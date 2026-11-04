"""MGeo-center lane changes with checked world boundaries and corridor gates."""
import math
from bisect import bisect_left

from .lane_geometry import EPS, arc, sample, corridor_covers, cross
from .lattice_planner import LatticePlanner
from .corridor_diagnostics import inspect_corridor, _describe


def multiply(a,b):
    result=[0.]*(len(a)+len(b)-1)
    for i,x in enumerate(a):
        for j,y in enumerate(b):
            result[i+j]+=x*y
    return result


def derivative(coefficients):
    return [i*c for i,c in enumerate(coefficients) if i] or [0.]


def add(a,b,scale=1.):
    result=list(a)+[0.]*max(0,len(b)-len(a))
    for i,value in enumerate(b):
        result[i]+=scale*value
    return result


class WorldTransition:
    """Two-dimensional quintic Hermite with position/tangent/curvature boundaries."""
    def __init__(self,start,first,second,end,end_first,end_second,length):
        self.length=length
        self.end,self.end_first,self.end_second=end,end_first,end_second
        self.coefficients=[]
        for axis in range(2):
            c0=start[axis];c1=first[axis]*length;c2=.5*second[axis]*length**2
            r0=end[axis]-c0-c1-c2
            r1=end_first[axis]*length-c1-2*c2
            r2=end_second[axis]*length**2-2*c2
            self.coefficients.append([c0,c1,c2,10*r0-4*r1+.5*r2,
                                      -15*r0+7*r1-r2,6*r0-3*r1+.5*r2])

    def state(self,distance):
        u=max(0.,min(1.,distance/self.length))
        values=[]
        for order in range(3):
            axes=[]
            for coefficients in self.coefficients:
                for _ in range(order):
                    coefficients=derivative(coefficients)
                axes.append(LatticePlanner._polynomial_value(coefficients,u)/self.length**order)
            values.append(axes)
        return values

    def project_parameter(self,position):
        # Closest world boundary state, independent of road-vs-curve arc length.
        equation=[0.]
        for coefficients,value in zip(self.coefficients,position):
            relative=list(coefficients);relative[0]-=value
            equation=add(equation,multiply(relative,derivative(coefficients)))
        scale=max(map(abs,equation))
        roots=LatticePlanner._unit_roots([v/scale for v in equation]) if scale else []
        u=min([0.,1.]+roots,key=lambda u:sum(
            (LatticePlanner._polynomial_value(c,u)-v)**2 for c,v in zip(self.coefficients,position)))
        return u*self.length

    def maximum_curvature(self):
        x,y=self.coefficients
        dx,dy=derivative(x),derivative(y)
        ddx,ddy=derivative(dx),derivative(dy)
        numerator=add(multiply(dx,ddy),multiply(dy,ddx),-1)
        speed2=add(multiply(dx,dx),multiply(dy,dy))
        equation=add(multiply(derivative(numerator),speed2),
                     multiply(numerator,derivative(speed2)),-1.5)
        scale=max(map(abs,equation))
        roots=LatticePlanner._unit_roots([v/scale for v in equation]) if scale else []
        ds=derivative(speed2);scale=max(map(abs,ds))
        speed_roots=LatticePlanner._unit_roots([v/scale for v in ds]) if scale else []
        if min(LatticePlanner._polynomial_value(speed2,u) for u in [0.,1.]+speed_roots)<=1e-12:
            return math.inf
        return max(abs(LatticePlanner._polynomial_value(numerator,u))/
                   LatticePlanner._polynomial_value(speed2,u)**1.5 for u in [0.,1.]+roots)


    def corridor_safe(self,cells):
        """Clip the entire polynomial against the convex-cell union in u.

        Published chords alone cannot exclude an excursion between samples.
        The sampled gate is unchanged. This additional continuous check
        admits only floating-point roundoff, so an EPS-accepted excursion
        cannot become an invalid start at a differently sampled next cycle.
        """
        controls=[[sum(c[i]*math.comb(k,i)/math.comb(5,i) for i in range(k+1))
                   for k in range(6)] for c in self.coefficients]
        xmin,xmax=min(controls[0]),max(controls[0]);ymin,ymax=min(controls[1]),max(controls[1])
        intervals=[]
        for cell in cells:
            if (xmax<min(p[0] for p in cell)-EPS or xmin>max(p[0] for p in cell)+EPS or
                    ymax<min(p[1] for p in cell)-EPS or ymin>max(p[1] for p in cell)+EPS):
                continue
            area=sum(cross(a,b) for a,b in zip(cell,cell[1:]+cell[:1]))
            if abs(area)<=1e-10:
                continue
            sign=1. if area>0 else -1.;polynomials=[];cuts={0.,1.}
            for a,b in zip(cell,cell[1:]+cell[:1]):
                edge=(b[0]-a[0],b[1]-a[1])
                polynomial=[sign*(edge[0]*y-edge[1]*x) for x,y in zip(*self.coefficients)]
                coordinate_scale=max(abs(self.coefficients[0][0]),abs(self.coefficients[1][0]),
                                     abs(a[0]),abs(a[1]),1.)
                roundoff=min(EPS,8*math.ulp(coordinate_scale)*math.hypot(*edge))
                polynomial[0]+=sign*(edge[1]*a[0]-edge[0]*a[1])+roundoff
                scale=max(map(abs,polynomial))
                if scale:cuts.update(LatticePlanner._unit_roots([v/scale for v in polynomial]))
                polynomials.append(polynomial)
            cuts=sorted(cuts)
            for low,high in zip(cuts,cuts[1:]):
                if all(LatticePlanner._polynomial_value(c,(low+high)/2)>=0. for c in polynomials):
                    intervals.append((low,high))
        covered=0.
        for low,high in sorted(intervals):
            if low>covered+1e-12:return False,(low+covered)/2
            covered=max(covered,high)
            if covered>=1-1e-12:return True,None
        return False,(covered+1)/2


class AdjacentLanePlanner(LatticePlanner):
    """Keep the original Dijkstra route; only replace the local avoidance path.

    The corridor covers Ego reference centers between two actual lane centers.
    It is not a verified lane-boundary or full-body swept-polygon guarantee.
    """
    def __init__(self,network,*,min_transition_time_sec=3.,**geometry):
        if not math.isfinite(min_transition_time_sec) or min_transition_time_sec<=0:
            raise ValueError('min_transition_time_sec must be finite and positive')
        self.network=network
        self.min_transition_time_sec=min_transition_time_sec
        super().__init__(**geometry)

    def reset(self):
        super().reset()
        self._lane_pair=None
        self._world_transition=None
        self._transition_start_progress=None
        self._transition_end_progress=None
        self._transition_end_station=None
        self._transition_last_parameter=0.
        self._entry_completion_progress=None
        self._recovery_completion_progress=None
        self._phase='CLEAR'
        self._corridor_debug_cache=None
        self._maneuver_context={}

    def _result(self,*args,**kwargs):
        result=super()._result(*args,**kwargs)
        result.update(lane_change_phase='BLOCKED' if result['blocked'] else 'CLEAR',
                      selected_side='',adjacent_link_id='',original_link_id='',
                      adjacent_separation=math.nan,drivable_safe=False,
                      original_lane_points=[],adjacent_lane_points=[],
                      entry_points=[],hold_points=[],recovery_points=[],recovery_gate_points=[],
                      corridor_diagnostics=None,corridor_evaluation=None)
        return result

    def _boundary(self,position,yaw):
        magnitude,tangential,kappa=1.,0.,0.
        if self._world_transition is not None:
            delta=max(self._transition_last_parameter,self._world_transition.project_parameter(position))
            self._transition_last_parameter=delta
            if delta<self._world_transition.length:
                _,first,second=self._world_transition.state(delta)
                magnitude=math.hypot(*first)
                if magnitude<=1e-9:
                    return None
                tangential=sum(a*b for a,b in zip(first,second))/magnitude
                kappa=(first[0]*second[1]-first[1]*second[0])/magnitude**3
        tangent=(math.cos(yaw),math.sin(yaw));normal=(-tangent[1],tangent[0])
        first=[magnitude*v for v in tangent]
        second=[tangential*t+kappa*magnitude*magnitude*n for t,n in zip(tangent,normal)]
        return first,second

    def _candidate(self,pair,position,yaw,ego_s,progress,front_s,rear_s,obstacles,
                   speed_kmh,horizon,recovery):
        lane='original' if recovery else 'adjacent'
        at_ego=pair.at(ego_s)
        if at_ego is None:
            return None
        target_geometry=pair.original if recovery else pair.adjacent
        target_index=pair.original_index if recovery else pair.adjacent_index
        nearest_target_s=max(0.,min(target_index.lengths[-1],target_index.project(position)))
        nearest_target=sample(target_geometry,nearest_target_s)
        if nearest_target is None:
            return None
        target_at=nearest_target[0]
        distance=math.dist(position,target_at[:2])
        boundary=self._boundary(position,yaw)
        if boundary is None:
            return None
        first,second=boundary
        target_tangent=nearest_target[1]
        aligned=(sum(a*b for a,b in zip((math.cos(yaw),math.sin(yaw)),target_tangent))>1-1e-5)
        on_center=distance<=1e-5 and aligned
        entry_completion=(self._entry_completion_progress
                          if self._phase!='RECOVERING' and self._entry_completion_progress is not None
                          else self._transition_end_progress)
        holding=(not recovery and self._lane_pair is not None and
                 distance<=self.recovery_threshold_m and
                 (self._phase=='ADJACENT_LANE_HOLD' or
                  (entry_completion is not None and progress>=entry_completion)))
        available=min(horizon,pair.length-ego_s)
        if not recovery and not holding and not on_center:
            available=min(available,math.nextafter(front_s,-math.inf))
        if available<=1e-6:
            return None
        continuing=(self._world_transition is not None and
                    (self._phase=='RECOVERING')==recovery)
        if continuing:
            preferred=max(0.,self._transition_end_station-ego_s)
        elif holding:
            # An acquired lane needs a bounded residual correction, not a
            # fresh five-metre entry after the zero-length HOLD optimization.
            preferred=max(.25,self._required_transition_length(distance))
        else:
            preferred=max(self.transition_before_m,self._required_transition_length(distance),
                          speed_kmh/3.6*self.min_transition_time_sec)
        lengths=[0.] if on_center else [min(available,max(preferred,.25))]
        if not on_center and lengths[0]<available:
            lengths.extend(lengths[0]+(available-lengths[0])*i/12 for i in range(1,13))
        if continuing and not recovery and not on_center and preferred<=available and lengths[0]>.25:
            # Carried heading/curvature makes feasibility nonmonotonic in
            # length. Allow at most 50% shortening of the approved remainder,
            # retaining the 0.25 m floor and every downstream safety gate.
            minimum=max(.25,lengths[0]*.5)
            lengths.extend(lengths[0]-(lengths[0]-minimum)*i/12 for i in range(1,13))
            lengths.sort(key=lambda length:(abs(length-preferred),length<preferred))
        if continuing and not recovery and not on_center and preferred<=available:
            # Constant-turn geometry gives a terminal longitudinal span
            # d*cot(|heading error|/2). It is a proposal, not an approval or
            # a new minimum: all existing gates and the half-remainder floor
            # still apply. This avoids coarse horizon-grid gaps near the goal.
            dot=sum(a*b for a,b in zip((math.cos(yaw),math.sin(yaw)),target_tangent))
            sine=abs(math.cos(yaw)*target_tangent[1]-math.sin(yaw)*target_tangent[0])
            proposals=[self._required_transition_length(distance)]
            if sine>1e-9:
                proposals.append(distance*(1+dot)/sine)
            floor=max(.25,min(available,max(preferred,.25))*.5)
            lengths=sorted(set(lengths+[v for v in proposals if floor<=v<=available]),
                           key=lambda length:(abs(length-preferred),length<preferred))
        best=None
        envelope=pair.corridor_envelope(max(0.,ego_s-.5),ego_s+min(horizon,pair.length-ego_s))
        quads=envelope['quads']
        start_in_corridor=corridor_covers([list(position),list(position)],quads)
        boundaries=[(first,second,'carried',1.)]
        if self._world_transition is not None and not on_center:
            magnitude=math.hypot(*first)
            kappa=(first[0]*second[1]-first[1]*second[0])/magnitude**3
            heading_tangent=(math.cos(yaw),math.sin(yaw))
            # The previous polynomial's parameter speed/acceleration are not
            # vehicle speed/acceleration. With tracking error preserve actual
            # pose, heading and signed geometric curvature (G2), while allowing
            # canonical arc-length boundary derivatives as a fallback.
            start_kappa=kappa
            canonical=(list(heading_tangent),[-kappa*heading_tangent[1],kappa*heading_tangent[0]])
            boundaries.append((*canonical,'geometric',1.))
            boundaries.extend((*canonical,'corridor_handles',factor) for factor in (1.,.5))
        for base_first,base_second,mode,end_scale in boundaries:
            for length in lengths:
                first,second=base_first,base_second
                end_speed=end_scale
                endpoint=pair.at(ego_s+length)
                if endpoint is None:
                    continue
                end,end_tangent=nearest_target if on_center else endpoint[lane]
                end_s=nearest_target_s if on_center else ego_s+length if recovery else endpoint['adjacent_s']
                if mode=='corridor_handles':
                    # A quintic's first two Bezier controls are p+H*t/5 and
                    # p+2H*t/5+k*H^2*n/20. Bound their handles by the actual
                    # strip, rather than forcing the old parameter speed.
                    for ratio in (1.,.5,.25,.125,.0625,.03125,.015625):
                        handle=length*ratio
                        p1=[position[i]+handle*heading_tangent[i]/5 for i in range(2)]
                        normal=(-heading_tangent[1],heading_tangent[0])
                        p2=[position[i]+2*handle*heading_tangent[i]/5+start_kappa*handle**2*normal[i]/20 for i in range(2)]
                        if corridor_covers([position,p1,p2],quads):
                            break
                    scale=ratio*end_scale
                    first=[scale*v for v in heading_tangent]
                    second=[start_kappa*scale**2*v for v in normal]
                    # Last controls stay on the actual incoming target segment.
                    # No smoothed normal or invented lane width is involved.
                    segment=max(0,bisect_left(target_index.lengths,end_s)-1)
                    straight=end_s-target_index.lengths[segment]
                    end_speed=min(1.,2.5*straight/length) if length else 1.
                # The actual MGeo suffix is piecewise linear. Inside its current
                # segment the boundary curvature is zero; importing a smoothed
                # triplet curvature here can push the quintic outside that segment.
                # All map vertices are still covered by the world polyline gate.
                end_curvature=0.
                end_second=(-end_curvature*end_tangent[1],end_curvature*end_tangent[0])
                transition=None
                retained_suffix=False
                if length>0:
                    if continuing and mode=='carried' and abs(length-preferred)<=1e-9:
                        previous=self._world_transition
                        delta=self._transition_last_parameter
                        old_position,old_first,old_second=previous.state(delta)
                        remainder=previous.length-delta
                        same_heading=sum(a*b for a,b in zip(first,old_first))>0 and abs(
                            first[0]*old_first[1]-first[1]*old_first[0])<=1e-9
                        if (remainder>1e-8 and math.dist(position,old_position)<=1e-8 and same_heading
                                and math.dist(end[:2],previous.end[:2])<=1e-8):
                            # Road station and curve parameter are different. Retain
                            # the exact suffix using its own remaining parameter;
                            # fitting it again with a road-length parameter changes
                            # the polynomial even for perfect tracking.
                            transition=WorldTransition(position,old_first,old_second,
                                previous.end,previous.end_first,previous.end_second,remainder)
                            retained_suffix=True
                    if transition is None:
                        transition=WorldTransition(position,first,second,end,[v*end_speed for v in end_tangent],end_second,length)
                    count=max(2,math.ceil(length/.25))
                    transition_points=[[*transition.state(transition.length*i/count)[0],float(end[2])]
                                       for i in range(count+1)]
                    profile_curvature=transition.maximum_curvature()
                else:
                    transition_points=[list(end)]
                    profile_curvature=0.
                last=pair.at(ego_s+min(horizon,pair.length-ego_s))
                if last is None:
                    continue
                suffix_end=ego_s+min(horizon,pair.length-ego_s) if recovery else last['adjacent_s']
                suffix=pair.suffix(lane,end_s,suffix_end)
                candidate=transition_points+[p for p in suffix[1:]]
                polyline_curvature=self.maximum_curvature(candidate)
                kappa=max(profile_curvature,polyline_curvature)
                clearance=self._minimum_obstacle_distance(candidate,obstacles)
                road_safe=corridor_covers(candidate,quads)
                # Check forward motion against original route stations, including
                # world quintics that could otherwise loop inside the same strip.
                projected=[pair.original_index.project(p) for p in candidate]
                forward=all(b>=a-1e-6 for a,b in zip(projected,projected[1:]))
                if transition is not None:
                    tangent=at_ego['original'][1]
                    velocity=add([tangent[0]*v for v in derivative(transition.coefficients[0])],
                                 [tangent[1]*v for v in derivative(transition.coefficients[1])])
                    critical=derivative(velocity);scale=max(map(abs,critical))
                    roots=self._unit_roots([v/scale for v in critical]) if scale else []
                    forward=forward and min(self._polynomial_value(velocity,u) for u in [0.,1.]+roots)>1e-9
                complete=on_center or preferred<=available
                continuous_witness=None
                if (transition is not None and road_safe and forward and complete and
                        kappa<=self.usable_curvature_limit and clearance>=self.required_clearance_m):
                    road_safe,gap=transition.corridor_safe(quads)
                    if not road_safe:
                        continuous_witness=dict(parameter_fraction=gap,
                            world_xy=transition.state(gap*transition.length)[0])
                reason='+'.join(name for failed,name in (
                    (not complete,'ENTRY_DEADLINE'),(clearance<self.required_clearance_m,'CLEARANCE'),
                    (kappa>self.usable_curvature_limit,'CURVATURE'),(not road_safe,'DRIVABLE_CORRIDOR'),
                    (not forward,'DIRECTION_CONTINUITY')) if failed)
                result=dict(points=candidate,offset=0. if recovery else endpoint['separation'],clearance=clearance,
                    curvature=kappa,profile_curvature=profile_curvature,polyline_curvature=polyline_curvature,
                    feasible=not reason,reject_reason=reason,pair=pair,transition=transition,
                    continuous_corridor_witness=continuous_witness,
                    boundary_mode='retained_suffix' if retained_suffix else mode,end_parameter_scale=end_speed,start_first=transition.state(0.)[1] if transition else first,
                    start_second=transition.state(0.)[2] if transition else second,
                    previous_curve_parameter=self._transition_last_parameter,target_distance=distance,
                    target_heading_error_rad=math.atan2(math.sin(yaw-math.atan2(target_tangent[1],target_tangent[0])),
                        math.cos(yaw-math.atan2(target_tangent[1],target_tangent[0]))),
                    transition_length=length,end_station=ego_s+length,entry_length=math.nan if recovery else length,
                    full_offset_start_s=math.nan if recovery else length,
                    recovery_length=length if recovery else math.nan,road_safe=road_safe,
                    corridor_quads=quads,
                    corridor_envelope_info=dict(station_count=len(envelope['stations']),quad_count=len(quads),
                        normalization=envelope['normalization'],failure=envelope['failure']),
                    corridor_station_range=[max(0.,ego_s-.5),ego_s+min(horizon,pair.length-ego_s)],
                    separation=at_ego['separation'],current_ids=pair.current_ids(ego_s),holding=holding or on_center,entry_points=transition_points if length and not recovery and not holding else [],
                    hold_points=(candidate if holding else suffix) if not recovery else [],recovery_points=transition_points if recovery else [])
                if best is None or (not reason,kappa<=self.usable_curvature_limit,clearance)>\
                        (best['feasible'],best['curvature']<=self.usable_curvature_limit,best['clearance']):
                    best=result
                if not reason:
                    return result
                if not start_in_corridor:
                    return best
        return best

    def _package(self,candidate,status,current,front,rear,reason=''):
        if candidate is None:
            return self._stop([],current,0.,math.inf,front,rear,reason or 'NO_ADJACENT_LANE')
        result=self._result(status,candidate['points'],candidate['offset'],candidate['clearance'],
                            candidate['curvature'],front,rear,reason)
        result=self._transition_diagnostics(result,candidate)
        pair=candidate['pair']
        result['maneuver_evaluation']={name:candidate.get(name) for name in (
            'boundary_mode','start_first','start_second','previous_curve_parameter',
            'target_distance','target_heading_error_rad','transition_length','end_station')}
        result['maneuver_evaluation'].update(
            curve_parameter_length=candidate['transition'].length if candidate['transition'] else 0.,
            endpoint=candidate['transition'].end if candidate['transition'] else None,
            endpoint_tangent=candidate['transition'].end_first if candidate['transition'] else None,
            endpoint_second=candidate['transition'].end_second if candidate['transition'] else None)
        # Inspect the same evaluated candidate as the existing scalar topics,
        # including rejected geometry. Nothing from this report decides safety.
        quads=candidate['corridor_quads']
        key=(pair.original,pair.adjacent,candidate['points'],quads,candidate['corridor_envelope_info'],candidate.get('continuous_corridor_witness'))
        if self._corridor_debug_cache is None or key!=self._corridor_debug_cache[0]:
            self._corridor_debug_cache=(key,inspect_corridor(
                candidate['points'],quads,pair,candidate['corridor_station_range']))
            self._corridor_debug_cache[1]['envelope']=candidate['corridor_envelope_info']
            witness=candidate.get('continuous_corridor_witness')
            if witness is not None:
                from .corridor_diagnostics import _distance
                point=witness['world_xy'];u=witness['parameter_fraction']
                index=int(u*max(2,math.ceil(candidate['transition_length']/.25)))
                detail=_describe(index,point,_distance(point,quads),pair,quads,
                                 curve_parameter_fraction=u)
                self._corridor_debug_cache[1].update(safe=False,reason='UNCOVERED_TRANSITION_INTERIOR',
                    failed_index=index,first_failure=detail,continuous_transition_failure=detail)

        result.update(corridor_diagnostics=self._corridor_debug_cache[1],
                      corridor_evaluation=dict(points=candidate['points'],quads=quads,
                          original_lane_points=pair.original,adjacent_lane_points=pair.adjacent,
                          lane_pair_ids=pair.ids,side=pair.side,station_range_m=candidate['corridor_station_range']))
        result.update(lane_change_phase=('BLOCKED' if status=='BLOCKED' else 'RECOVERING' if status=='RECOVERING' else
            'ADJACENT_LANE_HOLD' if candidate.get('holding') else 'ENTRY'),
            selected_side=pair.side,adjacent_link_id=candidate.get('current_ids',pair.ids[0])[1],
            original_link_id=candidate.get('current_ids',pair.ids[0])[0],
            adjacent_separation=candidate.get('separation',math.nan),drivable_safe=candidate.get('road_safe',False),
            original_lane_points=pair.original,adjacent_lane_points=pair.adjacent,
            entry_points=candidate.get('entry_points',[]),hold_points=candidate.get('hold_points',[]),
            recovery_points=candidate.get('recovery_points',[]))
        if self._rear_point is not None:
            gate=pair.at(pair.original_index.project(self._rear_point)+self.obstacle_rear_margin_m)
            if gate is not None:
                result['recovery_gate_points']=[gate['original'][0],gate['adjacent'][0]]
        return result

    def plan(self,reference_points,obstacles,current_position=None,*,current_heading=None,
             obstacle_groups=None,road_progress_m=None,current_link_id='',route_link_ids=(),speed_kmh=0.):
        self._maneuver_context={}
        if (current_position is None or current_heading is None or len(reference_points)<3
                or not all(math.isfinite(v) for v in (*current_position,current_heading,speed_kmh))
                or speed_kmh<0):
            self.break_release_confirmation()
            return None
        self._actual_offset=self._current_lateral_offset(reference_points,current_position)
        current=self._actual_offset
        self._road_progress_m=road_progress_m
        progress=road_progress_m if road_progress_m is not None else (
            self._lane_pair.original_index.project(current_position) if self._lane_pair is not None else 0.)
        original=reference_points;lengths=arc(original)
        ego_s_local=self._project_reference(original,lengths,current_position)[0]
        observed=self._update_extent(original,lengths,list(obstacles),obstacle_groups,ego_s_local)
        front=self._project_reference(original,lengths,self._front_point)[0] if self._front_point else math.nan
        rear=self._project_reference(original,lengths,self._rear_point)[0] if self._rear_point else math.nan
        rear_pending=self._rear_point is not None and ego_s_local<=rear+self.obstacle_rear_margin_m
        self._maneuver_context=dict(current_progress=progress,obstacle_front_s=front,obstacle_rear_s=rear,
                                    rear_pending=rear_pending,obstacle_observed=observed)
        collision=list(obstacles)
        if rear_pending and not observed:
            collision.extend(sorted(self._remembered_obstacle_points-{tuple(p) for p in obstacles}))
        if not observed and not rear_pending and self._lane_pair is None:
            result=self._result('CLEAR',original,0.,self._minimum_obstacle_distance(original,collision),
                                self.maximum_curvature(original),front,rear)
            return self._approve(result,original,current)
        if self._lane_pair is None:
            pairs=[p for side in ('left','right') if
                   (p:=self.network.pair(current_link_id,side,list(route_link_ids))) is not None]
        else:
            pairs=[self._lane_pair]
        if not pairs:
            return self._stop(original,current,0.,math.inf,front,rear,'NO_ADJACENT_LANE')
        recovery=not observed and not rear_pending and self._lane_pair is not None
        candidates=[]
        for pair in pairs:
            ego_s=pair.original_index.project(current_position)
            at_ego=pair.at(ego_s)
            if at_ego is None:
                continue
            candidate_progress=progress if road_progress_m is not None else ego_s
            candidate=self._candidate(pair,current_position,current_heading,ego_s,candidate_progress,front-ego_s_local,rear-ego_s_local,
                                      collision,speed_kmh,max(0.,arc(original)[-1]-ego_s_local),recovery)
            if (recovery and self._phase=='ADJACENT_LANE_HOLD' and
                    (candidate is None or not candidate['feasible'])):
                # Passing the obstacle rear permits recovery; it does not make
                # an outward HOLD correction boundary a safe recovery launch.
                # Finish a freshly checked HOLD correction before trying again.
                launch=candidate
                correction=self._candidate(pair,current_position,current_heading,ego_s,candidate_progress,
                    front-ego_s_local,rear-ego_s_local,collision,speed_kmh,
                    max(0.,arc(original)[-1]-ego_s_local),False)
                if correction is not None and correction['feasible']:
                    candidate=correction;recovery=False
                    self._maneuver_context.update(recovery_deferred=True,
                        recovery_reject_reason=launch['reject_reason'] if launch else 'NO_CANDIDATE')
            if candidate is not None:
                candidate['planning_progress']=candidate_progress
                if math.isfinite(candidate['full_offset_start_s']):
                    candidate['full_offset_start_s']+=ego_s_local
                candidates.append(candidate)
        safe=[c for c in candidates if c['feasible']]
        if not safe:
            best=max(candidates,key=lambda c:c['clearance']) if candidates else None
            result=self._package(best,'BLOCKED',current,front,rear,'NO_SAFE_FEASIBLE_CANDIDATE')
            self._blocked=True;self.break_release_confirmation()
            result['selected_offset_m']=current
            result['entry_points']=result['hold_points']=result['recovery_points']=[]
            if self._lane_pair is None:
                # Failed geometry is evaluated, not an approved lane selection.
                result['selected_side']='';result['adjacent_link_id']=''
                result['adjacent_separation']=math.nan
                result['adjacent_lane_points']=[]
                result['recovery_gate_points']=[]
            return result
        selected=min(safe,key=lambda c:abs(c['offset']-current))
        result=self._package(selected,'RECOVERING' if recovery else 'AVOIDING',current,front,rear)
        corridor_report={k:result[k] for k in ('corridor_diagnostics','corridor_evaluation','maneuver_evaluation')}
        result=self._approve(result,original,current)
        # RELEASE_PENDING rebuilds the public result in the legacy helper.
        # Keep current evaluated diagnostics even while approval is pending.
        result.update(corridor_report)
        if result['status']!='BLOCKED':
            progress=selected['planning_progress']
            if self._maneuver_start_global_progress_m is None:
                self._maneuver_start_global_progress_m=progress
            self._maneuver_progress_m=max(self._maneuver_progress_m,
                                         progress-self._maneuver_start_global_progress_m)
            # Keep ENTRY acquisition tied to its approved maneuver goal. The
            # short correction below still rebases every cycle, but must not
            # push this phase boundary ahead of an almost-centered vehicle.
            if not recovery and (self._entry_completion_progress is None or self._phase=='RECOVERING'):
                self._entry_completion_progress=progress+selected['transition_length']
            if recovery and self._recovery_completion_progress is None:
                self._recovery_completion_progress=progress+selected['transition_length']
            elif not recovery:
                self._recovery_completion_progress=None
            self._lane_pair=selected['pair']
            self._world_transition=selected['transition']
            self._transition_start_progress=progress
            self._transition_end_progress=progress+selected['transition_length']
            self._transition_end_station=selected['end_station']
            self._transition_last_parameter=0.
            self._phase=result['lane_change_phase']
            if recovery:
                pair=selected['pair']
                target=sample(pair.original,max(0.,min(pair.length,pair.original_index.project(current_position))))
                aligned=sum(a*b for a,b in zip((math.cos(current_heading),math.sin(current_heading)),target[1]))>1-1e-5
                acquired=(math.dist(current_position,target[0][:2])<=self.recovery_threshold_m and aligned and
                          progress>=self._recovery_completion_progress)
                result['recovery_completion_progress']=self._recovery_completion_progress
                result['recovery_acquired']=acquired
                if acquired:
                    # Only hand off after approving the actual-pose correction
                    # through every gate. CLEAR still publishes that checked
                    # correction this cycle, never an unchecked center snap.
                    result.update(status='CLEAR',lane_change_phase='CLEAR',blocked=False,
                                  obstacle_avoid_requested=False,selected_offset_m=0.,
                                  selected_side='',adjacent_link_id='',adjacent_separation=math.nan,
                                  adjacent_lane_points=[],entry_points=[],hold_points=[],
                                  recovery_points=[],recovery_gate_points=[])
                    result=self._approve(result,original,current)
                    self._lane_pair=None;self._world_transition=None;self._phase='CLEAR'
                    self._entry_completion_progress=None;self._recovery_completion_progress=None
        else:
            result['lane_change_phase']='BLOCKED'
            result['drivable_safe']=selected.get('road_safe',False)
            result['entry_points']=result['hold_points']=result['recovery_points']=[]
            if self._lane_pair is not None:
                result.update(selected_side=self._lane_pair.side,
                    adjacent_link_id=selected['current_ids'][1],adjacent_separation=selected['separation'],
                    original_lane_points=self._lane_pair.original,
                    adjacent_lane_points=self._lane_pair.adjacent)
        return result
