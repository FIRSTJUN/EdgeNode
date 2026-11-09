"""One static-obstacle planner; fixed lattice maneuvers and hard safety first."""
from dataclasses import dataclass
from collections import deque
import math

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_control.steering_command import to_morai_front_steer

from .avoidance_geometry import convex_covers, convex_hull, intersects, obstacle_polygons, polygon_distance, swept_body, inflate
from .lane_geometry import PolylineIndex, sample, cross, arc
from .lattice_path import (LatticeCandidate, CostWeights, center_suffix, control_local_path, curvatures,
                           fixed_path_suffix, heading_error, poses, reference_cost, transition)
from .vehicle_geometry import sonata_geometry


@dataclass(frozen=True)
class AvoidanceConfig:
    transition_lengths_m: tuple = (12., 16., 20.)
    prediction_distance_m: float = 80.
    obstacle_margin_m: float = .25
    reference_clearance_m: float = 1.20
    return_margin_m: float = .5
    road_margin_m: float = 0.
    avoid_speed_kmh: float = 3.
    deceleration_mps2: float = 2.
    reaction_time_s: float = .3
    max_lateral_acceleration_mps2: float = 1.5
    max_wheel_rate_radps: float = .6
    wheelbase_m: float = 2.84
    max_wheel_angle_rad: float = .610865
    max_front_steer_normalized: float = .70
    steering_sign: float = -1.
    control_rate_hz: float = 30.
    lookahead_min_m: float = 2.5
    lookahead_max_m: float = 10.
    lookahead_preview_time_sec: float = .45
    rollout_seconds: float = 2.
    command_delay_s: float = .1
    acquisition_lateral_m: float = .15
    acquisition_heading_rad: float = math.radians(8.)
    acquisition_end_region_m: float = 2.
    acquisition_confirmations: int = 3
    tracking_limit_m: float = .30

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if name=='transition_lengths_m':
                if not value or any(not math.isfinite(v) or v<=0 for v in value):
                    raise ValueError('positive transition lengths required')
            elif isinstance(value, bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('invalid '+name)
            elif name!='steering_sign' and value<0:
                raise ValueError('negative '+name)
        if (min(self.wheelbase_m,self.max_wheel_angle_rad,self.avoid_speed_kmh,
                self.deceleration_mps2,self.rollout_seconds,self.tracking_limit_m)<=0
                or self.control_rate_hz<=0
                or not 0<self.max_front_steer_normalized<=1 or self.acquisition_confirmations<1
                or self.max_wheel_angle_rad*self.max_front_steer_normalized>=math.pi/2
                or self.reference_clearance_m<1.20 or self.obstacle_margin_m<.25
                or self.steering_sign not in (-1.,1.)):
            raise ValueError('invalid safety/controller configuration')

    @property
    def curvature_limit(self):
        return math.tan(self.max_wheel_angle_rad*self.max_front_steer_normalized)/self.wheelbase_m


def bicycle_step(pose, curvature, distance):
    x,y,yaw=pose; end=yaw+curvature*distance
    if abs(curvature)<1e-12:
        return (x+distance*math.cos(yaw),y+distance*math.sin(yaw),end)
    return (x+(math.sin(end)-math.sin(yaw))/curvature,
            y+(math.cos(yaw)-math.cos(end))/curvature,end)


def _axle_bounds_separated(first,second,sagitta,obstacle,limit):
    """Sufficient separation proof; ambiguous/touching bounds use exact distance.

    The existing inflated axle is contained in this box. No obstacle distance
    or road containment diagnostic is replaced. Padding keeps rounding near a
    threshold in the exact branch, including large map coordinates.
    """
    xs,ys=zip(*obstacle)
    gap=max(min(xs)-max(first[0],second[0])-sagitta,
            min(first[0],second[0])-sagitta-max(xs),
            min(ys)-max(first[1],second[1])-sagitta,
            min(first[1],second[1])-sagitta-max(ys))
    magnitude=max(abs(v) for v in (*first[:2],*second[:2],*xs,*ys,sagitta,limit))
    return gap>limit+max(1e-9,32*math.ulp(magnitude))


class StaticObstaclePlanner:
    """NORMAL → AVOID_ENTRY → AVOID_HOLD → RETURN → NORMAL, or latched BLOCKED.

    Unknown boundary data NEVER approves avoidance. Offline tests may inject
    surveyed synthetic DrivableArea cells. No road width is derived from lanes.
    The high-level PlanningFSM remains solely the readiness/drive-lease arbiter.
    """
    def __init__(self, network, *, drivable_area=None, config=None, weights=None):
        self.network=network;self.drivable_area=drivable_area
        self.config=config or AvoidanceConfig();self.weights=weights or CostWeights()
        self.body=sonata_geometry()
        self.pp=PurePursuit(self.config.wheelbase_m,self.config.max_wheel_angle_rad)
        self.reset()

    def reset(self):
        self.state='NORMAL';self.reason='';self.selected=None;self.pair=None
        self.candidates=();self.remembered=();self.confirmations=0
        self.entry_commits=self.return_commits=0
        self._hold_path=();self._last_progress=None;self._min_clearance=math.inf
        self._original=();self.left_lane=self.right_lane=()
        self._hazards=()
        self._validated_paths={}
        self._validated_context=None
        self.generated_transition_count=0
        self.candidate_lane_batches=0
        self._control_horizon_m=20.
        self._entry_reserve_m=0.
        self._entry_observation_m=None
        self._observation_curvature_limit=None

    def invalidate_observation(self):
        self.confirmations=0

    def stop(self, reason):
        """Latch an integration-level validation failure until explicit reset."""
        return self._block(reason)

    def needs_validation_stop(self, reference, groups):
        """Cheap guard before potentially expensive candidate/revision checks."""
        if self.state=='BLOCKED':
            return False
        detected=obstacle_polygons(groups)
        if self.state=='NORMAL':
            return bool(self._normal_hazards(reference,self.remembered+detected))
        return any(not any(convex_covers(old,o) for old in self.remembered) for o in detected)

    def _result(self, path=(), *, speed_cap=None):
        active=self.state!='NORMAL' or speed_cap is not None
        return dict(state=self.state,points=[] if self.state=='BLOCKED' else path,
            blocked=self.state=='BLOCKED',reason=self.reason,active=active,
            speed_cap_kmh=self.config.avoid_speed_kmh if active and speed_cap is None else speed_cap,
            candidates=self.candidates,original_lane=self._original,left_lane=self.left_lane,
            right_lane=self.right_lane,selected_lane=self.pair.current_ids(
                max(0.,min(self.pair.length,self._last_progress or 0.)))[1] if self.pair else '',
            selected_side=self.selected.side if self.selected else '',
            return_path=self.selected.return_path if self.selected else (),
            clearance_m=self._min_clearance,curvature_limit=self.config.curvature_limit,
            entry_commits=self.entry_commits,return_commits=self.return_commits,
            generated_transition_count=self.generated_transition_count,
            candidate_lane_batches=self.candidate_lane_batches,
            boundary_source=self.drivable_area.source if self.drivable_area else None)

    def _block(self, reason):
        self.state='BLOCKED';self.reason=reason
        return self._result()

    def _body_safety(self, first, second, obstacles, check_road=True):
        swept=swept_body(self.body,first,second)
        if check_road:
            if self.drivable_area is None:
                return 'ROAD_BOUNDARY_UNAVAILABLE'
            if not self.drivable_area.covers(inflate(swept,self.config.road_margin_m)):
                return 'DRIVABLE_AREA'
        for obstacle in obstacles:
            clearance=polygon_distance(swept,obstacle)
            self._min_clearance=min(self._min_clearance,clearance)
            if clearance<=self.config.obstacle_margin_m:
                return 'BODY_COLLISION'
            # Preserve the previous 1.20 m reference clearance floor as an
            # additional gate. It is NEVER a substitute for the full body.
            angle=abs(heading_error(second[2],first[2]))
            chord=math.dist(first[:2],second[:2])
            sagitta=(chord/(2*math.sin(angle/2))*(1-math.cos(angle/2))) if angle>1e-9 else 0.
            if not _axle_bounds_separated(first,second,sagitta,obstacle,self.config.reference_clearance_m):
                axle=inflate((tuple(first[:2]),tuple(second[:2])),sagitta)
                if polygon_distance(axle,obstacle)<self.config.reference_clearance_m:
                    return 'REFERENCE_CLEARANCE'
        return ''

    def _geometry_context(self):
        # All geometric inputs are immutable value objects. Replacing the
        # boundary/map object also changes the context even for equal values.
        return (id(self.network),id(self.drivable_area),self.drivable_area,
                self.body,self.config,self._observation_curvature_limit)

    def _path_safety(self, points, obstacles, speed, check_road=True, geometry_check=True,
                     body_cache=None):
        try:
            ks=curvatures(points)
        except (ValueError,OverflowError):
            return 'INVALID_PATH'
        if not ks or not all(math.isfinite(k) for k in ks):
            return 'INVALID_PATH'
        if geometry_check:
            if body_cache is not None:
                context=(self._geometry_context(),obstacles,check_road)
                if body_cache.get('context')!=context:
                    body_cache.clear();body_cache['context']=context
            path_poses=poses(points)
            for a,b in zip(path_poses,path_poses[1:]):
                if body_cache is None:
                    reason=self._body_safety(a,b,obstacles,check_road)
                else:
                    key=(a,b)
                    cached=body_cache.get(key)
                    if cached is None:
                        previous=self._min_clearance;self._min_clearance=math.inf
                        try:
                            reason=self._body_safety(a,b,obstacles,check_road)
                        finally:
                            clearance=self._min_clearance
                            self._min_clearance=min(previous,clearance)
                        body_cache[key]=(reason,clearance)
                    else:
                        reason,clearance=cached
                        self._min_clearance=min(self._min_clearance,clearance)
                if reason:
                    return reason
        if (self._observation_curvature_limit is not None
                and max(map(abs,ks))>self._observation_curvature_limit):
            return 'OBSERVATION_CURVATURE_SCOPE'
        if max(map(abs,ks))>self.config.curvature_limit:
            return 'CURVATURE'
        velocity=speed/3.6
        if velocity**2*max(map(abs,ks))>self.config.max_lateral_acceleration_mps2:
            return 'LATERAL_ACCELERATION'
        # Steering rate is a controller response, not a derivative of a
        # discretized MGeo polyline (native corners can be micrometres from a
        # transition endpoint). The production-PP rollout checks wheel rate in
        # time with the measured initial turn state and unchanged physical gate.
        return ''

    def _normal_hazards(self, points, obstacles):
        path_poses=poses(points)
        return tuple(o for o in obstacles if any(
            self._body_safety(a,b,(o,),False) in ('BODY_COLLISION','REFERENCE_CLEARANCE')
            for a,b in zip(path_poses,path_poses[1:])))

    def _remember(self, detected):
        # No IDs are present in PoseArray. Merge overlapping observed static
        # solids conservatively, rather than accumulating one noisy box/frame.
        result=list(self.remembered)
        for obstacle in detected:
            matches=[i for i,old in enumerate(result)
                     if intersects(inflate(old,self.config.obstacle_margin_m),obstacle)]
            combined=convex_hull(list(obstacle)+[p for i in matches for p in result[i]])
            result=[old for i,old in enumerate(result) if i not in matches]+[combined]
        self.remembered=tuple(sorted(result))

    def _rollout_distance(self, speed):
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        return 2*velocity*self.config.rollout_seconds+lookahead+2*self.config.tracking_limit_m

    def _rollout(self, points, pose, speed, actual_curvature, obstacles):
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        if abs(actual_curvature)>self.config.curvature_limit:
            return 'ACTUAL_CURVATURE'
        try:
            points=control_local_path(points,pose,self._control_horizon_m)
        except ValueError:
            return 'INVALID_CONTROL_WINDOW'
        # Keep the PP target within the validated window for the full horizon;
        # short configured windows are rejected rather than modeled as a
        # longer path than the actual Control input.
        if arc(points)[-1]<self._rollout_distance(speed):
            return 'SHORT_CONTROL_HORIZON'
        dt=1/self.config.control_rate_hz
        previous_angle=math.atan(actual_curvature*self.config.wheelbase_m)
        applied_angle=previous_angle
        queue=deque()
        for tick in range(math.ceil(self.config.rollout_seconds/dt)):
            response=self.pp.compute(points,*pose,lookahead_distance_m=lookahead)
            if response is None:
                return 'PP_UNTRACKABLE'
            if response['nearest_distance_m']>self.config.tracking_limit_m:
                return 'TRACKING_ERROR'
            if abs(response['steering_normalized'])>self.config.max_front_steer_normalized:
                return 'STEERING_CLAMP'
            command=to_morai_front_steer(response['steering_normalized'],self.config.steering_sign,
                self.config.max_front_steer_normalized)
            queue.append((tick*dt+self.config.command_delay_s,
                command/self.config.steering_sign*self.config.max_wheel_angle_rad))
            while queue and queue[0][0]<=tick*dt+1e-12:
                applied_angle=queue.popleft()[1]
            curvature=math.tan(applied_angle)/self.config.wheelbase_m
            wheel_angle=applied_angle
            if abs(wheel_angle-previous_angle)/dt>self.config.max_wheel_rate_radps:
                return 'STEERING_RATE'
            previous_angle=wheel_angle
            next_pose=bicycle_step(pose,curvature,velocity*dt)
            reason=self._body_safety(pose,next_pose,obstacles)
            if reason:
                return reason
            pose=next_pose
        return ''

    def _generate(self, pair, pose, speed, actual_curvature, front, rear, horizon=None,
                  body_cache=None):
        self.candidate_lane_batches+=1
        ego=pair.original_index.project(pose[:2]);entry_row=pair.at(ego)
        if entry_row is None:
            raise ValueError('NO_LANE_CORRESPONDENCE')
        supported_horizon=min(pair.length-self.body.wheelbase_m-self.body.front_overhang_m-1.,
                              ego+self.config.prediction_distance_m)
        horizon=supported_horizon if horizon is None else horizon
        return_start=rear+self.body.rear_overhang_m+self.config.return_margin_m+self.config.lookahead_min_m
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        # A PP target shifted at t=0 demands an instantaneous turn from a
        # straight vehicle. Reserve a fixed preparation arc covering its actual
        # lookahead and delayed travel; carry MEASURED turn, never planned kappa.
        preparation=lookahead+velocity*self.config.command_delay_s
        divisions=math.ceil(preparation/.25)
        prepared=tuple(bicycle_step(pose,actual_curvature,preparation*i/divisions)
                       for i in range(divisions+1))
        prepared_pose=prepared[-1]
        prepared_s=pair.original_index.project(prepared_pose[:2])
        prefix=tuple((x,y,float(entry_row['original'][0][2])) for x,y,angle in prepared)
        candidates=[]
        # Lifetime: one synchronous Planning call (or this direct batch). Exact
        # segment checks may also serve the selected-path validation below.
        if body_cache is None:body_cache={}
        projection_cache={}
        for length in self.config.transition_lengths_m:
            entry=entry_path=()
            try:
                end=prepared_s+length
                target=pair.at(end);back=pair.at(return_start)
                if target is None or back is None:
                    raise ValueError('INSUFFICIENT_ADJACENT_SUPPORT')
                entry=prefix+transition(prepared_pose[:2],prepared_pose[2],actual_curvature,
                    pair.adjacent,target['adjacent_s'],length)[1:]
                self.generated_transition_count+=1
                if horizon>supported_horizon+1e-9:
                    raise ValueError('INSUFFICIENT_COMMON_HORIZON')
                entry_path=entry+center_suffix(pair.adjacent,target['adjacent_s'],
                    pair.at(horizon)['adjacent_s'])[1:]
                return_end=return_start+length
                target_back=sample(pair.original,return_end)
                if target_back is None or return_end>=horizon or return_start<=end:
                    raise ValueError('RETURN_UNAVAILABLE')
                tangent=back['adjacent'][1]
                recovery=transition(back['adjacent'][0],math.atan2(tangent[1],tangent[0]),0.,
                                    pair.original,return_end,length)
                self.generated_transition_count+=1
                hold=center_suffix(pair.adjacent,target['adjacent_s'],back['adjacent_s'])
                return_path=hold+recovery[1:]+center_suffix(pair.original,return_end,horizon)[1:]
                combined=entry+return_path[1:]
                # Compare every candidate over the SAME prediction support.
                # A control reserve after return is safety geometry, not extra
                # scored progress. Its vertices come only from the actual
                # original lane in this connected, permitted lane pair.
                cost,terms=reference_cost(combined,pair.original,pair.adjacent,self.weights,
                                          projection_cache=projection_cache)
                reserve=self._rollout_distance(speed)
                support_end=return_end+reserve
                reason=('RETURN_CONTROL_SUPPORT' if support_end>
                    pair.length-self.body.wheelbase_m-self.body.front_overhang_m-1. else '')
                extension=max(0.,support_end-horizon) if not reason else 0.
                if extension:
                    # Preview truncation is different from absent map data.
                    # Validate the entire appended real-lane tail below before
                    # committing. Never extrapolate the map or re-fit a path.
                    return_path+=center_suffix(pair.original,horizon,support_end)[1:]
                    combined=entry+return_path[1:]
                if not reason and end+self.body.wheelbase_m+self.body.front_overhang_m+\
                        self.config.obstacle_margin_m+self._entry_reserve_m>front:
                    reason='ENTRY_DEADLINE'
                if (not reason and self._entry_observation_m is not None
                        and end-ego+self.body.wheelbase_m+self.body.front_overhang_m+
                        self.config.obstacle_margin_m+self._entry_reserve_m>self._entry_observation_m):
                    reason='ENTRY_OBSERVATION_DISTANCE'
                if not reason and any(sum(a*b for a,b in zip((b[0]-a[0],b[1]-a[1]),
                    sample(pair.original,pair.original_index.project(a),pair.original_arc)[1]))<=0
                    for a,b in zip(combined,combined[1:])):
                    reason='DIRECTION_CONTINUITY'
                if not reason:
                    reason=self._path_safety(combined,self.remembered,speed,body_cache=body_cache)
                if not reason:
                    reason=self._path_safety(entry_path,self.remembered,speed,body_cache=body_cache)
                if not reason:
                    reason=self._rollout(entry_path,pose,speed,actual_curvature,self.remembered)
                candidates.append(LatticeCandidate(pair.side,combined,end,return_start,return_end,length,
                    cost,terms,reason,entry_path,return_path,reserve,extension))
            except (ValueError,TypeError,IndexError,OverflowError) as error:
                # A rejected partial entry remains useful diagnostic geometry;
                # it is never an executable fallback without a verified return.
                partial=entry_path or entry
                candidates.append(LatticeCandidate(pair.side,partial,math.nan,math.nan,math.nan,length,
                    math.inf,{},str(error),partial,()))
        return candidates

    def _acquired(self, pose, target, completion, progress):
        row=sample(target,PolylineIndex(target).project(pose[:2]))
        if row is None:
            self.confirmations=0;return False
        p,tangent=row;delta=(pose[0]-p[0],pose[1]-p[1])
        yaw=math.atan2(tangent[1],tangent[0])
        eligible=(progress>=completion-self.config.acquisition_end_region_m and
            abs(cross(tangent,delta))<=self.config.acquisition_lateral_m and
            abs(heading_error(pose[2],yaw))<=self.config.acquisition_heading_rad and
            math.cos(heading_error(pose[2],yaw))>0)
        self.confirmations=self.confirmations+1 if eligible else 0
        return self.confirmations>=self.config.acquisition_confirmations

    def plan(self, reference, groups, pose, *, speed_kmh=0., actual_curvature=0.,
             current_link_id='', route_link_ids=(), observation_valid=True, control_horizon_m=20.,
             hazard_reference=None, entry_reserve_m=0., stopping_required_m=None,
             entry_observation_m=None, observation_curvature_limit=None):
        normal_reference=reference
        if hazard_reference is not None:
            reference=hazard_reference
        self._min_clearance=math.inf
        if self.state=='BLOCKED':
            return self._result()
        try:
            if (any(v is not None and (not math.isfinite(v) or v<0)
                    for v in (entry_observation_m, observation_curvature_limit))
                    or not math.isfinite(entry_reserve_m) or entry_reserve_m<0
                    or (stopping_required_m is not None and (not math.isfinite(stopping_required_m)
                        or stopping_required_m<=0)) or len(pose)!=3 or len(reference)<2 or speed_kmh<0 or not math.isfinite(control_horizon_m)
                    or control_horizon_m<=0 or not all(math.isfinite(v)
                    for v in (*pose,speed_kmh,actual_curvature)) or any(len(p)!=3 or
                    not all(math.isfinite(v) for v in p) for p in reference)):
                return self._block('INVALID_INPUT')
            detected=obstacle_polygons(groups)
        except (ValueError,TypeError,OverflowError):
            return self._block('INVALID_OBSTACLE_FOOTPRINT')
        if not observation_valid:
            return self._block('OBSTACLE_OBSERVATION_INVALID_OR_STALE')
        self._control_horizon_m=control_horizon_m
        self._entry_reserve_m=entry_reserve_m
        self._entry_observation_m=entry_observation_m
        self._observation_curvature_limit=observation_curvature_limit
        context=self._geometry_context()
        if context!=self._validated_context:
            self._validated_paths.clear();self._validated_context=context
        body_cache={}
        old_memory=self.remembered
        self._remember(detected)
        # A nominal centerline can miss a collision caused by current tracking
        # error. Check the measured solid body before pruning passed objects
        # or entering an expensive path search. This is also active in NORMAL.
        actual_reason=self._body_safety(pose,pose,self.remembered,False)
        if actual_reason:
            return self._block(actual_reason)
        if self.remembered!=old_memory:
            self._validated_paths.clear()
            if self.pair is not None:
                hazards=[]
                for obstacle in self.remembered:
                    laterals=[]
                    for p in obstacle:
                        row=sample(self.pair.original,self.pair.original_index.project(p))
                        if row:
                            center,tangent=row
                            laterals.append(cross(tangent,(p[0]-center[0],p[1]-center[1])))
                    width=self.body.width_m/2+self.config.obstacle_margin_m
                    if laterals and min(laterals)<=width and max(laterals)>=-width:
                        hazards.append(obstacle)
                if hazards:self._hazards=tuple(hazards)
                reason=self._path_safety(self.selected.return_path,self.remembered,speed_kmh,body_cache=body_cache)
                if reason:return self._block('RETURN_RESERVATION_'+reason)
        if self.state=='NORMAL':
            self._original=reference
            index=PolylineIndex(reference);ego=index.project(pose[:2])
            self.remembered=tuple(o for o in self.remembered if max(index.project(p) for p in o)
                >=ego-self.body.rear_overhang_m-self.config.return_margin_m)
            # Preserve normal local geometry exactly; only obstacle hazards
            # activate this planner. No boundary assumption is used here.
            blocking=self._normal_hazards(reference,self.remembered)
            if not blocking:
                return self._result(normal_reference)
            self._hazards=blocking
            current_reference=sample(reference,ego)
            if current_reference is None:
                return self._block('REFERENCE_RANGE')
            tangent=current_reference[1]
            if abs(heading_error(pose[2],math.atan2(tangent[1],tangent[0])))>math.radians(30.):
                return self._block('DIRECTION_CONTINUITY')
            front=min(index.project(p) for o in blocking for p in o)
            actual_front=max(index.project(p) for p in self.body.polygon(*pose))
            distance=front-actual_front-self.config.obstacle_margin_m
            velocity=speed_kmh/3.6
            if stopping_required_m is not None and front-ego<=stopping_required_m:
                return self._block('S4_OBSTACLE_STOPPING_DISTANCE')
            if distance<=velocity*self.config.reaction_time_s+velocity**2/(2*self.config.deceleration_mps2):
                return self._block('BRAKING_DISTANCE')
            if hazard_reference is not None and self.drivable_area is None:
                return self._block('ROAD_BOUNDARY_UNAVAILABLE')
            pairs=[];all_candidates=[];lane_pairs=[]
            for side in ('left','right'):
                try:
                    pair=self.network.pair(current_link_id,side,list(route_link_ids))
                    if pair is None:
                        continue
                    lane_pairs.append(pair)
                    if side=='left':self.left_lane=pair.adjacent
                    else:self.right_lane=pair.adjacent
                except (ValueError,TypeError,IndexError,OverflowError):
                    continue
            # Both directions start on the same current original link. Use
            # one original-road prediction support across ALL compared paths.
            # A shorter adjacent chain cannot win a cheaper truncated score;
            # it is rejected, without extending unverified map geometry.
            common_horizon=max((min(p.length-self.body.wheelbase_m-self.body.front_overhang_m-1.,
                p.original_index.project(pose[:2])+self.config.prediction_distance_m)
                for p in lane_pairs),default=0.)
            for pair in lane_pairs:
                try:
                    original_index=pair.original_index
                    front_s=min(original_index.project(p) for o in blocking for p in o)
                    rear_s=max(original_index.project(p) for o in blocking for p in o)
                    candidates=self._generate(pair,pose,speed_kmh,actual_curvature,front_s,rear_s,common_horizon,
                                               body_cache=body_cache)
                    all_candidates.extend(candidates);pairs.append((pair,candidates))
                except (ValueError,TypeError,IndexError,OverflowError):
                    continue
            self.candidates=tuple(all_candidates)
            safe=[(c,p) for p,cs in pairs for c in cs if not c.rejection]
            if not safe:
                reason=('ROAD_BOUNDARY_UNAVAILABLE' if any(c.rejection=='ROAD_BOUNDARY_UNAVAILABLE'
                    for c in all_candidates) else 'NO_SAFE_CANDIDATE' if all_candidates else 'NO_VALID_ADJACENT_LANE')
                return self._block(reason)
            if speed_kmh>self.config.avoid_speed_kmh+.2:
                # Decelerate before committing the lane change. A safe prefix
                # ends before the obstacle; no normal path through it is sent.
                stop_s=front-self.body.wheelbase_m-self.body.front_overhang_m-self.config.obstacle_margin_m-1.
                if stop_s<=ego+1.:
                    return self._block('BRAKING_DISTANCE')
                self.reason='DECELERATING'
                return self._result(center_suffix(reference,ego,stop_s),speed_cap=self.config.avoid_speed_kmh)
            self.selected,self.pair=min(safe,key=lambda row:row[0].cost)
            # Check the exact future RETURN representation while search is
            # already stopped. Most segments share this call's exact memo with
            # the full candidate, but endpoint poses are checked independently.
            reason=self._path_safety(self.selected.return_path,self.remembered,speed_kmh,
                                     body_cache=body_cache)
            if reason:
                return self._block('RETURN_RESERVATION_'+reason)
            self._validated_paths[(id(self.selected.return_path),self.remembered)]=(self.selected.return_path,
                                                                                 self.selected.return_path)
            # Progress belongs to this lane-pair coordinate frame. A later
            # independent maneuver may start on a different current link.
            self._last_progress=None
            self._original=self.pair.original
            # Acquisition may occur before the quintic endpoint. Retain the
            # same fixed entry/adjacent geometry through HOLD: no target jump
            # to a separately sampled centerline and no new long validation.
            self._hold_path=self.selected.entry_path
            self.state='AVOID_ENTRY';self.reason='';self.entry_commits=1;self.return_commits=0
        pair=self.pair;progress=pair.original_index.project(pose[:2])
        return_rollout_verified=False
        if not math.isfinite(progress):
            return self._block('INVALID_PROGRESS')
        if self._last_progress is not None and progress<self._last_progress-self.config.tracking_limit_m:
            return self._block('PROGRESS_REGRESSION')
        self._last_progress=progress
        current_reason=self._body_safety(pose,pose,self.remembered)
        if current_reason:
            return self._block(current_reason)
        if self.state=='AVOID_ENTRY' and self._acquired(pose,pair.adjacent,self.selected.entry_end_s,progress):
            self.state='AVOID_HOLD';self.confirmations=0
        if self.state=='AVOID_HOLD':
            rear=max(pair.original_index.project(p) for o in self._hazards for p in o)
            rear_cleared=min(pair.original_index.project(p) for p in self.body.polygon(*pose))>rear+self.config.return_margin_m
            if rear_cleared:
                reason=self._rollout(self.selected.return_path,pose,speed_kmh,actual_curvature,self.remembered)
                if reason:
                    return self._block('RETURN_'+reason)
                return_rollout_verified=True
                self.state='RETURN';self.confirmations=0;self.return_commits+=1
            elif progress>self.selected.return_start_s:
                return self._block('RETURN_RESERVATION_INVALIDATED')
        if self.state=='RETURN' and self._acquired(pose,pair.original,self.selected.return_end_s,progress):
            # Explicit acquisition guarantees progress back to NORMAL, rather
            # than hoping a cost weight eventually completes the maneuver.
            self.state='NORMAL';self.reason='';self.remembered=();self.selected=None
            self.pair=None;self.candidates=();self._hold_path=()
            return self._result(normal_reference)
        path=(self.selected.entry_path if self.state=='AVOID_ENTRY' else
              self._hold_path if self.state=='AVOID_HOLD' else self.selected.return_path)
        index=PolylineIndex(path);station=index.project(pose[:2]);row=sample(path,station)
        if row is None or math.dist(pose[:2],row[0][:2])>self.config.tracking_limit_m:
            return self._block('TRACKING_ERROR')
        if abs(heading_error(pose[2],math.atan2(row[1][1],row[1][0])))>math.radians(30.):
            return self._block('DIRECTION_CONTINUITY')
        # Revalidate the fixed future geometry against changed obstacles. Never
        # refit its endpoints to tracking error or flip the committed side.
        # The approved geometry is fixed. Re-scan full remaining geometry only
        # when the obstacle solids/path change; current body and PP response
        # remain checked on every cycle. Kinematics still use actual speed.
        key=(id(path),self.remembered)
        if key not in self._validated_paths:
            future=fixed_path_suffix(path,station)
            reason=self._path_safety(future,self.remembered,speed_kmh,body_cache=body_cache)
            if not reason:self._validated_paths[key]=(path,future)
        else:
            reason=self._path_safety(self._validated_paths[key][1],(),speed_kmh,geometry_check=False)
        # The entry into RETURN above already rolled out this exact path with
        # the same pose/speed/turn/obstacles in this synchronous call.
        if not reason and not return_rollout_verified:
            reason=self._rollout(path,pose,speed_kmh,actual_curvature,self.remembered)
        return self._block(reason) if reason else self._result(path)
