"""Phase A: keep the normal path and brake for hazards; no lateral search.

The v61 source modules are imported unchanged for body, source-time and map
contracts. Observed occupancy is an exact union in spatial tiles, without
convex filling or costly triangulation. Missing observations never clear it.
"""
import math,time
from shapely.geometry import Polygon,LineString,box
from shapely.ops import unary_union
from shapely.prepared import prep
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from edgenode_planning.avoidance_geometry import obstacle_polygons,inflate,swept_body
from edgenode_planning.lane_geometry import PolylineIndex,arc,sample
from edgenode_planning.lattice_path import poses
from edgenode_planning.route_clearance import boundary_contacts,_permission
from .provenance import implementation_sha256
from .observation_memory import NormalObservationMemory


class ExactObservedTiles:
    """Retain every padded observation; no temporal or unobserved free-space erase."""
    def __init__(self,size=4.):self.size=size;self.cells={};self.geometry=None
    def update(self,observed,index,ego,rear,padding):
        size=self.size;changed=False
        threshold=ego-rear-.5
        for key in list(self.cells):
            i,j=key;corners=((i*size,j*size),((i+1)*size,j*size),((i+1)*size,(j+1)*size),(i*size,(j+1)*size))
            if all(index.project(p)<threshold for p in corners):del self.cells[key];changed=True
        for polygon in observed:
            shape=Polygon(inflate(polygon,padding));x0,y0,x1,y1=shape.bounds
            for i in range(math.floor(x0/size),math.floor(x1/size)+1):
                for j in range(math.floor(y0/size),math.floor(y1/size)+1):
                    part=shape.intersection(box(i*size,j*size,(i+1)*size,(j+1)*size))
                    if part.is_empty or part.area<=0.:continue
                    old=self.cells.get((i,j))
                    if old is not None and old.covers(part):continue
                    self.cells[i,j]=part if old is None else old.union(part);changed=True
        if changed or self.geometry is None:self.geometry=unary_union(list(self.cells.values()))
        return self.geometry


class NormalSafetyPlanner(StaticObstaclePlanner):
    """Interface adapter for v61's route/FSM/Control; never calls its plan/search."""
    complete_offset_search=False
    def __init__(self,network,*,config,weights,body,route_provider):
        self.route_provider=route_provider
        super().__init__(network,config=config,weights=weights);self.body=body
    def reset(self):
        super().reset();self.occupancy=NormalObservationMemory();self.previous_pose=None;self.normal_cap=None;self.normal_debug={}
    def _result(self,path=(),*,speed_cap=None):
        result=super()._result(path,speed_cap=None)
        result.update(active=self.state=='BLOCKED',speed_cap_kmh=0. if self.state=='BLOCKED' else self.normal_cap,
            selected_strategy='NORMAL_SAFETY_ONLY',static_avoidance_method='NONE_PHASE_A',
            phase_a_implementation_sha256=implementation_sha256(),phase_a_normal_safety=self.normal_debug)
        return result
    def needs_validation_stop(self,reference,groups):
        # This NORMAL adapter checks the admitted frame synchronously before
        # any path/speed publish. There is no asynchronous lateral revision.
        return False
    def _search_entry(self,*args,**kwargs):raise RuntimeError('PHASE_A_LATERAL_SEARCH_FORBIDDEN')
    def _generate_return(self,*args,**kwargs):raise RuntimeError('PHASE_A_LATERAL_RETURN_FORBIDDEN')
    def _map_reason(self,first,second):
        polygon=swept_body(self.body,first,second);boundaries=self.network.boundaries
        road=getattr(self.network,'road_corridor',None)
        if road is None or not road.covers(polygon):return 'NORMAL_ROAD_CONTAINMENT_UNPROVEN'
        reason=boundaries.swept_reason(polygon)
        if reason.startswith('PHYSICAL_BOUNDARY_COLLISION'):return reason
        contacts=boundary_contacts(boundaries,polygon)
        if not contacts:return ''
        route=self.route_provider() or {};allowed=set()
        for certificate in route.get('normal_merge_corrections',[]):
            original=certificate['original_link_id'];target=certificate['target_link_id'];side=certificate['direction']
            source=_permission(boundaries,original,target,side)
            if source is None or source['ID']!=certificate['shared_boundary_id']:continue
            index=PolylineIndex(self.network.links[original]['points']);station=index.project(second[:2])
            if certificate['merge_start_m']-self.body.wheelbase_m-self.body.front_overhang_m<=station<=certificate['merge_start_m']+certificate['transition_m']+self.body.rear_overhang_m:
                allowed.add(source['ID'])
        forbidden=contacts-allowed
        return 'NORMAL_FORBIDDEN_BOUNDARY:'+next(iter(sorted(forbidden))) if forbidden else ''
    def plan(self,reference,groups,pose,**kwargs):
        began=time.perf_counter();speed=kwargs.get('speed_kmh',0.);now=kwargs.get('now_sec',time.monotonic())
        if self.state=='BLOCKED':return self._result()
        if (len(reference)<2 or not all(math.isfinite(v) for v in (*pose,speed,now)) or speed<0 or
            not kwargs.get('observation_valid',True) or self._last_now is not None and now<self._last_now):
            return self._block('NORMAL_INVALID_OR_STALE_INPUT')
        detected=obstacle_polygons(groups);self._last_now=now;self._last_pose=pose;self._detected_count=len(detected)
        self.normal_cap=None;self.reason='';self.normal_debug={};self._boundary_check_mode='normal_safety'
        reason=self._map_reason(self.previous_pose or pose,pose);self.previous_pose=pose
        if reason:return self._block(reason)
        index=PolylineIndex(reference);ego=max(0.,min(index.lengths[-1],index.project(pose[:2])))
        occupancy=self.occupancy.update(detected,index,ego,self.body.rear_overhang_m,self.config.observation_padding_m,pose=pose,stamp=kwargs.get('observation_stamp_ns',now),now=now)
        self.remembered=tuple(() for _ in self.occupancy.cells) # count only; geometry is never re-convexified
        self._min_clearance=math.inf
        if occupancy.is_empty:return self._result(reference)
        tree=self.occupancy.tree
        def near(shape,margin):
            return any(shape.distance(solid)<=margin for solid in tree.query(shape.buffer(margin+1e-9)))
        def intersects(shape):
            return any(shape.intersects(solid) for solid in tree.query(shape))
        actual=Polygon(self.body.polygon(*pose));self._min_clearance=actual.distance(tree.nearest(actual))
        if self._min_clearance<=self.config.obstacle_margin_m or near(LineString((pose[:2],(pose[0]+1e-10,pose[1]))),self.config.reference_clearance_m):
            self._failure_context=dict(phase='MEASURED_BODY',body_clearance_m=self._min_clearance)
            return self._block('NORMAL_BODY_OR_REFERENCE_COLLISION')
        path_poses=poses(reference);lengths=arc(reference);conflict=None
        # The first intersecting WHOLE swept body defines rear-axle travel to
        # the unchanged body/axle floors. Do not subtract its front overhang twice.
        for i,(a,b) in enumerate(zip(path_poses,path_poses[1:])):
            if lengths[i+1]<ego:continue
            solid=Polygon(swept_body(self.body,a,b));axle=LineString((a[:2],b[:2]))
            if near(solid,self.config.obstacle_margin_m) or near(axle,self.config.reference_clearance_m):
                conflict=max(0.,lengths[i]-ego);break
        self._path_obstacle_count=int(conflict is not None)
        required=speed/3.6*self.config.reaction_time_s+(speed/3.6)**2/(2*self.config.deceleration_mps2)
        self._distance_context=dict(scope='NORMAL_ONLY_NO_LATERAL_MANEUVER',available_distance_m=conflict,required_stopping_m=required,speed_kmh=speed,body_clearance_m=self._min_clearance)
        if conflict is not None:
            if conflict<=required:
                self._failure_context=dict(self._distance_context);return self._block('NORMAL_BRAKING_DISTANCE')
            available=max(0.,conflict-2.);a=self.config.deceleration_mps2;reaction=1.1
            self.normal_cap=3.6*(math.sqrt((a*reaction)**2+2*a*available)-a*reaction)
        # Same strict 2D full-body occlusion model, now against the exact SAME
        # retained observations used by collision safety. No fresh-looking age.
        source=(pose[0]+1.19*math.cos(pose[2]),pose[1]+1.19*math.sin(pose[2]))
        start=self.body.wheelbase_m+self.body.front_overhang_m+2.;visibility=None
        for distance in (start+2*i for i in range(max(0,math.floor((lengths[-1]-ego-start)/2)+1))):
            point,tangent=sample(reference,ego+distance,lengths);shape=inflate(self.body.polygon(point[0],point[1],math.atan2(tangent[1],tangent[0])),self.config.tracking_limit_m+self.config.obstacle_margin_m)
            if any(intersects(LineString((source,target))) for target in shape):visibility=max(0.,distance-2.);break
        if visibility is not None:
            a=self.config.deceleration_mps2;reaction=1.7 # original 1.1 + unchanged worst-case .6 source age
            cap=3.6*(math.sqrt((a*reaction)**2+2*a*max(0.,visibility-1.))-a*reaction)
            self.normal_cap=cap if self.normal_cap is None else min(self.normal_cap,cap)
        self.normal_debug=dict(lateral_planning_enabled=False,occupancy='CURRENT_PLUS_CONFIRMED_STATIC',observation_memory=self.occupancy.debug,memory_cells=len(self.occupancy.cells),first_conflict_distance_m=conflict,visible_distance_m=visibility,speed_cap_kmh=self.normal_cap,elapsed_ms=(time.perf_counter()-began)*1000)
        return self._result(reference)
