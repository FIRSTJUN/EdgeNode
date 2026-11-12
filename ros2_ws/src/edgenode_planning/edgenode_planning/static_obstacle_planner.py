"""One static-obstacle planner; fixed lattice maneuvers and hard safety first."""
from dataclasses import dataclass
from collections import deque
from bisect import bisect_right
import math
import time

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_control.steering_command import to_morai_front_steer, limit_steering_change

from .avoidance_geometry import convex_covers, convex_hull, intersects, obstacle_polygons, polygon_distance, swept_body, inflate, polygon_bounds, bounds_distance
from .lane_geometry import PolylineIndex, sample, cross, arc
from .lattice_path import (LatticeCandidate, CostWeights, center_suffix, control_local_path, curvatures,
                           fixed_path_suffix, heading_error, poses, reference_cost, transition)
from .vehicle_geometry import sonata_geometry


@dataclass(frozen=True)
class AvoidanceConfig:
    transition_lengths_m: tuple = (12., 16., 20.)
    prediction_distance_m: float = 80.
    obstacle_margin_m: float = .25
    observation_padding_m: float = .05
    reference_clearance_m: float = 1.20
    return_margin_m: float = .5
    hold_duration_s: float = 3.
    hold_clearance_m: float = 1.
    avoid_speed_kmh: float = 3.
    deceleration_mps2: float = 2.
    reaction_time_s: float = .3
    max_lateral_acceleration_mps2: float = 1.5
    max_wheel_rate_radps: float = .6
    steering_slew_limit_radps: float = 0.
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
                self.deceleration_mps2,self.rollout_seconds,self.tracking_limit_m,self.hold_duration_s)<=0
                or self.control_rate_hz<=0
                or self.steering_slew_limit_radps>self.max_wheel_rate_radps
                or not 0<self.max_front_steer_normalized<=1 or self.acquisition_confirmations<1
                or self.max_wheel_angle_rad*self.max_front_steer_normalized>=math.pi/2
                or self.reference_clearance_m<1.20 or self.obstacle_margin_m<.25
                or self.hold_clearance_m<self.obstacle_margin_m
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


class StaticObstaclePlanner:
    """One transition per call: NORMAL → DETECT → ENTRY → HOLD → RETURN.

    Each motion stage generates only its own geometry. LiDAR solids, actual
    vehicle geometry, MGeo permission and the existing PP rollout gate motion.
    """
    def __init__(self, network, *, config=None, weights=None):
        self.network=network
        self.config=config or AvoidanceConfig();self.weights=weights or CostWeights()
        self.body=sonata_geometry()
        self.pp=PurePursuit(self.config.wheelbase_m,self.config.max_wheel_angle_rad)
        self.reset()

    def reset(self):
        self.state='NORMAL';self.reason='';self.selected=None;self.pair=None
        self.candidates=();self.remembered=();self.confirmations=0
        self.entry_commits=self.return_commits=0
        self._hold_path=();self._return_path=();self._last_progress=None;self._last_pose=None;self._min_clearance=math.inf
        self._global_path=();self._hold_started=None;self._now=None;self._last_now=None
        self._return_retry_at=0.;self._hold_elapsed=0.;self._surround_clearance=math.inf
        self._detected_obstacles=();self._obstacle_bounds={}
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
        if self.state=='DETECT' or (self.state=='RETURN' and not self._return_path):
            return True
        changed=tuple(o for o in detected if not any(convex_covers(old,o) for old in self.remembered))
        if not changed:return False
        path=(self.selected.entry_path if self.state=='AVOID_ENTRY' else
              self._return_path if self.state=='RETURN' else self._hold_path or self.selected.entry_path)
        # New roadside returns do not revoke an already safe motion path.
        # Potential intersections still STOP before full validation. Every new
        # solid, including clear roadside solids, is revalidated by plan().
        if self._last_pose is not None:
            index=PolylineIndex(path)
            station=max(0.,min(index.lengths[-1],index.project(self._last_pose[:2])))
            path=fixed_path_suffix(path,station)
        return bool(self._normal_hazards(path,changed))

    def _result(self, path=(), *, speed_cap=None):
        active=self.state!='NORMAL' or speed_cap is not None
        return dict(state=self.state,stage=self.state,points=[] if self.state=='BLOCKED' else path,
            blocked=self.state=='BLOCKED',reason=self.reason,active=active,
            speed_cap_kmh=self.config.avoid_speed_kmh if active and speed_cap is None else speed_cap,
            candidates=self.candidates,original_lane=self._original,left_lane=self.left_lane,
            right_lane=self.right_lane,selected_lane=self.pair.current_ids(
                max(0.,min(self.pair.length,self._last_progress or 0.)))[1] if self.pair else '',
            selected_side=self.selected.side if self.selected else '',
            return_path=self._return_path,
            hold_elapsed_s=self._hold_elapsed,hold_required_s=self.config.hold_duration_s,
            surround_clearance_m=self._surround_clearance,
            detected_obstacles=self._detected_obstacles,
            clearance_m=self._min_clearance,clearance_is_lower_bound=True,curvature_limit=self.config.curvature_limit,
            entry_commits=self.entry_commits,return_commits=self.return_commits,
            generated_transition_count=self.generated_transition_count,
            candidate_lane_batches=self.candidate_lane_batches)

    def _block(self, reason):
        self.state='BLOCKED';self.reason=reason
        return self._result()

    def _body_safety(self, first, second, obstacles):
        swept=swept_body(self.body,first,second)
        bounds=polygon_bounds(swept)
        angle=abs(heading_error(second[2],first[2]))
        chord=math.dist(first[:2],second[:2])
        sagitta=(chord/(2*math.sin(angle/2))*(1-math.cos(angle/2))) if angle>1e-9 else 0.
        axle_bounds=(min(first[0],second[0])-sagitta,max(first[0],second[0])+sagitta,
                     min(first[1],second[1])-sagitta,max(first[1],second[1])+sagitta)
        for obstacle in obstacles:
            obstacle=tuple(map(tuple,obstacle))
            other=self._obstacle_bounds.get(obstacle)
            if other is None:
                other=polygon_bounds(obstacle);self._obstacle_bounds[obstacle]=other
            lower=bounds_distance(bounds,other)
            reference_lower=bounds_distance(axle_bounds,other)
            if (lower>self.config.obstacle_margin_m
                    and reference_lower>=self.config.reference_clearance_m):
                # Both hard safety floors are proven by lower bounds. Report
                # a conservative clearance instead of recomputing a far solid.
                self._min_clearance=min(self._min_clearance,lower)
                continue
            clearance=polygon_distance(swept,obstacle)
            self._min_clearance=min(self._min_clearance,clearance)
            if clearance<=self.config.obstacle_margin_m:
                return 'BODY_COLLISION'
            if reference_lower<self.config.reference_clearance_m:
                axle=inflate((tuple(first[:2]),tuple(second[:2])),sagitta)
                if polygon_distance(axle,obstacle)<self.config.reference_clearance_m:
                    return 'REFERENCE_CLEARANCE'
        return ''

    def _geometry_context(self):
        return (id(self.network),self.body,self.config,self._observation_curvature_limit)

    def _path_safety(self, points, obstacles, speed, geometry_check=True, body_cache=None, curvature_points=None):
        try:
            ks=curvatures(points if curvature_points is None else curvature_points)
        except (ValueError,OverflowError):
            return 'INVALID_PATH'
        if not ks or not all(math.isfinite(k) for k in ks):
            return 'INVALID_PATH'
        if geometry_check:
            if body_cache is not None:
                context=(self._geometry_context(),obstacles)
                if body_cache.get('context')!=context:
                    body_cache.clear();body_cache['context']=context
            path_poses=poses(points)
            for a,b in zip(path_poses,path_poses[1:]):
                if body_cache is None:
                    reason=self._body_safety(a,b,obstacles)
                else:
                    key=(a,b)
                    cached=body_cache.get(key)
                    if cached is None:
                        previous=self._min_clearance;self._min_clearance=math.inf
                        try:
                            reason=self._body_safety(a,b,obstacles)
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
            self._body_safety(a,b,(o,)) in ('BODY_COLLISION','REFERENCE_CLEARANCE')
            for a,b in zip(path_poses,path_poses[1:])))

    def _remember(self, detected):
        # No IDs are present in PoseArray. Merge overlapping observed static
        # solids conservatively, rather than accumulating one noisy box/frame.
        result=list(self.remembered)
        for obstacle in detected:
            # Validate a conservative envelope once. Contained measurement
            # jitter neither shrinks the solid nor restarts fresh-pose waits.
            if any(convex_covers(old,obstacle) for old in result):
                continue
            obstacle=inflate(obstacle,self.config.observation_padding_m)
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

    def _rollout(self, points, pose, speed, actual_curvature, obstacles, duration_s=None):
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        if abs(actual_curvature)>self.config.curvature_limit:
            return 'ACTUAL_CURVATURE'
        fixed_points=points
        try:
            points=control_local_path(fixed_points,pose,self._control_horizon_m)
        except ValueError:
            return 'INVALID_CONTROL_WINDOW'
        # Keep the PP target within the validated window for the full horizon;
        # short configured windows are rejected rather than modeled as a
        # longer path than the actual Control input.
        if arc(points)[-1]<self._rollout_distance(speed):
            return 'SHORT_CONTROL_HORIZON'
        dt=1/self.config.control_rate_hz
        previous_angle=math.atan(actual_curvature*self.config.wheelbase_m)
        applied_angle=requested_angle=previous_angle
        queue=deque()
        duration=self.config.rollout_seconds if duration_s is None else duration_s
        for tick in range(math.ceil(duration/dt)):
            if duration_s is not None and tick%max(1,round(self.config.control_rate_hz*.1))==0:
                points=control_local_path(fixed_points,pose,self._control_horizon_m)
            response=self.pp.compute(points,*pose,lookahead_distance_m=lookahead)
            if response is None:
                return 'PP_UNTRACKABLE'
            if response['nearest_distance_m']>self.config.tracking_limit_m:
                return 'TRACKING_ERROR'
            if (abs(response['steering_normalized'])>self.config.max_front_steer_normalized
                    and self.config.steering_slew_limit_radps==0):
                return 'STEERING_CLAMP'
            command=to_morai_front_steer(response['steering_normalized'],self.config.steering_sign,
                self.config.max_front_steer_normalized)
            queue.append((tick*dt+self.config.command_delay_s,
                command/self.config.steering_sign*self.config.max_wheel_angle_rad))
            while queue and queue[0][0]<=tick*dt+1e-12:
                requested_angle=queue.popleft()[1]
            applied_angle=limit_steering_change(requested_angle,previous_angle,
                self.config.steering_slew_limit_radps,dt)
            curvature=math.tan(applied_angle)/self.config.wheelbase_m
            if velocity**2*abs(curvature)>self.config.max_lateral_acceleration_mps2:
                return 'LATERAL_ACCELERATION'
            wheel_angle=applied_angle
            if abs(wheel_angle-previous_angle)/dt>self.config.max_wheel_rate_radps+1e-9:
                return 'STEERING_RATE'
            previous_angle=wheel_angle
            next_pose=bicycle_step(pose,curvature,velocity*dt)
            reason=self._body_safety(pose,next_pose,obstacles)
            if reason:
                return reason
            pose=next_pose
        return ''

    def _preparation(self, pose, speed, actual_curvature, z):
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        distance=lookahead+velocity*self.config.command_delay_s
        divisions=max(1,math.ceil(distance/.25))
        prepared=tuple(bicycle_step(pose,actual_curvature,distance*i/divisions)
                       for i in range(divisions+1))
        return prepared[-1],tuple((x,y,float(z)) for x,y,_ in prepared)

    def _generate_entry(self, pair, pose, speed, actual_curvature, front, body_cache=None):
        """STEP 2 only. No return target, reservation or return permission."""
        self.candidate_lane_batches+=1
        ego=pair.original_index.project(pose[:2]);row=sample(pair.original,ego)
        if row is None:
            raise ValueError('ORIGINAL_LANE_RANGE')
        # Neighbor links may start slightly ahead of the current cross-section.
        # Entry needs real target/tail correspondence, not one at its start.
        prepared,prefix=self._preparation(pose,speed,actual_curvature,row[0][2])
        prepared_s=pair.original_index.project(prepared[:2])
        supported=pair.length-self.body.wheelbase_m-self.body.front_overhang_m-1.
        reserve=self._rollout_distance(speed)
        candidates=[];projection_cache={}
        for length in self.config.transition_lengths_m:
            entry=path=();end=prepared_s+length;cost=math.inf;terms={}
            try:
                target=pair.at(end);tail=pair.at(min(supported,end+reserve))
                if target is None or tail is None or end+reserve>supported:
                    raise ValueError('INSUFFICIENT_ENTRY_SUPPORT')
                entry=prefix+transition(prepared[:2],prepared[2],actual_curvature,
                    pair.adjacent,target['adjacent_s'],length)[1:]
                self.generated_transition_count+=1
                # This short adjacent tail is PP support for completing entry.
                # STEP 3's driving path is created only after acquisition.
                path=entry+center_suffix(pair.adjacent,target['adjacent_s'],tail['adjacent_s'])[1:]
                reason=''
                needed=end+self.body.wheelbase_m+self.body.front_overhang_m+\
                    self.config.obstacle_margin_m+self._entry_reserve_m
                if needed>front:reason='ENTRY_DEADLINE'
                if not reason and self._entry_observation_m is not None and needed-ego>self._entry_observation_m:
                    reason='ENTRY_OBSERVATION_DISTANCE'
                if not reason:reason=self._path_safety(path,self.remembered,speed,body_cache=body_cache)
                if not reason:reason=self._rollout(path,pose,speed,actual_curvature,self.remembered)
                cost,terms=reference_cost(path,pair.original,pair.adjacent,self.weights,
                                          projection_cache=projection_cache)
            except (ValueError,TypeError,IndexError,OverflowError) as error:
                reason=str(error)
            candidates.append(LatticeCandidate(pair.side,path or entry,end,math.nan,math.nan,
                length,cost,terms,reason,path or entry,()))
        return candidates

    def _return_permission(self, start, end):
        reverse='right' if self.pair.side=='left' else 'left'
        begin=0.
        for (original,adjacent),finish in zip(self.pair.ids,self.pair.section_ends):
            if finish>=start and begin<=end:
                reason=self.network.permission_reason(adjacent,reverse)
                if reason:return reason
                if self.network.links[adjacent].get(reverse+'_lane_change_dst_link_idx')!=original:
                    return 'RETURN_LINK_MISMATCH'
            begin=finish
        if start<0 or end>self.pair.length:return 'RETURN_OUTSIDE_LANE_SUPPORT'
        return ''

    def _generate_return(self, pose, speed, actual_curvature, body_cache):
        """STEP 4 only, from the measured pose to the saved Global Path."""
        reference=self._global_path;index=PolylineIndex(reference)
        station=index.project(pose[:2]);row=sample(reference,station)
        if row is None:return 'SAVED_GLOBAL_PATH_UNAVAILABLE'
        prepared,prefix=self._preparation(pose,speed,actual_curvature,row[0][2])
        start=index.project(prepared[:2]);reserve=self._rollout_distance(speed)
        candidates=[]
        for length in self.config.transition_lengths_m:
            path=curve=();end=start+length;terms={};cost=math.inf
            try:
                target=sample(reference,end)
                if target is None or end+reserve>index.lengths[-1]:
                    raise ValueError('RETURN_CONTROL_SUPPORT')
                a=self.pair.original_index.project(pose[:2])
                b=self.pair.original_index.project(target[0])
                reason=self._return_permission(a,b)
                if reason:raise ValueError(reason)
                recovery=transition(prepared[:2],prepared[2],actual_curvature,reference,end,length)
                self.generated_transition_count+=1
                curve=prefix+recovery[1:]
                path=curve+center_suffix(reference,end,end+reserve)[1:]
                reason=self._path_safety(path,self.remembered,speed,body_cache=body_cache,curvature_points=curve)
                if not reason:
                    # Native map vertices are PP reference corners, not wheel
                    # curvature. Validate the full return plus controller tail
                    # with the same bounded actuator used by Control.
                    duration=arc(curve)[-1]/(max(speed,self.config.avoid_speed_kmh)/3.6)+self.config.rollout_seconds
                    reason=self._rollout(path,pose,speed,actual_curvature,self.remembered,duration_s=duration)
                cost,terms=reference_cost(path,self.pair.original,self.pair.adjacent,self.weights)
            except (ValueError,TypeError,IndexError,OverflowError) as error:
                reason=str(error)
            candidates.append(LatticeCandidate(self.pair.side,path,self.selected.entry_end_s,
                station,end,length,cost,terms,reason,self.selected.entry_path,path,curve))
        self.candidates=tuple(candidates)
        safe=[c for c in candidates if not c.rejection]
        if not safe:return 'NO_SAFE_RETURN'
        self.selected=min(safe,key=lambda c:c.cost)
        self._return_path=self.selected.return_path;self.return_commits+=1
        self._validated_paths.clear();self.reason=''
        return ''

    def _acquired(self, pose, target, completion, progress):
        row=sample(target,PolylineIndex(target).project(pose[:2]))
        if row is None:
            self.confirmations=0;return False
        p,tangent=row;delta=(pose[0]-p[0],pose[1]-p[1])
        yaw=math.atan2(tangent[1],tangent[0])
        eligible=(progress>=completion-self.config.acquisition_end_region_m and
            abs(cross(tangent,delta))<=self.config.acquisition_lateral_m and
            abs(heading_error(pose[2],yaw))<=self.config.acquisition_heading_rad)
        self.confirmations=self.confirmations+1 if eligible else 0
        return self.confirmations>=self.config.acquisition_confirmations

    def _validate_active(self, path, pose, speed, actual_curvature, body_cache):
        index=PolylineIndex(path);station=max(0.,min(index.lengths[-1],index.project(pose[:2])));row=sample(path,station)
        if row is None or math.dist(pose[:2],row[0][:2])>self.config.tracking_limit_m:
            return 'TRACKING_ERROR'
        if abs(heading_error(pose[2],math.atan2(row[1][1],row[1][0])))>math.radians(30.):
            return 'DIRECTION_CONTINUITY'
        # Subdivision retains MGeo corners and changes the 3-point curvature
        # estimate at those vertices. Use the native centerline for HOLD's
        # curvature, while checking collisions on the entire sampled path.
        if path is self._hold_path:
            start=self.pair.adjacent_index.project(pose[:2])
            end=self.pair.adjacent_index.project(path[-1])
            first=max(0,bisect_right(self.pair.adjacent_arc,start)-2)
            last=min(len(self.pair.adjacent),bisect_right(self.pair.adjacent_arc,end)+2)
            kinematic_points=self.pair.adjacent[first:last]
        else:
            kinematic_points=(self.selected.kinematic_points if path is self._return_path
                              and self.selected.kinematic_points else path)
        key=(id(path),self.remembered)
        if key not in self._validated_paths:
            future=fixed_path_suffix(path,station)
            reason=self._path_safety(future,self.remembered,speed,body_cache=body_cache,curvature_points=kinematic_points)
            if not reason:self._validated_paths[key]=(path,future)
        else:
            reason=self._path_safety(self._validated_paths[key][1],(),speed,geometry_check=False,curvature_points=kinematic_points)
        if not reason:reason=self._rollout(path,pose,speed,actual_curvature,self.remembered)
        return reason

    def _hold(self, pose, speed, actual_curvature, body_cache):
        """Generate/extend only a measured MGeo adjacent-centerline suffix."""
        station=self.pair.adjacent_index.project(pose[:2])
        if not self._hold_path or arc(self._hold_path)[-1]-PolylineIndex(self._hold_path).project(pose[:2])<self._rollout_distance(speed)+1.:
            end=min(self.pair.adjacent_arc[-1]-self.body.wheelbase_m-self.body.front_overhang_m-1.,
                    station+self.config.prediction_distance_m)
            if end<=station:return self._block('ADJACENT_LANE_END')
            self._hold_path=center_suffix(self.pair.adjacent,station,end)
            self._validated_paths.clear()
        reason=self._validate_active(self._hold_path,pose,speed,actual_curvature,body_cache)
        return self._block(reason) if reason else self._result(self._hold_path)

    def _detection_path(self, reference, pose):
        index=PolylineIndex(reference);ego=max(0.,min(index.lengths[-1],index.project(pose[:2])))
        front=min(index.project(p) for o in self._hazards for p in o)
        stop=front-self.body.wheelbase_m-self.body.front_overhang_m-self.config.obstacle_margin_m-1.
        if stop<=ego+1.:return self._block('BRAKING_DISTANCE')
        return self._result(center_suffix(reference,ego,min(stop,index.lengths[-1])))

    def plan(self, reference, groups, pose, *, speed_kmh=0., actual_curvature=0.,
             current_link_id='', route_link_ids=(), observation_valid=True, control_horizon_m=20.,
             hazard_reference=None, entry_reserve_m=0., stopping_required_m=None,
             entry_observation_m=None, observation_curvature_limit=None,
             now_sec=None, global_reference=None, return_search=None):
        normal_reference=reference
        reference=hazard_reference if hazard_reference is not None else reference
        self._min_clearance=math.inf
        if self.state=='BLOCKED':return self._result()
        try:
            now=time.monotonic() if now_sec is None else now_sec
            if (not math.isfinite(now) or (self._last_now is not None and now<self._last_now)
                    or any(v is not None and (not math.isfinite(v) or v<0)
                           for v in (entry_observation_m,observation_curvature_limit))
                    or not math.isfinite(entry_reserve_m) or entry_reserve_m<0
                    or (stopping_required_m is not None and (not math.isfinite(stopping_required_m) or stopping_required_m<=0))
                    or len(pose)!=3 or len(reference)<2 or speed_kmh<0
                    or not math.isfinite(control_horizon_m) or control_horizon_m<=0
                    or not all(math.isfinite(v) for v in (*pose,speed_kmh,actual_curvature))
                    or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in reference)):
                return self._block('INVALID_INPUT_OR_CLOCK')
            detected=obstacle_polygons(groups)
        except (ValueError,TypeError,OverflowError):
            return self._block('INVALID_OBSTACLE_FOOTPRINT')
        if not observation_valid:return self._block('OBSTACLE_OBSERVATION_INVALID_OR_STALE')
        self._last_now=self._now=now;self._last_pose=pose
        self._control_horizon_m=control_horizon_m;self._entry_reserve_m=entry_reserve_m
        self._entry_observation_m=entry_observation_m;self._observation_curvature_limit=observation_curvature_limit
        context=self._geometry_context()
        if context!=self._validated_context:
            self._validated_paths.clear();self._validated_context=context
        body_cache={};old_memory=self.remembered;self._remember(detected)
        if self.remembered!=old_memory:
            self._obstacle_bounds.clear()
            self._validated_paths.clear()
            if self._hazards:
                self._hazards=tuple(o for o in self.remembered if any(intersects(o,h) for h in self._hazards))
        reason=self._body_safety(pose,pose,self.remembered)
        if reason:return self._block(reason)

        if self.state in ('NORMAL','DETECT'):
            index=PolylineIndex(reference);ego=max(0.,min(index.lengths[-1],index.project(pose[:2])))
            if self.state=='NORMAL':
                self._original=reference
                self.remembered=tuple(o for o in self.remembered if max(index.project(p) for p in o)
                    >=ego-self.body.rear_overhang_m-self.config.return_margin_m)
                blocking=self._normal_hazards(reference,self.remembered)
                if not blocking:return self._result(normal_reference)
                self._hazards=blocking
            front=min(index.project(p) for o in self._hazards for p in o)
            actual_front=max(index.project(p) for p in self.body.polygon(*pose))
            distance=front-actual_front-self.config.obstacle_margin_m;velocity=speed_kmh/3.6
            if stopping_required_m is not None and front-ego<=stopping_required_m:
                return self._block('S4_OBSTACLE_STOPPING_DISTANCE')
            if distance<=velocity*self.config.reaction_time_s+velocity**2/(2*self.config.deceleration_mps2):
                return self._block('BRAKING_DISTANCE')
            row=sample(reference,ego)
            if row is None:return self._block('REFERENCE_RANGE')
            if abs(heading_error(pose[2],math.atan2(row[1][1],row[1][0])))>math.radians(30.):
                return self._block('DIRECTION_CONTINUITY')
            if self.state=='NORMAL':
                saved=global_reference if global_reference is not None else reference
                if len(saved)<2 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in saved):
                    return self._block('SAVED_GLOBAL_PATH_UNAVAILABLE')
                self._global_path=tuple(tuple(p) for p in saved)
                self._detected_obstacles=tuple(dict(
                    center_xy=tuple(sum(p[k] for p in o)/len(o) for k in (0,1)),
                    distance_m=polygon_distance(self.body.polygon(*pose),o),
                    size_xy_m=tuple(max(p[k] for p in o)-min(p[k] for p in o) for k in (0,1)))
                    for o in self._hazards)
                self.state='DETECT';self.reason='OBSTACLE_DETECTED'
                return self._detection_path(reference,pose)
            if speed_kmh>self.config.avoid_speed_kmh+.2:
                self.reason='DECELERATING';return self._detection_path(reference,pose)
            pairs=[];all_candidates=[]
            for side in ('left','right'):
                try:
                    pair=self.network.pair(current_link_id,side,list(route_link_ids))
                    if pair is None:continue
                    setattr(self,side+'_lane',pair.adjacent)
                    front_s=min(pair.original_index.project(p) for o in self._hazards for p in o)
                    candidates=self._generate_entry(pair,pose,speed_kmh,actual_curvature,front_s,body_cache)
                    all_candidates.extend(candidates);pairs.append((pair,candidates))
                except (ValueError,TypeError,IndexError,OverflowError):
                    continue
            self.candidates=tuple(all_candidates)
            safe=[(c,p) for p,cs in pairs for c in cs if not c.rejection]
            if not safe:return self._block('NO_SAFE_ENTRY' if all_candidates else 'NO_VALID_ADJACENT_LANE')
            self.selected,self.pair=min(safe,key=lambda row:row[0].cost)
            self._last_progress=None;self._original=self.pair.original
            self.state='AVOID_ENTRY';self.reason='';self.entry_commits+=1
            return self._result(self.selected.entry_path)

        progress=self.pair.original_index.project(pose[:2])
        if not math.isfinite(progress):return self._block('INVALID_PROGRESS')
        if self._last_progress is not None and progress<self._last_progress-self.config.tracking_limit_m:
            return self._block('PROGRESS_REGRESSION')
        self._last_progress=progress
        if self.state=='AVOID_ENTRY':
            reason=self._validate_active(self.selected.entry_path,pose,speed_kmh,actual_curvature,body_cache)
            if reason:return self._block(reason)
            if self._acquired(pose,self.pair.adjacent,self.selected.entry_end_s,progress):
                self.state='AVOID_HOLD';self.reason='HOLD_TIMER';self.confirmations=0
                self._hold_started=now;self._hold_elapsed=0.
            return self._result(self.selected.entry_path)

        if self.state=='AVOID_HOLD':
            result=self._hold(pose,speed_kmh,actual_curvature,body_cache)
            if result['blocked']:return result
            if speed_kmh>self.config.avoid_speed_kmh+.2:self._hold_started=now
            self._hold_elapsed=now-self._hold_started
            rear=max(self.pair.original_index.project(p) for o in self._hazards for p in o)
            rear_cleared=min(self.pair.original_index.project(p) for p in self.body.polygon(*pose))>rear+self.config.return_margin_m
            self._surround_clearance=min((polygon_distance(self.body.polygon(*pose),o)
                                         for o in self.remembered),default=math.inf)
            if self._hold_elapsed<self.config.hold_duration_s:self.reason='HOLD_TIMER'
            elif not rear_cleared:self.reason='WAIT_OBSTACLE_PASS'
            elif self._surround_clearance<self.config.hold_clearance_m:self.reason='WAIT_SURROUND_CLEARANCE'
            else:
                self.state='RETURN';self.reason='RETURN_READY';self.confirmations=0
            # Even when ready, no return geometry is generated by STEP 3.
            return self._result(self._hold_path)

        if self.state=='RETURN':
            if not self._return_path:
                # Recheck STEP 3 clearance against each new LiDAR frame before
                # a delayed/retried return search. Do not abandon the safe lane.
                self._surround_clearance=min((polygon_distance(self.body.polygon(*pose),o)
                                             for o in self.remembered),default=math.inf)
                rear=max(self.pair.original_index.project(p) for o in self._hazards for p in o)
                cleared=min(self.pair.original_index.project(p) for p in self.body.polygon(*pose))>rear+self.config.return_margin_m
                if not cleared or self._surround_clearance<self.config.hold_clearance_m:
                    self.reason='WAIT_SURROUND_CLEARANCE';return self._hold(pose,speed_kmh,actual_curvature,body_cache)
                if now<self._return_retry_at:return self._hold(pose,speed_kmh,actual_curvature,body_cache)
                reason=(self._generate_return(pose,speed_kmh,actual_curvature,body_cache)
                        if return_search is None else return_search(self,pose,speed_kmh,actual_curvature,body_cache))
                if reason=='RETURN_SEARCH_PENDING':
                    # Only STEP 4 may start this work. Remain stopped while the
                    # isolated search runs; no old HOLD path can authorize drive.
                    return dict(self._result(),state='VALIDATING',blocked=True,
                                points=(),reason=reason)
                if reason:
                    self.reason=reason;self._return_retry_at=now+1.
                    return self._hold(pose,speed_kmh,actual_curvature,body_cache)
            reason=self._validate_active(self._return_path,pose,speed_kmh,actual_curvature,body_cache)
            if reason:return self._block(reason)
            station=PolylineIndex(self._global_path).project(pose[:2])
            if self._acquired(pose,self._global_path,self.selected.return_end_s,station):
                self.state='NORMAL';self.reason='';self.remembered=();self.selected=None;self.pair=None
                self.candidates=();self._hold_path=();self._return_path=();self._hazards=()
                self._hold_started=None;self._last_progress=None;self._validated_paths.clear()
                return self._result(normal_reference)
            return self._result(self._return_path)
        return self._block('INVALID_STATE')
