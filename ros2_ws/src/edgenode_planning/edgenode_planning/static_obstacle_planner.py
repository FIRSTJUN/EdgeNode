"""Measured-pose, multi-candidate static avoidance with hard safety gates."""
from dataclasses import dataclass,replace
from copy import copy
from collections import deque
from collections import OrderedDict
from bisect import bisect_right
import math
import time

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance, path_lookahead_distance
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_control.steering_command import to_morai_front_steer, limit_steering_change

from .avoidance_geometry import convex_covers, convex_hull, intersects, obstacle_polygons, polygon_distance, swept_body, inflate, polygon_bounds, bounds_distance
from .lane_geometry import PolylineIndex, sample, cross, arc,curvature_at
from .lattice_path import (LatticeCandidate, CostWeights, center_suffix, control_local_path, curvatures,
                           fixed_path_suffix, heading_error, poses, reference_cost, transition)
from .vehicle_geometry import sonata_geometry
from .lane_continuation import continuation
from .maneuver_selection import ManeuverSelection,Situation,Strategy
from .planning_provenance import implementation_sha256
from .search_budget import check as check_search_budget


@dataclass(frozen=True)
class AvoidanceConfig:
    continuous_lane_follow: bool = False
    integrated_strategy: bool = False
    optimize_entry_onset: bool = False
    native_reference_rollout: bool = False
    observed_union_memory: bool = False
    native_obstacle_geometry: bool = False
    transition_lengths_m: tuple = (12., 16., 20.)
    entry_start_offsets_m: tuple = (0., 2., 4.)
    transition_search_step_m: float = 2.
    prediction_distance_m: float = 80.
    obstacle_margin_m: float = .25
    observation_padding_m: float = .05
    memory_merge_extent_m: float = 0.
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
    lookahead_heading_budget_rad: float = .4
    rollout_seconds: float = 2.
    command_delay_s: float = .1
    acquisition_lateral_m: float = .15
    acquisition_heading_rad: float = math.radians(8.)
    acquisition_end_region_m: float = 2.
    acquisition_confirmations: int = 3
    tracking_limit_m: float = .30
    normal_obstacle_confirmations: int = 1
    normal_detection_latency_s: float = 0.
    normal_detection_margin_m: float = 2.
    normal_prediction_margin_m: float = 0.
    entry_geometry_wait_distance_m: float = 0.

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if name in ('continuous_lane_follow','integrated_strategy','optimize_entry_onset','native_reference_rollout','observed_union_memory','native_obstacle_geometry'):
                if type(value) is not bool:raise ValueError('continuous lane follow must be boolean')
            elif name in ('transition_lengths_m', 'entry_start_offsets_m'):
                if (not value or any(isinstance(v,bool) or not math.isfinite(v) or
                        (v<=0 if name=='transition_lengths_m' else v<0) for v in value)):
                    raise ValueError('positive transition lengths required')
                object.__setattr__(self,name,tuple(value))
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
        if (self.transition_search_step_m<=0 or 0. not in self.entry_start_offsets_m
                or len(self.entry_start_offsets_m)>8):
            raise ValueError('bounded entry start search including zero required')
        if not isinstance(self.normal_obstacle_confirmations,int) or self.normal_obstacle_confirmations<1:
            raise ValueError('positive obstacle confirmation count required')

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
        self._implementation_sha256=implementation_sha256()
        self.reset()

    def reset(self):
        self.state='NORMAL';self.reason='';self.selected=None;self.pair=None
        self.candidates=();self.remembered=();self.confirmations=0
        self._observed_union_memory=None
        self._work_timing_ms={}
        self.entry_commits=self.return_commits=0
        self._hold_path=();self._return_path=();self._last_progress=None;self._last_pose=None;self._min_clearance=math.inf
        self._global_path=();self._hold_started=None;self._now=None;self._last_now=None
        self._return_retry_at=0.;self._hold_elapsed=0.;self._surround_clearance=math.inf
        self._detected_obstacles=();self._obstacle_bounds={}
        self._entry_context={}
        self._entry_wait_path=();self._entry_wait_origin=None;self._entry_wait_retry=None
        self._normal_detection_frames=0;self._normal_misses=0;self._normal_stamp=None
        self._normal_candidate=()
        self._normal_tracks=[];self._detection_trigger_m=None;self._confirmed_path_obstacles=0
        self._detected_count=0;self._path_obstacle_count=0
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
        self._boundary_contact_reason=''
        self._boundary_check_mode='diagnostic'
        self._road_area=None
        self._planning_tracking_reserve=0.
        self._collision_context={};self._failure_context={};self._check_phase=''
        self._map_sweep_cache=OrderedDict()
        self._fixed_sweep_cache=OrderedDict()
        self._detection_pairs={};self._entry_distance_basis={}
        self._distance_context={}
        self._route_links=();self._route_occurrence=None;self._continuation=None
        self._continuation_index=None;self.natural_rejoins=0
        self._maneuver_selection=ManeuverSelection();self._strategy=Strategy.NORMAL
        self._motion_authorized=True
        self._entry_pending_path=();self._entry_replan_guide=()

    def invalidate_observation(self):
        self.confirmations=0

    def stop(self, reason):
        """Latch an integration-level validation failure until explicit reset."""
        return self._block(reason)

    def needs_validation_stop(self, reference, groups):
        """Cheap guard before potentially expensive candidate/revision checks."""
        if self.state=='BLOCKED':
            return False
        if self.config.continuous_lane_follow:
            # MORAI continuity mode validates the current control window with
            # fresh solids in plan(); a prospective revision alone does not
            # revoke an already valid command before that check completes.
            return False
        detected=obstacle_polygons(groups)
        if self.state=='NORMAL':
            if self.config.normal_obstacle_confirmations>1:return False
            return bool(self._normal_hazards(reference,self.remembered+detected))
        if self.state=='DETECT':
            if self._entry_wait_path and self._last_pose is not None:
                progress=PolylineIndex(self._global_path).project(self._last_pose[:2])
                if progress < self._entry_wait_retry:
                    changed=tuple(o for o in detected if not any(convex_covers(old,o) for old in self.remembered))
                    return bool(changed and self._normal_hazards(self._entry_wait_path,changed))
            return True
        if (self.state=='RETURN' and not self._return_path and self._last_pose is not None
                and self._return_clearance_ready(self._last_pose) and self._now>=self._return_retry_at):
            return True
        changed=tuple(o for o in detected if not any(convex_covers(old,o) for old in self.remembered))
        if not changed:return False
        path=(self.selected.entry_path if self.state=='AVOID_ENTRY' else
              self._return_path if self.state=='RETURN' and self._return_path
              else self._hold_path or self.selected.entry_path)
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
            implementation_sha256=self._implementation_sha256,
            blocked=self.state=='BLOCKED',reason=self.reason,active=active,
            boundary_contact_reason=self._boundary_contact_reason,
            boundary_check_mode=self._boundary_check_mode,
            failure_context=self._failure_context,
            speed_cap_kmh=self.config.avoid_speed_kmh if active and speed_cap is None else speed_cap,
            candidates=self.candidates,original_lane=self._original,left_lane=self.left_lane,
            right_lane=self.right_lane,selected_lane=self.pair.current_ids(
                max(0.,min(self.pair.length,self._last_progress or 0.)))[1] if self.pair else '',
            selected_side=self.selected.side if self.selected else '',
            return_path=self._return_path,
            hold_elapsed_s=self._hold_elapsed,hold_required_s=self.config.hold_duration_s,
            surround_clearance_m=self._surround_clearance,
            detected_obstacles=self._detected_obstacles,
            entry_context=self._entry_context,
            detected_object_count=self._detected_count,path_obstacle_count=self._path_obstacle_count,
            remembered_object_count=len(self.remembered),
            work_timing_ms=dict(getattr(self,'_work_timing_ms',{})),
            selected_mode=getattr(self.pair,'mode','lane') if self.pair else '',
            obstacle_confirmation_frames=self._normal_detection_frames,
            confirmed_path_obstacles=self._confirmed_path_obstacles,
            detection_trigger_distance_m=self._detection_trigger_m,
            entry_distance_basis=self._entry_distance_basis,
            distance_context=self._distance_context,
            clearance_m=self._min_clearance,clearance_is_lower_bound=True,curvature_limit=self.config.curvature_limit,
            entry_commits=self.entry_commits,return_commits=self.return_commits,
            generated_transition_count=self.generated_transition_count,
            candidate_lane_batches=self.candidate_lane_batches,
            natural_rejoins=self.natural_rejoins,
            continuation_reason=self._continuation.reason if self._continuation else '',
            selected_strategy=self._strategy.value if self.config.integrated_strategy else 'LEGACY')

    def _block(self, reason):
        if reason in ('BRAKING_DISTANCE','S4_OBSTACLE_STOPPING_DISTANCE'):
            self._failure_context=dict(self._distance_context)
        if reason in ('BODY_COLLISION','REFERENCE_CLEARANCE','ROAD_AREA_UNPROVEN','LATERAL_ACCELERATION') or reason.startswith(('LANE_BOUNDARY_COLLISION:','PHYSICAL_BOUNDARY_COLLISION:')):
            self._failure_context=dict(self._collision_context)
        self.state='BLOCKED';self.reason=reason
        return self._result()

    def _inflate(self,polygon,margin):
        return inflate(polygon,margin,native=self.config.native_obstacle_geometry)

    def _body_geometry(self,first,second):
        swept=swept_body(self.body,first,second,native=self.config.native_obstacle_geometry)
        bounds=polygon_bounds(swept)
        angle=abs(heading_error(second[2],first[2]))
        chord=math.dist(first[:2],second[:2])
        sagitta=(chord/(2*math.sin(angle/2))*(1-math.cos(angle/2))) if angle>1e-9 else 0.
        axle_bounds=(min(first[0],second[0])-sagitta,max(first[0],second[0])+sagitta,
                     min(first[1],second[1])-sagitta,max(first[1],second[1])+sagitta)
        return swept,bounds,axle_bounds,sagitta

    def _body_safety(self, first, second, obstacles, *, prepared=None):
        geometry=self._body_geometry(first,second) if prepared is None else prepared
        # Immutable-map proof is independent of changing LiDAR observations.
        # Only fixed sampled sweeps use this cache; rollout poses are always
        # checked from the current measured state. Never cache solid clearance.
        boundaries=getattr(self.network,'boundaries',None)
        key=(self._geometry_context(),tuple(first),tuple(second)) if prepared is not None else None
        if key is not None and key in self._map_sweep_cache:
            reason,context=self._map_sweep_cache[key]
            self._map_sweep_cache.move_to_end(key)
            if reason:self._collision_context=dict(context,phase=self._check_phase);return reason
            return self._obstacle_safety(first,second,obstacles,prepared=geometry)
        reason=self._map_sweep_reason(first,second,geometry,boundaries)
        if key is not None:
            self._map_sweep_cache[key]=(reason,dict(self._collision_context) if reason else {})
            if len(self._map_sweep_cache)>12000:self._map_sweep_cache.popitem(last=False)
        if reason:return reason
        return self._obstacle_safety(first,second,obstacles,prepared=geometry)

    def _fixed_body_geometry(self, first, second):
        """Reuse exact sampled solids, never quantize measured/rollout poses."""
        key=(self.body,tuple(first),tuple(second))
        geometry=self._fixed_sweep_cache.get(key)
        if geometry is None:
            geometry=self._body_geometry(first,second)
            self._fixed_sweep_cache[key]=geometry
            if len(self._fixed_sweep_cache)>12000:self._fixed_sweep_cache.popitem(last=False)
        else:self._fixed_sweep_cache.move_to_end(key)
        return geometry

    def _map_sweep_reason(self,first,second,geometry,boundaries):
        if self._road_area is not None:
            margin=self.config.tracking_limit_m+self._planning_tracking_reserve
            contained=self._inflate(geometry[0],margin)
            if not self._road_area.covers(contained):
                self._collision_context=dict(phase=self._check_phase,first_pose=first,second_pose=second,
                    swept_polygon=geometry[0],containment_polygon=contained,
                    tracking_margin_m=margin,road_source=self._road_area.source)
                return 'ROAD_AREA_UNPROVEN'
        if boundaries is not None:
            boundary_polygon=geometry[0]
            if (getattr(self,'complete_offset_search',False) and
                    (self._road_area is not None or getattr(self,'_complete_return_proof_active',False) or
                     getattr(self.selected,'complete_return',False))):
                # The original measured-body gate is still mandatory. A fixed
                # offset also needs room for admitted tracking error at internal
                # solid dividers, not only at the outside of the road polygon.
                boundary_polygon=self._inflate(boundary_polygon,
                    self.config.tracking_limit_m+self._planning_tracking_reserve)
            boundary_reason = boundaries.swept_reason(boundary_polygon)
            if boundary_reason:
                self._collision_context=dict(phase=self._check_phase,first_pose=first,second_pose=second,
                    swept_polygon=geometry[0],boundary_reason=boundary_reason)
                return boundary_reason
        return ''

    def _obstacle_safety(self, first, second, obstacles, *, prepared=None):
        """LiDAR collision/clearance only; map markings cannot hide a solid."""
        swept,bounds,axle_bounds,sagitta=self._body_geometry(first,second) if prepared is None else prepared
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
            if self.config.native_obstacle_geometry:
                from .native_obstacle_geometry import distance
                clearance=distance(swept,obstacle)
            else:clearance=polygon_distance(swept,obstacle)
            self._min_clearance=min(self._min_clearance,clearance)
            if clearance<=self.config.obstacle_margin_m:
                self._collision_context=dict(phase=self._check_phase,first_pose=first,second_pose=second,
                    clearance_m=clearance,obstacle=obstacle,swept_polygon=swept)
                return 'BODY_COLLISION'
            if reference_lower<self.config.reference_clearance_m:
                axle=self._inflate((tuple(first[:2]),tuple(second[:2])),sagitta)
                separation=(distance(axle,obstacle) if self.config.native_obstacle_geometry else polygon_distance(axle,obstacle))
                if separation<self.config.reference_clearance_m:
                    self._collision_context=dict(phase=self._check_phase,first_pose=first,second_pose=second,
                        clearance_m=clearance,obstacle=obstacle,swept_polygon=swept)
                    return 'REFERENCE_CLEARANCE'
        return ''

    def _geometry_context(self):
        # Retain the loaded map objects, so object-ID reuse cannot revive an
        # old proof. Map reloads replace these objects; live LiDAR is excluded.
        return (self.network,getattr(self.network,'boundaries',None),self.body,
                self.config,self._observation_curvature_limit,self._road_area,self._planning_tracking_reserve,
                bool(getattr(self,'_complete_return_proof_active',False)),
                bool(getattr(self.selected,'complete_return',False)))

    def _path_safety(self, points, obstacles, speed, geometry_check=True, body_cache=None, curvature_points=None,
                     native_reference=False, geometry_station_range=None):
        check_search_budget(self)
        self._check_phase='PATH_SWEEP'
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
            lengths=arc(points) if geometry_station_range is not None else None
            if geometry_station_range is not None:
                low,high=geometry_station_range
                if (not all(math.isfinite(v) for v in (low,high)) or
                        low<0. or high<low or high>lengths[-1]+1e-6):return 'INVALID_PATH'
            # A horizon is a checking range, not a new trajectory. Preserve
            # the original neighbouring vertices used to orient each body;
            # retain whole sweeps straddling either end conservatively.
            segments=tuple((a,b,self._fixed_body_geometry(a,b))
                for i,(a,b) in enumerate(zip(path_poses,path_poses[1:]))
                if lengths is None or lengths[i+1]>=low and lengths[i]<=high)
            if not segments:return 'INVALID_PATH'
            if obstacles and segments and getattr(self,'complete_offset_search',False):
                # Exact bounds of the SAME swept solids/axle envelopes used
                # below. A distant roadside cluster cannot violate either
                # clearance floor; retain every possible collision for the
                # unchanged polygon-distance check. No spatial quantization.
                def union_bounds(values):
                    return (min(v[0] for v in values),max(v[1] for v in values),
                            min(v[2] for v in values),max(v[3] for v in values))
                body_bounds=union_bounds([g[1] for a,b,g in segments])
                axle_bounds=union_bounds([g[2] for a,b,g in segments])
                nearby=[]
                for obstacle in obstacles:
                    bounds=polygon_bounds(obstacle);lower=bounds_distance(body_bounds,bounds)
                    if (lower<=self.config.obstacle_margin_m or
                            bounds_distance(axle_bounds,bounds)<self.config.reference_clearance_m):
                        nearby.append(obstacle)
                    else:self._min_clearance=min(self._min_clearance,lower)
                obstacles=tuple(nearby)
            for a,b,geometry in segments:
                if body_cache is None:
                    reason=self._body_safety(a,b,obstacles,prepared=geometry)
                else:
                    key=(a,b)
                    cached=body_cache.get(key)
                    if cached is None:
                        previous=self._min_clearance;self._min_clearance=math.inf
                        try:
                            reason=self._body_safety(a,b,obstacles,prepared=geometry)
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
        if native_reference:
            # Explicit MORAI junction experiment only: a native polyline corner
            # is a PP reference, not a commanded wheel trajectory. Body/map/
            # observation checks above remain; _rollout below checks actual
            # steering, rate, lateral acceleration, tracking and live solids.
            return ''
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

    def _normal_hazards(self, points, obstacles, *, prediction_margin_m=0.):
        if not obstacles:return ()
        path_poses=poses(points)
        # Geometry belongs to the path segment, not to each roadside object.
        segments=tuple((a,b,self._fixed_body_geometry(a,b)) for a,b in zip(path_poses,path_poses[1:]))
        if prediction_margin_m:
            # Detection must anticipate the admitted tracking envelope. The
            # measured-body collision floor stays unchanged. Otherwise a
            # roadside object 0.251 m from the ideal path is ignored until a
            # few centimetres of valid PP tracking error brings it inside the
            # actual 0.25 m floor, too late for either braking or avoidance.
            segments=tuple((a,b,(expanded,polygon_bounds(expanded),g[2],g[3]))
                for a,b,g in segments for expanded in (self._inflate(g[0],prediction_margin_m),))
        if self.config.native_obstacle_geometry:
            from .native_obstacle_geometry import normal_hazards
            hazards,minimum=normal_hazards(segments,obstacles,self.config.obstacle_margin_m,
                self.config.reference_clearance_m)
            self._min_clearance=min(self._min_clearance,minimum)
            return hazards
        def envelope(rows,index):
            bounds=[row[2][index] for row in rows]
            return (min(b[0] for b in bounds),max(b[1] for b in bounds),
                    min(b[2] for b in bounds),max(b[3] for b in bounds))
        blocks=[]
        for start in range(0,len(segments),16):
            rows=segments[start:start+16]
            blocks.append((rows,envelope(rows,1),envelope(rows,2)))
        hazards=[]
        for obstacle in obstacles:
            solid=tuple(map(tuple,obstacle))
            other=self._obstacle_bounds.get(solid)
            if other is None:
                other=polygon_bounds(solid);self._obstacle_bounds[solid]=other
            found=False
            for rows,body_bounds,axle_bounds in blocks:
                if (bounds_distance(body_bounds,other)>self.config.obstacle_margin_m and
                        bounds_distance(axle_bounds,other)>=self.config.reference_clearance_m):
                    # A union box is a conservative lower bound for EVERY
                    # original sweep in this block. Both unchanged safety
                    # floors are proven; retain the exact per-sweep reported
                    # clearance, rather than substituting the coarser bound.
                    self._min_clearance=min(self._min_clearance,
                        min(bounds_distance(geometry[1],other) for a,b,geometry in rows))
                    continue
                for a,b,geometry in rows:
                    if self._obstacle_safety(a,b,(solid,),prepared=geometry) in ('BODY_COLLISION','REFERENCE_CLEARANCE'):
                        hazards.append(obstacle);found=True;break
                if found:break
        return tuple(hazards)

    def _remember(self, detected):
        if self.config.observed_union_memory:
            from .observed_union_memory import ObservedUnionMemory
            if self.state=='NORMAL' and not self._motion_authorized:
                # Disarmed preparation owns no moving maneuver. Capture the
                # complete current scene rather than accumulating minutes of
                # parked sensor jitter. Once armed, static occluded occupancy
                # persists through the maneuver and all collision gates.
                self._observed_union_memory=None;self.remembered=()
            if self._observed_union_memory is None:
                self._observed_union_memory=ObservedUnionMemory(self.remembered)
            self.remembered=self._observed_union_memory.update(detected,self.remembered,
                self.config.observation_padding_m,self.config.memory_merge_extent_m)
            return
        # No IDs are present in PoseArray. Merge overlapping observed static
        # solids conservatively, rather than accumulating one noisy box/frame.
        result=list(self.remembered)
        bounds={old:polygon_bounds(old) for old in result}
        for obstacle in detected:
            # Validate a conservative envelope once. Contained measurement
            # jitter neither shrinks the solid nor restarts fresh-pose waits.
            incoming=polygon_bounds(obstacle)
            if any(bounds[old][0]<=incoming[0] and bounds[old][1]>=incoming[1]
                   and bounds[old][2]<=incoming[2] and bounds[old][3]>=incoming[3]
                   and convex_covers(old,obstacle) for old in result):
                continue
            obstacle=self._inflate(obstacle,self.config.observation_padding_m)
            incoming=polygon_bounds(obstacle);envelope=incoming;matches=[]
            for i,old in enumerate(result):
                previous=bounds[old]
                if bounds_distance(previous,incoming)>self.config.obstacle_margin_m*math.sqrt(2):continue
                joined=(min(envelope[0],previous[0]),max(envelope[1],previous[1]),
                        min(envelope[2],previous[2]),max(envelope[3],previous[3]))
                limit=self.config.memory_merge_extent_m
                if limit and max(joined[1]-joined[0],joined[3]-joined[2])>limit:continue
                if intersects(self._inflate(old,self.config.obstacle_margin_m),obstacle):
                    matches.append(i);envelope=joined
            combined=convex_hull(list(obstacle)+[p for i in matches for p in result[i]])
            result=[old for i,old in enumerate(result) if i not in matches]+[combined]
            bounds[combined]=polygon_bounds(combined)
        self.remembered=tuple(sorted(result))

    def _rollout_distance(self, speed):
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        if self.config.continuous_lane_follow:
            # The unchanged rollout advances one duration. Include its PP
            # target, the full front body and tracking support. A second whole
            # duration is unnecessary for this continuously revalidated window.
            return (velocity*self.config.rollout_seconds+max(lookahead,
                self.body.wheelbase_m+self.body.front_overhang_m)+2*self.config.tracking_limit_m)
        return 2*velocity*self.config.rollout_seconds+lookahead+2*self.config.tracking_limit_m

    def _rollout(self, points, pose, speed, actual_curvature, obstacles, duration_s=None):
        self._check_phase='PP_ROLLOUT'
        velocity=max(speed,self.config.avoid_speed_kmh)/3.6
        lookahead=calculate_lookahead_distance(velocity*3.6,self.config.lookahead_min_m,
            self.config.lookahead_max_m,self.config.lookahead_preview_time_sec)
        if abs(actual_curvature)>self.config.curvature_limit:
            return 'ACTUAL_CURVATURE'
        from .local_path_planner import LocalPathPlanner
        fixed_points=LocalPathPlanner().prepare(points)
        if fixed_points is None:return 'INVALID_CONTROL_WINDOW'
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
            check_search_budget(self)
            if duration_s is not None and tick%max(1,round(self.config.control_rate_hz*.1))==0:
                points=control_local_path(fixed_points,pose,self._control_horizon_m)
            lookahead=path_lookahead_distance(velocity*3.6,points,pose,
                self.config.lookahead_min_m,self.config.lookahead_max_m,
                self.config.lookahead_preview_time_sec,self.config.lookahead_heading_budget_rad)
            if lookahead is None:return 'INVALID_LOOKAHEAD'
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
            # Control limits the outgoing command before transport delay.
            # Apply the same ordering here, including the measured initial turn.
            requested_angle=limit_steering_change(
                command/self.config.steering_sign*self.config.max_wheel_angle_rad,
                requested_angle,self.config.steering_slew_limit_radps,dt)
            queue.append((tick*dt+self.config.command_delay_s,requested_angle))
            while queue and queue[0][0]<=tick*dt+1e-12:
                applied_angle=queue.popleft()[1]
            curvature=math.tan(applied_angle)/self.config.wheelbase_m
            lateral=velocity**2*abs(curvature)
            if lateral>self.config.max_lateral_acceleration_mps2:
                self._collision_context=dict(phase='PP_ROLLOUT',prediction_t_s=tick*dt,
                    pose=tuple(pose),speed_kmh=velocity*3.6,curvature_m_inv=curvature,
                    lateral_acceleration_mps2=lateral,
                    limit_mps2=self.config.max_lateral_acceleration_mps2,
                    initial_curvature_m_inv=actual_curvature,applied_wheel_angle_rad=applied_angle)
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

    def _entry_preparation(self,pair,pose,speed,actual_curvature,z):
        prepared,prefix=self._preparation(pose,speed,actual_curvature,z)
        if not (getattr(self,'complete_offset_search',False) and
                getattr(pair,'mode','lane')=='road_offset'):
            return prepared,prefix,actual_curvature
        # _rollout proves the real command delay. The reference can already
        # follow the native bend used by Control while the worker runs. A
        # straight delay reference diverges on that bend before adoption.
        distance=arc(prefix)[-1]
        end=pair.original_index.project(pose[:2])+distance
        prefix=transition(pose[:2],pose[2],actual_curvature,pair.original,end,distance)
        target=sample(pair.original,end)
        prepared=(*target[0][:2],math.atan2(target[1][1],target[1][0]))
        return prepared,prefix,curvature_at(pair.original,end)

    def _entry_lengths(self):
        """Refine the configured family; physical gates decide admissibility."""
        base=self.config.transition_lengths_m
        step=self.config.transition_search_step_m
        low=max(step,min(base)-2*step)
        high=max(base)+2*step
        # Bound work independently of a pathological parameter value.
        count=min(16,math.ceil((high-low)/step)+1)
        return tuple(sorted(set(base)|{low+i*step for i in range(count)}))

    def _detection_entry_distance(self, pose, current, route, route_index):
        """Schedule detection from map geometry; never authorize a trajectory.

        A quintic lane displacement supplies curvature/rate lower bounds.
        The full swept body, boundaries, tracking rollout and live solids
        still decide every candidate. Missing/complex map evidence keeps the
        former conservative family bound. Only immutable map pairs are cached.
        """
        cfg=self.config;velocity=cfg.avoid_speed_kmh/3.6
        fallback=max(self._entry_lengths())+max(cfg.entry_start_offsets_m)
        key=(current,tuple(route),route_index)
        if key not in self._detection_pairs:
            pairs=[]
            for side in ('left','right'):
                try:
                    pair=self.network.pair(current,side,list(route),route_link_index=route_index)
                    if pair is not None:pairs.append(pair)
                except (ValueError,TypeError,IndexError,OverflowError):pass
            if len(self._detection_pairs)>=16:self._detection_pairs.clear()
            self._detection_pairs[key]=tuple(pairs)
        lengths=[];details=[]
        for pair in self._detection_pairs[key]:
            station=pair.original_index.project(pose[:2])
            rows=[pair.at(station+d) for d in (0.,6.,12.,18.,24.)]
            if any(row is None for row in rows):continue
            lateral=max(abs(row['separation']) for row in rows)
            native=max(abs(k) for k in curvatures([row['original'][0] for row in rows]))
            available=min(cfg.curvature_limit,cfg.max_lateral_acceleration_mps2/velocity**2)-native
            rate=cfg.steering_slew_limit_radps or cfg.max_wheel_rate_radps
            if available<=0 or rate<=0:continue
            curvature_length=math.sqrt((10./math.sqrt(3.))*lateral/available)
            rate_length=(60.*cfg.wheelbase_m*lateral*velocity/rate)**(1./3.)
            required=max(curvature_length,rate_length)
            supported=next((length for length in self._entry_lengths() if length>=required),None)
            if supported is None:continue
            lengths.append(supported)
            details.append(dict(side=pair.side,separation_m=lateral,native_curvature=native,
                curvature_length_m=curvature_length,steering_rate_length_m=rate_length,
                transition_m=supported))
        length=max(lengths) if lengths else fallback
        lookahead=calculate_lookahead_distance(cfg.avoid_speed_kmh,cfg.lookahead_min_m,
            cfg.lookahead_max_m,cfg.lookahead_preview_time_sec)
        preparation=lookahead+velocity*cfg.command_delay_s
        search_advance=max(4.,velocity*2.) if cfg.continuous_lane_follow else 0.
        self._entry_distance_basis=dict(source='map_kinematic_estimate' if lengths else 'conservative_fallback',
            transition_m=length,preparation_m=preparation,pairs=details,
            search_advance_m=search_advance,
            latency_s=cfg.normal_detection_latency_s,deceleration_mps2=cfg.deceleration_mps2)
        return length+preparation+search_advance

    def _road_offsets(self, road, reference, side):
        """Probe actual map-supported cross-section, never a guessed width.

        Sparse initial LiDAR faces need not limit the available passing gap.
        Every proposed value still gets whole-trajectory and PP validation.
        Sample every supported interval, including space beyond a map hole.
        Full swept containment still rejects trajectories crossing that hole.
        """
        index=PolylineIndex(reference)
        front=min(index.project(p) for o in self._hazards for p in o)
        row=sample(reference,max(0.,min(index.lengths[-1],front)))
        point,tangent=row;sign=1 if side=='left' else -1
        yaw=math.atan2(tangent[1],tangent[0]);normal=(-sign*tangent[1],sign*tangent[0])
        intervals=road.cross_section(point[:2],normal)
        # Nominal offset search leaves room for the admitted tracking error
        # AND the next measured-pose rollout's unchanged tracking envelope.
        margin=2*self.config.tracking_limit_m
        half=self.body.width_m/2+margin
        supported=[]
        complete_search=getattr(self,'complete_offset_search',False)
        passage_solids=(tuple(self._inflate(o,2*self.config.tracking_limit_m+self.config.observation_padding_m)
                              for o in self.remembered) if complete_search else self.remembered)
        passage_bounds=tuple((o,polygon_bounds(o)) for o in passage_solids)
        evidence=dict(source=road.source,side=side,probe_point_xy=tuple(point[:2]),
            intervals_m=intervals,reference_body_covered=road.covers(self._inflate(
                self.body.polygon(point[0],point[1],yaw),margin)),
            road_fit_probes=0,obstacle_rejected_probes=0,boundary_rejected_probes=0)
        self._entry_context.setdefault('road_cross_sections',[]).append(evidence)
        def covered(offset):
            check_search_budget(self)
            solid=self.body.polygon(point[0]+offset*normal[0],point[1]+offset*normal[1],yaw)
            if not road.covers(self._inflate(solid,margin)):return False
            evidence['road_fit_probes']+=1
            if self.config.continuous_lane_follow:
                bounds=polygon_bounds(solid);center=(point[0]+offset*normal[0],point[1]+offset*normal[1])
                center_bounds=(center[0],center[0],center[1],center[1])
                distance=polygon_distance
                if self.config.native_obstacle_geometry:
                    from .native_obstacle_geometry import distance
                # Exactly the old body/axle floors on the SAME padded solids.
                # Bounds eliminate only comparisons that cannot reach either
                # floor, including thousands of far probes at wide junctions.
                if any((bounds_distance(bounds,other)<=self.config.obstacle_margin_m and
                        distance(solid,o)<=self.config.obstacle_margin_m) or
                       (complete_search and bounds_distance(center_bounds,other)<self.config.reference_clearance_m and
                        distance((center,),o)<self.config.reference_clearance_m)
                       for o,other in passage_bounds):
                    evidence['obstacle_rejected_probes']+=1;return False
                boundaries=getattr(self.network,'boundaries',None)
                if boundaries is not None and boundaries.swept_reason(solid):
                    evidence['boundary_rejected_probes']+=1;return False
            return True
        for low,high in intervals:
            low=max(.2,low+half);high-=half
            if self.config.continuous_lane_follow:high=min(high,self.config.prediction_distance_m)
            if high<low:continue
            # A parked object can leave a narrow but fully margined interval.
            # The former 20 cm probes skipped those intervals and selected a
            # distant branch instead. Exact union predicates keep fine probes
            # inexpensive; every kept candidate still has a whole-body proof.
            step=.025 if self.config.continuous_lane_follow else .2
            divisions=max(1,math.ceil((high-low)/step))
            previous=None
            for i in range(divisions+1):
                offset=low+(high-low)*i/divisions
                valid=covered(offset)
                if valid:supported.append(offset)
                if previous is not None and valid!=previous[1]:
                    a,b=previous[0],offset
                    # Refine only an observed validity transition. Each kept
                    # point has its own proof; never infer coverage of a gap.
                    for _ in range(8):
                        mid=(a+b)/2
                        if covered(mid)==previous[1]:a=mid
                        else:b=mid
                    supported.append(a if previous[1] else b)
                previous=(offset,valid)
        evidence['supported_probes']=len(supported)
        evidence['classification']=('SUPPORTED_PASSING_SECTION' if supported else
            'NO_PASSING_CLEARANCE' if evidence['reference_body_covered'] else 'ROAD_EVIDENCE_MISSING')
        if not supported:return ()
        supported=sorted(set(supported))
        # Bound expensive trajectory searches while spanning the whole proven
        # section. Solids reject candidates later; they never clip this range.
        proposals={supported[round((len(supported)-1)*i/8)] for i in range(9)}
        if complete_search:
            # Wide junctions must not move the next proposal several metres
            # past a narrow useful gap. Retain the full-range proposals and
            # add nearby deviations, each independently proved by the SAME
            # road, obstacle, axle and physical-boundary cross-section gates.
            for i in range(16):
                value=supported[0]+.2*i
                if value<=supported[-1] and covered(value):proposals.add(value)
        return tuple(sorted(proposals))

    def _ordered_road_offsets(self,offsets):
        values=tuple(sorted(set(offsets)))
        if not values or not getattr(self,'complete_offset_search',False):return values
        # Initial faces can grow as a static object's sides become visible.
        # Prefer another admitted tracking/observation allowance when actual
        # surveyed space supports it. This is a soft search preference: every
        # smaller, tighter alternative remains available with unchanged gates.
        headroom=2*self.config.tracking_limit_m+self.config.observation_padding_m
        preferred=values[0]+headroom
        self._entry_context['offset_headroom_preference_m']=headroom
        return tuple(sorted(values,key=lambda x:(abs(x-preferred),x)))

    def _hazard_clearance(self, points):
        minimum=math.inf
        path_poses=poses(points)
        for a,b in zip(path_poses,path_poses[1:]):
            swept,bounds,_,_=self._body_geometry(a,b)
            for obstacle in self._hazards:
                other=polygon_bounds(obstacle)
                if bounds_distance(bounds,other)>=minimum:continue
                minimum=min(minimum,polygon_distance(swept,obstacle))
        return minimum

    def _entry_obstacles(self, pair):
        """Reserve is longitudinal sensing/braking uncertainty, not a deadline
        for reaching the neighboring center. Sweep against that uncertainty.
        """
        if not self._entry_reserve_m:return self.remembered
        reserved=[]
        for solid in self.remembered:
            station=min(pair.original_index.project(p) for p in solid)
            row=sample(pair.original,max(0.,min(pair.length,station)))
            if row is None:raise ValueError('ENTRY_RESERVE_REFERENCE')
            tx,ty=row[1];reserve=self._entry_reserve_m
            reserved.append(convex_hull(list(solid)+[(x-tx*reserve,y-ty*reserve) for x,y in solid]))
        return tuple(reserved)

    def _road_entry_offsets(self,pair,prepared_s,front):
        """Prioritize onsets inside measured crossing space, before full PP.

        This is an ordering probe only. Every accepted transition still gets
        the full swept road, marking, obstacle and controller checks.
        """
        budget=max(0.,front-prepared_s-min(self._entry_lengths())-
                   self.body.wheelbase_m-self.body.front_overhang_m-self.config.obstacle_margin_m)
        values=sorted(set(self.config.entry_start_offsets_m)|
                      {2.*i for i in range(math.floor(min(budget,40.)/2)+1)})
        floor=getattr(self,'_entry_onset_floor_xy',None)
        minimum=max(0.,pair.original_index.project(floor)-prepared_s) if floor is not None else 0.
        values=sorted({minimum}|{v for v in values if v>=minimum})
        ranked=[]
        for offset in values:
            if offset>budget:continue
            row=pair.at(prepared_s+offset)
            if row is None:continue
            source=sample(pair.original,prepared_s+offset)
            target=sample(pair.adjacent,row['adjacent_s'])
            yaw=math.atan2(source[1][1],source[1][0])
            a=(*source[0][:2],yaw);b=(*target[0][:2],yaw)
            reason=self._map_sweep_reason(a,b,self._body_geometry(a,b),getattr(self.network,'boundaries',None))
            ranked.append((bool(reason),offset))
        # Prefer staying on the reference until the last supported onset.
        # This only orders already enumerated proposals; full entry, passage,
        # return, body and PP proofs still decide admission. Legacy ordering
        # remains unchanged outside the complete-maneuver search.
        return tuple(value for _,value in sorted(ranked,key=lambda row:
            (row[0],-row[1] if getattr(self,'complete_offset_search',False) else row[1])))

    def _generate_entry(self, pair, pose, speed, actual_curvature, front, body_cache=None):
        """Search both transition length and entry position, without refitting
        an approved maneuver. Every accepted entry is tracked to completion.
        """
        self.candidate_lane_batches+=1
        area=getattr(pair,'road_area',None)
        if area is not None:
            margin=self.config.tracking_limit_m*(2 if getattr(pair,'mode','lane')=='road_offset' else 1)
            if not area.covers(self._inflate(self.body.polygon(*pose),margin)):
                # Every prospective trajectory starts at this same measured
                # body. No transition length or offset can repair absent
                # containment at its origin. Preserve the original gate,
                # but prove this necessary condition before enumerating curves.
                return [LatticeCandidate(pair.side,(),math.nan,math.nan,math.nan,
                    0.,math.inf,{},'ROAD_START_BODY_UNPROVEN',mode=getattr(pair,'mode','lane'))]
        ego=pair.original_index.project(pose[:2]);row=sample(pair.original,ego)
        if row is None:raise ValueError('ORIGINAL_LANE_RANGE')
        prepared,prefix,prepared_curvature=self._entry_preparation(pair,pose,speed,actual_curvature,row[0][2])
        prepared_s=pair.original_index.project(prepared[:2])
        floor=getattr(self,'_entry_onset_floor_xy',None)
        minimum_onset=max(0.,pair.original_index.project(floor)-prepared_s) if floor is not None else 0.
        supported=pair.length-self.body.wheelbase_m-self.body.front_overhang_m-1.
        reserve=self._rollout_distance(speed)
        candidates=[];projection_cache={}
        obstacles=self._entry_obstacles(pair)
        if self.config.continuous_lane_follow and getattr(pair,'mode','lane')=='road_offset':
            # A sparsely visible face grows as Ego approaches. Reserve the
            # admitted tracking envelope and observation padding when choosing
            # the offset; live validation retains the unchanged hard floors.
            observation_reserve=2*self.config.tracking_limit_m+self.config.observation_padding_m
            obstacles=tuple(self._inflate(o,observation_reserve) for o in obstacles)
        original_area=self._road_area
        original_reserve=self._planning_tracking_reserve
        self._road_area=getattr(pair,'road_area',None)
        self._planning_tracking_reserve=(self.config.tracking_limit_m
            if getattr(pair,'mode','lane')=='road_offset' else 0.)
        deferred=self.config.continuous_lane_follow and self.config.optimize_entry_onset and not self._entry_replan_guide
        def generate(length, offset):
            check_search_budget(self)
            entry=path=();end=prepared_s+offset+length;cost=math.inf;terms={};clearance=0.
            try:
                tail_s=end+reserve
                if getattr(pair,'mode','lane')=='road_offset':
                    rear=max(pair.original_index.project(p) for o in self._hazards for p in o)
                    # A small offset can fit entry yet collide during HOLD.
                    # Prove passage of the entire observed obstruction before
                    # committing to this non-centerline guide.
                    passage_support=(self.config.lookahead_min_m if self.config.continuous_lane_follow else reserve)
                    tail_s=max(tail_s,rear+self.body.rear_overhang_m+self.config.return_margin_m+passage_support)
                target=pair.at(end);tail=pair.at(min(supported,tail_s))
                if target is None or tail is None or tail_s>supported:
                    raise ValueError('INSUFFICIENT_ENTRY_SUPPORT')
                start_pose=prepared;initial=prefix;start_turn=prepared_curvature
                if offset:
                    start=sample(pair.original,prepared_s+offset)
                    if start is None:raise ValueError('ORIGINAL_LANE_RANGE')
                    if deferred and math.dist(pose[:2],row[0][:2])<=self.config.tracking_limit_m/4:
                        initial=center_suffix(pair.original,ego,prepared_s+offset)
                    else:
                        approach=transition(prepared[:2],prepared[2],prepared_curvature,
                            pair.original,prepared_s+offset,offset)
                        initial=prefix+approach[1:]
                    start_pose=(*start[0][:2],math.atan2(start[1][1],start[1][0]))
                    start_turn=curvature_at(pair.original,prepared_s+offset)
                recovery=transition(start_pose[:2],start_pose[2],start_turn,
                    pair.adjacent,target['adjacent_s'],length)
                entry=initial+recovery[1:]
                self.generated_transition_count+=1
                path=entry+center_suffix(pair.adjacent,target['adjacent_s'],tail['adjacent_s'])[1:]
                needed=end+self.body.wheelbase_m+self.body.front_overhang_m+\
                    self.config.obstacle_margin_m+self._entry_reserve_m
                if getattr(pair,'mode','lane')=='road_offset':
                    needed=max(needed,tail_s+self.body.wheelbase_m+self.body.front_overhang_m+
                        self.config.obstacle_margin_m+self._entry_reserve_m)
                reason=''
                if self._entry_observation_m is not None and needed-ego>self._entry_observation_m:
                    reason='ENTRY_OBSERVATION_DISTANCE'
                kinematic_curve=(recovery if deferred and self._observation_curvature_limit is None else
                    entry if self.config.native_reference_rollout and getattr(pair,'mode','lane')=='lane' else None)
                if not reason:
                    # Reject an untrackable curve before its expensive map
                    # sweeps. Accepted candidates still receive every sweep
                    # and the full measured-state controller rollout below.
                    reason=self._path_safety(path,(),speed,geometry_check=False,
                        curvature_points=kinematic_curve)
                if not reason:reason=self._path_safety(path,obstacles,speed,body_cache=body_cache,
                    curvature_points=kinematic_curve)
                if not reason:
                    duration=arc(entry)[-1]/(max(speed,self.config.avoid_speed_kmh)/3.6)+self.config.rollout_seconds
                    if getattr(pair,'mode','lane')=='road_offset':
                        duration=max(duration,(arc(path)[-1]-reserve)/(max(speed,self.config.avoid_speed_kmh)/3.6)+self.config.rollout_seconds)
                    reason=self._rollout(path,pose,speed,actual_curvature,obstacles,duration_s=duration)
                if not reason and getattr(pair,'mode','lane')=='road_offset':
                    clearance=self._hazard_clearance(path)
                if not reason or not self.config.continuous_lane_follow:
                    cost,terms=reference_cost(path,pair.original,pair.adjacent,self.weights,
                                              projection_cache=projection_cache)
            except (ValueError,TypeError,IndexError,OverflowError) as error:
                reason=str(error)
            candidate=LatticeCandidate(pair.side,path or entry,end,math.nan,math.nan,
                length,cost,terms,reason,path or entry,(),entry,
                entry_start_offset_m=offset,mode=getattr(pair,'mode','lane'),obstacle_clearance_m=clearance)
            if (not reason and getattr(self,'complete_offset_search',False) and candidate.mode=='road_offset'):
                # Entry length/onset and return opening are coupled. Do not
                # stop this family at an entry whose return cannot be proved:
                # a later onset or another length can reach the real opening.
                try:
                    if self._offset_return_proof(candidate,pair,speed) is None:
                        candidate=replace(candidate,rejection='OFFSET_RETURN_UNPROVEN')
                except (ValueError,TypeError,IndexError,OverflowError) as error:
                    candidate=replace(candidate,rejection='OFFSET_RETURN_ERROR:'+str(error))
            candidates.append(candidate)
        try:
            if deferred:
                request=self._entry_onset_request(pair,pose,speed,actual_curvature,front)
                for length in self._entry_lengths():
                    if length+1e-9<request['minimum_transition_m']:continue
                    offset=max(0.,front-self.body.wheelbase_m-self.body.front_overhang_m-
                        self.config.obstacle_margin_m-request['reserve_m']-length-prepared_s)
                    # PP begins responding before the geometric bend reaches
                    # the rear axle. Test that measured controller lead as a
                    # geometric onset compensation; every candidate still
                    # needs full nominal-body and actual-PP clearance proofs.
                    offset+=request['controller_lookahead_m']
                    # A safe original-lane approach is part of the fixed path;
                    # computation/commit happens now, not at the onset point.
                    attempts=min(16,math.ceil(offset/self.config.transition_search_step_m)+1)
                    for i in range(attempts):
                        generate(length,max(0.,offset-i*self.config.transition_search_step_m))
                        if not candidates[-1].rejection:break
                    if any(not c.rejection for c in candidates):break
            elif self.config.continuous_lane_follow and getattr(pair,'mode','lane')=='road_offset':
                lengths=tuple(dict.fromkeys(self.config.transition_lengths_m+self._entry_lengths()))
                for offset in self._road_entry_offsets(pair,prepared_s,front):
                    for length in lengths:
                        generate(length,offset)
                        if not candidates[-1].rejection:break
                    if any(not c.rejection for c in candidates):break
            else:
                for length in self.config.transition_lengths_m:generate(length,minimum_onset)
            if (not any(not c.rejection for c in candidates) and
                    not (self.config.continuous_lane_follow and getattr(pair,'mode','lane')=='road_offset')):
                for offset in (value+minimum_onset for value in self.config.entry_start_offsets_m):
                    for length in self._entry_lengths():
                        if offset==0. and length in self.config.transition_lengths_m:continue
                        generate(length,offset)
        finally:
            self._road_area=original_area
            self._planning_tracking_reserve=original_reserve
        return candidates

    def _entry_onset_request(self,pair,pose,speed,curvature,front):
        """Physical maneuver length and a separate post-computation reserve."""
        cfg=self.config;velocity=max(speed,cfg.avoid_speed_kmh)/3.6
        station=pair.original_index.project(pose[:2])
        rows=[pair.at(max(0.,min(pair.length,s))) for s in (station,(station+front)/2,front)]
        separation=max(abs(row['separation']) for row in rows if row is not None)
        start=max(station,front-max(cfg.transition_lengths_m))
        native=max(abs(curvature),*(abs(curvature_at(pair.original,max(0.,min(pair.length,s))))
                                   for s in (start,(start+front)/2,front)))
        available=min(cfg.curvature_limit,cfg.max_lateral_acceleration_mps2/velocity**2)-native
        rate=cfg.steering_slew_limit_radps or cfg.max_wheel_rate_radps
        curvature_length=math.sqrt((10./math.sqrt(3.))*separation/available) if available>0 else math.inf
        rate_length=(60.*cfg.wheelbase_m*separation*velocity/rate)**(1./3.) if rate>0 else math.inf
        row=sample(pair.original,max(0.,min(pair.length,station)))
        error=math.dist(pose[:2],row[0][:2])
        reserve=(cfg.tracking_limit_m+error+cfg.observation_padding_m+
                 velocity*(cfg.command_delay_s+1/cfg.control_rate_hz))
        return dict(minimum_transition_m=max(curvature_length,rate_length),
            separation_m=separation,curvature_length_m=curvature_length,
            steering_rate_length_m=rate_length,native_curvature=native,
            reserve_m=reserve,tracking_error_m=error,speed_kmh=speed,
            command_delay_s=cfg.command_delay_s,planning_speed_kmh=velocity*3.6,
            controller_lookahead_m=calculate_lookahead_distance(velocity*3.6,cfg.lookahead_min_m,
                cfg.lookahead_max_m,cfg.lookahead_preview_time_sec))

    def _search_entry(self, reference, pose, speed_kmh, actual_curvature, current_link_id,
                      route_link_ids, route_link_index, distance, body_cache=None, *,
                      lane_search_exhausted=False):
        pairs=[];all_candidates=[]
        self._entry_context=dict(current_link_id=current_link_id,
            route_occurrences=list(route_link_ids).count(current_link_id),pose=tuple(pose),sides={})
        self._entry_context['morai_unmarked_junction_experiment']=getattr(
            self.network,'morai_unmarked_junction_experiment',False)
        for side in ('left','right'):
            pair=None
            try:
                pair=self.network.pair(current_link_id,side,list(route_link_ids),route_link_index=route_link_index)
                self._entry_context['sides'][side]=dict(
                    target_link_id=self.network.links.get(current_link_id,{}).get(side+'_lane_change_dst_link_idx'),
                    permission_reason=self.network.permission_reason(current_link_id,side),
                    valid_neighbor=self.network.neighbor(current_link_id,side),
                    pair_available=pair is not None)
                if pair is None:continue
                setattr(self,side+'_lane',pair.adjacent)
                pairs.append((pair,[]))
            except (ValueError,TypeError,IndexError,OverflowError) as error:
                self._entry_context['sides'].setdefault(side,{})['error']=str(error)
                if (str(error)=='ORIGINAL_LANE_RANGE' and pair is not None
                        and -self.config.entry_geometry_wait_distance_m
                        <=pair.original_index.project(pose[:2])<0.):
                    all_candidates.append(LatticeCandidate(side,(),math.nan,math.nan,math.nan,
                        0.,math.inf,{},'ENTRY_START_BEFORE_LANE'))
                continue
        roads=[]
        explicit=getattr(self.network,'road_corridor',None)
        if explicit is not None:roads.append(explicit)
        boundaries=getattr(self.network,'boundaries',None)
        if boundaries is not None and hasattr(boundaries,'road_corridor_for_pair'):
            for pair,_ in pairs:
                try:area=boundaries.road_corridor_for_pair(self.network,pair)
                except (ValueError,TypeError,IndexError,OverflowError):area=None
                if area is not None:roads.append(area)
        self._entry_context['road_offset_evidence']=';'.join(r.source for r in roads) or 'UNAVAILABLE'
        chosen=None
        if self.config.integrated_strategy:
            kind=self.network.links.get(current_link_id,{}).get('link_type')
            chosen=self._maneuver_selection.select(Situation(
                context=(current_link_id,tuple(route_link_ids)),static_path_blocked=True,
                moving_conflict=getattr(self,'_moving_conflict',False),
                lane_topology_clear=not lane_search_exhausted and kind in ('6',6) and bool(pairs),
                road_area_verified=bool(roads),
                replan_required=True))
            self._strategy=chosen.strategy
            self._entry_context.update(strategy=chosen.strategy.value,strategy_reason=chosen.reason,
                strategy_generation=chosen.generation)
            if lane_search_exhausted:self._entry_context['lane_search_exhausted']=True
            if chosen.strategy==Strategy.BLOCKED:return self._block(chosen.reason)
            if chosen.strategy==Strategy.DYNAMIC_TRAFFIC:
                return self._block('DYNAMIC_TRAJECTORY_SEARCH_NOT_READY')
        if chosen is None or chosen.strategy==Strategy.LANE_STATIC:
            for i,(pair,_) in enumerate(pairs):
                try:
                    front_s=min(pair.original_index.project(p) for o in self._hazards for p in o)
                    candidates=self._generate_entry(pair,pose,speed_kmh,actual_curvature,front_s,body_cache)
                except (ValueError,TypeError,IndexError,OverflowError) as error:
                    self._entry_context['sides'][pair.side]['error']=str(error)
                    candidates=[]
                    if (str(error)=='ORIGINAL_LANE_RANGE' and
                            -self.config.entry_geometry_wait_distance_m<=pair.original_index.project(pose[:2])<0.):
                        candidates=[LatticeCandidate(pair.side,(),math.nan,math.nan,math.nan,
                            0.,math.inf,{},'ENTRY_START_BEFORE_LANE')]
                pairs[i]=(pair,candidates);all_candidates.extend(candidates)
        if chosen is not None and chosen.strategy==Strategy.LANE_STATIC:roads=[]
        if self.config.continuous_lane_follow and any(not c.rejection for _,cs in pairs for c in cs):
            roads=[]
            self._entry_context['road_offset_search_skipped']='VALID_NATIVE_LANE'
        complete_search=getattr(self,'complete_offset_search',False)
        early_offset_proofs={}
        for road in roads:
            if not road.covers(self._inflate(self.body.polygon(*pose),2*self.config.tracking_limit_m)):
                self._entry_context.setdefault('road_origin_pruning',[]).append(dict(
                    source=road.source,reason='ROAD_START_BODY_UNPROVEN',
                    classification='MISSING_MAP_CONTAINMENT_NOT_PHYSICAL_IMPOSSIBILITY'))
                all_candidates.append(LatticeCandidate('left',(),math.nan,math.nan,math.nan,
                    0.,math.inf,{},'ROAD_START_BODY_UNPROVEN',mode='road_offset'))
                continue
            for side in ('left','right'):
                if side not in road.allowed_sides:continue
                offsets=self._road_offsets(road,reference,side)
                if not offsets:
                    all_candidates.append(LatticeCandidate(side,(),math.nan,math.nan,math.nan,
                        0.,math.inf,{},'ROAD_AREA_UNPROVEN',mode='road_offset',obstacle_clearance_m=0.))
                    continue
                explored={};offset_rejections={};offset_collision_counts={};offset_map_counts={}
                def evaluate_offset(offset):
                    maximum=None
                    if self.config.continuous_lane_follow:
                        at=PolylineIndex(self.network.links[current_link_id]['points']).project(pose[:2])
                        maximum=max(0.,at)+self.config.prediction_distance_m+2*max(self.config.transition_lengths_m)
                    pair=road.offset_pair(self.network,current_link_id,side,list(route_link_ids),
                        offset,route_link_index,maximum_station_m=maximum)
                    if pair is None:return
                    front_s=min(pair.original_index.project(p) for o in self._hazards for p in o)
                    cs=self._generate_entry(pair,pose,speed_kmh,actual_curvature,front_s,body_cache)
                    all_candidates.extend(cs);pairs.append((pair,cs))
                    explored[offset]=any(not c.rejection for c in cs)
                    offset_rejections[offset]={c.rejection for c in cs}
                    offset_collision_counts[offset]=sum(c.rejection in ('BODY_COLLISION','REFERENCE_CLEARANCE') for c in cs)
                    offset_map_counts[offset]=sum(c.rejection=='ROAD_AREA_UNPROVEN' or
                        c.rejection.startswith(('LANE_BOUNDARY_COLLISION:','PHYSICAL_BOUNDARY_COLLISION:')) for c in cs)
                if self.config.continuous_lane_follow:
                    offsets=self._ordered_road_offsets(offsets)
                refinement_budget=8;previous_offset=None
                for offset in offsets:
                    evaluate_offset(offset)
                    if (self.config.continuous_lane_follow and previous_offset is not None and offset>previous_offset and
                            not explored.get(offset,False) and not explored.get(previous_offset,False) and
                            offset_rejections.get(previous_offset,set())&{'BODY_COLLISION','REFERENCE_CLEARANCE'} and
                            any(r=='ROAD_AREA_UNPROVEN' or r.startswith(('LANE_BOUNDARY_COLLISION:','PHYSICAL_BOUNDARY_COLLISION:'))
                                for r in offset_rejections.get(offset,set()))):
                        # A narrow passing interval can lie between a collision
                        # at the lower probe and a divider at the upper one.
                        # Refine that bracket before jumping into a distant lane.
                        low,high=previous_offset,offset
                        while refinement_budget and high-low>.01:
                            mid=(low+high)/2;refinement_budget-=1;evaluate_offset(mid)
                            if explored.get(mid,False):break
                            if offset_collision_counts.get(mid,0)>offset_map_counts.get(mid,0):low=mid
                            else:high=mid
                    if self.config.continuous_lane_follow and any(explored.values()):break
                    previous_offset=offset
                if self.config.continuous_lane_follow and any(explored.values()):
                    high=min(value for value,valid in explored.items() if valid)
                    lower=[value for value,valid in explored.items() if value<high and not valid]
                    if lower:
                        low=max(lower)
                        for _ in range(4):
                            if high-low<.01:break
                            mid=(low+high)/2;evaluate_offset(mid)
                            if explored.get(mid,False):high=mid
                            else:low=mid
                # Cross-section fit alone does not prove a curved transition.
                # Refine the gap beyond the farthest WHOLE-trajectory success
                # instead of losing up to one coarse step of usable road.
                for _ in range(4):
                    if self.config.continuous_lane_follow and any(explored.values()):break
                    safe_offsets=[value for value,valid in explored.items() if valid]
                    if not safe_offsets:break
                    low=max(safe_offsets)
                    higher=[value for value in explored if value>low]
                    if not higher:break
                    high=min(higher)
                    if high-low<.01:break
                    evaluate_offset((low+high)/2)
                if complete_search:
                    # A wider surveyed map exposes many distant offsets on
                    # the other side. Prove the best current out-and-back
                    # maneuver before enumerating those alternatives. An
                    # entry alone NEVER short-circuits the opposite side.
                    current_safe=sorted(((c,p) for p,cs in pairs for c in cs
                        if c.mode=='road_offset' and not c.rejection),
                        key=lambda row:(row[0].cost,-row[0].obstacle_clearance_m))
                    for candidate,pair in current_safe:
                        proof=self._offset_return_proof(candidate,pair,speed_kmh)
                        if proof is not None:
                            early_offset_proofs[id(candidate)]=proof
                            self._entry_context['offset_search_order']='FIRST_COMPLETE_PROOF_BEFORE_DISTANT_SIDE_ENUMERATION'
                            break
                    if early_offset_proofs:break
            if early_offset_proofs:break
        self.candidates=tuple(all_candidates)
        safe=[(c,p) for p,cs in pairs for c in cs if not c.rejection]
        offset_return_proofs={}
        if self.config.continuous_lane_follow:
            feasible=[];rejected={}
            # The complete-offset solver ranks by the same final cost. Prove
            # the best candidate first and stop at its first complete success;
            # proving every more expensive duplicate consumes the entry window.
            complete_search=getattr(self,'complete_offset_search',False)
            if complete_search:safe.sort(key=lambda row:(row[0].cost,-row[0].obstacle_clearance_m))
            for candidate,pair in safe:
                if candidate.mode!='road_offset':
                    feasible.append((candidate,pair));continue
                proof=early_offset_proofs.get(id(candidate)) or self._offset_return_proof(candidate,pair,speed_kmh)
                if proof is not None:
                    feasible.append((candidate,pair))
                    offset_return_proofs[id(candidate)]=proof
                    if complete_search:break
                else:rejected[id(candidate)]=replace(candidate,rejection='OFFSET_RETURN_UNPROVEN')
            if rejected:
                self.candidates=tuple(rejected.get(id(c),c) for c in self.candidates)
                self._entry_context['offset_return_rejected']=len(rejected)
                if not feasible:return self._block('OFFSET_RETURN_UNPROVEN')
            safe=feasible
        if not safe:
            # A lane junction may have safe entry geometry a little ahead.
            # Never waive candidate rejection. The S4 target may be used
            # only on the independently validated original lane.
            retryable=bool(all_candidates) and all(c.rejection in
                ('CURVATURE','INSUFFICIENT_ENTRY_SUPPORT','ENTRY_START_BEFORE_LANE') for c in all_candidates)
            progress=PolylineIndex(self._global_path).project(pose[:2])
            if self._entry_wait_origin is None:self._entry_wait_origin=progress
            remaining=self.config.entry_geometry_wait_distance_m-(progress-self._entry_wait_origin)
            reserve=self._rollout_distance(speed_kmh)+self.body.wheelbase_m+self.body.front_overhang_m+2.
            if retryable and remaining>1e-6 and distance>reserve:
                approach=self._detection_path(reference,pose)
                if approach['blocked']:return approach
                index=PolylineIndex(self._global_path)
                end=min(index.project(approach['points'][-1]),progress+self._control_horizon_m)
                path,reason=self._approach_candidates(reference,pose,speed_kmh,actual_curvature,end,body_cache)
                if reason:return self._block(reason)
                self._entry_wait_path=path
                self._entry_wait_retry=progress+min(2.,remaining)
                return self._entry_approach(pose,speed_kmh,actual_curvature,body_cache)
            available=bool(all_candidates) or any(v.get('pair_available') for v in self._entry_context['sides'].values())
            return self._block('NO_SAFE_ENTRY' if available else 'NO_VALID_ADJACENT_LANE')
        self._entry_wait_path=()
        # A road-offset path proves passage of the WHOLE obstruction, unlike
        # an entry-only lane tail. Prefer its largest verified obstacle gap,
        # then use the existing path cost; every hard floor stays mandatory.
        offsets=[row for row in safe if row[0].mode=='road_offset']
        self.selected,self.pair=(min(offsets,key=lambda row:(row[0].cost,-row[0].obstacle_clearance_m))
                                 if offsets and self.config.continuous_lane_follow else
                                 min(offsets,key=lambda row:(-row[0].obstacle_clearance_m,row[0].cost))
                                 if offsets else min(safe,key=lambda row:row[0].cost))
        self._road_area=getattr(self.pair,'road_area',None)
        if id(self.selected) in offset_return_proofs:
            self._entry_context['offset_return_proof']=offset_return_proofs[id(self.selected)]
        if self.config.continuous_lane_follow and self.config.optimize_entry_onset:
            front=min(self.pair.original_index.project(p) for o in self._hazards for p in o)
            timing=self._entry_onset_request(self.pair,pose,speed_kmh,actual_curvature,front)
            prepared,_=self._preparation(pose,speed_kmh,actual_curvature,self.selected.entry_path[0][2])
            onset=self.pair.original_index.project(prepared[:2])+self.selected.entry_start_offset_m
            timing.update(onset_station_m=onset,
                onset_obstacle_front_gap_m=front-onset-self.body.wheelbase_m-self.body.front_overhang_m,
                transition_length_m=self.selected.transition_length_m,
                approach_length_m=self.selected.entry_start_offset_m,
                calculation_deferred=False)
            self._entry_context['onset_plan']=timing
        if chosen is not None:self._maneuver_selection.commit(chosen,id(self.selected.entry_path),path_valid=True)
        self._last_progress=None;self._original=self.pair.original
        self.state='AVOID_ENTRY';self.reason='';self.entry_commits+=1
        return self._result(self.selected.entry_path)

    def _offset_return_proof(self,candidate,pair,speed):
        """Do not commit an offset that traps Ego behind a later solid line.

        This is a forecast from the observed static envelope, not permission
        to execute a future return. The live return still rechecks all inputs.
        Only anchors inside the already proved entry/hold guide are searched.
        """
        probe=copy(self);probe.pair=pair;probe.selected=candidate
        probe._road_area=pair.road_area;probe._validated_paths={}
        probe._obstacle_bounds=dict(self._obstacle_bounds)
        probe._map_sweep_cache=type(self._map_sweep_cache)()
        probe._fixed_sweep_cache=type(self._fixed_sweep_cache)()
        last=pair.original_index.project(candidate.entry_path[-1])
        rear=max(pair.original_index.project(p) for o in self._hazards for p in o)
        start=max(candidate.entry_end_s,rear+self.body.rear_overhang_m+self.config.return_margin_m)
        for advance in range(0,9,2):
            station=start+advance
            if station>last:break
            row=pair.at(station)
            if row is None:continue
            target=sample(pair.adjacent,row['adjacent_s'])
            pose=(*target[0][:2],math.atan2(target[1][1],target[1][0]))
            if not probe._return_clearance_ready(pose):continue
            from .lane_geometry import curvature_at
            reason=probe._generate_return(pose,speed,curvature_at(pair.adjacent,row['adjacent_s']),{})
            if not reason:
                return dict(source='LIVE_LIDAR_AND_VERIFIED_MAP',station_m=station,
                            transition_length_m=probe.selected.transition_length_m)
        return None

    def _return_permission(self, start, end):
        if start<0 or end>self.pair.length:return 'RETURN_OUTSIDE_LANE_SUPPORT'
        if getattr(self.pair,'mode','lane')=='road_offset':
            for original,side,target in getattr(self.pair.road_area,'crossings',()):
                reason=self.network.permission_reason(original,side)
                if reason:return reason
                if self.network.links[original].get(side+'_lane_change_dst_link_idx')!=target:
                    return 'RETURN_LINK_MISMATCH'
            return '' if self.pair.road_area.authorizes(self.pair.ids) else 'ROAD_PERMISSION_UNPROVEN'
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
        lengths=(tuple(sorted(self._entry_lengths(),key=lambda v:(abs(v-min(self.config.transition_lengths_m)),v)))
                 if self.config.continuous_lane_follow else self.config.transition_lengths_m)
        cache=getattr(self,'_return_component_cache',None)
        # Component proofs are independent of the preceding entry. Include
        # every observed solid and controller/map context; the joined-path
        # validator below ALWAYS reruns for the current entry candidate.
        cache_context=(self.pair,self._geometry_context(),tuple(pose),speed,actual_curvature,
            tuple(tuple(tuple(p) for p in o) for o in self.remembered),self._global_path,
            self._control_horizon_m,self.weights)
        for length in lengths:
            check_search_budget(self)
            path=curve=();end=start+length;terms={};cost=math.inf
            key=(cache_context,length)
            cached=cache.get(key) if cache is not None else None
            if cached is not None:
                path,curve,end,cost,terms,reason,context,clearance=cached
                self._collision_context=dict(context);self._min_clearance=min(self._min_clearance,clearance)
                self._entry_context['return_component_cache_hits']=self._entry_context.get('return_component_cache_hits',0)+1
            else:
                try:
                    target=sample(reference,end)
                    if target is None or end+reserve>index.lengths[-1]:raise ValueError('RETURN_CONTROL_SUPPORT')
                    a=self.pair.original_index.project(pose[:2]);b=self.pair.original_index.project(target[0])
                    reason=self._return_permission(a,b)
                    if reason:raise ValueError(reason)
                    recovery=transition(prepared[:2],prepared[2],actual_curvature,reference,end,length)
                    self.generated_transition_count+=1
                    curve=prefix+recovery[1:];path=curve+center_suffix(reference,end,end+reserve)[1:]
                    reason=self._path_safety(path,(),speed,geometry_check=False,curvature_points=curve)
                    if not reason:reason=self._path_safety(path,self.remembered,speed,body_cache=body_cache,curvature_points=curve)
                    if not reason:
                        duration=arc(curve)[-1]/(max(speed,self.config.avoid_speed_kmh)/3.6)+self.config.rollout_seconds
                        reason=self._rollout(path,pose,speed,actual_curvature,self.remembered,duration_s=duration)
                    if not reason or not self.config.continuous_lane_follow:
                        cost,terms=reference_cost(path,self.pair.original,self.pair.adjacent,self.weights)
                except (ValueError,TypeError,IndexError,OverflowError) as error:reason=str(error)
                if cache is not None:
                    cache[key]=(path,curve,end,cost,terms,reason,dict(self._collision_context),self._min_clearance)
            proposed=LatticeCandidate(self.pair.side,path,self.selected.entry_end_s,
                station,end,length,cost,terms,reason,self.selected.entry_path,path,curve,
                mode=getattr(self.pair,'mode','lane'))
            validator=getattr(self,'_complete_return_validator',None)
            if not reason and validator is not None:
                joined_reason=validator(proposed)
                if joined_reason:proposed=replace(proposed,rejection=joined_reason)
            candidates.append(proposed)
            if validator is not None and not proposed.rejection:break
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
            kinematic_points=self._hold_kinematic_points(path,pose)
        else:
            kinematic_points=(self.selected.kinematic_points if path is self._return_path
                              and self.selected.kinematic_points else path)
        # Only map proofs are persistent. Check all current solids against the
        # remaining path every call, even when a new frame has equal vertices.
        future=fixed_path_suffix(path,station)
        geometry_kwargs={}
        if self.config.continuous_lane_follow:
            # Static geometry was proved when this fixed maneuver was adopted.
            # Fresh solids affect the presently commanded/stopping window;
            # a distant tail is not a reason to erase a valid local path.
            stopping=(speed/3.6*self.config.reaction_time_s+
                (speed/3.6)**2/(2*self.config.deceleration_mps2)+5.)
            horizon=max(self._control_horizon_m,self._rollout_distance(speed)+1.,stopping)
            end=min(arc(path)[-1],station+horizon)
            if getattr(self.selected,'complete_return',False):
                # Re-sampling a clipped curve changes its centred tangents,
                # creating a different future body at a microscopic cap.
                # Check the same proved fixed sweeps against all latest
                # solids/map gates, limited to the live stopping window.
                future=path;geometry_kwargs['geometry_station_range']=(station,end)
            elif arc(future)[-1]>horizon:future=center_suffix(path,station,end)
        reference_experiment=(path is self._hold_path and getattr(
            self.network,'morai_unmarked_junction_experiment',False))
        reference_experiment=reference_experiment or (self.config.native_reference_rollout and
            getattr(self.selected,'mode',None)=='lane' and self.config.continuous_lane_follow)
        active_started=time.monotonic()
        reason=self._path_safety(future,self.remembered,speed,body_cache=body_cache,
            curvature_points=kinematic_points,native_reference=reference_experiment,**geometry_kwargs)
        self._work_timing_ms['active_path_safety']=(time.monotonic()-active_started)*1000.
        active_started=time.monotonic()
        if not reason:reason=self._rollout(path,pose,speed,actual_curvature,self.remembered)
        self._work_timing_ms['active_pp_rollout']=(time.monotonic()-active_started)*1000.
        return reason

    def _hold(self, pose, speed, actual_curvature, body_cache):
        """Generate/extend only a measured MGeo adjacent-centerline suffix."""
        guide=self.pair.adjacent;index=self.pair.adjacent_index;lengths=self.pair.adjacent_arc
        if self.config.continuous_lane_follow and getattr(self.pair,'mode','lane')=='lane':
            if self._continuation is None:
                self._continuation=continuation(self.network,self.pair,self._route_links,self._route_occurrence)
                self._continuation_index=PolylineIndex(self._continuation.points)
            guide=self._continuation.points;index=self._continuation_index;lengths=index.lengths
        station=index.project(pose[:2])
        if not self._hold_path or arc(self._hold_path)[-1]-PolylineIndex(self._hold_path).project(pose[:2])<self._rollout_distance(speed)+1.:
            end=min(lengths[-1]-self.body.wheelbase_m-self.body.front_overhang_m-1.,
                    station+self.config.prediction_distance_m)
            if end<=station:return self._block('ADJACENT_LANE_END')
            path=center_suffix(guide,station,end)
            if self._road_area is not None:
                reason=''
                if not getattr(self.network,'morai_unmarked_junction_experiment',False):
                    end,reason=self._hold_kinematic_end(station,end,speed)
                # station/end belong to the selected continuation guide.
                # A surveyed native lane may extend beyond the original pair;
                # never interpret its stations on that shorter polyline.
                path=center_suffix(guide,station,end) if end>station else ()
                path,map_reason=self._map_supported_prefix(path)
                minimum=(self._rollout_distance(speed)+1. if self.config.continuous_lane_follow else
                         max(self._control_horizon_m,self._rollout_distance(speed)+1.))
                if not path or arc(path)[-1]<minimum:
                    return self._block(map_reason or reason or 'ROAD_AREA_UNPROVEN')
            self._hold_path=path
            self._validated_paths.clear()
        reason=self._validate_active(self._hold_path,pose,speed,actual_curvature,body_cache)
        return self._block(reason) if reason else self._result(self._hold_path)

    def _offset_entry_replan(self,reason,reference,pose,speed,curvature,current,route,route_index,entry_search):
        if (not self.config.continuous_lane_follow or self.selected.mode!='road_offset' or
                entry_search is None or reason not in ('BODY_COLLISION','REFERENCE_CLEARANCE','ROAD_AREA_UNPROVEN') or
                self._collision_context.get('phase')!='PATH_SWEEP'):
            return None
        failure=dict(self._collision_context);index=PolylineIndex(self.selected.entry_path)
        station=index.project(pose[:2]);end=index.project(failure['first_pose'][:2])-.75
        if end<=station:return None
        safe=center_suffix(self.selected.entry_path,station,end)
        if self._validate_active(safe,pose,speed,curvature,{}):
            self._collision_context=failure;return None
        self._entry_pending_path=safe
        if not self._entry_replan_guide:self._entry_replan_guide=self.selected.entry_path
        result=entry_search(self,reference,pose,speed,curvature,current,route,route_index,end-station,{})
        self._entry_context['active_replan_reason']=reason
        return result

    def _hold_kinematic_points(self, path, pose):
        index=self._continuation_index or self.pair.adjacent_index
        points=index.points;lengths=index.lengths
        start=index.project(pose[:2]);end=index.project(path[-1])
        first=max(0,bisect_right(lengths,start)-2)
        last=min(len(points),bisect_right(lengths,end)+2)
        return points[first:last]

    def _lane_obstacle_preview(self,pose,speed):
        """Look beyond the control window before committing to a lane end.

        This requests the existing background return search; it neither
        discards a valid guide nor treats a distant object as a collision now.
        """
        index=self._continuation_index or self.pair.adjacent_index
        at=max(0.,index.project(pose[:2]));end=min(index.lengths[-1],at+self.config.prediction_distance_m)
        if end<=at:return None
        ahead=center_suffix(index.points,at,end)
        hazards=self._normal_hazards(ahead,self.remembered)
        if not hazards:return None
        front=min(index.project(p) for solid in hazards for p in solid)
        gap=front-at-self.body.wheelbase_m-self.body.front_overhang_m-self.config.obstacle_margin_m
        reserve=(max(speed,self.config.avoid_speed_kmh)/3.6*4.
                 +max(self.config.transition_lengths_m)+self._rollout_distance(speed))
        self._entry_context.update(lane_obstacle_gap_m=gap,lane_obstacle_replan_distance_m=reserve)
        return gap if gap<=reserve else None

    def _hold_kinematic_end(self, start, end, speed):
        """Stop the guide before a future untrackable corner, never smooth it."""
        first=max(0,bisect_right(self.pair.adjacent_arc,start)-2)
        last=min(len(self.pair.adjacent),bisect_right(self.pair.adjacent_arc,end)+2)
        native=self.pair.adjacent[first:last]
        for i,k in enumerate(curvatures(native)):
            reason=('OBSERVATION_CURVATURE_SCOPE' if self._observation_curvature_limit is not None
                    and abs(k)>self._observation_curvature_limit else
                    'CURVATURE' if abs(k)>self.config.curvature_limit else
                    'LATERAL_ACCELERATION' if (speed/3.6)**2*abs(k)>self.config.max_lateral_acceleration_mps2 else '')
            if reason:
                # _hold_kinematic_points includes two native neighbors beyond
                # the guide endpoint. Keep that SAME conservative stencil.
                return min(end,self.pair.adjacent_arc[first+i]-1e-6),reason
        return end,''

    def _map_supported_prefix(self, points):
        """End HOLD before the first unproven sweep; never bridge a map gap.

        Recheck the clipped endpoint's headings as well. The caller requires
        full control/rollout support and validates every live solid afterward.
        """
        reason=''
        while len(points)>2:
            self._check_phase='HOLD_MAP_SUPPORT'
            path_poses=poses(points)
            for i,(a,b) in enumerate(zip(path_poses,path_poses[1:])):
                failure=self._body_safety(a,b,(),prepared=self._fixed_body_geometry(a,b))
                if failure:
                    reason=failure
                    points=tuple(points[:i])
                    break
            else:return points,reason
        return (),reason

    def _return_clearance_ready(self,pose):
        """Use the same STEP 4 clearance for its pre-search STOP and execution.

        New observations can extend the obstacle after STEP 3 first cleared it.
        The validated HOLD path must then remain available for advancing clear;
        restarting the search interlock on every HOLD tick would deadlock it.
        """
        body=self.body.polygon(*pose)
        self._surround_clearance=min((polygon_distance(body,o) for o in self.remembered),default=math.inf)
        rear=max(self.pair.original_index.project(p) for o in self._hazards for p in o)
        cleared=min(self.pair.original_index.project(p) for p in body)>rear+self.config.return_margin_m
        if self.config.continuous_lane_follow and self.selected.mode=='road_offset':
            # Offset return is a swept out-and-back curve, not a discrete lane
            # switch. Spatial clearance may permit beginning that curve while
            # still alongside the object; its complete sweep/rollout decides.
            reserve=(self.config.obstacle_margin_m+self.config.tracking_limit_m+
                     self.config.observation_padding_m)
            return self._surround_clearance>=reserve
        return cleared and self._surround_clearance>=self.config.hold_clearance_m

    def _accept_motion_progress(self,progress):
        if not self._motion_authorized:return ''
        if self._last_progress is not None and progress<self._last_progress-self.config.tracking_limit_m:
            self._failure_context=dict(previous_motion_progress_m=self._last_progress,
                current_progress_m=progress,pose=self._last_pose)
            return 'PROGRESS_REGRESSION'
        self._last_progress=progress
        return ''

    def _detection_path(self, reference, pose):
        index=PolylineIndex(reference);ego=max(0.,min(index.lengths[-1],index.project(pose[:2])))
        front=min(index.project(p) for o in self._hazards for p in o)
        stop=front-self.body.wheelbase_m-self.body.front_overhang_m-self.config.obstacle_margin_m-1.
        self._distance_context.update(detection_stop_station_m=stop,
            detection_ego_station_m=ego,minimum_detection_path_m=1.)
        if stop<=ego+1.:return self._block('BRAKING_DISTANCE')
        return self._result(center_suffix(reference,ego,min(stop,index.lengths[-1])))

    def _approach_candidates(self, reference, pose, speed, actual_curvature, end, body_cache):
        """Try native geometry and several curvature-continuous rejoin points.

        A certified road surface replaces the center-distance heuristic only
        when it proves whole-body containment. Without it, keep the existing
        narrow corridor as a conservative fallback, never infer a lane width.
        """
        index=PolylineIndex(self._global_path);progress=index.project(pose[:2])
        row=sample(self._global_path,progress)
        corridor=PolylineIndex(reference)
        nearest=sample(reference,max(0.,min(corridor.lengths[-1],corridor.project(pose[:2]))))
        initial=math.dist(pose[:2],nearest[0][:2])
        if initial>self.config.tracking_limit_m:return (), 'TRACKING_ERROR'
        prepared,prefix=self._preparation(pose,speed,actual_curvature,row[0][2])
        start=index.project(prepared[:2]);paths=[];rejections=[]
        if end<=start+1.:return (), 'NO_SAFE_ENTRY'
        paths.append(center_suffix(self._global_path,progress,end))
        for length in sorted(set((4.,8.,12.,end-start))):
            anchor=min(end,start+length)
            if anchor<=start+1.:continue
            curve=prefix+transition(prepared[:2],prepared[2],actual_curvature,
                self._global_path,anchor,anchor-start)[1:]
            if end>anchor+1e-6:curve+=center_suffix(self._global_path,anchor,end)[1:]
            paths.append(curve)
        area=getattr(self.network,'road_corridor',None)
        current=self._entry_context.get('current_link_id')
        if area is not None and not area.authorizes(((current,current),)):area=None
        original_area=self._road_area;self._road_area=area
        try:
            for path in paths:
                reason=''
                if area is None:
                    limit=max(min(.1,self.config.tracking_limit_m),initial)
                    if any(math.dist(p[:2],sample(reference,max(0.,min(corridor.lengths[-1],
                            corridor.project(p[:2]))))[0][:2])>limit+1e-9 for p in path):
                        reason='APPROACH_ROAD_AREA_UNPROVEN'
                if not reason:reason=self._path_safety(path,self.remembered,speed,body_cache=body_cache)
                if not reason:
                    duration=max(0.,arc(path)[-1]-self._rollout_distance(speed))/(max(speed,self.config.avoid_speed_kmh)/3.6)
                    reason=self._rollout(path,pose,speed,actual_curvature,self.remembered,
                        duration_s=max(self.config.rollout_seconds,duration))
                rejections.append(reason)
                if not reason:
                    self._entry_context['approach_rejections']=rejections
                    self._entry_context['approach_evidence']=area.source if area else 'NATIVE_ROUTE_NARROW_CORRIDOR'
                    return path,''
        finally:self._road_area=original_area
        self._entry_context['approach_rejections']=rejections
        return (), 'NO_SAFE_ENTRY_APPROACH'

    def _entry_approach(self, pose, speed, actual_curvature, body_cache):
        """Use the S4 target only on an independently verified original-lane path."""
        original_area=self._road_area
        road=getattr(self.network,'road_corridor',None)
        current=self._entry_context.get('current_link_id')
        if road is not None and road.authorizes(((current,current),)):self._road_area=road
        try:reason=self._validate_active(self._entry_wait_path,pose,speed,actual_curvature,body_cache)
        finally:self._road_area=original_area
        if reason:return self._block(reason)
        self.reason='WAIT_ENTRY_GEOMETRY'
        result=self._result(self._entry_wait_path)
        return result

    def _confirm_normal_hazards(self,blocking,detected,stamp):
        fresh=stamp!=self._normal_stamp
        self._normal_stamp=stamp;tracks=[];confirmed=[];retire=[]
        for solid in blocking:
            previous=next((row for row in self._normal_tracks if intersects(row['solid'],solid)),None)
            row=dict(previous or dict(count=0,misses=0,confirmed=False),solid=solid)
            if fresh:
                if any(intersects(solid,d) for d in detected):
                    row['count']+=1;row['misses']=0
                    row['confirmed']=row['confirmed'] or row['count']>=self.config.normal_obstacle_confirmations
                elif not row['confirmed']:
                    row['count']=0;row['misses']+=1
            if not row['confirmed'] and row['misses']>=3:retire.append(solid);continue
            tracks.append(row)
            if row['confirmed']:confirmed.append(solid)
        self._normal_tracks=tracks
        self._normal_detection_frames=max((row['count'] for row in tracks),default=0)
        self._confirmed_path_obstacles=len(confirmed)
        if retire:
            self.remembered=tuple(o for o in self.remembered if o not in retire)
            self._path_obstacle_count-=len(retire)
        return tuple(confirmed)

    def plan(self, reference, groups, pose, *, speed_kmh=0., actual_curvature=0.,
             current_link_id='', route_link_ids=(), observation_valid=True, control_horizon_m=20.,
             hazard_reference=None, entry_reserve_m=0., stopping_required_m=None,
             entry_observation_m=None, observation_curvature_limit=None,
             now_sec=None, global_reference=None, return_search=None,
             route_link_index=None, observation_stamp_ns=None, entry_search=None,motion_authorized=True):
        if type(motion_authorized) is not bool:return self._block('INVALID_MOTION_AUTHORIZATION')
        self._motion_authorized=motion_authorized
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
            self._detected_count=len(detected)
        except (ValueError,TypeError,OverflowError):
            return self._block('INVALID_OBSTACLE_FOOTPRINT')
        if not observation_valid:return self._block('OBSTACLE_OBSERVATION_INVALID_OR_STALE')
        self._last_now=self._now=now;self._last_pose=pose
        self._control_horizon_m=control_horizon_m;self._entry_reserve_m=entry_reserve_m
        self._entry_observation_m=entry_observation_m;self._observation_curvature_limit=observation_curvature_limit
        context=self._geometry_context()
        if context!=self._validated_context:
            self._validated_paths.clear();self._validated_context=context
        self._work_timing_ms={};work_started=time.monotonic()
        body_cache={};old_memory=self.remembered;self._remember(detected)
        self._work_timing_ms['observed_memory']=(time.monotonic()-work_started)*1000.
        work_started=time.monotonic()
        if self.remembered!=old_memory:
            self._obstacle_bounds.clear()
            self._validated_paths.clear()
            if self._hazards and not self.config.observed_union_memory:
                self._hazards=tuple(o for o in self.remembered if any(intersects(o,h) for h in self._hazards))
        geometry=self._body_geometry(pose,pose)
        self._check_phase='MEASURED_BODY'
        boundaries=getattr(self.network,'boundaries',None)
        self._boundary_contact_reason=boundaries.swept_reason(geometry[0]) if boundaries is not None else ''
        self._boundary_check_mode='diagnostic' if self.state=='NORMAL' else 'avoidance'
        # NORMAL owns the unchanged base route. A map-marking contact is a
        # tracking diagnostic there, not an S4 emergency-stop request. Actual
        # LiDAR body collision/clearance and observation failures still stop.
        reason=(self._obstacle_safety(pose,pose,self.remembered,prepared=geometry)
                if self.state=='NORMAL' else self._body_safety(pose,pose,self.remembered,prepared=geometry))
        if reason:return self._block(reason)
        self._work_timing_ms['measured_body']=(time.monotonic()-work_started)*1000.

        if self.state in ('NORMAL','DETECT'):
            index=PolylineIndex(reference);ego=max(0.,min(index.lengths[-1],index.project(pose[:2])))
            if self.state=='NORMAL':
                work_started=time.monotonic()
                self._original=reference
                threshold=ego-self.body.rear_overhang_m-self.config.return_margin_m
                projected={}
                def keep_observation(obstacle):
                    # A single vertex ahead of the retirement threshold keeps
                    # the WHOLE observed solid. Shared union vertices need an
                    # exact projection only once for this immutable index.
                    for point in obstacle:
                        if point not in projected:projected[point]=index.project(point)
                        if projected[point]>=threshold:return True
                    return False
                self.remembered=tuple(o for o in self.remembered if keep_observation(o))
                self._work_timing_ms['retirement']=(time.monotonic()-work_started)*1000.
                work_started=time.monotonic()
                blocking=self._normal_hazards(reference,self.remembered,
                    prediction_margin_m=self.config.normal_prediction_margin_m)
                self._work_timing_ms['normal_hazards']=(time.monotonic()-work_started)*1000.
                self._path_obstacle_count=len(blocking)
                if not blocking:
                    self._normal_detection_frames=0;self._normal_candidate=()
                    self._normal_tracks=[];self._confirmed_path_obstacles=0
                    self._hazards=();self._detection_trigger_m=None;self._entry_distance_basis={}
                    self._distance_context={}
                    return self._result(normal_reference)
                self._hazards=blocking
            elif self.config.observed_union_memory:
                # Repartitioning a concave observed union changes fragment
                # identities. Adjacency to a previous triangle is not evidence
                # that a whole roadside wall blocks the path: repeated SAT
                # touch matching otherwise propagates the hazard toward Ego.
                # Reclassify against the SAME body/reference hard floors.
                # Retain captured static occupancy if no new blocker remains.
                blocking=self._normal_hazards(reference,self.remembered,
                    prediction_margin_m=self.config.normal_prediction_margin_m)
                if blocking:self._hazards=blocking
            front=min(index.project(p) for o in self._hazards for p in o)
            actual_front=max(index.project(p) for p in self.body.polygon(*pose))
            distance=front-actual_front-self.config.obstacle_margin_m;velocity=speed_kmh/3.6
            self._distance_context=dict(observation_stamp_ns=observation_stamp_ns,
                pose=tuple(pose),speed_kmh=speed_kmh,ego_station_m=ego,
                obstacle_front_station_m=front,body_front_station_m=actual_front,
                available_distance_m=distance,reaction_time_s=self.config.reaction_time_s,
                reaction_distance_m=velocity*self.config.reaction_time_s,
                deceleration_mps2=self.config.deceleration_mps2,
                braking_distance_m=velocity**2/(2*self.config.deceleration_mps2),
                required_stopping_m=velocity*self.config.reaction_time_s+velocity**2/(2*self.config.deceleration_mps2),
                stopping_required_m=stopping_required_m,hazard_polygons=self._hazards,
                candidate_search_deferred=speed_kmh>self.config.avoid_speed_kmh+.2)
            if stopping_required_m is not None and front-ego<=stopping_required_m:
                return self._block('S4_OBSTACLE_STOPPING_DISTANCE')
            if distance<=velocity*self.config.reaction_time_s+velocity**2/(2*self.config.deceleration_mps2):
                return self._block('BRAKING_DISTANCE')
            row=sample(reference,ego)
            if row is None:return self._block('REFERENCE_RANGE')
            if abs(heading_error(pose[2],math.atan2(row[1][1],row[1][0])))>math.radians(30.):
                return self._block('DIRECTION_CONTINUITY')
            if self.state=='NORMAL':
                if self.config.normal_obstacle_confirmations>1:
                    stamp=observation_stamp_ns if observation_stamp_ns is not None else now
                    self._hazards=self._confirm_normal_hazards(blocking,detected,stamp)
                    if not self._hazards:
                        return self._result(normal_reference)
                    front=min(index.project(p) for o in self._hazards for p in o)
                    distance=front-actual_front-self.config.obstacle_margin_m
                if self.config.normal_detection_latency_s>0:
                    target=self.config.avoid_speed_kmh/3.6
                    entry_distance=self._detection_entry_distance(pose,current_link_id,route_link_ids,route_link_index)
                    self._detection_trigger_m=(velocity*self.config.normal_detection_latency_s
                        +max(0.,velocity**2-target**2)/(2*self.config.deceleration_mps2)
                        +entry_distance
                        +self.config.obstacle_margin_m+self.config.normal_detection_margin_m)
                    if distance>self._detection_trigger_m:return self._result(normal_reference)
                saved=global_reference if global_reference is not None else reference
                if len(saved)<2 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in saved):
                    return self._block('SAVED_GLOBAL_PATH_UNAVAILABLE')
                self._global_path=tuple(tuple(p) for p in saved)
                self._route_links=tuple(route_link_ids);self._route_occurrence=route_link_index
                self._continuation=self._continuation_index=None
                self._entry_context=dict(current_link_id=current_link_id,
                    route_occurrences=list(route_link_ids).count(current_link_id),pose=tuple(pose))
                self._detected_obstacles=tuple(dict(
                    center_xy=tuple(sum(p[k] for p in o)/len(o) for k in (0,1)),
                    distance_m=polygon_distance(self.body.polygon(*pose),o),
                    size_xy_m=tuple(max(p[k] for p in o)-min(p[k] for p in o) for k in (0,1)))
                    for o in self._hazards)
                # Only confirmed, in-range path hazards activate S4's hard
                # boundary gate. Every candidate and rollout uses it as well.
                self._boundary_check_mode='avoidance'
                reason=self._body_safety(pose,pose,self.remembered,prepared=geometry)
                if reason:return self._block(reason)
                self.state='DETECT';self.reason='OBSTACLE_DETECTED'
                return self._detection_path(reference,pose)
            if speed_kmh>self.config.avoid_speed_kmh+.2:
                self.reason='DECELERATING'
                if (self.config.continuous_lane_follow and entry_search is not None and
                        (self.config.optimize_entry_onset or entry_search.active)):
                    return entry_search(self,reference,pose,speed_kmh,actual_curvature,current_link_id,
                        route_link_ids,route_link_index,distance,body_cache)
                return self._detection_path(reference,pose)
            if self._entry_wait_path:
                progress=PolylineIndex(self._global_path).project(pose[:2])
                if progress < self._entry_wait_retry:
                    return self._entry_approach(pose,speed_kmh,actual_curvature,body_cache)
                self._entry_wait_path=()
            arguments=(reference,pose,speed_kmh,actual_curvature,current_link_id,
                       route_link_ids,route_link_index,distance,body_cache)
            return self._search_entry(*arguments) if entry_search is None else entry_search(self,*arguments)

        progress=self.pair.original_index.project(pose[:2])
        if not math.isfinite(progress):return self._block('INVALID_PROGRESS')
        pending_return=self.state=='RETURN' and not self._return_path
        if not pending_return:
            reason=self._accept_motion_progress(progress)
            if reason:return self._block(reason)
        if getattr(self.selected,'complete_return',False):
            return self._follow_complete_maneuver(normal_reference,pose,speed_kmh,actual_curvature,progress,body_cache)
        if self.state=='AVOID_ENTRY':
            reason=self._validate_active(self.selected.entry_path,pose,speed_kmh,actual_curvature,body_cache)
            if reason:
                result=self._offset_entry_replan(reason,reference,pose,speed_kmh,actual_curvature,
                    current_link_id,route_link_ids,route_link_index,entry_search)
                if result is not None:return result
                return self._block(reason)
            if (self.config.continuous_lane_follow and self.selected.mode=='road_offset' and
                    return_search is not None and
                    self.selected.entry_end_s-progress<=self.config.acquisition_end_region_m+max(self.config.lookahead_min_m,
                        speed_kmh/3.6*(getattr(return_search,'elapsed_ms',0.)/1000.+self.config.command_delay_s)) and
                    self._return_clearance_ready(pose)):
                return_search.prefetch(self,pose,speed_kmh,actual_curvature)
            if self._acquired(pose,self.pair.adjacent,self.selected.entry_end_s,progress):
                self.state='AVOID_HOLD';self.reason='HOLD_TIMER';self.confirmations=0
                self._hold_started=now;self._hold_elapsed=0.
            return self._result(self.selected.entry_path)

        if self.state=='AVOID_HOLD':
            if (self.config.continuous_lane_follow and self.selected.mode=='road_offset' and
                    return_search is not None and self._return_clearance_ready(pose) and
                    getattr(return_search,'_future',None) is not None and return_search._future.done()):
                reason=return_search(self,pose,speed_kmh,actual_curvature,body_cache)
                if not reason:
                    self.state='RETURN';self.reason='OFFSET_RETURN_PREPARED';self.confirmations=0
                    return self._result(self._return_path)
            result=self._hold(pose,speed_kmh,actual_curvature,body_cache)
            if result['blocked']:return result
            if speed_kmh>self.config.avoid_speed_kmh+.2:self._hold_started=now
            self._hold_elapsed=now-self._hold_started
            rear=max(self.pair.original_index.project(p) for o in self._hazards for p in o)
            rear_cleared=min(self.pair.original_index.project(p) for p in self.body.polygon(*pose))>rear+self.config.return_margin_m
            self._surround_clearance=min((polygon_distance(self.body.polygon(*pose),o)
                                         for o in self.remembered),default=math.inf)
            if (self.config.continuous_lane_follow and
                    self._lane_obstacle_preview(pose,speed_kmh) is not None):
                self.state='RETURN';self.reason='NEW_LANE_OBSTACLE_RETURN_SEARCH';self.confirmations=0
                return self._result(self._hold_path)
            if self.config.continuous_lane_follow and self._continuation is not None:
                follow=self._continuation;at=self._continuation_index.project(pose[:2])
                if follow.merge_station_m is not None:
                    if at>=follow.merge_station_m and self._acquired(pose,self._global_path,0.,progress):
                        self.natural_rejoins+=1;self.state='NORMAL';self.reason='NATURAL_GLOBAL_PATH_REJOIN'
                        self.remembered=();self.selected=None;self.pair=None;self.candidates=()
                        self._hold_path=();self._return_path=();self._hazards=();self._road_area=None
                        self._last_progress=None;self._validated_paths.clear()
                        return self._result(normal_reference)
                    self.reason='AVOID_LANE_CRUISE_NATURAL_JOIN_AHEAD'
                    return self._result(self._hold_path)
                remaining=self._continuation_index.lengths[-1]-at
                preparation=self.config.lookahead_min_m+max(speed_kmh,self.config.avoid_speed_kmh)/3.6*4.
                reserve=preparation+max(self.config.transition_lengths_m)+self._rollout_distance(speed_kmh)+10.
                self._entry_context['lane_cruise_remaining_m']=remaining
                self._entry_context['return_preview_required_m']=reserve
                if remaining>reserve:
                    self.reason='AVOID_LANE_CRUISE';return self._result(self._hold_path)
            hold_required=0. if self.config.continuous_lane_follow else self.config.hold_duration_s
            offset_return_ready=(self.config.continuous_lane_follow and self.selected.mode=='road_offset'
                                 and self._return_clearance_ready(pose))
            if self._hold_elapsed<hold_required:self.reason='HOLD_TIMER'
            elif not rear_cleared and not offset_return_ready:
                self.reason='WAIT_OBSTACLE_PASS'
            elif self._surround_clearance<self.config.hold_clearance_m and not offset_return_ready:
                self.reason='WAIT_SURROUND_CLEARANCE'
            else:
                self.state='RETURN';self.reason='RETURN_READY';self.confirmations=0
            # Even when ready, no return geometry is generated by STEP 3.
            return self._result(self._hold_path)

        if self.state=='RETURN':
            if not self._return_path:
                # Recheck STEP 3 clearance against each new LiDAR frame before
                # a delayed/retried return search. Do not abandon the safe lane.
                if not self._return_clearance_ready(pose):
                    reason=self._accept_motion_progress(progress)
                    if reason:return self._block(reason)
                    self.reason='WAIT_SURROUND_CLEARANCE';return self._hold(pose,speed_kmh,actual_curvature,body_cache)
                if now<self._return_retry_at:
                    reason=self._accept_motion_progress(progress)
                    if reason:return self._block(reason)
                    return self._hold(pose,speed_kmh,actual_curvature,body_cache)
                reason=(self._generate_return(pose,speed_kmh,actual_curvature,body_cache)
                        if return_search is None else return_search(self,pose,speed_kmh,actual_curvature,body_cache))
                if reason=='RETURN_SEARCH_PENDING':
                    if self.config.continuous_lane_follow:
                        failure=self._accept_motion_progress(progress)
                        if failure:return self._block(failure)
                        self.reason=reason
                        return self._hold(pose,speed_kmh,actual_curvature,body_cache)
                    # Only STEP 4 may start this work. Remain stopped while the
                    # isolated search runs; no old HOLD path can authorize drive.
                    return dict(self._result(),state='VALIDATING',blocked=True,
                                points=(),reason=reason)
                if reason:
                    progress_reason=self._accept_motion_progress(progress)
                    if progress_reason:return self._block(progress_reason)
                    self.reason=reason;self._return_retry_at=now+1.
                    return self._hold(pose,speed_kmh,actual_curvature,body_cache)
                # STOP-only search poses can overshoot then settle as the
                # filter fuses stationary GPS. They never authorized motion;
                # compare the adopted path to the last actual motion stage.
                reason=self._accept_motion_progress(progress)
                if reason:return self._block(reason)
            reason=self._validate_active(self._return_path,pose,speed_kmh,actual_curvature,body_cache)
            if reason:return self._block(reason)
            station=PolylineIndex(self._global_path).project(pose[:2])
            if self._acquired(pose,self._global_path,self.selected.return_end_s,station):
                self.state='NORMAL';self.reason='';self.remembered=();self.selected=None;self.pair=None
                self.candidates=();self._hold_path=();self._return_path=();self._hazards=();self._road_area=None
                self._hold_started=None;self._last_progress=None;self._validated_paths.clear()
                return self._result(normal_reference)
            return self._result(self._return_path)
        return self._block('INVALID_STATE')
