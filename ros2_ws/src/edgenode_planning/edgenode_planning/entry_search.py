"""Isolated bounded entry search; live ROS inputs and STOP keep their cadence."""
from concurrent.futures import ProcessPoolExecutor
from copy import copy
from multiprocessing import get_context
import math
import os
import time
from dataclasses import replace

from .lattice_path import heading_error
from .lane_geometry import PolylineIndex,sample,curvature_at,arc
from .lattice_path import center_suffix


class EntrySearch:
    def __init__(self, executor=None):
        self._executor=executor if executor is not None else ProcessPoolExecutor(
            max_workers=1,mp_context=get_context('spawn'))
        self._warmup=self._executor.submit(os.getpid) if executor is None else None
        self._future=self._context=self._pose=None
        self.elapsed_ms=0.
        self._prefix=()
        self._pending_prefix=()
        self._anchor={}

    def cancel(self):
        if self._future is not None:self._future.cancel()
        self._future=self._context=self._pose=None
        self._prefix=()
        self._pending_prefix=()
        self._anchor={}

    def close(self):
        self.cancel();self._executor.shutdown(wait=False,cancel_futures=True)

    @staticmethod
    def _search(snapshot, arguments):
        started=time.perf_counter();generated=snapshot.generated_transition_count
        result=snapshot._search_entry(*arguments)
        return result,snapshot,snapshot.generated_transition_count-generated,(time.perf_counter()-started)*1000.

    def __call__(self, planner, reference, pose, speed, curvature, current, route, route_index, distance, body_cache):
        context=(id(planner),id(planner._global_path),planner.config,tuple(route))
        context+=(id(planner._entry_replan_guide),)
        if not planner.config.continuous_lane_follow:context+=(current,route_index)
        if context!=self._context:self.cancel()
        def pending():
            if planner.config.continuous_lane_follow:
                if planner._entry_pending_path:
                    planner.reason='ENTRY_REPLAN_PENDING'
                    result=planner._result(planner._entry_pending_path,speed_cap=min(5.,planner.config.avoid_speed_kmh))
                else:
                    planner.reason=('DECELERATING_ENTRY_SEARCH' if planner.config.optimize_entry_onset and
                        speed>planner.config.avoid_speed_kmh+.2 else 'ENTRY_SEARCH_PENDING')
                    result=planner._detection_path(reference,pose)
                if self._prefix and not result['blocked']:
                    # Do not drive past the worker's captured future origin.
                    # A slow search may consume its entire approach prefix;
                    # keep creeping while there is support, then stop rather
                    # than waiting at full speed until the obstacle brake gate.
                    index=PolylineIndex(self._pending_prefix or self._prefix)
                    remaining=max(0.,index.lengths[-1]-index.project(pose[:2])-
                        planner.config.tracking_limit_m)
                    reaction=planner.config.reaction_time_s+planner.config.command_delay_s
                    deceleration=planner.config.deceleration_mps2
                    stopping=math.sqrt((deceleration*reaction)**2+2*deceleration*remaining)-deceleration*reaction
                    approach=remaining/max(2.,self.elapsed_ms/1000.+1.)
                    cap=3.6*min(stopping,approach)
                    result['speed_cap_kmh']=min(result['speed_cap_kmh'],cap)
                    planner._entry_context['search_approach_remaining_m']=remaining
                    planner._entry_context['search_approach_cap_kmh']=result['speed_cap_kmh']
                return result
            return dict(planner._result(),state='VALIDATING',stage='DETECT',blocked=True,
                        points=(),reason='ENTRY_SEARCH_PENDING')
        if self._future is None:
            # Repeated parked resets must not queue obsolete geometry jobs
            # ahead of the first authorized moving search.
            if planner.config.continuous_lane_follow and not planner._motion_authorized:
                return pending()
            # STOP must settle before capturing an expensive search origin.
            # Speed is km/h. Keep servicing live inputs while coasting ends.
            if not math.isfinite(speed) or (not planner.config.continuous_lane_follow and abs(speed)>.2):return pending()
            search_pose=pose;search_curvature=curvature;search_current=current;search_index=route_index;prefix=()
            if planner.config.continuous_lane_follow:
                guide=planner._entry_replan_guide or planner._global_path
                index=PolylineIndex(guide);station=max(0.,index.project(pose[:2]))
                advance=max(4.,max(speed,planner.config.avoid_speed_kmh)/3.6*max(2.,self.elapsed_ms/1000.+1.))
                requested_advance=advance
                if planner.config.optimize_entry_onset:
                    target_speed=planner.config.avoid_speed_kmh/3.6;velocity=max(0.,speed)/3.6
                    braking=max(0.,velocity**2-target_speed**2)/(2*planner.config.deceleration_mps2)
                    # The fixed candidate itself contains its native approach.
                    # Capturing an additional two seconds ahead can consume
                    # the entire legal onset window before computation starts.
                    advance=max(planner.config.tracking_limit_m,
                                braking+velocity*planner.config.command_delay_s)
                if planner._entry_replan_guide:
                    available=arc(planner._entry_pending_path)[-1]
                    advance=min(advance,max(1.,available-planner._rollout_distance(speed)))
                elif not planner.config.optimize_entry_onset:
                    # A slow previous solver must not move this search origin
                    # through the CURRENT obstruction. Reserve the native
                    # transition and PP preparation before its nearest face.
                    basis=planner._entry_distance_basis
                    transition=(basis['transition_m'] if basis.get('source')=='map_kinematic_estimate'
                                else min(planner.config.transition_lengths_m))
                    velocity=max(speed,planner.config.avoid_speed_kmh)/3.6
                    from edgenode_control.dynamic_lookahead import calculate_lookahead_distance
                    preparation=calculate_lookahead_distance(velocity*3.6,planner.config.lookahead_min_m,
                        planner.config.lookahead_max_m,planner.config.lookahead_preview_time_sec)+velocity*planner.config.command_delay_s
                    budget=max(0.,distance-transition-preparation-
                        planner.config.tracking_limit_m-planner.config.observation_padding_m)
                    advance=min(advance,budget)
                    self._anchor=dict(requested_advance_m=requested_advance,advance_m=advance,
                        available_obstacle_gap_m=distance,transition_reserve_m=transition,
                        preparation_reserve_m=preparation,previous_search_ms=self.elapsed_ms)
                target=sample(guide,station+advance)
                if target is None:return pending()
                point,tangent=target;search_pose=(*point[:2],math.atan2(tangent[1],tangent[0]))
                search_curvature=curvature_at(guide,station+advance)
                if advance>1e-6:prefix=center_suffix(guide,station,station+advance)
                else:search_pose=pose;search_curvature=curvature
                if planner.config.optimize_entry_onset and not planner._entry_replan_guide:
                    entry_distance=planner._detection_entry_distance(pose,current,route,route_index)
                    basis=planner._entry_distance_basis
                    transition=(basis['transition_m'] if basis['source']=='map_kinematic_estimate'
                                else min(planner.config.transition_lengths_m))
                    needed=transition+basis['preparation_m']+planner.config.tracking_limit_m+planner.config.observation_padding_m
                    budget=max(advance,distance-needed)
                    end=min(index.lengths[-1],station+budget)
                    self._pending_prefix=center_suffix(guide,station,end)
                global_station=PolylineIndex(planner._global_path).project(point[:2])
                cumulative=0.
                for i,key in enumerate(route):
                    cumulative+=arc(planner.network.links[key]['points'])[-1]
                    if global_station<=cumulative:
                        search_current=key;search_index=i;break
            snapshot=copy(planner);snapshot._obstacle_bounds=dict(planner._obstacle_bounds)
            snapshot._entry_context=dict(planner._entry_context)
            snapshot._validated_paths={}
            snapshot._map_sweep_cache=type(planner._map_sweep_cache)()
            snapshot._fixed_sweep_cache=type(planner._fixed_sweep_cache)()
            search_speed=min(speed,planner.config.avoid_speed_kmh) if planner.config.optimize_entry_onset else speed
            arguments=(planner._global_path if prefix else reference,search_pose,search_speed,search_curvature,
                       search_current,route,search_index,distance-(advance if prefix else 0.),{})
            self._context=context;self._pose=search_pose;self._prefix=prefix
            self._future=self._executor.submit(self._search,snapshot,arguments)
            return pending()
        if not self._future.done():return pending()
        if planner.config.optimize_entry_onset and speed>planner.config.avoid_speed_kmh+.2:
            return pending()  # Compute during braking; never execute a lateral maneuver at that speed.
        future=self._future;search_pose=self._pose;prefix=self._prefix
        anchor=self._anchor
        self._future=self._context=self._pose=None
        self._prefix=()
        result,snapshot,generated,self.elapsed_ms=future.result()
        planner.generated_transition_count+=generated
        # The search began while STOP was being applied. Coasting can change
        # its initial constraints. Search again from the fresh stopped pose;
        # never latch a geometry failure from an obsolete starting position.
        if not planner.config.continuous_lane_follow and (math.dist(pose[:2],search_pose[:2])>planner.config.tracking_limit_m/2 or
                abs(heading_error(pose[2],search_pose[2]))>planner.config.acquisition_heading_rad/2):
            return pending()
        for name in ('candidates','_entry_context','left_lane','right_lane','candidate_lane_batches'):
            setattr(planner,name,getattr(snapshot,name))
        planner._entry_context['entry_search_elapsed_ms']=self.elapsed_ms
        if anchor:planner._entry_context['entry_search_anchor']=anchor
        if result['blocked']:
            if planner.config.continuous_lane_follow:
                if result['reason']=='OFFSET_RETURN_UNPROVEN':
                    return planner._block(result['reason'])
                planner._entry_context['last_search_rejection']=result['reason']
                return pending()
            return planner._block(result['reason'])
        adopted=('selected','pair','_road_area','_original','_entry_wait_path',
                 '_entry_wait_origin','_entry_wait_retry','_strategy','_maneuver_selection')
        previous={name:getattr(planner,name) for name in adopted}
        for name in adopted:
            setattr(planner,name,getattr(snapshot,name))
        planner._validated_paths.clear()
        if snapshot.state=='AVOID_ENTRY':
            if prefix:
                path=prefix+planner.selected.entry_path[1:]
                curve=prefix+planner.selected.kinematic_points[1:]
                planner.selected=replace(planner.selected,points=path,entry_path=path,kinematic_points=curve)
            # The original-lane bridge is part of the maneuver, including any
            # portion beyond the live control window. Never attach a bridge
            # through an obstacle to a safe trajectory that starts beyond it.
            reason=(planner._path_safety(prefix,planner.remembered,speed,
                native_reference=getattr(planner.network,'morai_unmarked_junction_experiment',False))
                if len(prefix)>=3 else '')
            if not reason:reason=planner._validate_active(planner.selected.entry_path,pose,speed,curvature,{})
            if reason:
                if planner.config.continuous_lane_follow:
                    for name,value in previous.items():setattr(planner,name,value)
                    planner._validated_paths.clear()
                    planner._entry_context['last_live_rejection']=reason
                    return pending()
                return planner._block(reason)
            planner.state='AVOID_ENTRY';planner.reason='';planner.entry_commits+=1
            onset=planner._entry_context.get('onset_plan')
            if onset:
                onset['commit_to_onset_m']=onset['onset_station_m']-planner.pair.original_index.project(pose[:2])
            planner._entry_pending_path=planner._entry_replan_guide=()
            planner._last_progress=None
            return planner._result(planner.selected.entry_path)
        return planner._entry_approach(pose,speed,curvature,{})
