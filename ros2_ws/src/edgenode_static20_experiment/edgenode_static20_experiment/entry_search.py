"""Isolated bounded entry search; live ROS inputs and STOP keep their cadence."""
from concurrent.futures import ProcessPoolExecutor
from copy import copy
from multiprocessing import get_context
import math
import os
import time
from dataclasses import replace

from edgenode_planning.lattice_path import heading_error
from edgenode_planning.lane_geometry import PolylineIndex,sample,curvature_at,arc
from edgenode_planning.lattice_path import center_suffix,transition
from edgenode_planning.search_budget import SearchBudgetExceeded


def matched_approach_prefix(guide,pose,curvature,station,advance):
    """Fixed C2 bridge from the measured pose to the future search origin.

    A clipped native polyline can end on a microscopic partial edge, whose
    tangent disagrees with the worker's preparation. Match position, tangent
    and turn instead; every ordinary curvature/body/PP gate still applies.
    """
    return transition(pose[:2],pose[2],curvature,guide,station+advance,advance)


class EntrySearch:
    def __init__(self, executor=None, *, budget_s=0.):
        if not math.isfinite(budget_s) or budget_s<0:raise ValueError('invalid search budget')
        self.budget_s=budget_s
        self._executor=executor if executor is not None else ProcessPoolExecutor(
            max_workers=1,mp_context=get_context('spawn'))
        self._warmup=self._executor.submit(os.getpid) if executor is None else None
        self._future=self._context=self._pose=None
        self.elapsed_ms=0.
        self._prefix=()
        self._pending_prefix=()
        self._anchor={}
        self._episode_context=None
        self._episode_started=None
        self._future_has_approach=False

    @property
    def active(self):
        return self._future is not None

    def cancel(self,*,reset_episode=True):
        if self._future is not None:self._future.cancel()
        self._future=self._context=self._pose=None
        self._prefix=()
        self._pending_prefix=()
        self._anchor={}
        self._future_has_approach=False
        if reset_episode:self._episode_context=self._episode_started=None

    def close(self):
        self.cancel();self._executor.shutdown(wait=False,cancel_futures=True)

    @staticmethod
    def _search(snapshot, arguments):
        started=time.perf_counter();generated=snapshot.generated_transition_count
        capture=os.environ.get('EDGENODE_ENTRY_SNAPSHOT_DIR');capture_prefix=None
        if capture:
            import json,pickle
            from pathlib import Path
            capture_prefix=Path(capture)/('entry-'+str(os.getpid())+'-'+str(time.monotonic_ns()))
            capture_prefix.with_suffix('.input.pickle').write_bytes(pickle.dumps((snapshot,arguments)))
            capture_prefix.with_suffix('.metadata.json').write_text(json.dumps(getattr(snapshot,'_entry_capture',{})))
        budget=getattr(snapshot,'_worker_search_budget_sec',0.)
        snapshot._search_deadline_sec=started+budget if budget else None
        try:
            result=snapshot._search_entry(*arguments)
        except SearchBudgetExceeded:
            snapshot._entry_context['search_budget']=dict(limit_s=budget,
                classification='COMPUTATION_LIMIT_NOT_PHYSICAL_IMPOSSIBILITY')
            result=snapshot._block('ENTRY_SEARCH_BUDGET_EXHAUSTED')
        finally:
            snapshot._search_deadline_sec=None
            if capture_prefix is not None:
                capture_prefix.with_suffix('.result.pickle').write_bytes(pickle.dumps(snapshot))
        return result,snapshot,snapshot.generated_transition_count-generated,(time.perf_counter()-started)*1000.

    def __call__(self, planner, reference, pose, speed, curvature, current, route, route_index, distance, body_cache):
        context=(id(planner),id(planner._global_path),planner.config,tuple(route))
        context+=(id(planner._entry_replan_guide),tuple(getattr(planner,'_route_waypoints',())))
        if not planner.config.continuous_lane_follow:context+=(current,route_index)
        if context!=self._context:self.cancel(reset_episode=False)
        if context!=self._episode_context:
            self._episode_context=context;self._episode_started=None
        if self.budget_s and self._episode_started is not None and time.monotonic()-self._episode_started>=self.budget_s:
            planner._entry_context['search_budget']=dict(limit_s=self.budget_s,
                classification='COMPUTATION_LIMIT_NOT_PHYSICAL_IMPOSSIBILITY',
                scope='TOTAL_ENTRY_EPISODE_INCLUDING_REJECTED_ATTEMPTS')
            self.cancel(reset_episode=False)
            return planner._block('ENTRY_SEARCH_BUDGET_EXHAUSTED')
        def pending():
            if planner.config.continuous_lane_follow:
                if planner._entry_pending_path:
                    planner.reason='ENTRY_REPLAN_PENDING'
                    result=planner._result(planner._entry_pending_path,speed_cap=min(5.,planner.config.avoid_speed_kmh))
                else:
                    planner.reason=('DECELERATING_ENTRY_SEARCH' if planner.config.optimize_entry_onset and
                        speed>planner.config.avoid_speed_kmh+.2 else 'ENTRY_SEARCH_PENDING')
                    result=planner._detection_path(reference,pose)
                if self._future is not None and not result['blocked']:
                    if self._anchor.get('approach_speed_cap_kmh') is not None:
                        result['speed_cap_kmh']=min(result['speed_cap_kmh'],self._anchor['approach_speed_cap_kmh'])
                    # Do not drive past the worker's captured future origin.
                    # A slow search may consume its entire approach prefix;
                    # keep creeping while there is support, then stop rather
                    # than waiting at full speed until the obstacle brake gate.
                    support=self._pending_prefix or self._prefix
                    if support:
                        index=PolylineIndex(support)
                        remaining=max(0.,index.lengths[-1]-index.project(pose[:2])-
                            planner.config.tracking_limit_m)
                    else:
                        # No reserved approach means there is no proved
                        # distance available while the worker is computing.
                        # Keep the brake gate; never drift into stale geometry.
                        remaining=0.
                    reaction=planner.config.reaction_time_s+planner.config.command_delay_s
                    deceleration=planner.config.deceleration_mps2
                    from .search_pacing import approach_cap
                    episode_left=(max(0.,self.budget_s-(time.monotonic()-self._episode_started))
                        if self.budget_s and self._episode_started is not None else None)
                    cap,horizon=approach_cap(remaining,reaction,deceleration,self.elapsed_ms,episode_left)
                    result['speed_cap_kmh']=min(result['speed_cap_kmh'],cap)
                    planner._entry_context['search_approach_remaining_m']=remaining
                    planner._entry_context['search_approach_cap_kmh']=result['speed_cap_kmh']
                    planner._entry_context['search_approach_pacing_horizon_s']=horizon
                    planner._entry_context['search_episode_remaining_s']=episode_left
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
            self._anchor={}
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
                    requested_budget=budget;lower_basis=None
                    # If the request leaves too little approach even to slow
                    # before the captured origin, also consider the already
                    # supported lower-speed family. This estimates an origin,
                    # never authorizes a path: every candidate still needs its
                    # complete map/body/PP proof and current-input adoption.
                    stop_reserve=(velocity*(planner.config.reaction_time_s+planner.config.command_delay_s)
                        +velocity**2/(2*planner.config.deceleration_mps2)
                        +planner.config.tracking_limit_m+planner.config.observation_padding_m)
                    floor=min(10.,planner.requested_avoidance_kmh)
                    if budget<max(4.,stop_reserve) and floor<planner.config.avoid_speed_kmh:
                        low=copy(planner);low.config=replace(planner.config,avoid_speed_kmh=floor)
                        low._detection_entry_distance(pose,current,route,route_index)
                        lower_basis=low._entry_distance_basis
                        lower_transition=(lower_basis['transition_m'] if lower_basis.get('source')=='map_kinematic_estimate'
                            else min(planner.config.transition_lengths_m))
                        lower_preparation=lower_basis['preparation_m']
                        lower_budget=max(0.,distance-lower_transition-lower_preparation-
                            planner.config.tracking_limit_m-planner.config.observation_padding_m)
                        if lower_budget>budget:
                            budget=lower_budget
                            self._anchor['approach_speed_cap_kmh']=floor
                    advance=min(advance,budget)
                    self._anchor.update(requested_advance_m=requested_advance,advance_m=advance,
                        available_obstacle_gap_m=distance,transition_reserve_m=transition,
                        preparation_reserve_m=preparation,previous_search_ms=self.elapsed_ms,
                        requested_speed_origin_budget_m=requested_budget,
                        conditional_lower_speed_basis=lower_basis)
                target=sample(guide,station+advance)
                if target is None:return pending()
                point,tangent=target;search_pose=(*point[:2],math.atan2(tangent[1],tangent[0]))
                search_curvature=curvature_at(guide,station+advance)
                if (getattr(planner,'complete_offset_search',False) and
                        not planner.config.optimize_entry_onset and not planner._entry_replan_guide and
                        abs(heading_error(pose[2],search_pose[2]))>planner.config.acquisition_heading_rad/2):
                    # Stay on the existing validated guide while finishing
                    # its bend, using the same half acquisition reserve as
                    # adoption, rather than proving thousands of lateral
                    # entries from an already turning actuator state. Waiting
                    # consumes the same bounded entry episode as computation.
                    if self._episode_started is None:self._episode_started=time.monotonic()
                    planner._entry_context['entry_alignment']=dict(
                        heading_error_rad=abs(heading_error(pose[2],search_pose[2])),
                        limit_rad=planner.config.acquisition_heading_rad/2,
                        action='FOLLOW_VALIDATED_ORIGINAL_GUIDE_BEFORE_SEARCH')
                    result=pending();planner.reason='ENTRY_ALIGNMENT_PENDING'
                    result['reason']=planner.reason
                    return result
                if advance>1e-6:
                    prefix=matched_approach_prefix(guide,pose,curvature,station,advance)
                    self._anchor['prefix_kind']='QUINTIC_POSE_TANGENT_CURVATURE_MATCH'
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
            if self._episode_started is None:self._episode_started=time.monotonic()
            snapshot._worker_search_budget_sec=(max(1e-9,self.budget_s-(time.monotonic()-self._episode_started))
                if self.budget_s else 0.)
            snapshot._entry_context=dict(planner._entry_context)
            snapshot._validated_paths={}
            snapshot._map_sweep_cache=type(planner._map_sweep_cache)()
            snapshot._fixed_sweep_cache=type(planner._fixed_sweep_cache)()
            self._future_has_approach=bool(prefix and getattr(planner,'complete_offset_search',False))
            if self._future_has_approach:
                # Predict the whole path from the measured state, reserving
                # an original-lane approach while computation runs. Starting
                # the PP proof at a guessed future pose misses controller lead
                # from the approach into the lateral bend.
                snapshot._entry_onset_floor_xy=search_pose[:2]
                search_pose=pose;search_curvature=curvature
                search_current=current;search_index=route_index
                self._anchor['path_origin_kind']='MEASURED_POSE_WITH_RESERVED_APPROACH'
            search_speed=min(speed,planner.config.avoid_speed_kmh) if planner.config.optimize_entry_onset else speed
            arguments=(planner._global_path if prefix else reference,search_pose,search_speed,search_curvature,
                       search_current,route,search_index,distance-(advance if prefix and not self._future_has_approach else 0.),{})
            self._context=context;self._pose=search_pose;self._prefix=prefix
            if os.environ.get('EDGENODE_ENTRY_SNAPSHOT_DIR'):
                snapshot._entry_capture=dict(submitted_ns=time.monotonic_ns(),pose=pose,
                    curvature=curvature,speed_kmh=speed,prefix=prefix,current_link=current,
                    path_has_approach=self._future_has_approach)
            self._future=self._executor.submit(self._search,snapshot,arguments)
            return pending()
        if not self._future.done():return pending()
        if planner.config.optimize_entry_onset and speed>planner.config.avoid_speed_kmh+.2:
            return pending()  # Compute during braking; never execute a lateral maneuver at that speed.
        ready_result,ready_snapshot,_,_=self._future.result()
        selected_speed=ready_snapshot.config.avoid_speed_kmh
        if not ready_result['blocked'] and speed>selected_speed+.2:
            result=pending();result['speed_cap_kmh']=min(result.get('speed_cap_kmh',selected_speed),selected_speed)
            planner._entry_context['speed_proof_adoption_wait_kmh']=selected_speed
            return result
        future=self._future;search_pose=self._pose
        prefix=() if self._future_has_approach else self._prefix
        anchor=self._anchor
        self._future=self._context=self._pose=None
        self._prefix=()
        result,snapshot,generated,self.elapsed_ms=future.result()
        speed_config_before=planner.config
        if not result['blocked']:
            if replace(snapshot.config,avoid_speed_kmh=planner.config.avoid_speed_kmh)!=planner.config:
                return planner._block('INVALID_WORKER_SPEED_CONFIGURATION')
            planner.config=replace(planner.config,avoid_speed_kmh=snapshot.config.avoid_speed_kmh)
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
                if result['reason'] in ('OFFSET_RETURN_UNPROVEN','ENTRY_SEARCH_BUDGET_EXHAUSTED'):
                    return planner._block(result['reason'])
                if planner._entry_context.get('road_origin_pruning') and all(
                        c.rejection=='ROAD_START_BODY_UNPROVEN' for c in planner.candidates):
                    return planner._block('ROAD_MAP_CONTAINMENT_UNPROVEN')
                planner._entry_context['last_search_rejection']=result['reason']
                return pending()
            return planner._block(result['reason'])
        adopted=('selected','pair','_road_area','_original','_entry_wait_path',
                 '_entry_wait_origin','_entry_wait_retry','_strategy','_maneuver_selection')
        previous={name:getattr(planner,name) for name in adopted}
        previous['config']=speed_config_before
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
            reason=(planner._path_safety(prefix,planner.remembered,speed)
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
