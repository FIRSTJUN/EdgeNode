"""Isolated STEP 4 search; ROS callbacks keep servicing live safety inputs."""
from concurrent.futures import ProcessPoolExecutor
from copy import copy
from multiprocessing import get_context
import os
import math
import time
from dataclasses import replace

from .lattice_path import heading_error
from .lane_geometry import sample,curvature_at
from .lattice_path import center_suffix
from .search_budget import SearchBudgetExceeded


class ReturnSearch:
    def __init__(self, executor=None, *, budget_s=0.):
        if not math.isfinite(budget_s) or budget_s<0:raise ValueError('invalid search budget')
        self.budget_s=budget_s
        # A CPU-bound Python geometry thread contends with ROS callbacks for
        # the GIL. Spawn a worker without inheriting the live DDS threads.
        self._executor = executor if executor is not None else ProcessPoolExecutor(
            max_workers=1, mp_context=get_context('spawn'))
        self._warmup = self._executor.submit(os.getpid) if executor is None else None
        self._future = None
        self._context = None
        self._pose = None
        self._prefix = ()
        self.elapsed_ms = 0.
        self.worker_pid = None

    def cancel(self):
        if self._future is not None:
            self._future.cancel()
        self._future = self._context = self._pose = None
        self._prefix=()

    def close(self):
        self.cancel()
        self._executor.shutdown(wait=False, cancel_futures=True)

    def prefetch(self,planner,pose,speed,curvature):
        """Start geometry work; adoption remains an explicit later call."""
        if self._future is None:
            if planner.selected.mode=='road_offset':
                guide=planner.pair.adjacent;index=planner.pair.adjacent_index
                station=max(0.,index.project(pose[:2]));anchor=station+planner.config.lookahead_min_m
                target=sample(guide,anchor)
                if target is None:return
                point,tangent=target
                self(planner,(*point[:2],math.atan2(tangent[1],tangent[0])),speed,
                     curvature_at(guide,anchor),{})
                self._prefix=center_suffix(guide,station,anchor)
            else:self(planner,pose,speed,curvature,{})

    @staticmethod
    def _search(snapshot, pose, speed, curvature):
        started = time.perf_counter()
        generated = snapshot.generated_transition_count
        budget=getattr(snapshot,'_worker_search_budget_sec',0.)
        snapshot._search_deadline_sec=started+budget if budget else None
        try:
            reason = snapshot._generate_return(pose, speed, curvature, {})
        except SearchBudgetExceeded:
            reason='RETURN_SEARCH_BUDGET_EXHAUSTED'
        finally:
            snapshot._search_deadline_sec=None
        return reason, snapshot, snapshot.generated_transition_count-generated, (time.perf_counter()-started)*1000., os.getpid()

    def __call__(self, planner, pose, speed, curvature, body_cache):
        # Geometry/configuration belongs to the current maneuver. New LiDAR
        # solids are checked on the ROS thread before any adopted path moves.
        context = (id(planner), id(planner.selected), id(planner.pair),
                   id(planner._global_path), planner.config)
        if self._context != context:
            self.cancel()
        if self._future is None:
            moving=planner.config.continuous_lane_follow
            if not math.isfinite(speed) or (not moving and abs(speed)>.2):
                return 'RETURN_SEARCH_PENDING'
            search_pose=pose;search_curvature=curvature;prefix=()
            if moving and planner.selected.mode!='road_offset':
                guide=planner.pair.adjacent;index=planner.pair.adjacent_index
                station=max(0.,index.project(pose[:2]))
                reserve=max(8.,max(speed,planner.config.avoid_speed_kmh)/3.6*
                            max(4.,self.elapsed_ms/1000.+1.))
                anchor=station+reserve
                target=sample(guide,anchor)
                if target is None:return 'RETURN_SEARCH_NO_FORWARD_ANCHOR'
                point,tangent=target
                search_pose=(*point[:2],math.atan2(tangent[1],tangent[0]))
                search_curvature=curvature_at(guide,anchor)
                prefix=center_suffix(guide,station,anchor)
            snapshot = copy(planner)
            snapshot._worker_search_budget_sec=self.budget_s
            snapshot._obstacle_bounds = dict(planner._obstacle_bounds)
            snapshot._validated_paths = {}
            snapshot._map_sweep_cache = type(planner._map_sweep_cache)()
            snapshot._fixed_sweep_cache = type(planner._fixed_sweep_cache)()
            self._context = context
            self._pose = tuple(search_pose);self._prefix=prefix
            self._future = self._executor.submit(self._search, snapshot, search_pose, speed, search_curvature)
            return 'RETURN_SEARCH_PENDING'
        if not self._future.done():
            return 'RETURN_SEARCH_PENDING'
        future = self._future
        search_pose = self._pose
        prefix=self._prefix
        self._future = self._context = self._pose = None
        self._prefix=()
        reason, snapshot, generated, self.elapsed_ms, self.worker_pid = future.result()
        planner.candidates = snapshot.candidates
        planner.generated_transition_count += generated
        # Localization can settle backward after measured Ego speed reaches
        # zero. As for EntrySearch, discard an obsolete origin BEFORE adopting
        # either its path or its failure. All latest-input validation remains.
        if not planner.config.continuous_lane_follow and (math.dist(pose[:2],search_pose[:2])>planner.config.tracking_limit_m/2 or
                abs(heading_error(pose[2],search_pose[2]))>planner.config.acquisition_heading_rad/2):
            return 'RETURN_SEARCH_PENDING'
        if not reason:
            if planner.config.continuous_lane_follow:
                # Offset return already includes the controller's straight
                # preparation/command-delay prefix. An additional future
                # anchor can skip its finite return opening. Revalidate even
                # when that extra prefix is empty and the origin is measured.
                path=prefix+snapshot._return_path[1:] if prefix else snapshot._return_path
                curve=prefix+snapshot.selected.kinematic_points[1:] if prefix else snapshot.selected.kinematic_points
                candidate=replace(snapshot.selected,points=path,return_path=path,kinematic_points=curve)
                previous=planner.selected
                previous_path=planner._return_path
                planner.selected=candidate;planner._return_path=path
                try:
                    failure=planner._validate_active(path,pose,speed,curvature,{})
                finally:planner.selected=previous;planner._return_path=previous_path
                if failure:return 'RETURN_SEARCH_LIVE_RETRY:'+failure
                snapshot.selected=candidate;snapshot._return_path=path
            planner.selected = snapshot.selected
            planner._return_path = snapshot._return_path
            planner.return_commits += 1
            # Never inherit worker validation against its older observations.
            # plan() now runs full path/body/PP checks with current live inputs,
            # followed by the integration's distinct-fresh-pose interlock.
            planner._validated_paths.clear()
            planner.reason = ''
        return reason
