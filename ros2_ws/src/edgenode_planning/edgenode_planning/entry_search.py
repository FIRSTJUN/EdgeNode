"""Isolated bounded entry search; live ROS inputs and STOP keep their cadence."""
from concurrent.futures import ProcessPoolExecutor
from copy import copy
from multiprocessing import get_context
import math
import time

from .lattice_path import heading_error


class EntrySearch:
    def __init__(self, executor=None):
        self._executor=executor if executor is not None else ProcessPoolExecutor(
            max_workers=1,mp_context=get_context('spawn'))
        self._future=self._context=self._pose=None
        self.elapsed_ms=0.

    def cancel(self):
        if self._future is not None:self._future.cancel()
        self._future=self._context=self._pose=None

    def close(self):
        self.cancel();self._executor.shutdown(wait=False,cancel_futures=True)

    @staticmethod
    def _search(snapshot, arguments):
        started=time.perf_counter();generated=snapshot.generated_transition_count
        result=snapshot._search_entry(*arguments)
        return result,snapshot,snapshot.generated_transition_count-generated,(time.perf_counter()-started)*1000.

    def __call__(self, planner, reference, pose, speed, curvature, current, route, route_index, distance, body_cache):
        context=(id(planner),id(planner._global_path),planner.config,current,tuple(route),route_index)
        if context!=self._context:self.cancel()
        def pending():
            return dict(planner._result(),state='VALIDATING',stage='DETECT',blocked=True,
                        points=(),reason='ENTRY_SEARCH_PENDING')
        if self._future is None:
            snapshot=copy(planner);snapshot._obstacle_bounds=dict(planner._obstacle_bounds)
            snapshot._validated_paths={}
            snapshot._map_sweep_cache=type(planner._map_sweep_cache)()
            arguments=(reference,pose,speed,curvature,current,route,route_index,distance,{})
            self._context=context;self._pose=pose
            self._future=self._executor.submit(self._search,snapshot,arguments)
            return pending()
        if not self._future.done():return pending()
        future=self._future;search_pose=self._pose
        self._future=self._context=self._pose=None
        result,snapshot,generated,self.elapsed_ms=future.result()
        planner.generated_transition_count+=generated
        # The search began while STOP was being applied. Coasting can change
        # its initial constraints. Search again from the fresh stopped pose;
        # never latch a geometry failure from an obsolete starting position.
        if (math.dist(pose[:2],search_pose[:2])>planner.config.tracking_limit_m/2 or
                abs(heading_error(pose[2],search_pose[2]))>planner.config.acquisition_heading_rad/2):
            return pending()
        for name in ('candidates','_entry_context','left_lane','right_lane','candidate_lane_batches'):
            setattr(planner,name,getattr(snapshot,name))
        if result['blocked']:return planner._block(result['reason'])
        for name in ('selected','pair','_road_area','_original','_entry_wait_path',
                     '_entry_wait_origin','_entry_wait_retry'):
            setattr(planner,name,getattr(snapshot,name))
        planner._validated_paths.clear()
        if snapshot.state=='AVOID_ENTRY':
            reason=planner._validate_active(planner.selected.entry_path,pose,speed,curvature,{})
            if reason:return planner._block(reason)
            planner.state='AVOID_ENTRY';planner.reason='';planner.entry_commits+=1
            planner._last_progress=None
            return planner._result(planner.selected.entry_path)
        return planner._entry_approach(pose,speed,curvature,{})
