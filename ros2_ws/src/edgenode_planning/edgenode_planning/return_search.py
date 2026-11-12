"""Isolated STEP 4 search; ROS callbacks keep servicing live safety inputs."""
from concurrent.futures import ThreadPoolExecutor
from copy import copy
import time


class ReturnSearch:
    def __init__(self):
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='s4_return')
        self._future = None
        self._context = None
        self.elapsed_ms = 0.

    def cancel(self):
        if self._future is not None:
            self._future.cancel()
        self._future = self._context = None

    def close(self):
        self.cancel()
        self._executor.shutdown(wait=False, cancel_futures=True)

    @staticmethod
    def _search(snapshot, pose, speed, curvature):
        started = time.perf_counter()
        generated = snapshot.generated_transition_count
        reason = snapshot._generate_return(pose, speed, curvature, {})
        return reason, snapshot, snapshot.generated_transition_count-generated, (time.perf_counter()-started)*1000.

    def __call__(self, planner, pose, speed, curvature, body_cache):
        # Geometry/configuration belongs to the current maneuver. New LiDAR
        # solids are checked on the ROS thread before any adopted path moves.
        context = (id(planner), id(planner.selected), id(planner.pair),
                   id(planner._global_path), planner.config)
        if self._context != context:
            self.cancel()
        if self._future is None:
            snapshot = copy(planner)
            snapshot._obstacle_bounds = dict(planner._obstacle_bounds)
            snapshot._validated_paths = {}
            self._context = context
            self._future = self._executor.submit(self._search, snapshot, pose, speed, curvature)
            return 'RETURN_SEARCH_PENDING'
        if not self._future.done():
            return 'RETURN_SEARCH_PENDING'
        future = self._future
        self._future = self._context = None
        reason, snapshot, generated, self.elapsed_ms = future.result()
        planner.candidates = snapshot.candidates
        planner.generated_transition_count += generated
        if not reason:
            planner.selected = snapshot.selected
            planner._return_path = snapshot._return_path
            planner.return_commits += 1
            # Never inherit worker validation against its older observations.
            # plan() now runs full path/body/PP checks with current live inputs,
            # followed by the integration's distinct-fresh-pose interlock.
            planner._validated_paths.clear()
            planner.reason = ''
        return reason
