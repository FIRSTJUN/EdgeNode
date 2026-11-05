"""S4 obstacle/return decisions around the scenario-independent fixed primitive."""
import math
from bisect import bisect_right

from .lane_change_planner import LaneChangePlanner
from .lane_change_path import LaneChangePathError, prepare_target_centerline
from .lane_change_types import LaneChangeDirection, LaneChangeRequest, LaneChangeState
from .lane_geometry import PolylineIndex, arc, corridor_covers, sample
from .lattice_planner import LatticePlanner


class StaticObstacleLaneChangeBehavior(LatticePlanner):
    """Reuse existing footprint association/clearance, never legacy trajectories.

    A geometry safety failure latches STOP until explicit reset. This adapter
    approves fixed geometry and the current pose; it makes no rollout claim.
    """
    def __init__(self, network, *, min_transition_time_sec=3., transition_length_m=20., **geometry):
        for value in (min_transition_time_sec, transition_length_m):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('transition duration/length must be finite and positive')
        self.network = network
        self.min_transition_time_sec = min_transition_time_sec
        self.transition_length_m = transition_length_m
        super().__init__(**geometry)

    def reset(self):
        super().reset()
        self.primitive = LaneChangePlanner(curvature_limit_m_inv=self.usable_curvature_limit)
        self._lane_pair = None
        self._failure = ''
        self._cells = []
        self._checked_paths = {}
        self._xyz_paths = {}
        self._complete_published = False
        self.return_trigger_progress_m = None

    def break_release_confirmation(self):
        super().break_release_confirmation()
        if hasattr(self, 'primitive'):
            self.primitive.update()  # Missing planning observation breaks acquisition.

    def _xyz(self, path):
        key = id(path)
        if key not in self._xyz_paths:
            self._xyz_paths[key] = (path, tuple((x, y, 0.) for x, y in path.points))
        return self._xyz_paths[key][1]

    @staticmethod
    def _future(points, position):
        lengths = arc(points)
        station = PolylineIndex(points).project(position)
        row = sample(points, station, lengths)
        if row is None:
            return [], None
        index = bisect_right(lengths, station)
        return [row[0]] + list(points[index:]), row[1]

    def _safety(self, path, position, yaw, obstacles):
        if not corridor_covers([position, position], self._cells):
            return 'DRIVABLE_CORRIDOR', 0., []
        future, tangent = self._future(path.points, position)
        if len(future) < 2:
            return 'INVALID_PATH', 0., []
        connector = [position, future[0]]
        clearance = min(self._minimum_obstacle_distance(future, obstacles),
                        self._minimum_obstacle_distance(connector, obstacles))
        if path.maximum_curvature_m_inv > self.usable_curvature_limit:
            return 'CURVATURE', clearance, future
        if sum(a*b for a, b in zip(tangent, (math.cos(yaw), math.sin(yaw)))) <= 0:
            return 'DIRECTION_CONTINUITY', clearance, future
        if id(path) not in self._checked_paths:
            self._checked_paths[id(path)] = (path, corridor_covers(path.points, self._cells))
        if not self._checked_paths[id(path)][1] or not corridor_covers(connector, self._cells):
            return 'DRIVABLE_CORRIDOR', clearance, future
        if clearance < self.required_clearance_m:
            return 'CLEARANCE', clearance, future
        return '', clearance, future

    def _package(self, points, phase, clearance=math.inf, curvature=0., front=math.nan,
                 rear=math.nan, reason=''):
        status = ('BLOCKED' if reason else 'RECOVERING' if phase == 'RECOVERING' else
                  'CLEAR' if phase in ('NORMAL', 'COMPLETE') else 'AVOIDING')
        pair = self._lane_pair
        offset = pair.at(max(0., min(pair.length, self._ego_s)))['separation'] if pair else 0.
        result = super()._result(status, [] if reason else points,
            0. if phase in ('NORMAL', 'COMPLETE', 'RECOVERING') else offset,
            clearance, curvature, front, rear, reason)
        result.update(lane_change_phase='BLOCKED' if reason else phase,
            drivable_safe=not bool(reason), selected_side=pair.side if pair else '',
            adjacent_link_id=pair.current_ids(self._ego_s)[1] if pair else '',
            original_link_id=pair.current_ids(self._ego_s)[0] if pair else '',
            adjacent_separation=offset, evaluated_candidate_clearance=clearance,
            evaluated_candidate_reject_reason=reason,
            original_lane_points=pair.original if pair else [],
            adjacent_lane_points=pair.adjacent if pair else [],
            entry_points=self._xyz(self.primitive.entry_path) if self.primitive.entry_path else [],
            recovery_points=self._xyz(self.primitive.recovery_path) if self.primitive.recovery_path else [],
            hold_points=points if phase == 'TARGET_LANE_HOLD' else [], recovery_gate_points=[],
            entry_length=self.primitive.entry_path.end_s_m-self.primitive.entry_path.start_s_m
                if self.primitive.entry_path else math.nan,
            recovery_length=self.primitive.recovery_path.end_s_m-self.primitive.recovery_path.start_s_m
                if self.primitive.recovery_path else math.nan,
            obstacle_rear_progress_m=self._rear_s, return_trigger_progress_m=self.return_trigger_progress_m)
        if pair and math.isfinite(self._rear_s):
            gate = pair.at(self._rear_s+self.obstacle_rear_margin_m)
            if gate:
                result['recovery_gate_points'] = [gate['original'][0], gate['adjacent'][0]]
        return result

    def _block(self, reason, front=math.nan, rear=math.nan, clearance=0.):
        self._failure = reason
        return self._package([], 'BLOCKED', clearance, math.inf, front, rear, reason)

    def plan(self, reference_points, obstacles, current_position=None, *, current_heading=None,
             obstacle_groups=None, road_progress_m=None, current_link_id='', route_link_ids=(), speed_kmh=0.):
        self._ego_s = 0.
        self._rear_s = math.nan
        try:
            valid = (current_position is not None and len(current_position) == 2
                and len(reference_points) >= 2 and all(math.isfinite(v) for v in
                (*current_position, current_heading, speed_kmh)) and speed_kmh >= 0
                and all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in obstacles))
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            return self._block('INVALID_INPUT')
        if self._failure:
            return self._block(self._failure)
        if self._complete_published:
            # The previous cycle approved original-lane acquisition and its
            # centerline. End this lifecycle explicitly before checking new input.
            self.reset()
        pair = self._lane_pair
        geometry = pair.original if pair else self.network.links.get(current_link_id, {}).get('points')
        if not geometry:
            return self._block('INVALID_ORIGINAL_LANE')
        try:
            original = prepare_target_centerline(geometry, curvature_limit_m_inv=self.usable_curvature_limit)
        except LaneChangePathError:
            return self._block('INVALID_ORIGINAL_LANE')
        geometry = original.points
        self._actual_offset = self._current_lateral_offset(geometry, current_position)
        lengths = arc(geometry)
        self._ego_s = PolylineIndex(geometry).project(current_position)
        self._update_extent(geometry, lengths, list(obstacles), obstacle_groups, self._ego_s)
        front_s = PolylineIndex(geometry).project(self._front_point) if self._front_point else math.nan
        self._rear_s = PolylineIndex(geometry).project(self._rear_point) if self._rear_point else math.nan
        front, rear = front_s-self._ego_s, self._rear_s-self._ego_s
        collision = list(set(tuple(p) for p in obstacles) | self._remembered_obstacle_points)
        if pair is None:
            future, tangent = self._future(geometry, current_position)
            if len(future) < 2:
                return self._block('INVALID_ORIGINAL_LANE')
            clearance = self._minimum_obstacle_distance(future, collision)
            if clearance >= self.required_clearance_m:
                try:
                    normal = prepare_target_centerline(reference_points,
                        curvature_limit_m_inv=self.usable_curvature_limit)
                except LaneChangePathError:
                    return self._block('INVALID_PATH')
                if sum(a*b for a, b in zip(tangent, (math.cos(current_heading), math.sin(current_heading)))) <= 0:
                    return self._block('DIRECTION_CONTINUITY')
                return self._package(reference_points, 'NORMAL', clearance,
                    normal.maximum_curvature_m_inv, front, rear)
            length = max(self.transition_length_m, speed_kmh/3.6*self.min_transition_time_sec)
            failures = []
            for side, direction in (('left', LaneChangeDirection.LEFT), ('right', LaneChangeDirection.RIGHT)):
                try:
                    candidate_pair = self.network.pair(current_link_id, side, list(route_link_ids))
                    if candidate_pair is not None:
                        for ids in candidate_pair.ids:
                            for link_id in ids:
                                prepare_target_centerline(self.network.links[link_id]['points'],
                                    curvature_limit_m_inv=self.usable_curvature_limit)
                        prepare_target_centerline(candidate_pair.adjacent,
                            curvature_limit_m_inv=self.usable_curvature_limit)
                except (LaneChangePathError, ValueError, TypeError, IndexError, OverflowError):
                    failures.append('INVALID_ADJACENT_GEOMETRY'); continue
                if candidate_pair is None:
                    continue
                ego_s = candidate_pair.original_index.project(current_position)
                row = candidate_pair.at(ego_s)
                if row is None:
                    failures.append('INVALID_ADJACENT_GEOMETRY'); continue
                if not math.isfinite(front_s) or ego_s+length > front_s-self.transition_before_m:
                    failures.append('ENTRY_DEADLINE'); continue
                envelope = candidate_pair.corridor_envelope(0., candidate_pair.length)
                if envelope['failure']:
                    failures.append('DRIVABLE_CORRIDOR'); continue
                candidate = LaneChangePlanner(curvature_limit_m_inv=self.usable_curvature_limit)
                result = candidate.start(LaneChangeRequest(candidate_pair.current_ids(ego_s)[1],
                    direction, True, 'STATIC_OBSTACLE'), candidate_pair.original, start_s_m=ego_s,
                    end_s_m=ego_s+length, lateral_offset_m=row['separation'],
                    target_reference_points=candidate_pair.adjacent)
                if result.state is LaneChangeState.BLOCKED:
                    failures.append(result.blocked_reason); continue
                self._cells = envelope['quads']
                reason, clearance, _ = self._safety(result.path, current_position, current_heading, collision)
                if reason:
                    failures.append(reason); continue
                self.primitive, self._lane_pair = candidate, candidate_pair
                pair = candidate_pair
                break
            if pair is None:
                return self._block('+'.join(dict.fromkeys(failures)) or 'NO_SAFE_ADJACENT_LANE',
                                   front, rear, clearance)
        result = self.primitive.update(current_position, current_heading)
        if result.state is LaneChangeState.TARGET_LANE_HOLD and self._ego_s > self._rear_s+self.obstacle_rear_margin_m:
            start = pair.adjacent_index.project(current_position)
            length = max(self.transition_length_m, speed_kmh/3.6*self.min_transition_time_sec)
            row = pair.at(self._ego_s)
            if row is None:
                return self._block('INVALID_ADJACENT_GEOMETRY', front, rear)
            if start+length > pair.adjacent_arc[-1]:
                return self._block('INSUFFICIENT_RECOVERY_DISTANCE', front, rear)
            result = self.primitive.request_return(LaneChangeRequest(pair.current_ids(self._ego_s)[0],
                LaneChangeDirection.RIGHT if pair.side == 'left' else LaneChangeDirection.LEFT,
                reason='RETURN'), pair.adjacent, start_s_m=start, end_s_m=start+length,
                lateral_offset_m=-row['separation'], target_reference_points=pair.original)
            self.return_trigger_progress_m = self._ego_s
        if result.state is LaneChangeState.BLOCKED:
            return self._block(result.blocked_reason, front, rear)
        if (result.state is LaneChangeState.RECOVERING and self._ego_s <= self._rear_s+self.obstacle_rear_margin_m):
            return self._block('RETURN_GATE_INVALIDATED', front, rear)
        reason, clearance, _ = self._safety(result.path, current_position, current_heading, collision)
        if reason:
            return self._block(reason, front, rear, clearance)
        if result.state is LaneChangeState.COMPLETE:
            self._complete_published = True
        return self._package(self._xyz(result.path), result.state.value, clearance,
            result.path.maximum_curvature_m_inv, front, rear)
