import math


DEFAULT_CANDIDATE_OFFSETS_M = (
    0.0, 0.6, -0.6, 0.8, -0.8, 1.2, -1.2, 1.5, -1.5,
    1.8, -1.8, 2.1, -2.1, 2.4, -2.4,
)


def _smoothstep5(t):
    """Quintic smoothstep from 0 to 1."""
    t = max(0.0, min(1.0, float(t)))
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


class LatticePlanner:
    """Generate smooth lateral avoidance paths around a reference path.

    Positive offset = left side of reference path.
    Negative offset = right side. Clearance inflates observed obstacle samples
    by ego half-width plus a margin; it is not a full oriented-body sweep or
    a check of drivable lane boundaries. Width defaults are MORAI test values.
    """

    def __init__(
        self,
        candidate_offsets_m=None,
        obstacle_clearance_m=None,
        obstacle_influence_m=2.5,
        transition_before_m=5.0,
        transition_after_m=10.0,
        hold_half_length_m=1.0,
        recovery_threshold_m=0.15,
        *,
        ego_half_width_m=0.95,
        obstacle_safety_margin_m=0.25,
    ):
        if candidate_offsets_m is None:
            candidate_offsets_m = DEFAULT_CANDIDATE_OFFSETS_M
        self.candidate_offsets_m = tuple(float(x) for x in candidate_offsets_m)
        if not self.candidate_offsets_m or not all(
                math.isfinite(x) for x in self.candidate_offsets_m):
            raise ValueError('candidate_offsets_m must contain finite offsets')
        self.ego_half_width_m = float(ego_half_width_m)
        self.obstacle_safety_margin_m = float(obstacle_safety_margin_m)
        for name in ('ego_half_width_m', 'obstacle_safety_margin_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and nonnegative')
        legacy_floor = 0.0 if obstacle_clearance_m is None else float(obstacle_clearance_m)
        if not math.isfinite(legacy_floor) or legacy_floor < 0.0:
            raise ValueError('obstacle_clearance_m must be finite and nonnegative')
        # Keep old callers/configs as a minimum floor; they cannot silently
        # reduce the ego half-width plus margin used for footprint points.
        self.required_clearance_m = max(
            legacy_floor, self.ego_half_width_m + self.obstacle_safety_margin_m)
        if self.required_clearance_m <= 0.0:
            raise ValueError('required clearance must be positive')
        self.obstacle_clearance_m = self.required_clearance_m
        self.obstacle_influence_m = float(obstacle_influence_m)
        self.transition_before_m = float(transition_before_m)
        self.transition_after_m = float(transition_after_m)
        self.hold_half_length_m = float(hold_half_length_m)
        self.recovery_threshold_m = float(recovery_threshold_m)
        for name in ('obstacle_influence_m', 'transition_before_m', 'transition_after_m',
                     'hold_half_length_m', 'recovery_threshold_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0 or (
                    name != 'recovery_threshold_m' and value == 0.0):
                raise ValueError(f'{name} must be finite and positive (recovery may be zero)')
        self.obstacle_influence_m = max(self.obstacle_influence_m, self.required_clearance_m)

        # RECOVERING은 실제 장애물 회피 이후에만 허용한다.
        # 시작 위치가 기준 경로에서 약간 벗어나 있다는 이유만으로
        # OBSTACLE_AVOID 상태에 들어가는 것을 방지한다.
        self._avoidance_active = False

    @staticmethod
    def _arc_lengths(points):
        s = [0.0]

        for i in range(1, len(points)):
            dx = float(points[i][0]) - float(points[i - 1][0])
            dy = float(points[i][1]) - float(points[i - 1][1])

            s.append(
                s[-1] + math.hypot(dx, dy)
            )

        return s

    @staticmethod
    def _normals(points):
        normals = []

        for i in range(len(points)):
            if i == 0:
                dx = float(points[1][0]) - float(points[0][0])
                dy = float(points[1][1]) - float(points[0][1])

            elif i == len(points) - 1:
                dx = float(points[-1][0]) - float(points[-2][0])
                dy = float(points[-1][1]) - float(points[-2][1])

            else:
                dx = float(points[i + 1][0]) - float(points[i - 1][0])
                dy = float(points[i + 1][1]) - float(points[i - 1][1])

            norm = math.hypot(dx, dy)

            if norm <= 1e-9:
                normals.append((0.0, 0.0))
                continue

            tx = dx / norm
            ty = dy / norm

            normals.append((-ty, tx))

        return normals

    @staticmethod
    def _nearest_reference_index(points, obstacle):
        ox, oy = obstacle

        best_index = 0
        best_distance = float('inf')

        for i, point in enumerate(points):
            distance = math.hypot(
                float(point[0]) - ox,
                float(point[1]) - oy,
            )

            if distance < best_distance:
                best_distance = distance
                best_index = i

        return best_index, best_distance

    @staticmethod
    def _current_lateral_offset(points, current_position):
        """Signed lateral displacement of vehicle from reference path."""

        if current_position is None or len(points) < 2:
            return 0.0

        vx = float(current_position[0])
        vy = float(current_position[1])

        best_distance = float('inf')
        best_offset = 0.0

        for i in range(len(points) - 1):
            x0 = float(points[i][0])
            y0 = float(points[i][1])
            x1 = float(points[i + 1][0])
            y1 = float(points[i + 1][1])

            dx = x1 - x0
            dy = y1 - y0

            length2 = dx * dx + dy * dy

            if length2 <= 1e-12:
                continue

            t = (
                (vx - x0) * dx +
                (vy - y0) * dy
            ) / length2

            t = max(0.0, min(1.0, t))

            px = x0 + t * dx
            py = y0 + t * dy

            ex = vx - px
            ey = vy - py

            distance = math.hypot(ex, ey)

            if distance >= best_distance:
                continue

            length = math.sqrt(length2)

            left_x = -dy / length
            left_y = dx / length

            signed_offset = (
                ex * left_x +
                ey * left_y
            )

            best_distance = distance
            best_offset = signed_offset

        return best_offset

    def _relevant_obstacles(self, points, obstacles):
        relevant = []

        for obstacle in obstacles:
            ox = float(obstacle[0])
            oy = float(obstacle[1])

            index, distance = self._nearest_reference_index(
                points,
                (ox, oy),
            )

            if distance <= self.obstacle_influence_m:
                relevant.append({
                    'point': (ox, oy),
                    'index': index,
                    'reference_distance_m': distance,
                })

        return relevant

    def _avoidance_offset_profile(
        self,
        s,
        obstacle_s,
        current_offset,
        target_offset,
    ):
        """Blend current vehicle offset into avoidance and back to center."""

        full_start = max(
            0.5,
            obstacle_s - self.hold_half_length_m,
        )

        full_end = (
            obstacle_s +
            self.hold_half_length_m
        )

        fall_end = (
            full_end +
            self.transition_after_m
        )

        # Current vehicle position -> target avoidance offset
        if s < full_start:
            t = s / max(full_start, 1e-6)

            blend = _smoothstep5(t)

            return (
                current_offset +
                (target_offset - current_offset) * blend
            )

        # Hold full avoidance offset around obstacle
        if s <= full_end:
            return target_offset

        # Return smoothly to reference path
        if s < fall_end:
            t = (
                s - full_end
            ) / max(
                fall_end - full_end,
                1e-6,
            )

            return (
                target_offset *
                (1.0 - _smoothstep5(t))
            )

        return 0.0

    def _recovery_offset_profile(
        self,
        s,
        current_offset,
    ):
        """Return smoothly from current lateral displacement to center."""

        if s >= self.transition_after_m:
            return 0.0

        t = s / max(
            self.transition_after_m,
            1e-6,
        )

        return (
            current_offset *
            (1.0 - _smoothstep5(t))
        )

    def _generate_avoidance_candidate(
        self,
        points,
        arc_lengths,
        normals,
        obstacle_s,
        current_offset,
        target_offset,
    ):
        candidate = []

        for point, s, normal in zip(
            points,
            arc_lengths,
            normals,
        ):
            d = self._avoidance_offset_profile(
                s,
                obstacle_s,
                current_offset,
                target_offset,
            )

            x = float(point[0]) + normal[0] * d
            y = float(point[1]) + normal[1] * d
            z = (
                float(point[2])
                if len(point) > 2
                else 0.0
            )

            candidate.append([x, y, z])

        return candidate

    def _generate_recovery_candidate(
        self,
        points,
        arc_lengths,
        normals,
        current_offset,
    ):
        candidate = []

        for point, s, normal in zip(
            points,
            arc_lengths,
            normals,
        ):
            d = self._recovery_offset_profile(
                s,
                current_offset,
            )

            x = float(point[0]) + normal[0] * d
            y = float(point[1]) + normal[1] * d
            z = (
                float(point[2])
                if len(point) > 2
                else 0.0
            )

            candidate.append([x, y, z])

        return candidate

    def _minimum_obstacle_distance(
        self,
        candidate,
        obstacles,
    ):
        """Distance to the complete candidate polyline, including between samples.

        Anonymous PoseArray points may be legacy centers or footprint samples;
        neither their grouping nor rectangle interiors can be inferred here.
        """
        if not obstacles:
            return float('inf')
        minimum = float('inf')
        # A one-point candidate is useful in direct diagnostics as well.
        segments = zip(candidate, candidate[1:]) if len(candidate) > 1 else (
            (point, point) for point in candidate)
        for start, end in segments:
            ax, ay = float(start[0]), float(start[1])
            dx, dy = float(end[0]) - ax, float(end[1]) - ay
            length2 = dx * dx + dy * dy
            for ox, oy in obstacles:
                fraction = (((ox - ax) * dx + (oy - ay) * dy) / length2
                            if length2 > 0.0 else 0.0)
                fraction = max(0.0, min(1.0, fraction))
                minimum = min(minimum, math.hypot(
                    ox - ax - fraction * dx, oy - ay - fraction * dy))
        return minimum

    def plan(
        self,
        reference_points,
        obstacles,
        current_position=None,
    ):
        """Return an avoidance candidate meeting sample clearance, or BLOCKED."""

        if reference_points is None or len(reference_points) < 3:
            return None

        points = [
            [
                float(point[0]),
                float(point[1]),
                float(point[2])
                if len(point) > 2
                else 0.0,
            ]
            for point in reference_points
        ]

        obstacle_points = [
            (
                float(obstacle[0]),
                float(obstacle[1]),
            )
            for obstacle in obstacles
        ]

        arc_lengths = self._arc_lengths(points)
        normals = self._normals(points)

        current_offset = self._current_lateral_offset(
            points,
            current_position,
        )

        relevant = self._relevant_obstacles(
            points,
            obstacle_points,
        )

        # No obstacle remains ahead:
        # smoothly recover from the current lateral displacement.
        if not relevant:
            if (
                self._avoidance_active
                and abs(current_offset) > self.recovery_threshold_m
            ):
                recovery_points = self._generate_recovery_candidate(
                    points,
                    arc_lengths,
                    normals,
                    current_offset,
                )

                return {
                    'status': 'RECOVERING',
                    'points': recovery_points,
                    'selected_offset_m': current_offset,
                    'minimum_clearance_m': float('inf'),
                    'obstacle_avoid_requested': True,
                    'blocked': False,
                }

            self._avoidance_active = False

            return {
                'status': 'CLEAR',
                'points': points,
                'selected_offset_m': 0.0,
                'minimum_clearance_m': float('inf'),
                'obstacle_avoid_requested': False,
                'blocked': False,
            }

        first_obstacle = min(
            relevant,
            key=lambda item: arc_lengths[item['index']],
        )

        obstacle_s = arc_lengths[
            first_obstacle['index']
        ]

        candidates = []

        for target_offset in self.candidate_offsets_m:
            candidate_points = self._generate_avoidance_candidate(
                points,
                arc_lengths,
                normals,
                obstacle_s,
                current_offset,
                target_offset,
            )

            clearance = self._minimum_obstacle_distance(
                candidate_points,
                obstacle_points,
            )

            collision = (
                clearance <
                self.obstacle_clearance_m
            )

            # Prefer minimum change from current lateral position.
            score = abs(
                target_offset -
                current_offset
            )

            candidates.append({
                'offset': target_offset,
                'points': candidate_points,
                'clearance': clearance,
                'collision': collision,
                'score': score,
            })

        safe_candidates = [
            candidate
            for candidate in candidates
            if not candidate['collision']
        ]

        if not safe_candidates:
            return {
                'status': 'BLOCKED',
                'points': points,
                'selected_offset_m': current_offset,
                'minimum_clearance_m': max(
                    candidate['clearance']
                    for candidate in candidates
                ),
                'obstacle_avoid_requested': False,
                'blocked': True,
            }

        selected = min(
            safe_candidates,
            key=lambda candidate: candidate['score'],
        )

        self._avoidance_active = True

        return {
            'status': 'AVOIDING',
            'points': selected['points'],
            'selected_offset_m': selected['offset'],
            'minimum_clearance_m': selected['clearance'],
            'obstacle_avoid_requested': True,
            'blocked': False,
        }
