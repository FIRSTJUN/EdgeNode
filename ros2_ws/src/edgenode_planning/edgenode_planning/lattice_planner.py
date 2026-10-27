import math


def _smoothstep5(t):
    """Quintic smoothstep from 0 to 1."""
    t = max(0.0, min(1.0, float(t)))
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


class LatticePlanner:
    """Generate smooth lateral avoidance paths around a reference path.

    Positive offset = left side of reference path.
    Negative offset = right side.
    """

    def __init__(
        self,
        candidate_offsets_m=None,
        obstacle_clearance_m=0.55,
        obstacle_influence_m=2.5,
        transition_before_m=5.0,
        transition_after_m=6.0,
        hold_half_length_m=1.0,
        recovery_threshold_m=0.15,
    ):
        if candidate_offsets_m is None:
            candidate_offsets_m = (
                0.0,
                0.6,
                -0.6,
                0.8,
                -0.8,
            )

        self.candidate_offsets_m = tuple(
            float(x) for x in candidate_offsets_m
        )

        self.obstacle_clearance_m = float(obstacle_clearance_m)
        self.obstacle_influence_m = float(obstacle_influence_m)
        self.transition_before_m = float(transition_before_m)
        self.transition_after_m = float(transition_after_m)
        self.hold_half_length_m = float(hold_half_length_m)
        self.recovery_threshold_m = float(recovery_threshold_m)

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
        if not obstacles:
            return float('inf')

        minimum = float('inf')

        for point in candidate:
            px = float(point[0])
            py = float(point[1])

            for obstacle in obstacles:
                ox, oy = obstacle

                distance = math.hypot(
                    px - ox,
                    py - oy,
                )

                minimum = min(
                    minimum,
                    distance,
                )

        return minimum

    def plan(
        self,
        reference_points,
        obstacles,
        current_position=None,
    ):
        """Return a continuous collision-free local avoidance path."""

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
            if abs(current_offset) > self.recovery_threshold_m:
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

        relevant_points = [
            item['point']
            for item in relevant
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
                relevant_points,
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

        return {
            'status': 'AVOIDING',
            'points': selected['points'],
            'selected_offset_m': selected['offset'],
            'minimum_clearance_m': selected['clearance'],
            'obstacle_avoid_requested': True,
            'blocked': False,
        }
