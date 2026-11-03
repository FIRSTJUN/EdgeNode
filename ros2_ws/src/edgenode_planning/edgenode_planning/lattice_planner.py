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
        wheelbase_m=2.84,
        max_wheel_angle_rad=0.610865,
        max_front_steer_normalized=0.70,
        curvature_safety_factor=1.0,
        obstacle_rear_margin_m=0.5,
        blocked_release_cycles=3,
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

        self.wheelbase_m = float(wheelbase_m)
        self.max_wheel_angle_rad = float(max_wheel_angle_rad)
        self.max_front_steer_normalized = float(max_front_steer_normalized)
        self.curvature_safety_factor = float(curvature_safety_factor)
        self.obstacle_rear_margin_m = float(obstacle_rear_margin_m)
        for name in ('wheelbase_m', 'max_wheel_angle_rad'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('max_front_steer_normalized', 'curvature_safety_factor'):
            if not math.isfinite(getattr(self, name)) or not 0 < getattr(self, name) <= 1:
                raise ValueError(f'{name} must be in (0, 1]')
        if self.max_front_steer_normalized * self.max_wheel_angle_rad >= math.pi / 2:
            raise ValueError('usable wheel angle must be below pi/2')
        if not math.isfinite(self.obstacle_rear_margin_m) or self.obstacle_rear_margin_m < 0:
            raise ValueError('obstacle_rear_margin_m must be finite and nonnegative')
        if (isinstance(blocked_release_cycles, bool)
                or not isinstance(blocked_release_cycles, int) or blocked_release_cycles < 2):
            raise ValueError('blocked_release_cycles must be an integer >= 2')
        self.blocked_release_cycles = blocked_release_cycles
        self.usable_curvature_limit = self.curvature_safety_factor * math.tan(
            self.max_front_steer_normalized * self.max_wheel_angle_rad) / self.wheelbase_m
        self.reset()

    def reset(self):
        """Forget maneuver history only on an explicit route replacement/reset."""
        self._avoidance_active = False
        self._selected_offset = None
        self._front_point = None
        self._rear_point = None
        self._blocked = False
        self._active_obstacle_bounds = None
        # Keep all associated footprint samples until actual rear passage.
        # A missing near cluster must not erase known occupancy during HOLD.
        self._remembered_obstacle_points = set()
        self._maneuver = None
        self._maneuver_progress_m = 0.0
        self._maneuver_start_global_progress_m = None
        self._road_progress_m = None
        self._actual_offset = math.nan
        self.break_release_confirmation()

    def break_release_confirmation(self):
        """Unavailable planning input breaks consecutive-safe confirmation."""
        self._release_count = 0
        self._release_key = None

    @staticmethod
    def maximum_curvature(points):
        """
        Return maximum absolute XY circumcircle curvature, independent of speed.

        Consecutive duplicate XY points are ignored; invalid/degenerate paths
        and collinear reversals fail closed. This is a sampled-path feasibility
        gate, not a full dynamics or initial-heading tracking guarantee.
        """
        xy = []
        for point in points:
            x, y = float(point[0]), float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                return math.inf
            if not xy or math.hypot(x - xy[-1][0], y - xy[-1][1]) > 1e-9:
                xy.append((x, y))
        if len(xy) < 2:
            return math.inf
        maximum = 0.0
        for a, b, c in zip(xy, xy[1:], xy[2:]):
            ux, uy = b[0] - a[0], b[1] - a[1]
            vx, vy = c[0] - b[0], c[1] - b[1]
            cross = ux * vy - uy * vx
            if ux * vx + uy * vy < 0 and abs(cross) <= 1e-12:
                return math.inf
            denominator = math.hypot(ux, uy) * math.hypot(vx, vy) * math.hypot(
                c[0] - a[0], c[1] - a[1])
            if denominator <= 1e-18:
                return math.inf
            maximum = max(maximum, 2.0 * abs(cross) / denominator)
        return maximum

    def steering_feasible(self, points):
        """Check the sampled XY path against the actual command geometry."""
        return self.maximum_curvature(points) <= self.usable_curvature_limit

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

    @staticmethod
    def _project_reference(points, arc_lengths, obstacle):
        """
        Return signed route arc and lateral distance, extrapolating endpoint tangents.

        Negative arc is essential: a forward-only local path must not clamp an
        already passed rear to s=0 and lose the proof of longitudinal clearance.
        """
        best = None
        for i, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            raw = ((obstacle[0] - a[0]) * dx + (obstacle[1] - a[1]) * dy) / length**2
            fraction = max(0.0, min(1.0, raw))
            distance = math.hypot(obstacle[0] - a[0] - fraction * dx,
                                  obstacle[1] - a[1] - fraction * dy)
            if best is None or distance < best[0]:
                along = raw if (arc_lengths[i] == 0 and raw < 0) or (
                    arc_lengths[i + 1] == arc_lengths[-1] and raw > 1) else fraction
                lateral = abs((obstacle[0] - a[0]) * dy
                              - (obstacle[1] - a[1]) * dx) / length
                best = (distance, arc_lengths[i] + along * length, lateral)
        return (best[1], best[2]) if best else (math.nan, math.inf)

    def _relevant_obstacles(self, points, obstacles, ego_s=0.0):
        arc = self._arc_lengths(points)
        return [dict(point=obstacle, s=s, reference_distance_m=distance)
                for obstacle in obstacles
                for s, distance in [self._project_reference(points, arc, obstacle)]
                if distance <= self.obstacle_influence_m
                and s + self.obstacle_rear_margin_m >= ego_s
                and s <= arc[-1] + self.obstacle_influence_m]

    @staticmethod
    def _quintic_maximum_curvature(delta, length):
        """Return the continuous straight-reference quintic curvature peak."""
        delta = abs(delta)
        if delta <= 1e-12:
            return 0.0
        if length <= 0.0:
            return math.inf
        ratio2 = (delta / length)**2
        low, high = 0.0, (3.0 - math.sqrt(3.0)) / 6.0
        # Symmetric peaks. The positive peak lies before max(f'') because
        # curvature's (1 + d'^2) denominator shifts it toward the start.
        for _ in range(28):
            t = (low + high) / 2.0
            first = 30.0 * t**2 * (1.0 - t)**2
            second = 60.0 * t * (1.0 - t) * (1.0 - 2.0 * t)
            third = 60.0 * (1.0 - 6.0 * t + 6.0 * t**2)
            derivative = (third * (1.0 + ratio2 * first**2)
                          - 3.0 * ratio2 * first * second**2)
            if derivative > 0.0:
                low = t
            else:
                high = t
        t = (low + high) / 2.0
        first = 30.0 * t**2 * (1.0 - t)**2
        second = 60.0 * t * (1.0 - t) * (1.0 - 2.0 * t)
        return delta * second / length**2 / (1.0 + ratio2 * first**2)**1.5

    def _required_transition_length(self, delta):
        """Solve quintic length from lateral displacement and unchanged geometry."""
        if abs(delta) <= 1e-12:
            return 0.0
        low = 0.0
        # max |f''|=10/sqrt(3); dropping the >=1 denominator gives a
        # guaranteed feasible upper bracket on a straight reference.
        high = math.sqrt(10.0 / math.sqrt(3.0) * abs(delta) / self.usable_curvature_limit)
        for _ in range(28):
            length = (low + high) / 2.0
            if self._quintic_maximum_curvature(delta, length) > self.usable_curvature_limit:
                low = length
            else:
                high = length
        return high

    @staticmethod
    def _profile_reference(points, arc, normals, full_start_s):
        """Insert the exact full-offset endpoint without extending the horizon."""
        samples = list(zip(points, arc, normals))
        if full_start_s <= 0 or full_start_s >= arc[-1]:
            return samples
        if any(abs(s - full_start_s) <= 1e-9 for s in arc):
            return samples
        for i in range(len(points) - 1):
            if arc[i] < full_start_s < arc[i + 1]:
                fraction = (full_start_s - arc[i]) / (arc[i + 1] - arc[i])
                point = [(1.0 - fraction) * a + fraction * b
                         for a, b in zip(points[i], points[i + 1])]
                nx, ny = [(1.0 - fraction) * a + fraction * b
                          for a, b in zip(normals[i], normals[i + 1])]
                length = math.hypot(nx, ny)
                normal = (nx / length, ny / length) if length > 1e-12 else (0.0, 0.0)
                samples.insert(i + 1, (point, full_start_s, normal))
                break
        return samples

    def _avoidance_offset_profile(self, s, entry_length, current_offset, target_offset,
                                  start_s=0.0):
        """Blend over the explicitly solved entry, then hold with no return."""
        if entry_length <= 0.0:
            return target_offset
        blend = _smoothstep5((s - start_s) / entry_length)
        return current_offset + (target_offset - current_offset) * blend

    def _recovery_offset_profile(self, s, current_offset, transition_length=None, start_s=0.0):
        """Return smoothly over an explicit, curvature-checked recovery length."""
        length = self.transition_after_m if transition_length is None else transition_length
        return self._avoidance_offset_profile(s, length, current_offset, 0.0, start_s)

    @staticmethod
    def _boundary_coefficients(current, target, length, first=0.0, second=0.0):
        """Quintic Hermite boundary: (d,d',d'') -> (target,0,0)."""
        a, b, delta = first * length, 0.5 * second * length**2, target - current
        return (current, a, b, 10*delta - 6*a - 3*b,
                -15*delta + 8*a + 3*b, 6*delta - 3*a - b)

    @staticmethod
    def _boundary_state(coefficients, length, distance):
        if length <= 0.0 or distance >= length:
            return sum(coefficients), 0.0, 0.0
        u = max(0.0, distance / length)
        d = sum(c * u**i for i, c in enumerate(coefficients))
        first = sum(i*c*u**(i-1) for i, c in enumerate(coefficients) if i >= 1) / length
        second = sum(i*(i-1)*c*u**(i-2) for i, c in enumerate(coefficients) if i >= 2) / length**2
        return d, first, second

    @staticmethod
    def _polynomial_value(coefficients, u):
        value = 0.0
        for c in reversed(coefficients):
            value = value*u + c
        return value

    @classmethod
    def _unit_roots(cls, coefficients):
        """Isolate real polynomial roots on [0,1] by derivative monotonicity."""
        coefficients = list(coefficients)
        while len(coefficients) > 1 and abs(coefficients[-1]) < 1e-14:
            coefficients.pop()
        if len(coefficients) <= 1:
            return []
        if len(coefficients) == 2:
            root = -coefficients[0]/coefficients[1]
            return [root] if 0 <= root <= 1 else []
        critical = cls._unit_roots([i*c for i, c in enumerate(coefficients) if i])
        boundaries = [0.0] + critical + [1.0]
        roots = [u for u in boundaries if abs(cls._polynomial_value(coefficients, u)) < 1e-12]
        for low, high in zip(boundaries, boundaries[1:]):
            a, b = cls._polynomial_value(coefficients, low), cls._polynomial_value(coefficients, high)
            if a*b >= 0:
                continue
            for _ in range(44):
                middle = (low+high)/2
                value = cls._polynomial_value(coefficients, middle)
                if a*value > 0:
                    low, a = middle, value
                else:
                    high = middle
            roots.append((low+high)/2)
        return sorted(set(roots))

    @classmethod
    def _boundary_maximum_curvature(cls, coefficients, length):
        """Continuous straight-reference curvature, including nonzero boundary state.

        Extrema solve third*(1+first^2)-3*first*second^2=0. Root isolation avoids
        missing a between-sample profile peak; world polyline is gated separately.
        """
        if length <= 0:
            return 0.0 if max(abs(c) for c in coefficients[1:]) <= 1e-12 else math.inf
        first = [i*c/length for i, c in enumerate(coefficients) if i >= 1]
        second = [i*c/length for i, c in enumerate(first) if i >= 1]
        third = [i*c/length for i, c in enumerate(second) if i >= 1]

        def multiply(a, b):
            product = [0.0]*(len(a)+len(b)-1)
            for i, x in enumerate(a):
                for j, y in enumerate(b):
                    product[i+j] += x*y
            return product

        denominator = multiply(first, first)
        denominator[0] += 1
        equation = multiply(third, denominator)
        subtract = multiply(first, multiply(second, second))
        for i, value in enumerate(subtract):
            equation[i] -= 3*value
        scale = max(abs(c) for c in equation)
        roots = cls._unit_roots([c/scale for c in equation]) if scale else []
        return max(abs(cls._polynomial_value(second, u)) /
                   (1+cls._polynomial_value(first, u)**2)**1.5 for u in [0., 1.] + roots)

    def _generate_avoidance_candidate(self, points, arc_lengths, normals, entry_length,
                                      current_offset, target_offset, start_s=0.0,
                                      first=0.0, second=0.0):
        coefficients = self._boundary_coefficients(
            current_offset, target_offset, entry_length, first, second)
        candidate = []
        for point, s, normal in self._profile_reference(
                points, arc_lengths, normals, start_s + entry_length):
            d = self._boundary_state(coefficients, entry_length, s - start_s)[0]
            candidate.append([point[0] + normal[0] * d, point[1] + normal[1] * d, point[2]])
        return candidate

    def _generate_recovery_candidate(self, points, arc_lengths, normals, current_offset,
                                     transition_length=None, start_s=0.0):
        length = self.transition_after_m if transition_length is None else transition_length
        return self._generate_avoidance_candidate(
            points, arc_lengths, normals, length, current_offset, 0.0, start_s)

    def _adaptive_transition(self, points, arc, normals, current_offset, target_offset,
                             available_length, preferred_length, start_s=0.0,
                             first=0.0, second=0.0):
        """
        Find a bounded entry/recovery meeting both nominal and published curvature.

        Geometry determines the required length. A bounded increasing-length
        search handles curved references; the existing candidate gate remains
        authoritative. No horizon extension or late offset completion is allowed.
        """
        delta = target_offset - current_offset
        # The rest-to-rest length is not a lower bound for a moving profile:
        # a valid suffix already carries lateral velocity and acceleration.
        required = (self._required_transition_length(delta)
                    if abs(first) + abs(second) <= 1e-12 else 0.0)
        available = max(0.0, available_length)
        initial = min(available, max(required, min(preferred_length, available)))
        if abs(delta) <= 1e-12 and abs(first) + abs(second) <= 1e-12:
            initial = 0.0

        def evaluate(length):
            candidate = self._generate_avoidance_candidate(
                points, arc, normals, length, current_offset, target_offset, start_s, first, second)
            coefficients = self._boundary_coefficients(
                current_offset, target_offset, length, first, second)
            profile_curvature = (self._quintic_maximum_curvature(delta, length)
                                 if abs(first) + abs(second) <= 1e-12 else
                                 self._boundary_maximum_curvature(coefficients, length))
            polyline_curvature = self.maximum_curvature(candidate)
            curvature = max(polyline_curvature, profile_curvature)
            complete = (required <= available and start_s + length <= arc[-1]
                        and (length > 1e-9 or abs(first) + abs(second) <= 1e-12))
            return dict(points=candidate, entry_length=length,
                        required_entry_length=required, full_offset_start_s=start_s + length,
                        curvature=curvature, profile_curvature=profile_curvature,
                        polyline_curvature=polyline_curvature,
                        coefficients=coefficients, profile_length=length, profile_start_s=start_s,
                        entry_complete=complete,
                        feasible=complete and curvature <= self.usable_curvature_limit)

        first_trial = evaluate(initial)
        if first_trial['feasible'] or required > available:
            return first_trial
        # Reference curvature can invalidate the straight-reference lower bound.
        # Scan toward the hard deadline, then refine the first feasible bracket.
        previous = initial
        best = first_trial
        for step in range(1, 17):
            length = initial + (available - initial) * step / 16.0
            trial = evaluate(length)
            if trial['curvature'] < best['curvature']:
                best = trial
            if trial['feasible']:
                low, high = previous, length
                for _ in range(24):
                    middle = (low + high) / 2.0
                    candidate = evaluate(middle)
                    if candidate['feasible']:
                        high, trial = middle, candidate
                    else:
                        low = middle
                return trial
            previous = length
        return best

    @staticmethod
    def _transition_diagnostics(result, transition):
        if transition is not None:
            for name in ('entry_length', 'full_offset_start_s', 'recovery_length',
                         'coefficients', 'profile_length', 'profile_start_s', 'offset', 'clearance',
                         'profile_curvature', 'polyline_curvature', 'reject_reason'):
                if name in transition:
                    result[name] = transition[name]
            result['evaluated_candidate_offset'] = transition.get(
                'offset', transition.get('evaluated_candidate_offset', math.nan))
            result['evaluated_candidate_clearance'] = transition.get(
                'clearance', transition.get('minimum_clearance_m', math.nan))
            for kind in ('profile', 'polyline'):
                result['evaluated_candidate_' + kind + '_curvature'] = transition.get(
                    kind + '_curvature', math.nan)
            result['evaluated_candidate_reject_reason'] = transition.get('reject_reason', '')
        return result

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

    def _result(self, status, points, offset, clearance, curvature, front_s, rear_s,
                reason=''):
        return {
            'status': status, 'points': points, 'selected_offset_m': offset,
            'minimum_clearance_m': clearance, 'maximum_curvature': curvature,
            'usable_curvature_limit': self.usable_curvature_limit,
            'obstacle_front_s': front_s, 'obstacle_rear_s': rear_s,
            'entry_length': math.nan, 'full_offset_start_s': math.nan,
            'recovery_length': math.nan,
            'blocked_reason': reason, 'blocked': status == 'BLOCKED',
            'obstacle_avoid_requested': status in ('AVOIDING', 'RECOVERING'),
            'actual_lateral_offset': self._actual_offset,
            'last_approved_offset': (self._selected_offset if self._selected_offset is not None
                                     else math.nan),
            'evaluated_candidate_offset': math.nan,
            'evaluated_candidate_clearance': math.nan,
            'evaluated_candidate_profile_curvature': math.nan,
            'evaluated_candidate_polyline_curvature': math.nan,
            'evaluated_candidate_reject_reason': 'NOT_EVALUATED',
        }

    def _stop(self, points, current_offset, clearance, curvature, front_s, rear_s, reason,
              pending=False, transition=None):
        self._blocked = True
        if not pending:
            self._release_count = 0
            self._release_key = None
        result = self._transition_diagnostics(
            self._result('BLOCKED', points, current_offset, clearance, curvature,
                         front_s, rear_s, reason), transition)
        if not result['evaluated_candidate_reject_reason']:
            result['evaluated_candidate_reject_reason'] = reason
        return result

    def _approve(self, result, reference, current_offset):
        """Require stable safe behavior/direction on consecutive Planning cycles."""
        key = (result['status'], math.copysign(1, result['selected_offset_m'])
               if result['selected_offset_m'] else 0)
        if self._blocked:
            self._release_count = self._release_count + 1 if key == self._release_key else 1
            self._release_key = key
            if self._release_count < self.blocked_release_cycles:
                return self._stop(
                    reference, current_offset, result['minimum_clearance_m'],
                    result['maximum_curvature'], result['obstacle_front_s'],
                    result['obstacle_rear_s'], 'RELEASE_PENDING', pending=True, transition=result)
        self._blocked = False
        self._release_count = 0
        self._release_key = None
        if result['status'] == 'AVOIDING':
            self._avoidance_active = True
            self._selected_offset = result['selected_offset_m']
        elif result['status'] == 'CLEAR':
            self._avoidance_active = False
            self._selected_offset = None
            self._front_point = self._rear_point = None
            self._active_obstacle_bounds = None
            self._remembered_obstacle_points.clear()
            self._maneuver = None
            self._maneuver_start_global_progress_m = None
            self._maneuver_progress_m = 0.0
        if result['status'] in ('AVOIDING', 'RECOVERING') and 'coefficients' in result:
            if self._maneuver_start_global_progress_m is None:
                self._maneuver_start_global_progress_m = self._road_progress_m
            saved_arc = self._arc_lengths(reference)
            end_s = result['profile_start_s'] + result['profile_length']
            end_sample = min(self._profile_reference(
                reference, saved_arc, self._normals(reference), end_s),
                key=lambda sample: abs(sample[1]-end_s))
            self._maneuver = dict(reference=reference, arc=saved_arc,
                                  end_point=end_sample[0], road_progress=self._road_progress_m,
                                  coefficients=result['coefficients'],
                                  length=result['profile_length'], start_s=result['profile_start_s'], progress=0.0,
                                  offset=result['selected_offset_m'], status=result['status'])
        result['last_approved_offset'] = (self._selected_offset
                                        if self._selected_offset is not None else math.nan)
        return result

    @staticmethod
    def _bounds(group):
        return (min(p[0] for p in group), min(p[1] for p in group),
                max(p[0] for p in group), max(p[1] for p in group))

    def _legacy_groups(self, obstacles):
        """Spatial fallback for anonymous centers; explicit footprint groups win.

        This is geometric association, not a semantic object/tracker identity.
        The radius only groups metadata and never changes collision samples.
        """
        remaining = list(obstacles)
        groups = []
        while remaining:
            group = [remaining.pop()]
            for point in group:
                nearby = [p for p in remaining if math.dist(p, point) <= 2*self.obstacle_influence_m]
                group.extend(nearby)
                remaining = [p for p in remaining if p not in nearby]
            groups.append(group)
        return groups

    def _update_extent(self, points, arc, obstacles, groups, ego_s):
        groups = groups if groups is not None else self._legacy_groups(obstacles)
        relevant_groups = [g for g in groups if g and self._relevant_obstacles(points, g, ego_s)]
        matched = []
        if self._active_obstacle_bounds is not None:
            a = self._active_obstacle_bounds
            for group in relevant_groups:
                b = self._bounds(group)
                gap = math.hypot(max(a[0]-b[2], b[0]-a[2], 0),
                                 max(a[1]-b[3], b[1]-a[3], 0))
                if gap <= self.required_clearance_m:
                    matched.append(group)
        elif relevant_groups:
            matched = [min(relevant_groups, key=lambda g: min(
                self._project_reference(points, arc, p)[0] for p in g))]
        rear_passed = (self._rear_point is not None and ego_s >
                       self._project_reference(points, arc, self._rear_point)[0]
                       + self.obstacle_rear_margin_m)
        if not matched and rear_passed and relevant_groups:
            # A subsequent object starts a new associated extent only after
            # the remembered rear is actually passed.
            self._front_point = self._rear_point = None
            self._active_obstacle_bounds = None
            self._remembered_obstacle_points.clear()
            matched = [min(relevant_groups, key=lambda g: min(
                self._project_reference(points, arc, p)[0] for p in g))]
        if matched:
            # Multiple ambiguous matches are not fused into one remembered
            # object. Keep the closest world bounds; every other group remains
            # in the collision set and can still block this maneuver.
            group = min(matched, key=lambda g: math.dist(
                self._bounds(g), self._active_obstacle_bounds)
                if self._active_obstacle_bounds is not None else 0.0)
            self._active_obstacle_bounds = self._bounds(group)
            self._remembered_obstacle_points.update(tuple(p) for p in group)
            observed = list(group)
            observed.extend(p for p in (self._front_point, self._rear_point) if p is not None)
            self._front_point = min(observed, key=lambda p: self._project_reference(points, arc, p)[0])
            self._rear_point = max(observed, key=lambda p: self._project_reference(points, arc, p)[0])
        return bool(matched)

    def _continuation_boundary(self, points, arc, current_position, current_heading):
        """Reproject a world-anchored approved profile, carrying its derivatives.

        The restricted Hermite polynomial is equivalent to the previous suffix
        under exact tracking. Tracking corrections retain measured position/yaw
        and previous d''; all regenerated candidates pass the normal gates.
        """
        first, second, remaining = 0.0, 0.0, None
        if self._maneuver is not None:
            saved = self._maneuver
            if self._road_progress_m is not None and saved['road_progress'] is not None:
                progress = max(saved['progress'], self._road_progress_m-saved['road_progress'])
                self._maneuver_progress_m = max(self._maneuver_progress_m,
                    self._road_progress_m-self._maneuver_start_global_progress_m)
            else:
                progress = max(saved['progress'], self._project_reference(
                    saved['reference'], saved['arc'], current_position or points[0])[0] - saved['start_s'])
                self._maneuver_progress_m += max(0.0, progress-saved['progress'])
            saved['progress'] = progress
            _, first, second = self._boundary_state(saved['coefficients'], saved['length'], progress)
            # Keep the completion boundary in world coordinates, including
            # localization jitter behind the monotone road progress origin.
            remaining = max(0.0, self._project_reference(points, arc, saved['end_point'])[0]
                            - self._project_reference(points, arc, current_position or points[0])[0])
        if current_heading is not None:
            tangent = math.atan2(points[1][1]-points[0][1], points[1][0]-points[0][0])
            difference = math.atan2(math.sin(current_heading-tangent), math.cos(current_heading-tangent))
            # A forward Frenet profile cannot represent a sideways/reversed Ego.
            if abs(difference) >= math.pi / 2:
                return math.inf, second, remaining
            normals = self._normals(points)
            distance = math.dist(points[0][:2], points[1][:2])
            nx, ny = normals[0]
            # Derivative of the normalized, interpolated normal at the start.
            dx = (normals[1][0]-nx)/distance if distance > 1e-12 else 0.0
            dy = (normals[1][1]-ny)/distance if distance > 1e-12 else 0.0
            radial = nx*dx + ny*dy
            normal_tangent_derivative = ((dx-radial*nx)*math.cos(tangent)
                                         + (dy-radial*ny)*math.sin(tangent))
            first = (1+self._actual_offset*normal_tangent_derivative)*math.tan(difference)
        return first, second, remaining

    def _ego_reference(self, points, position):
        """Trim/extrapolate geometry to Ego projection without resetting road progress."""
        arc = self._arc_lengths(points)
        ego_s = self._project_reference(points, arc, position)[0]
        samples = self._profile_reference(points, arc, self._normals(points), ego_s)
        # Projection roundoff around s=0 must not prepend a zero-length
        # segment: its undefined tangent/normal breaks heading and curvature.
        if -1e-9 <= ego_s < 0:
            ego_s = 0.0
        if ego_s < 0:
            a, b = points[:2]
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            origin = [a[0]+ego_s*(b[0]-a[0])/length,
                      a[1]+ego_s*(b[1]-a[1])/length, a[2]]
            return [origin] + points
        return [p for p, s, _ in samples if s >= ego_s - 1e-9]

    def plan(self, reference_points, obstacles, current_position=None, *,
             current_heading=None, obstacle_groups=None, road_progress_m=None):
        """Choose a newly checked path with world profile and object extent memory."""
        if reference_points is None or len(reference_points) < 3:
            self.break_release_confirmation()
            return None
        points = [[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0]
                  for p in reference_points]
        obstacle_points = [(float(p[0]), float(p[1])) for p in obstacles]
        # Keep the old pure-function API's geometry for archived fixture replay.
        # The ROS caller always supplies heading and uses exact Ego position.
        if (current_position is not None and current_heading is not None
                and (obstacle_points or self._avoidance_active)):
            points = self._ego_reference(points, current_position)
            if len(points) < 3:
                self._actual_offset = self._current_lateral_offset(reference_points, current_position)
                return self._stop(points, self._actual_offset, 0.0, math.inf,
                                  math.nan, math.nan, 'INVALID_AVOIDANCE_REFERENCE')
        arc = self._arc_lengths(points)
        normals = self._normals(points)
        current_offset = self._current_lateral_offset(points, current_position)
        self._actual_offset = current_offset
        if road_progress_m is not None:
            self._road_progress_m = (road_progress_m if self._road_progress_m is None
                                     else max(self._road_progress_m, road_progress_m))
        ego_s = self._project_reference(
            points, arc, current_position if current_position is not None else points[0])[0]
        first, second, remaining = self._continuation_boundary(
            points, arc, current_position, current_heading)
        if not math.isfinite(first) or not math.isfinite(second):
            return self._stop(points, current_offset, 0.0, math.inf, math.nan, math.nan,
                              'INVALID_HEADING_BOUNDARY')
        relevant = self._update_extent(points, arc, obstacle_points, obstacle_groups, ego_s)

        front_s = (self._project_reference(points, arc, self._front_point)[0]
                   if self._front_point is not None else math.nan)
        rear_s = (self._project_reference(points, arc, self._rear_point)[0]
                  if self._rear_point is not None else math.nan)
        rear_passed = (self._rear_point is not None
                       and ego_s > rear_s + self.obstacle_rear_margin_m)

        rear_pending = self._rear_point is not None and not rear_passed
        # Rear passage gates RECOVERING, not forward avoidance. Recheck the
        # remembered associated obstacle as well as every current observation.
        collision_points = list(obstacle_points)
        if rear_pending and not relevant:
            collision_points.extend(sorted(self._remembered_obstacle_points -
                                           {tuple(p) for p in obstacle_points}))
            if not relevant and not self._remembered_obstacle_points:
                # Imported/partial history without the associated footprint
                # cannot establish a collision-safe continuation.
                return self._stop(points, current_offset, 0.0, math.nan,
                                  front_s, rear_s, 'MISSING_ASSOCIATED_FOOTPRINT')

        if not relevant and not rear_pending:
            status = 'CLEAR'
            candidate = points
            offset = 0.0
            transition = None
            # Do not snap the final recovery suffix to the road merely because
            # d crossed the direction/hold threshold. Finish its boundary state.
            if self._avoidance_active and max(abs(current_offset), abs(first), abs(second)) > 1e-6:
                status = 'RECOVERING'
                offset = current_offset
                start_s = max(0.0, ego_s)
                transition = self._adaptive_transition(
                    points, arc, normals, offset, 0.0, arc[-1] - start_s,
                    (remaining if remaining is not None and self._maneuver['status'] == 'RECOVERING'
                     else self.transition_after_m), start_s, first, second)
                transition['offset'] = 0.0
                transition['recovery_length'] = transition['entry_length']
                transition['entry_length'] = math.nan
                transition['full_offset_start_s'] = math.nan
                candidate = transition['points']
            clearance = self._minimum_obstacle_distance(candidate, obstacle_points)
            if transition is not None:
                transition['clearance'] = clearance
                transition['reject_reason'] = ('CLEARANCE' if clearance < self.required_clearance_m else '')
            curvature = (transition['curvature'] if transition is not None
                         else self.maximum_curvature(candidate))
            if status == 'RECOVERING' and (
                    clearance < self.required_clearance_m or not transition['feasible']):
                return self._stop(points, current_offset, clearance, curvature,
                                  front_s, rear_s, 'RECOVERY_UNSAFE', transition=transition)
            result = self._result(status, candidate, offset, clearance, curvature, front_s, rear_s)
            return self._approve(self._transition_diagnostics(result, transition),
                                 points, current_offset)

        candidates = []
        start_s = max(0.0, ego_s)
        # Strictly before the obstacle front, and always within the real horizon.
        deadline = math.nextafter(min(front_s, arc[-1]), -math.inf)
        available = max(0.0, deadline - start_s)
        offsets = list(self.candidate_offsets_m)
        if (front_s <= start_s and self._avoidance_active and self._selected_offset
                and current_offset * self._selected_offset > 0
                and abs(current_offset - self._selected_offset) <= self.recovery_threshold_m
                and all(abs(current_offset - offset) > 1e-9 for offset in offsets)):
            # Alongside the obstacle, retain an already achieved safe offset
            # rather than inventing a new entry after its deadline. This zero-
            # shift hold still passes the same curvature and clearance gates.
            offsets.append(current_offset)
        def evaluate_offset(target_offset):
            preferred = self.transition_before_m
            if (remaining is not None and self._maneuver['status'] == 'AVOIDING'
                    and abs(target_offset - self._maneuver['offset']) < 1e-9):
                preferred = remaining
            transition = self._adaptive_transition(
                points, arc, normals, current_offset, target_offset, available,
                preferred, start_s, first, second)
            clearance = self._minimum_obstacle_distance(transition['points'], collision_points)
            transition['reject_reason'] = '+'.join(reason for failed, reason in (
                (clearance < self.required_clearance_m, 'CLEARANCE'),
                (not transition['entry_complete'], 'ENTRY_DEADLINE'),
                (transition['curvature'] > self.usable_curvature_limit, 'CURVATURE')) if failed)
            return dict(transition, offset=target_offset, clearance=clearance,
                        score=abs(target_offset - current_offset))

        hold_offset = None
        if (self._avoidance_active and self._selected_offset
                and current_offset * self._selected_offset > 0
                and abs(current_offset - self._selected_offset) <= self.recovery_threshold_m):
            # Near the approved target, try its remaining/near-constant profile
            # first. Measured heading and carried d'' still define the boundary;
            # neither prior approval nor closeness bypasses any feasibility gate.
            hold_offset = self._selected_offset
            hold = evaluate_offset(hold_offset)
            candidates.append(hold)
            if (hold['clearance'] >= self.required_clearance_m and hold['feasible']
                    and hold['curvature'] <= self.usable_curvature_limit):
                result = self._result('AVOIDING', hold['points'], hold['offset'],
                                      hold['clearance'], hold['curvature'], front_s, rear_s)
                result['maneuver_phase'] = 'HOLD'
                return self._approve(self._transition_diagnostics(result, hold),
                                     points, current_offset)
        for target_offset in offsets:
            if hold_offset is None or abs(target_offset-hold_offset) > 1e-9:
                candidates.append(evaluate_offset(target_offset))
        safe = [c for c in candidates if c['clearance'] >= self.required_clearance_m
                and c['feasible'] and c['curvature'] <= self.usable_curvature_limit]
        best = max(candidates, key=lambda c: c['clearance'])
        if not safe:
            return self._stop(points, current_offset, best['clearance'], best['curvature'],
                              front_s, rear_s, 'NO_SAFE_FEASIBLE_CANDIDATE', transition=best)

        if self._avoidance_active and self._selected_offset:
            same_side = [c for c in safe if c['offset'] * self._selected_offset > 0]
            if same_side:
                # Keep the previous command if still executable, then prefer
                # the smallest same-side adjustment. Score cannot flip sides.
                selected = min(same_side, key=lambda c: (
                    abs(c['offset'] - self._selected_offset), c['score']))
            elif not self._blocked or abs(current_offset) > self.recovery_threshold_m:
                # A committed lateral maneuver must stop. A new opposite-side
                # plan is only eligible after BLOCKED, near center, with the
                # normal collision/curvature checks and release persistence.
                return self._stop(points, current_offset, best['clearance'], best['curvature'],
                                  front_s, rear_s, 'DIRECTION_CHANGE_REQUIRES_STOP',
                                  transition=best)
            else:
                selected = min(safe, key=lambda c: c['score'])
        else:
            selected = min(safe, key=lambda c: c['score'])
        result = self._result('AVOIDING', selected['points'], selected['offset'],
                              selected['clearance'], selected['curvature'], front_s, rear_s)
        return self._approve(
            self._transition_diagnostics(result, selected), points, current_offset)
