"""ROS-independent three-level speed suggestions from local path geometry.

This module only returns a suggestion; it does not publish vehicle commands.
"""

import math


_MIN_SIDE_LENGTH_M = 1e-9
_MIN_PATH_LENGTH_M = 1e-6


def _finite_number(value):
    """Accept finite numeric inputs, excluding booleans and integer overflow."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class SpeedPlanner:
    """Estimate XY curvature with a fixed preview and discrete speed profile.

    Invalid configuration is rejected by ``plan`` returning None, rather than
    raising from the constructor. All speeds are in km/h, distances in metres,
    and curvature thresholds in inverse metres.
    """

    def __init__(self, cruise_speed_kmh=1.0, curve_speed_kmh=0.7,
                 sharp_curve_speed_kmh=0.5, preview_distance_m=10.0,
                 curve_curvature_threshold=0.04, sharp_curvature_threshold=0.10):
        self.cruise_speed_kmh = cruise_speed_kmh
        self.curve_speed_kmh = curve_speed_kmh
        self.sharp_curve_speed_kmh = sharp_curve_speed_kmh
        self.preview_distance_m = preview_distance_m
        self.curve_curvature_threshold = curve_curvature_threshold
        self.sharp_curvature_threshold = sharp_curvature_threshold

    def plan(self, local_path_points):
        """Return speed and geometry diagnostics, or None for unusable input.

        Validate every XYZ point, including points beyond the preview. Z never
        affects geometry. Remove consecutive identical XY points, then clip
        the polyline to the preview distance with a linearly interpolated end.
        The evaluated length must exceed 1e-6 m and contain at least one valid
        triplet. Triplets with any side at most 1e-9 m are skipped.

        Representative curvature is the nearest-rank 80th percentile: sorted
        values at zero-based index ceil(0.8 * count) - 1. This is deterministic,
        uses no external dependencies, and rejects sparse extreme values when
        they occupy at most 20% of the samples. ``max_curvature`` remains
        available for diagnostics. Input points are never mutated.
        """
        speeds = (self.cruise_speed_kmh, self.curve_speed_kmh, self.sharp_curve_speed_kmh)
        parameters = (*speeds, self.preview_distance_m,
                      self.curve_curvature_threshold, self.sharp_curvature_threshold)
        if not all(_finite_number(value) for value in parameters):
            return None
        if any(speed < 0 for speed in speeds) or self.preview_distance_m <= 0:
            return None
        if not 0 <= self.curve_curvature_threshold < self.sharp_curvature_threshold:
            return None
        if not isinstance(local_path_points, (list, tuple)) or len(local_path_points) < 3:
            return None

        points = []
        for point in local_path_points:
            if not isinstance(point, (list, tuple)) or len(point) != 3:
                return None
            if not all(_finite_number(value) for value in point):
                return None
            xy = (float(point[0]), float(point[1]))
            if not points or xy != points[-1]:
                points.append(xy)
        if len(points) < 3:
            return None

        lengths = [math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(points, points[1:])]
        if not all(math.isfinite(length) for length in lengths):
            return None
        evaluated = [points[0]]
        evaluated_length = 0.0
        for index, length in enumerate(lengths):
            remaining = float(self.preview_distance_m) - evaluated_length
            if remaining <= 0:
                break
            if length <= remaining:
                evaluated.append(points[index + 1])
                evaluated_length += length
            else:
                fraction = remaining / length
                endpoint = tuple(
                    (1.0 - fraction) * a + fraction * b
                    for a, b in zip(points[index], points[index + 1])
                )
                if not all(math.isfinite(value) for value in endpoint):
                    return None
                evaluated.append(endpoint)
                evaluated_length = float(self.preview_distance_m)
                break
        if evaluated_length <= _MIN_PATH_LENGTH_M or len(evaluated) < 3:
            return None

        curvatures = []
        for p0, p1, p2 in zip(evaluated, evaluated[1:], evaluated[2:]):
            dx1, dy1 = p1[0] - p0[0], p1[1] - p0[1]
            dx2, dy2 = p2[0] - p0[0], p2[1] - p0[1]
            a = math.hypot(dx1, dy1)
            b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            c = math.hypot(dx2, dy2)
            if not all(math.isfinite(side) for side in (a, b, c)):
                return None
            if min(a, b, c) <= _MIN_SIDE_LENGTH_M:
                continue
            cross = dx1 * dy2 - dy1 * dx2
            denominator = a * b * c
            if not math.isfinite(cross) or not math.isfinite(denominator):
                return None
            curvature = 2.0 * abs(cross) / denominator
            if not math.isfinite(curvature):
                return None
            curvatures.append(curvature)
        if not curvatures:
            return None

        curvatures.sort()
        representative = curvatures[math.ceil(0.8 * len(curvatures)) - 1]
        if representative < self.curve_curvature_threshold:
            speed_mode, speed = 'CRUISE', self.cruise_speed_kmh
        elif representative < self.sharp_curvature_threshold:
            speed_mode, speed = 'CURVE', self.curve_speed_kmh
        else:
            speed_mode, speed = 'SHARP_CURVE', self.sharp_curve_speed_kmh
        return {
            'target_speed_kmh': float(speed),
            'representative_curvature': representative,
            'max_curvature': curvatures[-1],
            'evaluated_path_length_m': evaluated_length,
            'evaluated_point_count': len(evaluated),
            'speed_mode': speed_mode,
        }
