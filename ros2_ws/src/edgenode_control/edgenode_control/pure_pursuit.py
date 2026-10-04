"""ROS-independent Pure Pursuit with a fixed arc-length lookahead.

Positive steering means LEFT; negative means RIGHT. MORAI command conversion
belongs to the future Control integration and is deliberately absent here.
"""

import math


_DEGENERATE_LENGTH_M = 1e-9


def _finite_number(value):
    """Accept finite numbers, excluding booleans and overflowing integers."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class PurePursuit:
    """Stateless steering calculation using ERP42 initial dimensions.

    Invalid dimensions are handled by ``compute`` returning None, so neither
    malformed configuration nor path/pose data raises a validation exception.
    """

    def __init__(self, wheelbase_m=1.04, max_wheel_angle_rad=0.49):
        self.wheelbase_m = wheelbase_m
        self.max_wheel_angle_rad = max_wheel_angle_rad

    def compute(self, local_path_points, vehicle_x, vehicle_y, vehicle_yaw_rad,
                lookahead_distance_m=2.5):
        """Return target/steering diagnostics, or None for unusable input.

        Project onto every XY segment and use the closest projection as the
        current arc position. Ties select the first segment in path order.
        Advance by the fixed lookahead, capped at the path end, and interpolate
        XYZ for the target. Z is validated and retained only as path data; all
        distances and steering use XY. Duplicate XY segments contribute zero
        length and are skipped. Paths with no segment longer than 1e-9 m fail.

        The input is never mutated. Target arc and current arc are measured
        from the first path point; target distance is the actual XY straight
        line distance from the vehicle, which may differ from the lookahead.
        """
        values = (vehicle_x, vehicle_y, vehicle_yaw_rad, lookahead_distance_m,
                  self.wheelbase_m, self.max_wheel_angle_rad)
        if not all(_finite_number(value) for value in values):
            return None
        if (self.wheelbase_m <= 0 or self.max_wheel_angle_rad <= 0
                or lookahead_distance_m <= 0):
            return None
        if not isinstance(local_path_points, (list, tuple)) or len(local_path_points) < 2:
            return None
        for point in local_path_points:
            if not isinstance(point, (list, tuple)) or len(point) != 3:
                return None
            if not all(_finite_number(value) for value in point):
                return None

        points = [[float(value) for value in point] for point in local_path_points]
        x, y, yaw = float(vehicle_x), float(vehicle_y), float(vehicle_yaw_rad)
        lengths = []
        cumulative = [0.0]
        for a, b in zip(points, points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            arc = cumulative[-1] + length
            if not math.isfinite(length) or not math.isfinite(arc):
                return None
            lengths.append(length)
            cumulative.append(arc)
        if max(lengths) <= _DEGENERATE_LENGTH_M:
            return None

        nearest_distance = math.inf
        current_arc = None
        nearest_segment_index = None
        for index, length in enumerate(lengths):
            if length == 0.0:
                continue
            a, b = points[index:index + 2]
            unit_x = (b[0] - a[0]) / length
            unit_y = (b[1] - a[1]) / length
            along = (x - a[0]) * unit_x + (y - a[1]) * unit_y
            if not math.isfinite(along):
                return None
            along = max(0.0, min(length, along))
            fraction = along / length
            projection_x = (1.0 - fraction) * a[0] + fraction * b[0]
            projection_y = (1.0 - fraction) * a[1] + fraction * b[1]
            distance = math.hypot(x - projection_x, y - projection_y)
            if not math.isfinite(distance):
                return None
            if distance < nearest_distance:
                nearest_distance = distance
                current_arc = cumulative[index] + along
                nearest_segment_index = index

        if current_arc is None:
            return None
        total_length = cumulative[-1]
        target_arc = current_arc + min(float(lookahead_distance_m), total_length - current_arc)
        target = None
        target_segment_index = None
        if target_arc >= total_length:
            target = points[-1].copy()
            target_segment_index = len(lengths) - 1
        else:
            for index, length in enumerate(lengths):
                if length == 0.0 or cumulative[index + 1] < target_arc:
                    continue
                fraction = max(0.0, min(1.0, (target_arc - cumulative[index]) / length))
                target = [
                    (1.0 - fraction) * a + fraction * b
                    for a, b in zip(points[index], points[index + 1])
                ]
                target_segment_index = index
                break

        if target is None:
            return None
        dx, dy = target[0] - x, target[1] - y
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        left = -sin_yaw * dx + cos_yaw * dy
        ld2 = dx * dx + dy * dy
        if not all(math.isfinite(value) for value in (*target, target_arc, forward, left, ld2)):
            return None
        if forward <= 0.1 or ld2 <= 0.0:
            return None

        curvature = 2.0 * left / ld2
        wheel_curvature = float(self.wheelbase_m) * curvature
        if not math.isfinite(curvature) or not math.isfinite(wheel_curvature):
            return None
        max_angle = float(self.max_wheel_angle_rad)
        angle = max(-max_angle, min(max_angle, math.atan(wheel_curvature)))
        normalized = max(-1.0, min(1.0, angle / max_angle))
        return {
            'target_point': target,
            'target_arc_m': target_arc,
            'target_distance_m': math.sqrt(ld2),
            'target_forward_m': forward,
            'target_left_m': left,
            'nearest_distance_m': nearest_distance,
            'current_arc_m': current_arc,
            'curvature': curvature,
            'steering_angle_rad': angle,
            'steering_normalized': normalized,
            'nearest_segment_index': nearest_segment_index,
            'target_segment_index': target_segment_index,
        }
