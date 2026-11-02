"""ROS-independent preview distance from measured vehicle speed."""

import math


def calculate_lookahead_distance(measured_speed_kmh, lookahead_min_m=2.5,
                                 lookahead_max_m=10.0,
                                 lookahead_preview_time_sec=0.45):
    """Return the clamped arc lookahead in metres, or None for invalid input.

    Speed is a nonnegative magnitude in km/h, matching EgoVehicleStatus handling
    in ControlNode. Invalid speed/configuration must use the existing safe stop.
    """
    values = (measured_speed_kmh, lookahead_min_m, lookahead_max_m,
              lookahead_preview_time_sec)
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        try:
            if not math.isfinite(value):
                return None
        except OverflowError:
            return None
    if (measured_speed_kmh < 0 or not 0 < lookahead_min_m <= lookahead_max_m
            or lookahead_preview_time_sec <= 0):
        return None

    v_mps = float(measured_speed_kmh) / 3.6
    preview_distance_m = v_mps * float(lookahead_preview_time_sec)
    return max(float(lookahead_min_m),
               min(float(lookahead_max_m), preview_distance_m))
