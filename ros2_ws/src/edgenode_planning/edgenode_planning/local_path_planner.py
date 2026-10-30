"""ROS-independent local path extraction from ordered global XYZ points."""

import math


def _finite_number(value):
    """Accept finite numeric coordinates, excluding booleans and overflow."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class LocalPathPlanner:
    """Select a forward slice starting at the nearest global path point.

    Distances use XY only. No interpolation, heading filter or smoothing is
    applied, and returned XYZ points are copies of the original values.
    """

    def extract(self, global_points, x, y, horizon_m=20.0,
                previous_index=None, search_back=3, search_forward=10):
        """Return points and distance diagnostics, or None for invalid input.

        Validate the entire path before extraction. Equal nearest distances
        select the first index. Include the first point reaching or exceeding
        the horizon; if the route ends sooner, return its remaining points.
        ``path_length_m`` is the cumulative XY length of the returned slice,
        excluding the vehicle-to-nearest-point distance.

        With no previous index, search the entire path so a vehicle can start
        mid-route. Otherwise search only the inclusive window around the
        previous index and never return a lower index. Thus a cycle advances
        by at most ``search_forward`` points. The caller owns progress and
        must pass None whenever the global route is replaced. No automatic
        full-path reacquisition occurs during tracking.
        """
        if not isinstance(global_points, (list, tuple)) or not global_points:
            return None
        if not all(_finite_number(value) for value in (x, y, horizon_m)):
            return None
        if horizon_m <= 0:
            return None
        for window in (search_back, search_forward):
            if isinstance(window, bool) or not isinstance(window, int) or window < 0:
                return None
        if previous_index is not None:
            if (isinstance(previous_index, bool) or not isinstance(previous_index, int)
                    or not 0 <= previous_index < len(global_points)):
                return None
        for point in global_points:
            if not isinstance(point, (list, tuple)) or len(point) != 3:
                return None
            if not all(_finite_number(value) for value in point):
                return None

        start = 0 if previous_index is None else max(0, previous_index - search_back)
        stop = (len(global_points) if previous_index is None
                else min(len(global_points), previous_index + search_forward + 1))
        distances = {
            index: math.hypot(float(global_points[index][0]) - float(x),
                              float(global_points[index][1]) - float(y))
            for index in range(start, stop)
        }
        segment_lengths = [
            math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            for a, b in zip(global_points, global_points[1:])
        ]
        if not all(math.isfinite(value) for value in (*distances.values(), *segment_lengths)):
            return None

        nearest_index = min(distances, key=distances.__getitem__)
        if previous_index is not None:
            nearest_index = max(previous_index, nearest_index)
        points = [list(global_points[nearest_index])]
        path_length_m = 0.0
        for index in range(nearest_index + 1, len(global_points)):
            path_length_m += segment_lengths[index - 1]
            if not math.isfinite(path_length_m):
                return None
            points.append(list(global_points[index]))
            if path_length_m >= horizon_m:
                break

        return {
            'points': points,
            'nearest_index': nearest_index,
            'nearest_distance_m': distances[nearest_index],
            'path_length_m': path_length_m,
        }
