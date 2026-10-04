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

    def extract(self, global_points, x, y, horizon_m=20.0):
        """Return points and distance diagnostics, or None for invalid input.

        Validate the entire path before extraction. Equal nearest distances
        select the first index. Include the first point reaching or exceeding
        the horizon; if the route ends sooner, return its remaining points.
        ``path_length_m`` is the cumulative XY length of the returned slice,
        excluding the vehicle-to-nearest-point distance.
        """
        if not isinstance(global_points, (list, tuple)) or not global_points:
            return None
        if not all(_finite_number(value) for value in (x, y, horizon_m)):
            return None
        if horizon_m <= 0:
            return None
        for point in global_points:
            if not isinstance(point, (list, tuple)) or len(point) != 3:
                return None
            if not all(_finite_number(value) for value in point):
                return None

        distances = [
            math.hypot(float(point[0]) - float(x), float(point[1]) - float(y))
            for point in global_points
        ]
        segment_lengths = [
            math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            for a, b in zip(global_points, global_points[1:])
        ]
        if not all(math.isfinite(value) for value in distances + segment_lengths):
            return None

        nearest_index = min(range(len(distances)), key=distances.__getitem__)
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
