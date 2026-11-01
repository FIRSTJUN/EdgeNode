"""ROS-independent local path extraction from ordered global XYZ points."""

import math


_MAX_RESAMPLED_POINTS = 10000


def _finite_number(value):
    """Accept finite numeric coordinates, excluding booleans and overflow."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class LocalPathPlanner:
    """Track bounded, monotonic global-segment progress and sample ahead.

    The output starts at the selected projection, not at a nearest vertex.
    Original corners are retained; resampled indices never represent progress.
    """

    def extract(self, global_points, x, y, horizon_m=20.0,
                previous_index=None, search_back=3, search_forward=10,
                resample_spacing_m=2.0, previous_arc_m=None):
        """Return points and distance diagnostics, or None for invalid input.

        Validate the entire path before extraction. Project onto original XY
        segments within the inclusive previous-index window. Without previous
        progress, search the whole route once for mid-route initialization.
        Enforce nondecreasing cumulative XY arc, including within a segment.
        With previous_index alone, its original vertex is the initial arc.

        A forward junction must be reachable within the distance from previous
        projection to current pose. Skipping several segments also requires the
        projection to have passed each skipped segment end. This keeps a nearby
        future crossing or parallel return leg from stealing progress. An adjacent
        corner can hand off without requiring a forward-facing turn. Ties prefer earlier arc;
        shared-vertex ties prefer the next segment within the search window.

        Clip the forward polyline from that projection at the XY horizon;
        if the route ends sooner, never extend or pad it. Subdivide each
        retained segment to at most resample_spacing_m, keeping its original
        vertices and interpolating XYZ. Consecutive duplicate XY points are
        skipped, retaining the first Z. No smoothing or corner cutting occurs.
        ``nearest_index`` is the raw segment start index (the final raw vertex
        at route end). Return the explicit segment, fraction and global arc too.
        Callers must reset both previous_index and previous_arc_m on a route
        replacement. Unbounded output requests fail closed.
        """
        if not isinstance(global_points, (list, tuple)) or not global_points:
            return None
        if not all(_finite_number(value) for value in
                   (x, y, horizon_m, resample_spacing_m)):
            return None
        if horizon_m <= 0 or resample_spacing_m <= 0:
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

        segment_lengths = [
            math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            for a, b in zip(global_points, global_points[1:])
        ]
        cumulative = [0.0]
        vertex_sources = [0]
        for length in segment_lengths:
            arc = cumulative[-1] + length
            if not math.isfinite(length) or not math.isfinite(arc):
                return None
            cumulative.append(arc)
            vertex_sources.append(vertex_sources[-1] if length == 0.0 else len(vertex_sources))
        if previous_arc_m is not None:
            if (previous_index is None or not _finite_number(previous_arc_m)
                    or not cumulative[previous_index] <= previous_arc_m <=
                    cumulative[min(previous_index + 1, len(global_points) - 1)]):
                return None
        minimum_arc = (float(previous_arc_m) if previous_arc_m is not None else
                       cumulative[previous_index] if previous_index is not None else 0.0)
        if cumulative[-1] == 0.0:
            point = list(global_points[0])
            distance = math.hypot(float(point[0]) - float(x), float(point[1]) - float(y))
            if not math.isfinite(distance):
                return None
            return {'points': [point], 'nearest_index': previous_index or 0,
                    'nearest_distance_m': distance, 'path_length_m': 0.0,
                    'progress_segment_index': 0, 'progress_fraction': 0.0,
                    'progress_arc_m': 0.0}

        anchor = min(previous_index, len(segment_lengths) - 1) if previous_index is not None else 0
        start = max(0, anchor - search_back) if previous_index is not None else 0
        stop = (min(len(segment_lengths), anchor + search_forward + 1)
                if previous_index is not None else len(segment_lengths))
        advance_budget = math.inf
        if previous_index is not None:
            a, b = global_points[anchor:anchor + 2]
            fraction = ((minimum_arc - cumulative[anchor]) / segment_lengths[anchor]
                        if segment_lengths[anchor] > 0 else 0.0)
            previous_x = (1.0 - fraction) * float(a[0]) + fraction * float(b[0])
            previous_y = (1.0 - fraction) * float(a[1]) + fraction * float(b[1])
            advance_budget = math.hypot(float(x) - previous_x, float(y) - previous_y)
            if not math.isfinite(advance_budget):
                return None
        best = None
        can_skip_forward = True
        for index in range(start, stop):
            if previous_index is not None and index > anchor:
                junction_distance = cumulative[index] - minimum_arc
                if (junction_distance > advance_budget
                        and not math.isclose(junction_distance, advance_budget,
                                             rel_tol=1e-12, abs_tol=1e-9)):
                    break
            if previous_index is not None and index > anchor + 1 and not can_skip_forward:
                break
            length = segment_lengths[index]
            if length == 0.0:
                continue
            a, b = global_points[vertex_sources[index]], global_points[index + 1]
            ux = (float(b[0]) - float(a[0])) / length
            uy = (float(b[1]) - float(a[1])) / length
            along = (float(x) - float(a[0])) * ux + (float(y) - float(a[1])) * uy
            if not math.isfinite(along):
                return None
            if index >= anchor:
                can_skip_forward = can_skip_forward and along >= length
            if cumulative[index + 1] < minimum_arc:
                continue
            fraction = max(0.0, min(1.0, along / length),
                           (minimum_arc - cumulative[index]) / length)
            fraction = min(1.0, fraction)
            point = (list(a) if fraction == 0.0 else list(b) if fraction == 1.0 else
                     [(1.0 - fraction) * float(v0) + fraction * float(v1)
                      for v0, v1 in zip(a, b)])
            arc = max(minimum_arc, cumulative[index] + fraction * length)
            distance = math.hypot(float(x) - float(point[0]), float(y) - float(point[1]))
            if not all(math.isfinite(value) for value in (*point, arc, distance)):
                return None
            tied = best is not None and math.isclose(distance, best[0], rel_tol=1e-12, abs_tol=1e-9)
            if (best is None or (distance < best[0] and not tied)
                    or (tied and (arc < best[1]
                                  or (arc == best[1] and index > best[2])))):
                best = (distance, arc, index, fraction, point)
        if best is None:
            return None
        nearest_distance, progress_arc, segment_index, projection_fraction, projection = best
        nearest_index = (len(global_points) - 1
                         if projection_fraction == 1.0 and cumulative[segment_index + 1] == cumulative[-1]
                         else segment_index)
        points = [projection]
        path_length_m = 0.0
        for index in range(segment_index + 1, len(global_points)):
            length = math.hypot(float(global_points[index][0]) - float(points[-1][0]),
                                float(global_points[index][1]) - float(points[-1][1]))
            if length == 0.0:
                continue
            next_length = path_length_m + length
            if not math.isfinite(next_length):
                return None
            if next_length > horizon_m:
                fraction = (horizon_m - path_length_m) / length
                points.append([(1.0 - fraction) * float(a) + fraction * float(b)
                               for a, b in zip(points[-1], global_points[index])])
                break
            points.append(list(global_points[index]))
            path_length_m = next_length
            if path_length_m >= horizon_m:
                break

        sampled = [points[0].copy()]
        for a, b in zip(points, points[1:]):
            length = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            if length == 0.0:
                continue
            divisions = length / resample_spacing_m
            if not math.isfinite(divisions) or divisions > _MAX_RESAMPLED_POINTS:
                return None
            # Avoid an extra subdivision from roundoff at exact multiples.
            count = max(1, math.ceil(divisions - 1e-9))
            if len(sampled) + count > _MAX_RESAMPLED_POINTS:
                return None
            for step in range(1, count + 1):
                fraction = step / count
                point = (b.copy() if step == count else
                         [(1.0 - fraction) * float(v0) + fraction * float(v1)
                          for v0, v1 in zip(a, b)])
                if not all(math.isfinite(value) for value in point):
                    return None
                sampled.append(point)
        path_length_m = math.fsum(
            math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            for a, b in zip(sampled, sampled[1:])
        )
        return {
            'points': sampled,
            'nearest_index': nearest_index,
            'nearest_distance_m': nearest_distance,
            'path_length_m': path_length_m,
            'progress_segment_index': segment_index,
            'progress_fraction': projection_fraction,
            'progress_arc_m': progress_arc,
        }
