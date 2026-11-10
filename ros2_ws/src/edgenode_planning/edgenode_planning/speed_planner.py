"""ROS-independent three-level speed suggestions from local path geometry.

This module only returns a suggestion; it does not publish vehicle commands.
"""

import math
from bisect import bisect_left, bisect_right
from collections import deque


_MIN_SIDE_LENGTH_M = 1e-9
_MIN_PATH_LENGTH_M = 1e-6
_HEADING_STEP_M = 0.25
_HEADING_LIMIT_M = 20.0


def _finite_number(value):
    """Accept finite numeric inputs, excluding booleans and integer overflow."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _distance_median(curvatures, positions, path_length, window_m):
    """Median by metres of support, so dense vertices do not outweigh a road.

    A sample owns the interval between neighbouring sample midpoints. The
    first/last samples extend to the preview ends; one valid triplet therefore
    describes the whole available path. No geometry is moved or resampled.
    """
    boundaries = [0.0] + [(a + b) / 2.0 for a, b in zip(positions, positions[1:])] + [path_length]
    filtered = []
    for index, position in enumerate(positions):
        left, right = max(0.0, position - window_m / 2.0), min(path_length, position + window_m / 2.0)
        first = max(0, bisect_right(boundaries, left) - 1)
        stop = min(len(curvatures), bisect_left(boundaries, right))
        weighted = []
        for sample in range(first, stop):
            weight = min(right, boundaries[sample + 1]) - max(left, boundaries[sample])
            if weight > 0:
                weighted.append((curvatures[sample], weight))
        ordered = sorted(weighted)
        half = math.fsum(weight for _, weight in weighted) / 2.0
        accumulated = 0.0
        median = curvatures[index]
        for rank, (value, weight) in enumerate(ordered):
            accumulated += weight
            if math.isclose(accumulated, half, rel_tol=1e-12, abs_tol=1e-15):
                median = (value + ordered[min(rank + 1, len(ordered) - 1)][0]) / 2.0
                break
            if accumulated >= half:
                median = value
                break
        filtered.append(median)
    return filtered, boundaries


def _persistent_curvature(curvatures, boundaries, minimum_length_m):
    """Largest curvature sustained over one contiguous distance window.

    Use a rolling minimum, then take the maximum across windows. Disconnected
    short spikes cannot add up to a curve. At the path end, require the whole
    available preview if it is shorter than the configured minimum.
    """
    required = min(minimum_length_m, boundaries[-1])
    tolerance = min(1e-9, required * 1e-9)
    minimums = deque()
    end = 0
    representative = 0.0
    for start in range(len(curvatures)):
        while end < len(curvatures) and boundaries[end] - boundaries[start] < required - tolerance:
            while minimums and curvatures[minimums[-1]] >= curvatures[end]:
                minimums.pop()
            minimums.append(end)
            end += 1
        if minimums and boundaries[end] - boundaries[start] >= required - tolerance:
            representative = max(representative, curvatures[minimums[0]])
        if minimums and minimums[0] == start:
            minimums.popleft()
    return representative


def _interpolate(positions, values, position):
    if position <= positions[0]:
        return values[0]
    if position >= positions[-1]:
        return values[-1]
    index = bisect_right(positions, position) - 1
    fraction = (position - positions[index]) / (positions[index + 1] - positions[index])
    return (1.0 - fraction) * values[index] + fraction * values[index + 1]


def _heading_trace(points, limit_m=_HEADING_LIMIT_M):
    """Measure heading on a physical grid without changing the driving path."""
    length = 0.0
    vertices, arcs = [points[0]], [0.0]
    limit_m = min(limit_m, _HEADING_LIMIT_M)
    for a, b in zip(points, points[1:]):
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        if distance <= 1e-10:
            continue
        if length + distance >= limit_m:
            fraction = (limit_m - length) / distance
            vertices.append(tuple((1.0 - fraction) * x + fraction * y for x, y in zip(a, b)))
            arcs.append(limit_m)
            length = limit_m
            break
        length += distance
        vertices.append(b)
        arcs.append(length)
    grid = [i * _HEADING_STEP_M for i in range(int((length + 1e-9) / _HEADING_STEP_M) + 1)]
    if length - grid[-1] > 0.05:
        grid.append(length)
    if len(grid) < 3:
        return None
    xs, ys = [p[0] for p in vertices], [p[1] for p in vertices]
    xy = [(_interpolate(arcs, xs, s), _interpolate(arcs, ys, s)) for s in grid]
    heading = []
    for a, b in zip(xy, xy[1:]):
        angle = math.atan2(b[1] - a[1], b[0] - a[0])
        if heading:
            angle = heading[-1] + math.remainder(angle - heading[-1], 2.0 * math.pi)
        heading.append(angle)
    turn = [b - a if abs(b - a) >= 1e-8 else 0.0 for a, b in zip(heading, heading[1:])]
    absolute = [0.0]
    for angle in turn:
        absolute.append(absolute[-1] + abs(angle))
    return {'grid': grid, 'xy': xy, 'positions': grid[1:-1], 'turn': turn,
            'heading': heading, 'hpos': [(a + b) / 2.0 for a, b in zip(grid, grid[1:])],
            'absolute': absolute, 'length': length}


def _heading_window(trace, left, right):
    net = (_interpolate(trace['hpos'], trace['heading'], right)
           - _interpolate(trace['hpos'], trace['heading'], left))
    total = (_interpolate(trace['hpos'], trace['absolute'], right)
             - _interpolate(trace['hpos'], trace['absolute'], left))
    return net, total


def _distributed_turn(positions, weights):
    """Reject an impulse even when its angle is large or its tails are long."""
    total = math.fsum(weights)
    if total < math.radians(8.0):
        return None
    cumulative = []
    value = 0.0
    for weight in weights:
        value += weight
        cumulative.append(value)
    first = min(len(positions) - 1, bisect_left(cumulative, total * 0.1))
    last = min(len(positions) - 1, bisect_left(cumulative, total * 0.9))
    if positions[last] - positions[first] < 2.0 - 1e-9:
        return None
    bins = {}
    for position, weight in zip(positions, weights):
        index = math.floor(position - positions[0])
        bins[index] = bins.get(index, 0.0) + weight
    if sum(weight >= math.radians(1.0) for weight in bins.values()) < 2:
        return None
    return first, last


def _heading_evidence(trace):
    best = {'curvature': 0.0, 'anchors': [], 'direction': 0, 'positions': []}
    if trace is None or trace['length'] < 4.0:
        return best
    for step in range(int((trace['length'] - 4.0 + 1e-9) / _HEADING_STEP_M) + 1):
        center = 2.0 + step * _HEADING_STEP_M
        width = min(8.0, 2.0 * center, 2.0 * (trace['length'] - center))
        net, total = _heading_window(trace, center - width / 2.0, center + width / 2.0)
        if total <= 1e-9:
            continue
        # A larger window may straddle both S-curve lobes; evaluate a single lobe.
        if abs(net) / total < 0.9:
            width = 4.0
            net, total = _heading_window(trace, center - 2.0, center + 2.0)
        if total <= 1e-9 or abs(net) / total < 0.9 or abs(net) < math.radians(8.0):
            continue
        left, right = center - width / 2.0, center + width / 2.0
        positions, weights, indices = [], [], []
        for index, angle in enumerate(trace['turn']):
            a, b = trace['hpos'][index:index + 2]
            weight = abs(angle) * max(0.0, min(right, b) - max(left, a)) / (b - a)
            if weight > 1e-8:
                positions.append(trace['positions'][index])
                weights.append(weight)
                indices.append(index)
        support = _distributed_turn(positions, weights) if positions else None
        curvature = abs(net) / width
        if support is not None and curvature > best['curvature']:
            first, last = support
            best = {'curvature': curvature, 'direction': 1 if net >= 0 else -1,
                    'anchors': [list(trace['xy'][index + 1]) for index in indices[first:last + 1]],
                    'positions': positions[first:last + 1]}
    return best


def _resolved_lobe(trace):
    if trace is None:
        return False
    groups = []
    for position, angle in zip(trace['positions'], trace['turn']):
        if abs(angle) <= 1e-8:
            continue
        sign = 1 if angle > 0 else -1
        if not groups or groups[-1][0] != sign:
            groups.append((sign, [], []))
        groups[-1][1].append(position)
        groups[-1][2].append(abs(angle))
    for _, positions, weights in groups:
        for start in range(len(positions)):
            stop = bisect_right(positions, positions[start] + 8.0 - _HEADING_STEP_M)
            for end in range(start + 2, stop + 1):
                ps, ws = positions[start:end], weights[start:end]
                if (_distributed_turn(ps, ws) is not None
                        and math.fsum(ws) / (ps[-1] - ps[0] + 2.0 * _HEADING_STEP_M) >= 0.025):
                    return True
    return False


def _outlier_intervals(trace):
    """Confirm a short excursion returning to the same line and tangent.

    This quality check can use up to 20m of available input, including points
    past the speed preview. It never creates curve evidence beyond that preview.
    """
    if trace is None or math.fsum(abs(x) for x in trace['turn']) < math.radians(60.0):
        return []
    grid, xy = trace['grid'], trace['xy']
    directions = [_interpolate(trace['hpos'], trace['heading'], s) for s in grid]
    absolute = [_interpolate(trace['hpos'], trace['absolute'], s) for s in grid]
    regions = []
    for i in range(2, len(grid) - 8):
        sine, cosine = math.sin(directions[i]), math.cos(directions[i])
        if max(abs(directions[i] - directions[i - k]) for k in (1, 2)) > math.radians(1.0):
            continue
        for j in range(i + 8, min(len(grid) - 2, i + 41)):
            dx, dy = xy[j][0] - xy[i][0], xy[j][1] - xy[i][1]
            angle = abs((directions[j] - directions[i] + math.pi) % (2.0 * math.pi) - math.pi)
            if (abs(-sine * dx + cosine * dy) > 0.08 or angle > math.radians(2.0)
                    or absolute[j] - absolute[i] < math.radians(60.0)):
                continue
            if max(abs(directions[j + k] - directions[j]) for k in (1, 2)) > math.radians(1.0):
                continue
            deviation = max(abs(-sine * (p[0] - xy[i][0]) + cosine * (p[1] - xy[i][1]))
                            for p in xy[i:j + 1])
            signs = [1 if angle > 0 else -1 for position, angle in zip(trace['positions'], trace['turn'])
                     if grid[i] <= position <= grid[j] and abs(angle) > math.radians(0.1)]
            reversals = sum(a != b for a, b in zip(signs, signs[1:]))
            if deviation < 0.3 and reversals < 4:
                continue
            if _resolved_lobe(_heading_trace(xy[i:j + 1])):
                continue
            regions.append((grid[i], grid[j]))
            break
    merged = []
    for start, end in sorted(regions):
        if merged and start <= merged[-1][1] + _HEADING_STEP_M:
            merged[-1] = (merged[-1][0], max(end, merged[-1][1]))
        else:
            merged.append((start, end))
    return merged


def _valid_curve_state(state):
    if state is None:
        return True
    if not isinstance(state, (list, tuple)) or len(state) > 8:
        return False
    for event in state:
        if (not isinstance(event, dict) or type(event.get('strength')) is not int
                or event['strength'] not in (1, 2) or type(event.get('direction')) is not int
                or event['direction'] not in (-1, 1)):
            return False
        anchors = event.get('anchors')
        if not isinstance(anchors, (list, tuple)) or not 1 <= len(anchors) <= 256:
            return False
        if not all(isinstance(p, (list, tuple)) and len(p) == 2
                   and all(_finite_number(v) for v in p) for p in anchors):
            return False
    return True


def _forward_anchors(anchors, points):
    segments, arc = [], 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        squared = dx * dx + dy * dy
        if squared > 1e-12:
            length = math.sqrt(squared)
            segments.append((a, dx, dy, squared, length, arc))
            arc += length
    remaining = []
    for x, y in anchors:
        best = (math.inf, 0.0)
        for a, dx, dy, squared, length, arc in segments:
            fraction = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / squared))
            distance = math.hypot(x - a[0] - fraction * dx, y - a[1] - fraction * dy)
            if distance < best[0]:
                best = (distance, arc + fraction * length)
        if best[0] <= 1e-5 and best[1] > 1e-5:
            remaining.append([x, y])
    return remaining


def _speed_level(curvature, previous_mode, curve, sharp, hysteresis):
    if curvature >= sharp or (previous_mode == 'SHARP_CURVE' and curvature >= max(0.0, sharp - hysteresis)):
        return 2
    if curvature >= curve or (previous_mode in ('CURVE', 'SHARP_CURVE') and curvature >= max(0.0, curve - hysteresis)):
        return 1
    return 0


def _track_curve(points, trace, cue, previous_state, base_level, cue_level, current_level, threshold):
    """Hold only a rescued, still-visible curve; never delay an upward change."""
    active = []
    if trace is not None and math.fsum(abs(x) for x in trace['turn']) >= math.radians(6.0):
        for event in previous_state or []:
            anchors = _forward_anchors(event['anchors'], points)
            if anchors:
                active.append(dict(event, anchors=anchors))
    if cue['curvature'] >= threshold and cue['anchors']:
        matches = [i for i, event in enumerate(active) if event['direction'] == cue['direction']
                   and min(math.hypot(a[0] - b[0], a[1] - b[1])
                           for a in event['anchors'] for b in cue['anchors']) <= 0.75]
        if matches:
            strength = max([current_level] + [active[i]['strength'] for i in matches])
            merged = [p for i in matches for p in active[i]['anchors']] + cue['anchors']
            buckets = {}
            for point in reversed(merged):
                buckets[tuple(round(v / _HEADING_STEP_M) for v in point)] = point
            active = [event for i, event in enumerate(active) if i not in matches]
            active.append({'anchors': list(buckets.values()), 'strength': strength, 'direction': cue['direction']})
        elif cue_level > base_level:
            active.append({'anchors': cue['anchors'], 'strength': current_level, 'direction': cue['direction']})
    active = sorted(active, key=lambda event: event['strength'], reverse=True)[:8]
    return max([current_level] + [event['strength'] for event in active]), active


def curve_support(points, planner):
    """Locate support in a validated <=20m context, without temporal state.

    Local ``plan`` stays unchanged. Level zero support supplies lateral limits
    on gentle bends below the mode thresholds. Evidence is not a probability.
    """
    if len(points) < 3:
        return []
    xy = [tuple(p[:2]) for p in points]
    arcs = [0.0]
    for a, b in zip(xy, xy[1:]):
        arcs.append(arcs[-1] + math.dist(a, b))
    curvatures, positions = [], []
    for i in range(1, len(xy)-1):
        p0, p1, p2 = xy[i-1:i+2]
        a, b, c = math.dist(p0,p1), math.dist(p1,p2), math.dist(p0,p2)
        if min(a,b,c) <= _MIN_SIDE_LENGTH_M:
            continue
        cross = ((p1[0]-p0[0])*(p2[1]-p0[1])-(p1[1]-p0[1])*(p2[0]-p0[0]))
        curvatures.append(2.0*cross/(a*b*c))
        positions.append(arcs[i])
    if not curvatures:
        return []
    filtered, boundaries = _distance_median(
        curvatures,positions,arcs[-1],planner.curvature_smoothing_window_m)
    trace = _heading_trace(xy)
    cue = _heading_evidence(trace)
    regions = _outlier_intervals(trace)
    supported = [0.0 if any(a <= s <= b for a,b in regions) else k
                 for s,k in zip(positions,filtered)]
    result = []
    for level,threshold in ((0,1e-6),(1,planner.curve_curvature_threshold),
                            (2,planner.sharp_curvature_threshold)):
        first = None
        for i in range(len(supported)+1):
            yes = i < len(supported) and abs(supported[i]) >= max(1e-6,threshold)
            if yes and first is None:
                first = i
            if not yes and first is not None:
                if (boundaries[i]-boundaries[first]
                        >= min(planner.curvature_persistence_m,arcs[-1])-1e-9):
                    for j in range(first,i):
                        k = supported[j]
                        result.append((boundaries[j],boundaries[j+1],level,
                                       abs(k),1 if k > 0 else -1,1))
                first = None
    if cue['positions'] and cue['curvature'] >= max(1e-6,planner.curve_curvature_threshold):
        ps = [s for s in cue['positions'] if not any(a <= s <= b for a,b in regions)]
        if ps:
            result.append((max(0.0,min(ps)-_HEADING_STEP_M/2),
                           min(arcs[-1],max(ps)+_HEADING_STEP_M/2),
                           2 if cue['curvature'] >= planner.sharp_curvature_threshold else 1,
                           cue['curvature'],cue['direction'],2))
    return result


class SpeedPlanner:
    """Estimate XY curvature with a fixed preview and discrete speed profile.

    Invalid configuration is rejected by ``plan`` returning None, rather than
    raising from the constructor. All speeds are in km/h, distances in metres,
    and curvature thresholds in inverse metres.
    """

    def __init__(self, cruise_speed_kmh=1.0, curve_speed_kmh=0.7,
                 sharp_curve_speed_kmh=0.5, preview_distance_m=10.0,
                 curve_curvature_threshold=0.04, sharp_curvature_threshold=0.10,
                 curvature_smoothing_window_m=2.0, curvature_persistence_m=2.0,
                 curvature_hysteresis=0.005):
        self.cruise_speed_kmh = cruise_speed_kmh
        self.curve_speed_kmh = curve_speed_kmh
        self.sharp_curve_speed_kmh = sharp_curve_speed_kmh
        self.preview_distance_m = preview_distance_m
        self.curve_curvature_threshold = curve_curvature_threshold
        self.sharp_curvature_threshold = sharp_curvature_threshold
        self.curvature_smoothing_window_m = curvature_smoothing_window_m
        self.curvature_persistence_m = curvature_persistence_m
        self.curvature_hysteresis = curvature_hysteresis

    def plan(self, local_path_points, previous_speed_mode=None, previous_curve_state=None):
        """Return speed and geometry diagnostics, or None for unusable input.

        Validate every XYZ point, including points beyond the preview. Z never
        affects geometry. Remove consecutive identical XY points, then clip
        the polyline to the preview distance with a linearly interpolated end.
        The evaluated length must exceed 1e-6 m and contain at least one valid
        triplet. Triplets with any side at most 1e-9 m are skipped.

        Apply a distance-weighted spatial median to signed triplet curvature,
        then require its magnitude to persist over a contiguous path distance.
        Keep that estimate as ``legacy_representative_curvature``. The final
        ``representative_curvature`` also includes heading evidence and a held
        curve floor; ``max_curvature`` stays raw. Raw p80 and the median-filtered
        maximum remain available for debugging.
        Heading evidence supplements the existing estimate. Confirmed compact
        out-and-return artifacts are excluded from curvature support. Optional
        curve state retains the severity of a rescued curve only while its
        anchors remain ahead; callers own it, so there is no hidden state.
        """
        speeds = (self.cruise_speed_kmh, self.curve_speed_kmh, self.sharp_curve_speed_kmh)
        parameters = (*speeds, self.preview_distance_m,
                      self.curve_curvature_threshold, self.sharp_curvature_threshold,
                      self.curvature_smoothing_window_m, self.curvature_persistence_m,
                      self.curvature_hysteresis)
        if not all(_finite_number(value) for value in parameters):
            return None
        if any(speed < 0 for speed in speeds) or self.preview_distance_m <= 0:
            return None
        if not 0 <= self.curve_curvature_threshold < self.sharp_curvature_threshold:
            return None
        if (self.curvature_smoothing_window_m <= 0 or self.curvature_persistence_m <= 0
                or self.curvature_hysteresis < 0):
            return None
        if previous_speed_mode not in (None, 'CRUISE', 'CURVE', 'SHARP_CURVE'):
            return None
        if not _valid_curve_state(previous_curve_state):
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
        positions = []
        sample_arc = 0.0
        for p0, p1, p2 in zip(evaluated, evaluated[1:], evaluated[2:]):
            dx1, dy1 = p1[0] - p0[0], p1[1] - p0[1]
            dx2, dy2 = p2[0] - p0[0], p2[1] - p0[1]
            a = math.hypot(dx1, dy1)
            b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            c = math.hypot(dx2, dy2)
            if not all(math.isfinite(side) for side in (a, b, c)):
                return None
            sample_arc += a
            if min(a, b, c) <= _MIN_SIDE_LENGTH_M:
                continue
            cross = dx1 * dy2 - dy1 * dx2
            denominator = a * b * c
            if not math.isfinite(cross) or not math.isfinite(denominator):
                return None
            curvature = 2.0 * cross / denominator
            if not math.isfinite(curvature):
                return None
            curvatures.append(curvature)
            positions.append(sample_arc)
        if not curvatures:
            return None

        filtered, boundaries = _distance_median(
            curvatures, positions, evaluated_length, self.curvature_smoothing_window_m,
        )
        filtered = [abs(value) for value in filtered]
        legacy = _persistent_curvature(filtered, boundaries, self.curvature_persistence_m)
        trace = _heading_trace(evaluated, self.preview_distance_m)
        cue = _heading_evidence(trace)
        regions = []
        quality_trace = None
        if max(legacy, cue['curvature']) >= max(0.0, self.curve_curvature_threshold - self.curvature_hysteresis):
            quality_trace = _heading_trace(points)
            regions = _outlier_intervals(quality_trace)
        supported = [0.0 if any(a <= s <= b for a, b in regions) else k
                     for s, k in zip(positions, filtered)]
        quality_curvature = _persistent_curvature(supported, boundaries, self.curvature_persistence_m)
        if cue['positions'] and all(any(a <= s <= b for a, b in regions) for s in cue['positions']):
            cue = {'curvature': 0.0, 'anchors': [], 'direction': 0, 'positions': []}
        representative = max(quality_curvature, cue['curvature'])
        def level(value):
            return _speed_level(value, previous_speed_mode, self.curve_curvature_threshold,
                                self.sharp_curvature_threshold, self.curvature_hysteresis)
        speed_level, curve_state = _track_curve(
            points, trace, cue, previous_curve_state, level(quality_curvature),
            level(cue['curvature']), level(representative), self.curve_curvature_threshold,
        )
        if curve_state:
            held = max(event['strength'] for event in curve_state)
            representative = max(representative, (0.0, self.curve_curvature_threshold,
                                                   self.sharp_curvature_threshold)[held])
        speed_mode = ('CRUISE', 'CURVE', 'SHARP_CURVE')[speed_level]
        speed = speeds[speed_level]
        ordered = sorted(abs(value) for value in curvatures)
        return {
            'target_speed_kmh': float(speed),
            'representative_curvature': representative,
            'max_curvature': ordered[-1],
            'raw_representative_curvature': ordered[math.ceil(0.8 * len(ordered)) - 1],
            'filtered_max_curvature': max(filtered),
            'evaluated_path_length_m': evaluated_length,
            'evaluated_point_count': len(evaluated),
            'speed_mode': speed_mode,
            'legacy_representative_curvature': legacy,
            'heading_curvature': cue['curvature'],
            'quality_filtered_curvature': quality_curvature,
            'quality_rejected_samples': sum(a != b for a, b in zip(filtered, supported)),
            'heading_evaluated_length_m': trace['length'] if trace else 0.0,
            'quality_context_length_m': quality_trace['length'] if quality_trace else 0.0,
            'curve_state': curve_state,
        }
