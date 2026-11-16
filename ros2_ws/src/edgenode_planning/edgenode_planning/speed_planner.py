"""ROS-independent two-mode speed targets from local path geometry.

This module only returns a suggestion; it does not publish vehicle commands.
"""

import math
from bisect import bisect_left, bisect_right
from collections import deque


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


def _coherent_turn_mean(curvatures, turns, positions, boundaries, path_length, window_m):
    """Retain short real map turns that a spatial median can erase.

    Average signed curvature by metres, only where the surrounding exact
    heading changes agree. A point spike turns out and back, so its cancelling
    heading changes must not acquire the support of a real road corner.
    The caller still requires the existing contiguous persistence distance.
    """
    result = []
    for position in positions:
        begin = bisect_left(positions, position - window_m)
        end = bisect_right(positions, position + window_m)
        absolute_turn = math.fsum(abs(value) for value in turns[begin:end])
        net_turn = abs(math.fsum(turns[begin:end]))
        if absolute_turn == 0. or net_turn < .5 * absolute_turn:
            result.append(0.)
            continue
        left = max(0., position - window_m / 2.)
        right = min(path_length, position + window_m / 2.)
        first = max(0, bisect_right(boundaries, left) - 1)
        stop = min(len(curvatures), bisect_left(boundaries, right))
        integral = math.fsum(curvatures[index] * (
            min(right, boundaries[index + 1]) - max(left, boundaries[index]))
            for index in range(first, stop))
        result.append(integral / (right - left))
    return result


def _supported_mask(values,boundaries,threshold,minimum_length,path_length):
    mask=[False]*len(values);begin=0
    while begin<len(values):
        if values[begin]<threshold:begin+=1;continue
        end=begin+1
        while end<len(values) and values[end]>=threshold:end+=1
        if boundaries[end]-boundaries[begin]>=min(minimum_length,path_length)-1e-9:
            mask[begin:end]=[True]*(end-begin)
        begin=end
    return mask


class SpeedPlanner:
    """Estimate XY curvature with a fixed preview and discrete speed profile.

    Invalid configuration is rejected by ``plan`` returning None, rather than
    raising from the constructor. All speeds are in km/h, distances in metres,
    and curvature thresholds in inverse metres.
    """

    def __init__(self, cruise_speed_kmh=60.0, curve_speed_kmh=None,
                 sharp_curve_speed_kmh=30.0, preview_distance_m=10.0,
                 curve_curvature_threshold=0.04, sharp_curvature_threshold=0.10,
                 curvature_smoothing_window_m=2.0, curvature_persistence_m=2.0,
                 curvature_hysteresis=0.005, anticipatory=False,
                 braking_deceleration_mps2=2., reaction_time_s=.6,
                 max_lateral_acceleration_mps2=1.5,
                 physical_lateral_acceleration_mps2=0.):
        self.cruise_speed_kmh = cruise_speed_kmh
        # Accepted for old callers only; ordinary curves use the NORMAL target.
        self.curve_speed_kmh = curve_speed_kmh
        self.sharp_curve_speed_kmh = sharp_curve_speed_kmh
        self.preview_distance_m = preview_distance_m
        self.curve_curvature_threshold = curve_curvature_threshold
        self.sharp_curvature_threshold = sharp_curvature_threshold
        self.curvature_smoothing_window_m = curvature_smoothing_window_m
        self.curvature_persistence_m = curvature_persistence_m
        self.curvature_hysteresis = curvature_hysteresis
        self.anticipatory=anticipatory
        self.braking_deceleration_mps2=braking_deceleration_mps2
        self.reaction_time_s=reaction_time_s
        self.max_lateral_acceleration_mps2=max_lateral_acceleration_mps2
        self.physical_lateral_acceleration_mps2=physical_lateral_acceleration_mps2

    def plan(self, local_path_points, previous_speed_mode=None, measured_speed_kmh=0.,
             measured_curvature=0.):
        """Return speed and geometry diagnostics, or None for unusable input.

        Validate every XYZ point, including points beyond the preview. Z never
        affects geometry. Remove consecutive identical XY points, then clip
        the polyline to the preview distance with a linearly interpolated end.
        The evaluated length must exceed 1e-6 m and contain at least one valid
        triplet. Triplets with any side at most 1e-9 m are skipped.

        Apply a distance-weighted spatial median to signed triplet curvature,
        then require its magnitude to persist over a contiguous path distance.
        The result is ``representative_curvature``; ``max_curvature`` stays raw.
        Raw p80 and the median-filtered maximum are also returned for debugging.
        An optional previous mode enables lower release thresholds; callers
        own that state, so this planner remains stateless and ROS-independent.
        """
        speeds = (self.cruise_speed_kmh, self.sharp_curve_speed_kmh)
        parameters = (*speeds, self.preview_distance_m,
                      self.curve_curvature_threshold, self.sharp_curvature_threshold,
                      self.curvature_smoothing_window_m, self.curvature_persistence_m,
                      self.curvature_hysteresis,self.physical_lateral_acceleration_mps2)
        if not all(_finite_number(value) for value in parameters):
            return None
        if any(speed < 0 for speed in speeds) or self.preview_distance_m <= 0:
            return None
        if not 0 <= self.curve_curvature_threshold < self.sharp_curvature_threshold:
            return None
        if (self.curvature_smoothing_window_m <= 0 or self.curvature_persistence_m <= 0
                or self.curvature_hysteresis < 0 or self.physical_lateral_acceleration_mps2<0):
            return None
        if previous_speed_mode not in (None, 'NORMAL', 'SHARP_CURVE'):
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
        turns = []
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
            turns.append(math.atan2(cross, dx1 * (p2[0] - p1[0]) + dy1 * (p2[1] - p1[1])))
            positions.append(sample_arc)
        if not curvatures:
            return None

        filtered, boundaries = _distance_median(
            curvatures, positions, evaluated_length, self.curvature_smoothing_window_m,
        )
        if self.anticipatory:
            turn_mean = _coherent_turn_mean(curvatures, turns, positions, boundaries,
                evaluated_length, self.curvature_smoothing_window_m)
            filtered = [mean if abs(mean) > abs(median) else median
                        for median, mean in zip(filtered, turn_mean)]
        filtered = [abs(value) for value in filtered]
        representative = _persistent_curvature(filtered, boundaries, self.curvature_persistence_m)
        sharp_release = max(0.0, self.sharp_curvature_threshold - self.curvature_hysteresis)
        threshold = sharp_release if previous_speed_mode == 'SHARP_CURVE' else self.sharp_curvature_threshold
        speed_mode = 'SHARP_CURVE' if representative >= threshold else 'NORMAL'
        speed = self.sharp_curve_speed_kmh if speed_mode == 'SHARP_CURVE' else self.cruise_speed_kmh
        ordered = sorted(abs(value) for value in curvatures)
        constraint_distance = 0.0
        profile_speed = speed
        constraint_reason='SHARP_CURVE' if speed_mode=='SHARP_CURVE' else 'NORMAL_TARGET'
        if self.anticipatory:
            if (not all(_finite_number(v) for v in (measured_speed_kmh, measured_curvature,
                    self.braking_deceleration_mps2, self.reaction_time_s))
                    or measured_speed_kmh < 0 or self.braking_deceleration_mps2 <= 0
                    or self.reaction_time_s < 0):
                return None
            mask=_supported_mask(filtered,boundaries,threshold,self.curvature_persistence_m,evaluated_length)
            physical=self.physical_lateral_acceleration_mps2
            physical_threshold=(physical/(self.sharp_curve_speed_kmh/3.6)**2
                if physical>0 and self.sharp_curve_speed_kmh>0 else math.inf)
            physical_mask=_supported_mask(filtered,boundaries,physical_threshold,
                self.curvature_persistence_m,evaluated_length)
            speed = profile_speed = self.cruise_speed_kmh
            speed_mode = 'NORMAL'
            constraint_reason='NORMAL_TARGET'
            reaction_distance = measured_speed_kmh / 3.6 * self.reaction_time_s
            for i, is_sharp in enumerate(mask):
                if not is_sharp and not physical_mask[i]:
                    continue
                limit = self.sharp_curve_speed_kmh
                reason='SHARP_CURVE'
                if physical_mask[i]:
                    physical_limit=3.6*math.sqrt(physical/filtered[i])
                    if physical_limit<limit:
                        limit=physical_limit;reason='LATERAL_FRICTION'
                distance = max(0.0, boundaries[i] - reaction_distance)
                allowed = 3.6 * math.sqrt((limit / 3.6) ** 2
                    + 2 * self.braking_deceleration_mps2 * distance)
                if allowed < speed:
                    speed, speed_mode = allowed, 'SHARP_CURVE'
                    profile_speed, constraint_distance = limit, boundaries[i]
                    constraint_reason=reason
            # Maintain the configured sharp target until actual steering unwinds.
            if abs(measured_curvature) >= threshold and self.sharp_curve_speed_kmh < speed:
                speed = profile_speed = self.sharp_curve_speed_kmh
                speed_mode, constraint_distance = 'SHARP_CURVE', 0.0
                constraint_reason='STEERING_UNWIND'
            if physical>0 and abs(measured_curvature)>physical_threshold:
                limit=3.6*math.sqrt(physical/abs(measured_curvature))
                if limit<speed:
                    speed=profile_speed=limit;speed_mode='SHARP_CURVE';constraint_distance=0.
                    constraint_reason='LATERAL_FRICTION'
        result = {
            'target_speed_kmh': float(speed),
            'representative_curvature': representative,
            'max_curvature': ordered[-1],
            'raw_representative_curvature': ordered[math.ceil(0.8 * len(ordered)) - 1],
            'filtered_max_curvature': max(filtered),
            'evaluated_path_length_m': evaluated_length,
            'evaluated_point_count': len(evaluated),
            'speed_mode': speed_mode,
            'braking_constraint_distance_m': constraint_distance,
            'constraint_speed_kmh': float(profile_speed),
        }
        if self.physical_lateral_acceleration_mps2>0:
            result.update(constraint_reason=constraint_reason,
                physical_lateral_acceleration_mps2=self.physical_lateral_acceleration_mps2)
        return result
