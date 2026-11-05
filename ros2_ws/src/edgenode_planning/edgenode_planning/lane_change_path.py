# Portions Copyright 2023 TIER IV, Inc. (path_shifter.cpp)
# Portions Copyright 2021 Tier IV, Inc. (path_shifter.hpp)
# SPDX-License-Identifier: Apache-2.0
#
# Mathematical adaptation from Autoware Universe (Apache-2.0), commit
# 1fc25fcec6219f39d6ae89c36b0720951c45173a:
# planning/behavior_path_planner/autoware_behavior_path_planner_common/src/
# utils/path_shifter/path_shifter.cpp, get_base_lengths_without_accel_limit().
# Its header documents constant-jerk spline shifting. Only the unbounded-
# lateral-acceleration, constant-longitudinal-speed shape is adapted here.
# License: ../docs/autoware_path_shifter_LICENSE.md
"""One-shot spatial lateral shift on a plain reference polyline; no ROS state."""
from bisect import bisect_right
import math

from .lane_change_types import LaneCenterlinePath, LaneChangePath
from .lattice_planner import LatticePlanner


_MAX_POINTS = 10000
_MAX_SPACING_M = 0.25


class LaneChangePathError(ValueError):
    def __init__(self, reason, detail):
        super().__init__(detail)
        self.reason = reason


def _finite(value):
    try:
        return (not isinstance(value, bool) and isinstance(value, (int, float))
                and math.isfinite(value))
    except OverflowError:
        return False


def steering_curvature_limit(wheelbase_m=2.84, max_wheel_angle_rad=0.610865,
                             max_front_steer_normalized=0.70):
    """Existing Sonata command-geometry baseline; not a tyre/dynamics guarantee."""
    if (not all(_finite(v) for v in
                (wheelbase_m, max_wheel_angle_rad, max_front_steer_normalized))
            or wheelbase_m <= 0 or max_wheel_angle_rad <= 0
            or not 0 < max_front_steer_normalized <= 1
            or max_wheel_angle_rad * max_front_steer_normalized >= math.pi / 2):
        raise ValueError('invalid steering geometry')
    return math.tan(max_wheel_angle_rad * max_front_steer_normalized) / wheelbase_m


DEFAULT_CURVATURE_LIMIT = steering_curvature_limit()


def constant_jerk_profile(u):
    """Return normalized shift, first and second spatial derivatives.

    Autoware's base knots are (0,0), (1/4,1/12), (3/4,11/12), (1,1).
    Integrating normalized jerk +32, -32, +32 across those intervals gives
    this C2 piecewise cubic, with zero first/second derivatives at both ends.
    Reflecting the second half avoids cancellation near the terminal offset.
    """
    if not _finite(u):
        raise ValueError('profile parameter must be finite')
    if u <= 0:
        return 0.0, 0.0, 0.0
    if u >= 1:
        return 1.0, 0.0, 0.0
    if u > 0.5:
        value, first, second = constant_jerk_profile(1.0 - u)
        return 1.0 - value, first, -second
    if u <= 0.25:
        return 16.0/3.0*u**3, 16.0*u*u, 32.0*u
    v = u - 0.25
    return (16.0/3.0*(u**3 - 2.0*v**3),
            16.0*(u*u - 2.0*v*v), 32.0*(u - 2.0*v))


def _reference(points):
    if not isinstance(points, (list, tuple)) or not 2 <= len(points) <= _MAX_POINTS:
        raise LaneChangePathError('INVALID_REFERENCE', 'reference needs 2..10000 XY points')
    clean = []
    for point in points:
        if (not isinstance(point, (list, tuple)) or len(point) not in (2, 3)
                or not all(_finite(v) for v in point)):
            raise LaneChangePathError('INVALID_REFERENCE', 'finite XY or XYZ points required')
        xy = (float(point[0]), float(point[1]))
        if not clean or xy != clean[-1]:
            clean.append(xy)
    if len(clean) < 2:
        raise LaneChangePathError('INVALID_REFERENCE', 'reference has no nonzero segment')
    lengths, tangents = [0.0], []
    for a, b in zip(clean, clean[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if not math.isfinite(length) or length < 1e-6:
            raise LaneChangePathError('INVALID_REFERENCE', 'unresolvable reference segment')
        lengths.append(lengths[-1] + length)
        tangents.append((dx/length, dy/length))
    if not math.isfinite(lengths[-1]):
        raise LaneChangePathError('INVALID_REFERENCE', 'reference arc overflows')
    normals = [(-t[1], t[0]) for t in tangents]
    joins = [normals[0]]
    for before, after, n0, n1 in zip(tangents, tangents[1:], normals, normals[1:]):
        denominator = 1.0 + before[0]*after[0] + before[1]*after[1]
        if denominator <= 1e-6:
            raise LaneChangePathError('REFERENCE_REVERSAL', 'reference reverses at a vertex')
        join = ((n0[0]+n1[0])/denominator, (n0[1]+n1[1])/denominator)
        if math.hypot(*join) > 4.0:
            raise LaneChangePathError('REFERENCE_REVERSAL', 'reference corner has an excessive miter')
        joins.append(join)
    joins.append(normals[-1])
    return clean, lengths, tangents, joins


def _project_start(points, lengths, position):
    if (not isinstance(position, (list, tuple)) or len(position) != 2
            or not all(_finite(v) for v in position)):
        raise LaneChangePathError('INVALID_START', 'start_position needs finite XY')
    best = (math.inf, math.inf)
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = lengths[i+1]-lengths[i]
        along = max(0.0, min(length,
            ((position[0]-a[0])*dx + (position[1]-a[1])*dy)/length))
        distance = math.hypot(position[0]-a[0]-along*dx/length,
                              position[1]-a[1]-along*dy/length)
        best = min(best, (distance, lengths[i]+along))
    return best[1]


def prepare_target_centerline(reference_points, *, curvature_limit_m_inv=DEFAULT_CURVATURE_LIMIT):
    """Copy a target XY/XYZ polyline once, without lateral shifting or refitting."""
    try:
        points, _, _, _ = _reference(reference_points)
    except LaneChangePathError as error:
        raise LaneChangePathError('INVALID_TARGET_REFERENCE', str(error)) from error
    if not _finite(curvature_limit_m_inv) or curvature_limit_m_inv <= 0:
        raise LaneChangePathError('INVALID_TARGET_REFERENCE', 'invalid target curvature limit')
    curvature = LatticePlanner.maximum_curvature(points)
    if not math.isfinite(curvature) or curvature > curvature_limit_m_inv:
        raise LaneChangePathError('TARGET_CURVATURE_LIMIT', 'target centerline exceeds steering feasibility')
    return LaneCenterlinePath(tuple(points), curvature, float(curvature_limit_m_inv))


def generate_shift_path(reference_points, *, end_s_m, lateral_offset_m,
                        start_s_m=None, start_position=None, sample_spacing_m=0.25,
                        curvature_limit_m_inv=DEFAULT_CURVATURE_LIMIT):
    """Generate once; positive offset is LEFT, negative is RIGHT.

    Stations are XY arc length along the original reference, not shifted-path
    arc. Supply start_s_m OR an XY start_position projected onto that reference.
    Projection selects a station only; this does not fit an off-center pose/yaw.
    Prefix shift is zero and the terminal offset persists to the reference end.
    Native vertices are retained; miter joins keep an offset polyline connected.
    No map smoothing or arbitrary adjacent-center correspondence is inferred.
    """
    points, lengths, tangents, joins = _reference(reference_points)
    if start_s_m is None:
        start_s_m = _project_start(points, lengths, start_position)
    elif start_position is not None:
        raise LaneChangePathError('INVALID_START', 'choose start_s_m or start_position')
    if (not all(_finite(v) for v in (start_s_m, end_s_m, lateral_offset_m,
                                     sample_spacing_m, curvature_limit_m_inv))
            or not 0 <= start_s_m < end_s_m <= lengths[-1]
            or not 0 < sample_spacing_m <= _MAX_SPACING_M
            or curvature_limit_m_inv <= 0 or lateral_offset_m == 0):
        raise LaneChangePathError('INVALID_SHIFT', 'invalid station range, offset or sampling/curvature limit')
    start, end, offset = float(start_s_m), float(end_s_m), float(lateral_offset_m)
    span = end - start
    derivative_bound = 8.0 * abs(offset) / span / span
    # On a straight reference |kappa|=|d''|/(1+d'^2)^(3/2) <= |d''|.
    # Keep this conservative length guard even on curved references; the full
    # sampled XY curvature is checked separately. Coarse input cannot hide a
    # transition that is much too short.
    if derivative_bound > curvature_limit_m_inv:
        raise LaneChangePathError('TRANSITION_TOO_SHORT',
            'shift second-derivative bound exceeds the steering curvature limit')
    if lengths[-1] / sample_spacing_m > _MAX_POINTS-1:
        raise LaneChangePathError('RESOURCE_LIMIT', 'reference exceeds the sampling budget')
    for i, tangent in enumerate(tangents):
        low, high = (lengths[i]-start)/span, (lengths[i+1]-start)/span
        slope = max(constant_jerk_profile(low)[1], constant_jerk_profile(high)[1],
                    2.0 if low <= 0.5 <= high else 0.0) * abs(offset)/span
        lateral = abs(offset) * constant_jerk_profile(high)[0]
        before = sum(a*b for a,b in zip(joins[i], tangent))
        after = sum(a*b for a,b in zip(joins[i+1], tangent))
        normal_gradient = abs(after-before)/(lengths[i+1]-lengths[i])
        # Conservative lower bound on P'(s) projected onto this segment.
        if 1.0-slope*max(abs(before),abs(after))-lateral*normal_gradient <= 0:
            raise LaneChangePathError('DIRECTION_CONTINUITY', 'continuous offset path may fold')
    anchors = sorted(set(lengths + [start, start+span/4, start+span/2, start+3*span/4, end]))
    stations = [anchors[0]]
    for a, b in zip(anchors, anchors[1:]):
        count = math.ceil((b-a)/sample_spacing_m)
        if len(stations) + count > _MAX_POINTS:
            raise LaneChangePathError('RESOURCE_LIMIT', 'generated path exceeds 10000 points')
        stations.extend(a + (b-a)*i/count for i in range(1, count))
        stations.append(b)
    shifted, offsets = [], []
    for station in stations:
        i = min(len(tangents)-1, bisect_right(lengths, station)-1)
        fraction = (station-lengths[i])/(lengths[i+1]-lengths[i])
        base = tuple(points[i][k] + fraction*(points[i+1][k]-points[i][k]) for k in range(2))
        normal = tuple(joins[i][k] + fraction*(joins[i+1][k]-joins[i][k]) for k in range(2))
        lateral = offset * constant_jerk_profile((station-start)/span)[0]
        xy = tuple(base[k] + lateral*normal[k] for k in range(2))
        if not all(math.isfinite(v) for v in xy):
            raise LaneChangePathError('NONFINITE_PATH', 'shifted coordinates overflow')
        shifted.append(xy)
        offsets.append(lateral)
    for s0, s1, a, b in zip(stations, stations[1:], shifted, shifted[1:]):
        i = min(len(tangents)-1, bisect_right(lengths, (s0+s1)/2)-1)
        forward = sum((b[k]-a[k])*tangents[i][k] for k in range(2))
        if forward <= 1e-9:
            raise LaneChangePathError('DIRECTION_CONTINUITY', 'shifted path folds or has a degenerate step')
    curvature = LatticePlanner.maximum_curvature(shifted)
    if not math.isfinite(curvature) or curvature > curvature_limit_m_inv:
        raise LaneChangePathError('CURVATURE_LIMIT', 'sampled XY curvature exceeds steering feasibility')
    return LaneChangePath(tuple(shifted), tuple(stations), tuple(offsets), start, end,
                          offset, curvature, derivative_bound, float(curvature_limit_m_inv))
