"""Read-only measurements of the existing centerline-quad corridor gate.

Distances are Euclidean metres to the quad union, not vehicle-body clearance.
The scalar cross-section is explanatory; only corridor_covers decides safety.
"""
import math

from .lane_geometry import EPS, arc, corridor_covers, cross, segment_interval


def _closest(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx+dy*dy
    u = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2)) if length2 else 0.
    q = [a[0]+u*dx, a[1]+u*dy]
    return math.dist(point[:2], q), q, u


def _projection(points, point):
    lengths = arc(points)
    choices = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = lengths[i+1]-lengths[i]
        if length > 1e-9:
            distance, q, u = _closest(point, a, b)
            choices.append((distance, i, q, u, length))
    if not choices:
        return None
    distance, i, q, u, length = min(choices, key=lambda v: (v[0], v[1]))
    tangent = [(points[i+1][k]-points[i][k])/length for k in range(2)]
    relative = [point[k]-q[k] for k in range(2)]
    return dict(segment_index=i, station_m=lengths[i]+u*length, world_xy=q,
                tangent=tangent, normal=[-tangent[1], tangent[0]], distance_m=distance,
                signed_lateral_m=cross(tangent, relative),
                longitudinal_residual_m=sum(a*b for a, b in zip(tangent, relative)))


def _distance(point, quads):
    # Exact geometric distance, independent of the gate's cross-product EPS.
    best = math.inf
    for quad in quads:
        area = sum(cross(a, b) for a, b in zip(quad, quad[1:]+quad[:1]))
        if abs(area) <= 1e-10:
            continue
        turns = [cross((b[0]-a[0], b[1]-a[1]), (point[0]-a[0], point[1]-a[1]))
                 for a, b in zip(quad, quad[1:]+quad[:1])]
        if all(v >= 0. for v in turns) or all(v <= 0. for v in turns):
            return 0.
        best = min(best, *(_closest(point, a, b)[0]
                          for a, b in zip(quad, quad[1:]+quad[:1])))
    return best


def _cross_section(quads, origin, tangent):
    """Intersect the actual quad union with one original-lane normal line."""
    intervals = []
    for quad in quads:
        local = [(sum((p[k]-origin[k])*tangent[k] for k in range(2)),
                  cross(tangent, (p[0]-origin[0], p[1]-origin[1]))) for p in quad]
        lateral = []
        for (x, y), (xx, yy) in zip(local, local[1:]+local[:1]):
            if abs(x) <= 1e-10:
                lateral.append(y)
            if x*xx < 0.:
                lateral.append(y+(yy-y)*(-x)/(xx-x))
        if lateral:
            intervals.append((min(lateral), max(lateral)))
    merged = []
    for low, high in sorted(intervals):
        if merged and low <= merged[-1][1]+1e-10:
            merged[-1][1] = max(merged[-1][1], high)
        else:
            merged.append([low, high])
    return merged


def _describe(index, point, violation, pair, quads, **extra):
    result = dict(candidate_point_index=index, world_xy=list(point[:2]),
                  violation_m=violation, **extra)
    if quads:
        nearest = min(range(len(quads)), key=lambda i: _distance(point, [quads[i]]))
        quad = quads[nearest]
        area = sum(cross(a, b) for a, b in zip(quad, quad[1:]+quad[:1]))
        sign = 1. if area > 0. else -1.
        edges = []
        for a, b in zip(quad, quad[1:]+quad[:1]):
            edge = [b[0]-a[0], b[1]-a[1]]
            length = math.hypot(*edge)
            edges.append(dict(signed_distance_m=sign*cross(edge, [point[0]-a[0], point[1]-a[1]])/length
                              if length else None, effective_gate_tolerance_m=EPS/length if length else None))
        result.update(nearest_quad_index=nearest, nearest_quad=quad, nearest_quad_edges=edges)
    if pair is None:
        return result
    original = _projection(pair.original, point)
    adjacent = _projection(pair.adjacent, point)
    result.update(original_projection=original, adjacent_projection=adjacent)
    if original is None:
        return result
    intervals = _cross_section(quads, original['world_xy'], original['tangent'])
    lateral = original['signed_lateral_m']
    bounds = min(intervals, key=lambda v: max(v[0]-lateral, lateral-v[1], 0.)) if intervals else None
    row = pair.at(original['station_m'])
    result.update(bounds_frame='normal_at_original_projection',
                  signed_lateral_m=lateral, allowed_lateral_ranges_m=intervals,
                  corridor_lower_m=bounds[0] if bounds else None,
                  corridor_upper_m=bounds[1] if bounds else None,
                  lateral_violation_m=max(bounds[0]-lateral, lateral-bounds[1], 0.) if bounds else None,
                  nominal_center_bounds_m=sorted([0., row['separation']]) if row else None,
                  normal_angle_difference_rad=math.atan2(cross(original['tangent'], adjacent['tangent']),
                      sum(a*b for a, b in zip(original['tangent'], adjacent['tangent']))) if adjacent else None)
    return result


def _first_segment_gap(points, quads):
    """Use precisely the existing clipping and interval-merging tolerances."""
    bounded = [(q, min(p[0] for p in q), max(p[0] for p in q), min(p[1] for p in q), max(p[1] for p in q))
               for q in quads]
    for i, (a, b) in enumerate(zip(points, points[1:])):
        xmin, xmax = min(a[0], b[0]), max(a[0], b[0])
        ymin, ymax = min(a[1], b[1]), max(a[1], b[1])
        intervals = sorted(v for q, qxmin, qxmax, qymin, qymax in bounded
                           if xmax >= qxmin-EPS and xmin <= qxmax+EPS and ymax >= qymin-EPS and ymin <= qymax+EPS
                           if (v := segment_interval(a, b, q)) is not None)
        covered = 0.
        for low, high in intervals:
            if low > covered+EPS:
                return i, covered, low
            covered = max(covered, high)
            if covered >= 1-EPS:
                break
        if covered < 1-EPS:
            return i, covered, 1.
    return None


def _envelope_failure(pair, station_range):
    """Use the builder's actual normalized sections and rejection reason."""
    if pair is None or station_range is None:
        return None
    return pair.corridor_envelope(*station_range)['failure'] or dict(reason='NO_QUADS')


def inspect_corridor(points, quads, pair=None, station_range=None):
    """Measure rejected candidate vertices; also locate between-vertex failure.

    max_violation_m measures rejected candidate vertices (zero if accepted),
    NOT a continuous swept-path maximum. A separate interior witness reports
    holes even when every vertex passes. No diagnostics feed the safety gate.
    """
    safe = corridor_covers(points, quads)
    result = dict(evaluated=True, safe=safe, reason='COVERED' if safe else 'OUTSIDE_ENVELOPE',
                  candidate_point_count=len(points), failed_point_count=0, failed_index=-1,
                  max_violation_m=0., maximum_scope='rejected_candidate_vertices',
                  first_failure=None, maximum_violation_point=None, first_segment_failure=None,
                  gate_epsilon=EPS, gate_epsilon_units='cross_product_m2_and_segment_fraction')
    if not quads or len(points) < 2:
        result.update(reason='EMPTY_OR_INVALID_ENVELOPE' if not quads else 'INVALID_CANDIDATE',
                      max_violation_m=None, failed_point_count=len(points),
                      failed_index=0 if points else -1,
                      envelope_failure=_envelope_failure(pair, station_range))
        if points:
            result['first_failure'] = _describe(0, points[0], None, pair, quads)
        return result
    # Accepted paths need no additional per-vertex scan in the 20 Hz loop.
    # An EPS-accepted microscopic excursion is not a rejected point violation.
    if safe:
        result['reference_cross_section'] = _describe(0, points[0], _distance(points[0], quads), pair, quads)
        return result
    violations = [_distance(p, quads) for p in points]
    failed = [i for i, p in enumerate(points) if not corridor_covers([p, p], quads)]
    maximum = max(range(len(points)), key=violations.__getitem__)
    result.update(failed_point_count=len(failed), max_violation_m=violations[maximum],
                  maximum_violation_point=_describe(maximum, points[maximum], violations[maximum], pair, quads))
    if failed:
        i = failed[0]
        result.update(failed_index=i, first_failure=_describe(i, points[i], violations[i], pair, quads))
    gap = _first_segment_gap(points, quads)
    if gap is not None:
        i, low, high = gap
        u = (low+high)/2.
        point = [(1-u)*a+u*b for a, b in zip(points[i], points[i+1])]
        witness = _describe(i, point, _distance(point, quads), pair, quads,
                            segment_start_index=i, segment_fraction=u, uncovered_fraction_range=[low, high])
        result['first_segment_failure'] = witness
        if not failed:
            result.update(reason='UNCOVERED_SEGMENT_INTERIOR', failed_index=i, first_failure=witness)
    return result
