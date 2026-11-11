"""Bounded route re-projection for an explicitly disabled, stationary vehicle."""
import math


def reanchor_progress(points, x, y, yaw, previous_arc_m, max_retreat_m=3.0):
    """Find a heading-compatible projection within [-3m, +0.25m] of progress.

    Never search another route leg or change geometry. The caller owns the
    disarmed/stationary/freshness gates. A distant teleport must fail closed.
    """
    values = (x, y, yaw, previous_arc_m, max_retreat_m)
    try:
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) for v in values):
            return None
    except OverflowError:
        return None
    if not isinstance(points, (list, tuple)) or len(points) < 2 or not 0 < max_retreat_m <= 3:
        return None
    arc = 0.0
    best = None
    low, high = max(0.0, previous_arc_m-max_retreat_m), previous_arc_m+.25
    try:
        for a, b in zip(points, points[1:]):
            if len(a) != 3 or len(b) != 3 or not all(math.isfinite(v) for v in (*a, *b)):
                return None
        for index, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if not math.isfinite(length):
                return None
            end = arc+length
            if length > 1e-9 and end >= low and arc <= high:
                difference = abs(math.remainder(math.atan2(dy, dx)-yaw, 2*math.pi))
                if difference <= math.pi/4:
                    along = ((x-a[0])*dx+(y-a[1])*dy)/length
                    along = max(0.0, low-arc, min(length, high-arc, along))
                    distance = math.hypot(x-a[0]-along*dx/length, y-a[1]-along*dy/length)
                    candidate = (distance, arc+along, -index)
                    if best is None or candidate < best:
                        best = candidate
            arc = end
    except (TypeError, ValueError, OverflowError):
        return None
    if not 0 <= previous_arc_m <= arc or best is None or best[0] > .75:
        return None
    return dict(nearest_index=-best[2], progress_arc_m=best[1],
                distance_m=best[0], retreat_m=previous_arc_m-best[1])
