"""Planar solid geometry. No inferred road width or centerline road polygon."""
import math


EPS = 1e-9


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def arc(points):
    lengths = [0.]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1]+math.dist(a[:2], b[:2]))
    return lengths


def convex_hull(points,*,native=False):
    if native:
        import numpy as np
        from . import _geometry_native
        xy=np.ascontiguousarray([tuple(p[:2]) for p in points],dtype=np.float64)
        return tuple(_geometry_native.convex_hull(xy))
    points = sorted(set(tuple(p[:2]) for p in points))
    def half(values):
        result = []
        for p in values:
            while len(result)>1 and cross((result[-1][0]-result[-2][0], result[-1][1]-result[-2][1]),
                                         (p[0]-result[-1][0], p[1]-result[-1][1])) <= 0:
                result.pop()
            result.append(p)
        return result
    return tuple(half(points)[:-1]+half(list(reversed(points)))[:-1]) if len(points)>1 else tuple(points)


def area(polygon):
    if len(polygon)<3:return 0.
    # Translate before the shoelace sum. Tiny exact survey-cap fragments at
    # large map coordinates otherwise acquire artificial area by cancellation.
    x,y=polygon[0]
    return abs(math.fsum((a[0]-x)*(b[1]-y)-(a[1]-y)*(b[0]-x)
        for a,b in zip(polygon,polygon[1:]+polygon[:1])))/2


def convex_covers(container, points):
    return all(cross((b[0]-a[0],b[1]-a[1]),(p[0]-a[0],p[1]-a[1]))>=0
               for a,b in zip(container,container[1:]+container[:1]) for p in points)


def inflate(polygon, margin,*,native=False):
    if margin==0:
        return tuple(polygon)
    # An axis-aligned square contains the requested Euclidean margin disk.
    return convex_hull([(x+dx, y+dy) for x, y in polygon
                        for dx, dy in ((-margin,-margin),(-margin,margin),(margin,-margin),(margin,margin))],native=native)


def intersects(a, b):
    """Separating-axis test for convex solids; touching is collision."""
    for poly in (a, b):
        for p, q in zip(poly, poly[1:]+poly[:1]):
            axis = (p[1]-q[1], q[0]-p[0])
            aa = [x*axis[0]+y*axis[1] for x, y in a]
            bb = [x*axis[0]+y*axis[1] for x, y in b]
            if max(aa)<min(bb)-EPS or max(bb)<min(aa)-EPS:
                return False
    return True


def polygon_bounds(polygon):
    return (min(p[0] for p in polygon),max(p[0] for p in polygon),
            min(p[1] for p in polygon),max(p[1] for p in polygon))


def bounds_distance(a,b):
    """Lower bound on solid separation; never proves a nearby solid safe."""
    return math.hypot(max(a[0]-b[1],b[0]-a[1],0.),max(a[2]-b[3],b[2]-a[3],0.))


def polygon_distance(a, b):
    if intersects(a, b):
        return 0.
    distance = math.inf
    for points, edges in ((a,b),(b,a)):
        segments=[]
        for x,y in zip(edges,edges[1:]+edges[:1]):
            dx,dy=y[0]-x[0],y[1]-x[1]
            segments.append((x,dx,dy,dx*dx+dy*dy))
        for p in points:
            for x,dx,dy,length2 in segments:
                u = max(0., min(1., ((p[0]-x[0])*dx+(p[1]-x[1])*dy)/length2)) if length2 else 0.
                distance = min(distance, math.hypot(p[0]-x[0]-u*dx, p[1]-x[1]-u*dy))
    return distance


def obstacle_polygons(groups):
    result = []
    for points in groups:
        if len(points)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('complete finite obstacle footprint required')
        hull = convex_hull(points)
        if len(hull)<3 or area(hull)<=EPS:
            raise ValueError('degenerate obstacle footprint')
        result.append(hull)  # Convex hull is conservative for concave detections.
    return tuple(result)


def swept_body(body, first, second, margin=0.,*,native=False):
    """Conservative continuous rigid-body enclosure between two poses.

    Interpolation is linear translation + shortest-angle yaw, or a constant-
    curvature bicycle step. The endpoint hull already contains every linearly
    interpolated corner. For either reference motion, the chord-interpolation
    error is bounded by sup|corner''(u)|/8 = (R_axle + R_body)*angle**2/8.
    This follows from the linear-interpolation remainder for u in [0,1].
    R_axle=chord/(2*sin(angle/2)) for a bicycle arc (zero for linear motion).
    The square inflation contains that Euclidean error disk. Unlike adding
    half the translation again, this bound is continuous at zero rotation.
    """
    if first==second:
        return inflate(body.polygon(*first),margin,native=native)
    angle = abs(math.atan2(math.sin(second[2]-first[2]), math.cos(second[2]-first[2])))
    if angle==0.:
        # Constant orientation + straight translation has an exact convex sweep.
        # Half the longitudinal step is not a lateral uncertainty: adding it
        # made harmless guardrails collide when path spacing increased.
        return inflate(convex_hull(body.polygon(*first)+body.polygon(*second),native=native),margin+EPS,native=native)
    radius = max(math.hypot(x,y) for x,y in body.polygon(0.,0.,0.))
    chord=math.dist(first[:2],second[:2])
    axle_error=chord*angle**2/(16*math.sin(angle/2)) if angle>1e-9 else chord*angle/8
    bound = axle_error+radius*angle**2/8
    return inflate(body.polygon(*first)+body.polygon(*second), margin+bound,native=native)
