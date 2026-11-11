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


def convex_hull(points):
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
    return abs(sum(cross(a, b) for a, b in zip(polygon, polygon[1:]+polygon[:1])))/2


def convex_covers(container, points):
    return all(cross((b[0]-a[0],b[1]-a[1]),(p[0]-a[0],p[1]-a[1]))>=0
               for a,b in zip(container,container[1:]+container[:1]) for p in points)


def inflate(polygon, margin):
    if margin==0:
        return tuple(polygon)
    # An axis-aligned square contains the requested Euclidean margin disk.
    return convex_hull([(x+dx, y+dy) for x, y in polygon
                        for dx, dy in ((-margin,-margin),(-margin,margin),(margin,-margin),(margin,margin))])


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


def swept_body(body, first, second, margin=0.):
    """Conservative continuous rigid-body enclosure between two poses.

    Every intermediate corner is within half-step translation + R*half-angle
    of its nearer endpoint. Inflating the endpoint hull by that bound covers
    curved no-slip motion as well as linearly interpolated path samples.
    """
    if first==second:
        return inflate(body.polygon(*first),margin)
    angle = abs(math.atan2(math.sin(second[2]-first[2]), math.cos(second[2]-first[2])))
    radius = max(math.hypot(x,y) for x,y in body.polygon(0.,0.,0.))
    chord=math.dist(first[:2],second[:2])
    travel=chord*angle/(2*math.sin(angle/2)) if angle>1e-9 else chord
    bound = travel/2 + radius*angle/2
    return inflate(body.polygon(*first)+body.polygon(*second), margin+bound)
