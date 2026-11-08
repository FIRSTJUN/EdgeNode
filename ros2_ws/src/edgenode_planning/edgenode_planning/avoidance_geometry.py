"""Planar solid geometry. No inferred road width or centerline road polygon."""
from dataclasses import dataclass
import json
import math
import hashlib
from pathlib import Path


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


def polygon_distance(a, b):
    if intersects(a, b):
        return 0.
    distance = math.inf
    for points, edges in ((a,b),(b,a)):
        for p in points:
            for x, y in zip(edges, edges[1:]+edges[:1]):
                dx, dy = y[0]-x[0], y[1]-x[1]
                length2 = dx*dx+dy*dy
                u = max(0., min(1., ((p[0]-x[0])*dx+(p[1]-x[1])*dy)/length2)) if length2 else 0.
                distance = min(distance, math.hypot(p[0]-x[0]-u*dx, p[1]-x[1]-u*dy))
    return distance


def _half_plane(poly, a, b, inside):
    result = []
    def value(p):
        return cross((b[0]-a[0],b[1]-a[1]), (p[0]-a[0],p[1]-a[1])) * (1 if inside else -1)
    for p, q in zip(poly, poly[1:]+poly[:1]):
        v, w = value(p), value(q)
        if v >= 0:
            result.append(p)
        if (v >= 0) != (w >= 0):
            t = v/(v-w)
            result.append((p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1])))
    return tuple(result)


@dataclass(frozen=True)
class DrivableArea:
    """Externally verified convex cell union in the map frame, including holes.

    No default cells. Source/provenance is mandatory; centerlines never populate
    this interface. Concave surveyed polygons must be supplied as convex cells.
    """
    cells: tuple
    source: str

    def __post_init__(self):
        if not self.source.strip() or not self.cells:
            raise ValueError('verified boundary source/cells required')
        clean = []
        for cell in self.cells:
            if (len(cell)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in cell)):
                raise ValueError('invalid boundary cell')
            hull = convex_hull(cell)
            if len(hull)!=len(cell) or area(hull)<=EPS or abs(area(tuple(map(tuple,cell)))-area(hull))>EPS:
                raise ValueError('boundary cells must be nondegenerate convex polygons')
            clean.append(hull)
        object.__setattr__(self, 'cells', tuple(clean))
        object.__setattr__(self, '_bounds', tuple((min(p[0] for p in c),max(p[0] for p in c),
            min(p[1] for p in c),max(p[1] for p in c)) for c in clean))
        object.__setattr__(self, '_rectangular', tuple(len(c)==4 and
            len({p[0] for p in c})==2 and len({p[1] for p in c})==2 for c in clean))

    def covers(self, polygon):
        # Subtract each convex road cell from the solid query. Merely checking
        # vertices or its perimeter would miss a road hole inside the vehicle.
        remaining = [tuple(polygon)]
        bounds=(min(p[0] for p in polygon),max(p[0] for p in polygon),
                min(p[1] for p in polygon),max(p[1] for p in polygon))
        # Exact fast path: one convex verified cell containing all vertices
        # contains the entire solid. This does not fill holes in a cell union.
        for cell,box,rectangular in zip(self.cells,self._bounds,self._rectangular):
            if box[0]<=bounds[0] and bounds[1]<=box[1] and box[2]<=bounds[2] and bounds[3]<=box[3]:
                if rectangular or convex_covers(cell,polygon):
                    return True
        for cell, (xmin,xmax,ymin,ymax) in zip(self.cells,self._bounds):
            if xmax<bounds[0] or xmin>bounds[1] or ymax<bounds[2] or ymin>bounds[3]:
                continue
            residual = []
            for piece in remaining:
                if not intersects(piece, cell):
                    residual.append(piece); continue
                inside = piece
                for a, b in zip(cell,cell[1:]+cell[:1]):
                    outside = _half_plane(inside,a,b,False) if inside else ()
                    if len(outside)>=3 and area(outside)>EPS:
                        residual.append(outside)
                    inside = _half_plane(inside,a,b,True) if inside else ()
            remaining = residual
            if not remaining:
                return True
        return not remaining


def load_drivable_area(filename, *, approved_sha256=None, mgeo_dir=None):
    """S4 accepts only an operator-reviewed, pinned real-map artifact.

    Hash approval is an explicit trust input, not an automatic survey verdict.
    No approval hashes are supplied by the map audit tool or default config.
    None preserves the legacy contract; an empty S4 pin rejects nonempty files.
    """
    if not filename:
        return None
    raw = Path(filename).read_bytes()
    if approved_sha256 is not None:
        if (not isinstance(approved_sha256, str) or len(approved_sha256) != 64 or
                any(c not in '0123456789abcdef' for c in approved_sha256) or
                hashlib.sha256(raw).hexdigest() != approved_sha256):
            raise ValueError('S4 boundary requires a matching explicitly approved SHA256')
    data = json.loads(raw)
    if approved_sha256 is not None:
        if data.get('data_kind') != 'real_map' or not mgeo_dir:
            raise ValueError('S4 boundary must be a reviewed real-map artifact')
        expected = {name: hashlib.sha256((Path(mgeo_dir) / name).read_bytes()).hexdigest()
                    for name in ('global_info.json', 'node_set.json', 'link_set.json')}
        if data.get('mgeo_sha256') != expected:
            raise ValueError('S4 boundary is not bound to this exact MGeo revision')
    if data.get('frame_id')!='map':
        raise ValueError('boundary data must be registered to the MGeo map frame')
    return DrivableArea(tuple(data['cells']), data['source'])


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
