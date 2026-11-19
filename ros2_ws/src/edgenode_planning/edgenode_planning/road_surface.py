"""Exact simple surface triangulation; no convex filling of road gaps."""
import math

from .avoidance_geometry import area,convex_covers,cross,intersects


def triangulate(ring):
    points=[tuple(p[:2]) for p in ring]
    if points and points[0]==points[-1]:points.pop()
    if (not 3<=len(points)<=1000 or len(set(points))!=len(points)
            or any(not all(math.isfinite(v) for v in p) for p in points)):
        raise ValueError('finite simple ring required')
    n=len(points)
    for i in range(n):
        for j in range(i+1,n):
            if j in (i,(i+1)%n) or i==(j+1)%n:continue
            a,b=points[i],points[(i+1)%n];c,d=points[j],points[(j+1)%n]
            # SAT using only segment normals cannot separate disjoint
            # collinear segments: both project to the same single value.
            # Original surveyed rings often contain exactly those edges.
            # A bounding-box separation is an exact rejection, not a map
            # repair or tolerance. Real crossings/touches remain rejected.
            separated=(max(a[0],b[0])<min(c[0],d[0]) or max(c[0],d[0])<min(a[0],b[0]) or
                       max(a[1],b[1])<min(c[1],d[1]) or max(c[1],d[1])<min(a[1],b[1]))
            if not separated and intersects((a,b),(c,d)):
                # The generic SAT helper has a contact tolerance. Surveyed
                # caps can contain distinct edges separated by nanometres;
                # confirm topology with GEOS rather than calling those edges
                # a crossing or changing their original coordinates.
                from shapely.geometry import LineString
                if LineString((a,b)).intersects(LineString((c,d))):
                    raise ValueError('self-intersecting road surface')
    signed=sum(cross(a,b) for a,b in zip(points,points[1:]+points[:1]))
    if signed<0:points.reverse()
    expected=area(tuple(points));cells=[]
    while len(points)>3:
        for i,b in enumerate(points):
            a=points[i-1];c=points[(i+1)%len(points)]
            turn=cross((b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1]))
            if abs(turn)<=1e-10:
                points.pop(i);break
            if turn<0:continue
            triangle=(a,b,c)
            if any(convex_covers(triangle,(p,)) for p in points if p not in triangle):continue
            cells.append(triangle);points.pop(i);break
        else:raise ValueError('road surface cannot be triangulated')
    if len(points)==3 and area(tuple(points))>1e-10:cells.append(tuple(points))
    if not cells or not math.isclose(sum(area(c) for c in cells),expected,abs_tol=1e-6):
        raise ValueError('road surface triangulation lost area')
    return tuple(cells)


def cells_from_geometry(geometry):
    """Partition actual polygon surfaces including surveyed interior holes.

    Delaunay triangles are clipped to the ORIGINAL surface. Hole interiors
    never become road cells. This is neither a convex hull nor gap closing.
    """
    from shapely.geometry import Polygon
    from shapely.ops import triangulate as geos_triangles,unary_union
    if geometry.is_empty:return ()
    if geometry.geom_type=='MultiPolygon':
        return tuple(c for p in geometry.geoms for c in cells_from_geometry(p))
    if geometry.geom_type!='Polygon' or not geometry.is_valid:raise ValueError('valid surveyed surface required')
    if not geometry.interiors:return triangulate(tuple(geometry.exterior.coords)[:-1])
    cells=[]
    def polygons(value):
        if value.geom_type=='Polygon':yield value
        else:
            for child in getattr(value,'geoms',()):yield from polygons(child)
    for triangle in geos_triangles(geometry):
        clipped=triangle.intersection(geometry)
        for part in polygons(clipped):
            if part.geom_type!='Polygon' or part.is_empty or part.area<=1e-8:continue
            if part.interiors:raise ValueError('survey hole was not partitioned')
            cells.extend(triangulate(tuple(part.exterior.coords)[:-1]))
    covered=unary_union([Polygon(c) for c in cells])
    if geometry.symmetric_difference(covered).area>1e-6:raise ValueError('survey hole partition lost area')
    return tuple(cells)
