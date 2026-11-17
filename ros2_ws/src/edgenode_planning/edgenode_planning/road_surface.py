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
            if intersects((points[i],points[(i+1)%n]),(points[j],points[(j+1)%n])):
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
