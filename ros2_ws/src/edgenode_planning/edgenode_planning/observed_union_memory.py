"""Static observed occupancy, without convex filling between curved surfaces.

Every retained piece is convex for the existing SAT/distance gates. Concave
unions are partitioned exactly; holes or failed partitions retain the original
pieces. No observed occupancy is erased to make a route pass.
"""
from shapely.geometry import Polygon
from shapely.ops import unary_union,triangulate as geos_triangulate

from .avoidance_geometry import inflate
from .road_surface import triangulate


def conservative_compaction(observed):
    """Bound complexity without shrinking or cumulatively inflating history.

    Five-millimetre simplification is accepted only after exact containment
    proves every observed occupied point remains. The result must be inside
    the observed union's conservative 15 mm neighbourhood. It is NEVER used
    as the next raw observation union, so the allowance cannot accumulate.
    """
    if observed.geom_type!='Polygon' or len(observed.exterior.coords)<20:return observed
    candidate=observed.simplify(.005,preserve_topology=True).buffer(.005,join_style=2,mitre_limit=2.)
    if (candidate.geom_type=='Polygon' and len(candidate.exterior.coords)<len(observed.exterior.coords)
            and candidate.covers(observed) and observed.buffer(.015,resolution=4).covers(candidate)):return candidate
    return observed


def partition(shape, fallback):
    components=list(shape.geoms) if shape.geom_type=='MultiPolygon' else [shape]
    parts=[]
    try:
        for component in components:
            if component.geom_type!='Polygon' or component.interiors:
                return tuple(fallback)
            ring=tuple(component.exterior.coords)[:-1]
            if component.equals(component.convex_hull):parts.append(ring)
            else:
                cells=[t for t in geos_triangulate(component) if component.covers(t)]
                # GEOS triangulation is fast, but is not constrained. Accept
                # it only when its union equals the observed component. The
                # exact ear partition remains the fallback for missing cells.
                if component.symmetric_difference(unary_union(cells)).area<=1e-8:
                    parts.extend(tuple(t.exterior.coords)[:-1] for t in cells)
                else:parts.extend(triangulate(ring))
        result=unary_union([Polygon(p) for p in parts])
        if shape.symmetric_difference(result).area>1e-8:
            return tuple(fallback)
        return tuple(parts)
    except (ValueError,TypeError,OverflowError):
        return tuple(fallback)


class ObservedUnionMemory:
    def __init__(self, initial=()):
        self.groups=[(Polygon(p),(p,),Polygon(p).bounds) for p in initial]

    def update(self, observed, retained, padding, extent):
        allowed=set(retained);groups=[]
        for shape,parts,bounds in self.groups:
            keep=tuple(p for p in parts if p in allowed)
            if keep:
                retained_shape=shape if keep==parts else shape.intersection(unary_union([Polygon(p) for p in keep]))
                if not retained_shape.is_empty:groups.append((retained_shape,keep,retained_shape.bounds))
        for raw in observed:
            piece=inflate(raw,padding);incoming=Polygon(piece)
            x0,y0,x1,y1=incoming.bounds
            if any(a<=x0 and b<=y0 and c>=x1 and d>=y1 and shape.covers(incoming)
                   for shape,_,(a,b,c,d) in groups):continue
            matches=[];combined=incoming
            lo,bottom,hi,top=incoming.bounds;envelope=incoming.bounds
            for i,(shape,parts,(a,b,c,d)) in enumerate(groups):
                if hi<a or c<lo or top<b or d<bottom:continue
                if not shape.intersects(incoming):continue
                bounds=(min(envelope[0],a),min(envelope[1],b),max(envelope[2],c),max(envelope[3],d))
                if extent and max(bounds[2]-bounds[0],bounds[3]-bounds[1])>extent:continue
                combined=combined.union(shape);matches.append(i);envelope=bounds
            fallback=[piece]+[p for i in matches for p in groups[i][1]]
            compact=conservative_compaction(combined)
            parts=partition(compact,fallback)
            groups=[g for i,g in enumerate(groups) if i not in matches]
            groups.append((combined,parts,envelope))
        self.groups=groups
        return tuple(sorted({p for _,parts,_ in groups for p in parts}))
