"""Explicit, directed drivable-area evidence for optional offset maneuvers.

Cells must come from a verified map surface, including its exclusions. Neither
lane width nor a convex hull of road markings is evidence of a road surface.
No K-City area is inferred or enabled by this module.
"""
import math

from .avoidance_geometry import area, convex_hull, cross, polygon_bounds
from .lane_geometry import LanePair, arc, sample


def _split(polygon, a, b):
    """Partition a convex polygon by a directed half-plane, without sampling."""
    inside=[];outside=[]
    edge=(b[0]-a[0],b[1]-a[1])
    for p,q in zip(polygon,polygon[1:]+polygon[:1]):
        u=cross(edge,(p[0]-a[0],p[1]-a[1]))
        v=cross(edge,(q[0]-a[0],q[1]-a[1]))
        (inside if u>=0 else outside).append(p)
        if (u<0<v) or (v<0<u):
            t=u/(u-v);r=(p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1]))
            inside.append(r);outside.append(r)
        elif u==0:
            outside.append(p)
    return tuple(inside),tuple(outside)


def _subtract(polygon, cell):
    remaining=polygon;pieces=[]
    for a,b in zip(cell,cell[1:]+cell[:1]):
        remaining,outside=_split(remaining,a,b)
        if len(outside)>=3 and area(outside)>1e-10:pieces.append(outside)
        if len(remaining)<3 or area(remaining)<=1e-10:break
    return pieces


class VerifiedRoadCorridor:
    """Union of explicit convex cells, with directed route/crossing scope.

    The caller is responsible for map semantics, registration, exclusions and
    traffic permission for each out-and-back offset maneuver. Mandatory
    provenance and link/side scope make that
    contract visible; geometric containment alone never creates permission.
    Missing evidence leaves LaneNetwork.road_corridor as None (the default).
    """
    def __init__(self, cells, *, source, link_ids, allowed_sides):
        if not isinstance(source,str) or not source.strip():raise ValueError('road provenance required')
        self.source=source;self.link_ids=frozenset(link_ids);self.allowed_sides=frozenset(allowed_sides)
        if (not self.link_ids or any(not isinstance(k,str) or not k for k in self.link_ids)
                or not self.allowed_sides or not self.allowed_sides<={'left','right'}):
            raise ValueError('directed link and crossing scope required')
        self.cells=[];self.grid={}
        for cell in cells:
            if len(cell)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in cell):
                raise ValueError('finite convex road cells required')
            hull=convex_hull(cell)
            if area(hull)<=1e-8 or abs(area(hull)-area(tuple(cell)))>1e-8:
                raise ValueError('nonconvex/degenerate road cell')
            index=len(self.cells);bounds=polygon_bounds(hull);self.cells.append((hull,bounds))
            x0,x1,y0,y1=bounds
            for x in range(math.floor(x0/10),math.floor(x1/10)+1):
                for y in range(math.floor(y0/10),math.floor(y1/10)+1):
                    self.grid.setdefault((x,y),set()).add(index)
        if not self.cells:raise ValueError('road cells required')

    def authorizes(self, pairs, side=None):
        return (bool(pairs) and all(a in self.link_ids and b in self.link_ids for a,b in pairs)
                and (side is None or side in self.allowed_sides))

    def covers(self, polygon):
        """Whole swept solid coverage, including interior holes and cell gaps.

        Iterated convex subtraction avoids assuming that covered vertices or
        edges imply a covered interior. Overlapping cells are safe as well.
        """
        x0,x1,y0,y1=polygon_bounds(polygon);nearby=set()
        for x in range(math.floor(x0/10),math.floor(x1/10)+1):
            for y in range(math.floor(y0/10),math.floor(y1/10)+1):
                nearby.update(self.grid.get((x,y),()))
        pieces=[tuple(polygon)]
        for i in sorted(nearby):
            cell,(a,b,c,d)=self.cells[i]
            if b<x0 or x1<a or d<y0 or y1<c:continue
            pieces=[part for piece in pieces for part in _subtract(piece,cell)]
            if not pieces:return True
            # Fail closed if a fragmented map would require unbounded work.
            if len(pieces)>256:return False
        return False

    def cross_section(self, origin, direction):
        """Exact line intervals in the cell union; preserve unsupported gaps.

        This only bounds offset search. It never proves vehicle containment.
        """
        intervals=[]
        for cell,_ in self.cells:
            low,high=-math.inf,math.inf
            for a,b in zip(cell,cell[1:]+cell[:1]):
                edge=(b[0]-a[0],b[1]-a[1])
                value=cross(edge,(origin[0]-a[0],origin[1]-a[1]))
                slope=cross(edge,direction)
                if abs(slope)<1e-12:
                    if value<0:break
                elif slope>0:low=max(low,-value/slope)
                else:high=min(high,-value/slope)
                if low>high:break
            else:
                if math.isfinite(low) and math.isfinite(high) and low<=high:
                    intervals.append((low,high))
        merged=[]
        for low,high in sorted(intervals):
            if merged and low<=merged[-1][1]:merged[-1]=(merged[-1][0],max(high,merged[-1][1]))
            else:merged.append((low,high))
        return tuple(merged)

    def offset_pair(self, network, current, side, route, offset, route_index=None):
        if side not in self.allowed_sides or not math.isfinite(offset) or offset<=0:return None
        if route_index is None:
            if route.count(current)!=1:return None
            route_index=route.index(current)
        if not 0<=route_index<len(route) or route[route_index]!=current:return None
        original=[];ids=[];ends=[];previous=None
        for key in route[route_index:]:
            if key not in self.link_ids or key not in network.valid:break
            link=network.links[key]
            field='can_move_'+side+'_lane'
            if field in link and link[field] is not True:break
            if previous is not None and (previous['to_node_idx']!=link['from_node_idx'] or
                    math.dist(original[-1][:2],link['points'][0][:2])>1e-7):break
            original.extend(tuple(p) for p in link['points'][bool(original):])
            ids.append((key,key));ends.append(arc(original)[-1]);previous=link
        if len(original)<2:return None
        total=arc(original)[-1];target=[];sign=1 if side=='left' else -1
        for i in range(math.ceil(total/.5)+1):
            p,t=sample(original,min(total,i*.5))
            target.append((p[0]-sign*offset*t[1],p[1]+sign*offset*t[0],p[2]))
        pair=LanePair(original,target,ids,side,ends)
        pair.mode='road_offset';pair.road_area=self
        return pair
