"""MGeo lane pairs and conservative center-to-center corridors; no routing edits."""
import math
from bisect import bisect_left

from .avoidance_geometry import arc, cross


EPS = 1e-7


def project(points, point, lengths=None):
    return PolylineIndex(points).project(point)


def sample(points, s, lengths=None):
    """Interpolate within actual geometry; never extend a map link."""
    lengths = arc(points) if lengths is None else lengths
    if not points or s < -EPS or s > lengths[-1]+EPS:
        return None
    s = max(0., min(s, lengths[-1]))
    # Arc stations are monotone. Keep the original tolerant, earliest-segment
    # tie rule (including tiny/zero segments); only locate the scan start faster.
    first=max(0,bisect_left(lengths,s)-1)
    while first>0 and s<=lengths[first]+EPS:
        first-=1
    for i in range(first,len(points)-1):
        a,b=points[i],points[i+1]
        length = lengths[i+1]-lengths[i]
        if length > 1e-9 and s <= lengths[i+1]+EPS:
            u = max(0., min(1., (s-lengths[i])/length))
            point = [(1-u)*x+u*y for x,y in zip(a,b)]
            tangent = ((b[0]-a[0])/length, (b[1]-a[1])/length)
            return point, tangent
    return None


def curvature_at(points, s):
    lengths = arc(points)
    a,b,c = (sample(points, max(0., min(lengths[-1], v))) for v in (s-.5,s,s+.5))
    if not a or not b or not c:
        return 0.
    a,b,c = a[0],b[0],c[0]
    denominator = math.dist(a[:2],b[:2])*math.dist(b[:2],c[:2])*math.dist(a[:2],c[:2])
    return 2*((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))/denominator if denominator > 1e-12 else 0.


def segment_interval(a,b,polygon):
    """Exact convex clipping interval on a candidate segment, or None."""
    area = sum(cross(polygon[i],polygon[(i+1)%len(polygon)]) for i in range(len(polygon)))
    if abs(area) <= 1e-10:
        return None
    sign = 1 if area > 0 else -1
    low,high = 0.,1.
    direction = (b[0]-a[0],b[1]-a[1])
    for p,q in zip(polygon,polygon[1:]+polygon[:1]):
        edge = (q[0]-p[0],q[1]-p[1])
        constant = sign*cross(edge,(a[0]-p[0],a[1]-p[1]))
        slope = sign*cross(edge,direction)
        if abs(slope) <= 1e-12:
            if constant < -EPS:
                return None
        elif slope > 0:
            low = max(low,(-EPS-constant)/slope)
        else:
            high = min(high,(-EPS-constant)/slope)
        if high < low:
            return None
    return max(0.,low),min(1.,high)


def corridor_covers(points, quads):
    """Require every whole path segment to be covered by the quad union."""
    if len(points)<2 or not quads:
        return False
    bounded=[(q,min(p[0] for p in q),max(p[0] for p in q),min(p[1] for p in q),max(p[1] for p in q)) for q in quads]
    for a,b in zip(points,points[1:]):
        xmin,xmax=min(a[0],b[0]),max(a[0],b[0])
        ymin,ymax=min(a[1],b[1]),max(a[1],b[1])
        intervals = sorted(v for q,qxmin,qxmax,qymin,qymax in bounded
            if xmax>=qxmin-EPS and xmin<=qxmax+EPS and ymax>=qymin-EPS and ymin<=qymax+EPS
            if (v := segment_interval(a,b,q)) is not None)
        covered = 0.
        for low,high in intervals:
            if low > covered+EPS:
                break
            covered = max(covered,high)
            if covered >= 1-EPS:
                break
        if covered < 1-EPS:
            return False
    return True


class PolylineIndex:
    """Exact nearest-segment projection with block AABB pruning, no map edits."""
    def __init__(self,points):
        self.points,self.lengths=points,arc(points)
        self.segments=[]
        for i,(a,b) in enumerate(zip(points,points[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(dx,dy)
            if length>1e-9:
                self.segments.append((i,a,dx,dy,length))
        self.blocks=[]
        for first in range(0,len(self.segments),32):
            segments=self.segments[first:first+32]
            vertices=[p for i,a,dx,dy,length in segments for p in (a,points[i+1])]
            self.blocks.append((min(p[0] for p in vertices),max(p[0] for p in vertices),
                                min(p[1] for p in vertices),max(p[1] for p in vertices),segments))

    def project(self,point):
        ordered=sorted((max(lowx-point[0],point[0]-highx,0.)**2+
                        max(lowy-point[1],point[1]-highy,0.)**2,k)
                       for k,(lowx,highx,lowy,highy,_) in enumerate(self.blocks))
        best=(math.inf,math.inf,math.nan)
        for lower,k in ordered:
            if lower>best[0]:
                break
            for i,a,dx,dy,length in self.blocks[k][4]:
                raw=((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length**2
                fraction=max(0.,min(1.,raw))
                distance=(point[0]-a[0]-fraction*dx)**2+(point[1]-a[1]-fraction*dy)**2
                if (distance,i)<best[:2]:
                    along=raw if i==0 and raw<0 or i==len(self.points)-2 and raw>1 else fraction
                    best=(distance,i,self.lengths[i]+along*length)
        return best[2]


    def normal_intersection(self,point,tangent,side):
        candidates=[]
        sign=1 if side=='left' else -1
        for lowx,highx,lowy,highy,segments in self.blocks:
            values=[(x-point[0])*tangent[0]+(y-point[1])*tangent[1]
                    for x,y in ((lowx,lowy),(lowx,highy),(highx,lowy),(highx,highy))]
            if min(values)>EPS or max(values)<-EPS:
                continue
            for i,a,dx,dy,length in segments:
                denominator=dx*tangent[0]+dy*tangent[1]
                if denominator<=1e-9:
                    continue
                fraction=-((a[0]-point[0])*tangent[0]+(a[1]-point[1])*tangent[1])/denominator
                if -EPS<=fraction<=1+EPS:
                    fraction=max(0.,min(1.,fraction))
                    q=[a[0]+fraction*dx,a[1]+fraction*dy,
                       (1-fraction)*a[2]+fraction*self.points[i+1][2]]
                    lateral=cross(tangent,(q[0]-point[0],q[1]-point[1]))
                    if lateral*sign>EPS:
                        candidates.append((abs(lateral),i,self.lengths[i]+fraction*length,q,(dx/length,dy/length)))
        if not candidates:
            return None
        _,_,station,q,direction=min(candidates,key=lambda v:(v[0],v[1]))
        return station,(q,direction)


class LaneNetwork:
    """Reuse indexed MGeo records; directed neighbor fields declare lane changes.

    Synthetic lazy lane-change links are metadata only and never trajectories.
    Reciprocal neighbor relation, valid ordinary endpoints, same direction and
    connected successor pairs are required. Unknown geometry fails closed.
    """
    def __init__(self, planner, *, require_explicit_permission=False):
        if type(require_explicit_permission) is not bool:
            raise ValueError('explicit permission policy must be boolean')
        self.require_explicit_permission = require_explicit_permission
        self.links = planner.links_by_id
        self.valid = {edge['link_id'] for edges in planner.graph.values() for edge in edges}

    def permission_reason(self, original_id, side):
        """Topology alone is never positive permission in the S4 mode.

        Require both outward and reserved return permissions. This is only a
        metadata gate; it does not certify line crossing or road containment.
        """
        if side not in ('left', 'right') or original_id not in self.valid:
            return 'INVALID_LANE_REFERENCE'
        original = self.links[original_id]
        target_id = original.get(side + '_lane_change_dst_link_idx')
        if target_id not in self.valid:
            return 'NO_VALID_ADJACENT_LANE'
        reverse = 'right' if side == 'left' else 'left'
        default = None if self.require_explicit_permission else True
        values = (original.get('can_move_' + side + '_lane', default),
                  self.links[target_id].get('can_move_' + reverse + '_lane', default))
        if any(value is False for value in values):
            return 'LANE_CHANGE_FORBIDDEN'
        if any(value is not True for value in values):
            return 'UNKNOWN_LANE_CHANGE_PERMISSION'
        if self.require_explicit_permission and any(
                link.get('link_type') not in ('6', 6) for link in (original, self.links[target_id])):
            # NGII 2019 table 24: only ordinary travel lanes are in this S4
            # verification profile. Intersection/bus/reversible/unknown types
            # need separate, direction/section-specific crossing evidence.
            return 'UNVERIFIED_LANE_TYPE'
        return ''

    def neighbor(self, original_id, side):
        if self.permission_reason(original_id, side):
            return None
        field = side+'_lane_change_dst_link_idx'
        opposite = ('right' if side=='left' else 'left')+'_lane_change_dst_link_idx'
        if original_id not in self.valid:
            return None
        original = self.links[original_id]
        target_id = original.get(field)
        if target_id not in self.valid:
            return None
        target = self.links[target_id]
        if target.get(opposite) != original_id:
            return None
        if any(not isinstance(link.get('max_speed'), (int,float)) or
               not math.isfinite(link['max_speed']) or link['max_speed'] <= 0
               for link in (original,target)):
            return None
        for u in (0.,.5,1.):
            a = sample(original['points'],arc(original['points'])[-1]*u)
            b = sample(target['points'],max(0.,min(arc(target['points'])[-1],project(target['points'],a[0]))))
            if not b or sum(x*y for x,y in zip(a[1],b[1])) <= 0:
                return None
            displacement = (b[0][0]-a[0][0],b[0][1]-a[0][1])
            separation = cross(a[1],displacement)
            if separation*(1 if side=='left' else -1) <= EPS:
                return None
        return target_id

    def pair(self, original_id, side, route_link_ids):
        if route_link_ids.count(original_id)!=1:
            return None
        originals,adjacents,ids,section_ends = [],[],[],[]
        start = route_link_ids.index(original_id)
        previous_original = previous_adjacent = None
        for link_id in route_link_ids[start:]:
            adjacent_id = self.neighbor(link_id,side)
            if adjacent_id is None:
                break
            original,adjacent = self.links[link_id],self.links[adjacent_id]
            if previous_original is not None:
                if (previous_original['to_node_idx'] != original['from_node_idx'] or
                        previous_adjacent['to_node_idx'] != adjacent['from_node_idx'] or
                        math.dist(originals[-1][:2],original['points'][0][:2]) > EPS or
                        math.dist(adjacents[-1][:2],adjacent['points'][0][:2]) > EPS):
                    break
            for destination,source in ((originals,original['points']),(adjacents,adjacent['points'])):
                for p in source:
                    if not destination or math.dist(destination[-1][:2],p[:2]) > 1e-9:
                        destination.append(list(p))
            ids.append((link_id,adjacent_id))
            section_ends.append(arc(originals)[-1])
            previous_original,previous_adjacent = original,adjacent
        return LanePair(originals,adjacents,ids,side,section_ends) if ids else None


class LanePair:
    def __init__(self, original, adjacent, ids, side, section_ends):
        self.original,self.adjacent,self.ids,self.side = original,adjacent,ids,side
        self.section_ends=section_ends
        self.original_arc,self.adjacent_arc=arc(original),arc(adjacent)
        self.length = self.original_arc[-1]
        self.original_index,self.adjacent_index=PolylineIndex(original),PolylineIndex(adjacent)
        self.adjacent_stations=[self.original_index.project(p) for p in adjacent]

    def current_ids(self,s):
        for ids,end in zip(self.ids,self.section_ends):
            if s<end-1e-9:
                return ids
        return self.ids[-1]

    def at(self,s):
        original = sample(self.original,s,self.original_arc)
        if original is None:
            return None
        intersection=self.adjacent_index.normal_intersection(original[0],original[1],self.side)
        if intersection is None:
            return None
        adjacent_s,adjacent=intersection
        separation = cross(original[1],(adjacent[0][0]-original[0][0],adjacent[0][1]-original[0][1]))
        if separation*(1 if self.side=='left' else -1) <= EPS:
            return None
        return dict(original=original,adjacent=adjacent,adjacent_s=adjacent_s,separation=separation)

    def corridor_envelope(self,start,end):
        """Triangulate the exact two piecewise-linear centerline boundaries.

        Interior normal cross-sections are not geometry anchors: at a map
        corner their adjacent intersections may run backwards. Every real
        vertex stays in the boundary; supplemental half-metre rows are wholly
        redundant for piecewise-linear geometry and are not generated.
        The two horizon caps still use the same normal intersections as at().
        """
        result=dict(quads=[],stations=[],normalization=[],failure=None)
        if not all(math.isfinite(s) for s in (start,end)) or end<=start:
            result['failure']=dict(reason='INVALID_STATION_RANGE',station_range_m=[start,end])
            return result
        first,last=self.at(start),self.at(end)
        if first is None or last is None:
            result['failure']=dict(reason='NORMAL_INTERSECTION_UNAVAILABLE',station_range_m=[start,end])
            return result
        low,high=first['adjacent_s'],last['adjacent_s']
        if high<=low:
            result['failure']=dict(reason='NONMONOTONIC_CROSS_SECTION',station_range_m=[start,end],
                                   adjacent_station_range_m=[low,high])
            return result
        original=[(start,first['original'][0][:2])]+[(s,p[:2]) for s,p in
            zip(self.original_arc,self.original) if start<s<end]+[(end,last['original'][0][:2])]
        adjacent=[(start,first['adjacent'][0][:2])]+[(max(start,min(end,s)),p[:2]) for a,s,p in
            zip(self.adjacent_arc,self.adjacent_stations,self.adjacent) if low<a<high]+[(end,last['adjacent'][0][:2])]
        if any(b[0]<a[0]-1e-9 for a,b in zip(adjacent,adjacent[1:])):
            result['failure']=dict(reason='NONMONOTONIC_BOUNDARY',station_range_m=[start,end])
            return result
        anchors={start,end}|{s for s in self.original_arc+self.adjacent_stations if start<s<end}
        result['stations']=sorted(anchors)
        result['normalization']=[dict(removed_grid_station_m=start+i*.5,
            reason='EXACT_PIECEWISE_LINEAR_BOUNDARY') for i in range(1,math.ceil((end-start)/.5))
            if start+i*.5<end and start+i*.5 not in anchors]
        polygon=[p for s,p in original]+[p for s,p in reversed(adjacent)]
        # Reject real boundary crossings, including crossing end caps. This is
        # separate from the obsolete normal-row ordering test.
        edges=list(zip(polygon,polygon[1:]+polygon[:1]))
        for i,(a,b) in enumerate(edges):
            for j in range(i+2,len(edges)):
                if i==0 and j==len(edges)-1:
                    continue
                c,d=edges[j]
                if (max(a[0],b[0])<min(c[0],d[0]) or max(c[0],d[0])<min(a[0],b[0]) or
                        max(a[1],b[1])<min(c[1],d[1]) or max(c[1],d[1])<min(a[1],b[1])):
                    continue
                ab=(b[0]-a[0],b[1]-a[1]);cd=(d[0]-c[0],d[1]-c[1])
                turns=(cross(ab,(c[0]-a[0],c[1]-a[1])),cross(ab,(d[0]-a[0],d[1]-a[1])),
                       cross(cd,(a[0]-c[0],a[1]-c[1])),cross(cd,(b[0]-c[0],b[1]-c[1])))
                if turns[0]*turns[1]<=0. and turns[2]*turns[3]<=0.:
                    result['failure']=dict(reason='CROSSING_BOUNDARIES',boundary_edge_indices=[i,j],
                                           station_range_m=[start,end])
                    return result
        # Zipper the two ordered vertex chains. Each diagonal must have the
        # declared orientation; an invalid/nonmonotone strip fails closed.
        sign=1. if self.side=='left' else -1.
        cells=[];i=j=0
        while i<len(original)-1 or j<len(adjacent)-1:
            if j==len(adjacent)-1 or (i<len(original)-1 and original[i+1][0]<=adjacent[j+1][0]):
                triangle=[original[i][1],original[i+1][1],adjacent[j][1]];i+=1
            else:
                triangle=[original[i][1],adjacent[j+1][1],adjacent[j][1]];j+=1
            a,b,c=triangle
            area=cross((b[0]-a[0],b[1]-a[1]),(c[0]-a[0],c[1]-a[1]))
            if sign*area<0.:
                result['failure']=dict(reason='NONMONOTONIC_BOUNDARY',quad_index=len(cells),
                                       quad=triangle,station_range_m=[original[i][0],adjacent[j][0]])
                return result
            if area:
                cells.append(triangle)
        result['quads']=cells
        return result

    def quads(self,start,end):
        return self.corridor_envelope(start,end)['quads']

    def suffix(self, lane, start, end):
        geometry=self.adjacent if lane=='adjacent' else self.original
        first,last=sample(geometry,start),sample(geometry,end)
        if not first or not last or end<start:
            return []
        points=[first[0]]
        for s,p in zip(arc(geometry),geometry):
            if start+1e-9<s<end-1e-9:
                points.append(list(p))
        if math.dist(points[-1][:2],last[0][:2])>1e-9:
            points.append(last[0])
        return points
