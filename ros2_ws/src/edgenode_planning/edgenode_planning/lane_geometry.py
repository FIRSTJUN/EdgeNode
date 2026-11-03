"""MGeo lane pairs and conservative center-to-center corridors; no routing edits."""
import math

from .lattice_planner import LatticePlanner


EPS = 1e-7


def arc(points):
    return LatticePlanner._arc_lengths(points)


def project(points, point, lengths=None):
    return LatticePlanner._project_reference(points, arc(points) if lengths is None else lengths, point)[0]


def sample(points, s, lengths=None):
    """Interpolate within actual geometry; never extend a map link."""
    lengths = arc(points) if lengths is None else lengths
    if not points or s < -EPS or s > lengths[-1]+EPS:
        return None
    s = max(0., min(s, lengths[-1]))
    for i, (a, b) in enumerate(zip(points, points[1:])):
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


def cross(a,b):
    return a[0]*b[1]-a[1]*b[0]


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
    def __init__(self, planner):
        self.links = planner.links_by_id
        self.valid = {edge['link_id'] for edges in planner.graph.values() for edge in edges}

    def neighbor(self, original_id, side):
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

    def quads(self,start,end):
        # Include exact map vertices in addition to the half-metre stations.
        stations = {start,end}
        stations.update(s for s in self.original_arc if start<s<end)
        stations.update(s for s in self.adjacent_stations if start<s<end)
        stations.update(start+i*.5 for i in range(1,math.ceil((end-start)/.5)) if start+i*.5<end)
        rows=[self.at(s) for s in sorted(stations)]
        if any(row is None for row in rows):
            return []
        quads=[]
        for a,b in zip(rows,rows[1:]):
            quad=[a['original'][0][:2],b['original'][0][:2],b['adjacent'][0][:2],a['adjacent'][0][:2]]
            turns=[cross((q[0]-p[0],q[1]-p[1]),(r[0]-q[0],r[1]-q[1]))
                   for p,q,r in zip(quad,quad[1:]+quad[:1],quad[2:]+quad[:2])]
            if not (all(v>=-EPS for v in turns) or all(v<=EPS for v in turns)):
                return []
            quads.append(quad)
        return quads

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
