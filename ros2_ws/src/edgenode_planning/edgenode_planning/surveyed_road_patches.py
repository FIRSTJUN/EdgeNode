"""Local road strips between actual lower C3 boundaries, without gap joining.

Only exact shared endpoints connect surveyed parts. Curbs/guardrails must stay
on opposite sides of a real connected MGeo route. Use the closest consistent
face on each side, preserve every source vertex, and keep unsupported caps out.
This is not a polygon hull of a junction or classification of unbounded land.
"""
from collections import defaultdict
import math

from .hdmap_audit import read_layer,translated,nearest
from .lane_geometry import PolylineIndex,arc,sample
from .route_clearance import _native_slice
from .road_surface import triangulate
from .avoidance_geometry import area


def _chains(parts):
    endpoints=defaultdict(list)
    for i,(ids,p) in enumerate(parts):
        endpoints[tuple(p[0][:2])].append(i);endpoints[tuple(p[-1][:2])].append(i)
    unused=set(range(len(parts)));result=[]
    while unused:
        first=min(unused);unused.remove(first);ids,p=parts[first];p=list(p);ids=set(ids)
        for reverse in (False,True):
            if reverse:p.reverse()
            while True:
                at=tuple(p[-1][:2]);connections=endpoints[at]
                choices=[i for i in connections if i in unused]
                if len(connections)!=2 or len(choices)!=1:break
                i=choices[0];unused.remove(i);other_ids,q=parts[i]
                if tuple(q[0][:2])!=at:q=tuple(reversed(q))
                p.extend(q[1:]);ids.update(other_ids)
        result.append((tuple(sorted(ids)),tuple(p)))
    return result


def local_patches(boundaries,reference):
    from shapely.geometry import Polygon,Point,LineString
    if len(reference)<2:return []
    if not hasattr(boundaries,'_road_protection_parts'):
        boundaries._road_protection_parts=[((f['attributes']['ID'],),p)
            for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')
            if f['attributes']['Type'] in ('2','3','4') and f['attributes']['LowHigh']=='2'
            for p in translated(f,boundaries.origin) if len(p)>1]
    xs=[p[0] for p in reference];ys=[p[1] for p in reference]
    bounds=(min(xs)-20,max(xs)+20,min(ys)-20,max(ys)+20)
    parts=[]
    for ids,p in boundaries._road_protection_parts:
        px=[q[0] for q in p];py=[q[1] for q in p]
        if max(px)<bounds[0] or min(px)>bounds[1] or max(py)<bounds[2] or min(py)>bounds[3]:continue
        parts.append((ids,p))
    candidates={'left':[],'right':[]};total=arc(reference)[-1]
    for ids,chain in _chains(parts):
        for direction in (chain,tuple(reversed(chain))):
            projection=[nearest(reference,p) for p in direction]
            for side,sign in (('left',1),('right',-1)):
                run=[];stations=[]
                def finish():
                    if len(run)>1 and stations[-1]-stations[0]>=8.:
                        candidates[side].append(dict(ids=ids,points=tuple(run),index=PolylineIndex(run),
                            low=stations[0],high=stations[-1]))
                for p,q in zip(direction,projection):
                    valid=.5<sign*q[2]<20.
                    if not valid or stations and q[1]<stations[-1]-.01:
                        finish();run=[];stations=[]
                    if valid:run.append(p);stations.append(q[1])
                finish()
    result=[];seen=set()
    for left in candidates['left']:
        for right in candidates['right']:
            low=max(0.,left['low'],right['low'])+.25
            high=min(total,left['high'],right['high'])-.25
            if high-low<8.:continue
            selected={'left':left,'right':right};valid=True;caps=[]
            for s in [low+(high-low)*f for f in (0.,.1,.3,.5,.7,.9,1.)]:
                p,t=sample(reference,s);row=[]
                for side in ('left','right'):
                    chosen=selected[side];hit=chosen['index'].normal_intersection(p,t,side)
                    if hit is None:valid=False;break
                    distance=math.dist(p[:2],hit[1][0][:2])
                    for other in candidates[side]:
                        if other is chosen or not other['low']<=s<=other['high']:continue
                        alternative=other['index'].normal_intersection(p,t,side)
                        if alternative and math.dist(p[:2],alternative[1][0][:2])<distance-1e-6:
                            valid=False;break
                    if not valid:break
                    row.append(hit)
                if not valid:break
                caps.append(row)
            if not valid:continue
            a=_native_slice(left['points'],caps[0][0][0],caps[-1][0][0])
            b=_native_slice(right['points'],caps[0][1][0],caps[-1][1][0])
            ring=tuple(p[:2] for p in a+tuple(reversed(b)));shape=Polygon(ring)
            if (not shape.is_valid or shape.is_empty or
                    not all(shape.covers(Point(sample(reference,low+(high-low)*f)[0][:2]))
                            for f in (.01,.25,.5,.75,.99))):continue
            selected_ids=set(left['ids']+right['ids'])
            if any(not selected_ids.intersection(ids) and shape.relate_pattern(LineString(p),'T********')
                   for ids,p in parts):continue
            excluded=False
            for feature in read_layer(boundaries.directory/'A3_DRIVEWAYSECTION'):
                if feature['attributes']['Kind']!='7':continue
                for part in translated(feature,boundaries.origin):
                    exclusion=Polygon([p[:2] for p in part])
                    if not exclusion.is_valid or shape.intersects(exclusion):excluded=True;break
                if excluded:break
            if excluded:continue
            token=ring
            if token in seen:continue
            seen.add(token)
            cells=tuple(c for c in triangulate(ring) if area(c)>1e-8)
            result.append(dict(cells=cells,ring=ring,source_ids=tuple(sorted(set(left['ids']+right['ids']))),
                station_range_m=(low,high),source='NGII_LOWER_C3_EXACT_CONNECTED_LOCAL_STRIP'))
    return result
