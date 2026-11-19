"""Exact surveyed lower road boundaries for ordinary single lanes.

Only existing full-length B2 or lower C3 guardrail/wall/curb faces qualify.
Missing sections and endpoints stay missing; no width or connector is inferred.
"""
import hashlib
import json
import math
from pathlib import Path

from .hdmap_audit import read_layer,translated,marking_definition,nearest
from .lane_geometry import PolylineIndex,arc,sample,curvature_at
from .road_surface import triangulate
from .road_corridor import VerifiedRoadCorridor
from .route_clearance import _native_slice,boundary_contacts
from .lattice_path import poses,curvatures
from .avoidance_geometry import swept_body,inflate,area


def evidence(boundaries,network,key):
    from shapely.geometry import Polygon,Point,LineString
    link=network.links[key]
    if link.get('link_type') not in ('6',6):return None
    if any(link.get(side+'_lane_change_dst_link_idx') for side in ('left','right')):return None
    if key not in boundaries.aligned_links:return None
    center=link['points'];index=PolylineIndex(center);total=index.lengths[-1]
    if total<60.:return None
    known=[];missing=[]
    for side in ('left','right'):
        options=[(a,parts[0]) for a,parts in boundaries.observed_sides.get((key,side),())
            if len(parts)==1 and a['Kind'] in ('501','503','505','506') and a['Type']!='999'
            and marking_definition(a['Type'],a['Kind'])['definition_confirmed']]
        if len(options)==1 and abs(index.project(options[0][1][-1])-index.project(options[0][1][0]))>=.9*total:
            known.append((side,*options[0]))
        elif not options:missing.append(side)
        elif len(options)==1:missing.append(side)
        else:return None
    if not missing:return None
    if not hasattr(boundaries,'_concrete_curbs'):
        boundaries._concrete_curbs=[(f['attributes'],p) for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')
            if f['attributes']['Type'] in ('2','3','4') and f['attributes']['LowHigh']=='2'
            for p in translated(f,boundaries.origin) if len(p)>1]
    selected={side:(a,p) for side,a,p in known};center_line=LineString(center)
    for side in missing:
        sign=1 if side=='left' else -1;options=[]
        for curb_attr,part in boundaries._concrete_curbs:
            if LineString(part).distance(center_line)>12.:continue
            projection=[nearest(center,p) for p in part]
            if any(not .5<sign*r[2]<12. for r in projection):continue
            stations=[r[1] for r in projection]
            if stations[-1]<stations[0]:part=tuple(reversed(part));stations.reverse()
            if any(b<a-.01 for a,b in zip(stations,stations[1:])):continue
            if stations[-1]-stations[0]<.9*total:continue
            options.append((sum(sign*r[2] for r in projection)/len(projection),curb_attr,part))
        if not options:return None
        options.sort(key=lambda r:r[0]);_,a,p=options[0]
        # Multiple surveyed faces of one guardrail are allowed only when the
        # same closest face stays closest over every shared cross section.
        chosen=PolylineIndex(p)
        for _,_,other in options[1:]:
            alternative=PolylineIndex(other)
            for s in (total*f/10 for f in range(1,10)):
                q,t=sample(center,s);x=chosen.normal_intersection(q,t,side);y=alternative.normal_intersection(q,t,side)
                if x and y and math.dist(y[1][0][:2],q[:2])<math.dist(x[1][0][:2],q[:2])-1e-9:return None
        selected[side]=(a,p)
    lines=[]
    for side in ('left','right'):
        a,line=selected[side]
        if index.project(line[-1])<index.project(line[0]):line=tuple(reversed(line))
        lines.append(line)
    left,right=lines
    low=max(0.,index.project(left[0]),index.project(right[0]))+.5
    high=min(total,index.project(left[-1]),index.project(right[-1]))-.5
    if high-low<.85*total:return None
    li,ri=PolylineIndex(left),PolylineIndex(right)
    ends=[]
    for s in (low,high):
        p,t=sample(center,s);a=li.normal_intersection(p,t,'left');b=ri.normal_intersection(p,t,'right')
        if a is None or b is None:return None
        ends.append((a,b))
    l=_native_slice(left,ends[0][0][0],ends[1][0][0]);r=_native_slice(right,ends[0][1][0],ends[1][1][0])
    ring=[p[:2] for p in l+tuple(reversed(r))];shape=Polygon(ring)
    if not shape.is_valid or shape.is_empty:return None
    for feature in read_layer(boundaries.directory/'A3_DRIVEWAYSECTION'):
        if feature['attributes']['Kind']!='7':continue
        for part in translated(feature,boundaries.origin):
            exclusion=Polygon([p[:2] for p in part])
            if not exclusion.is_valid or shape.intersects(exclusion):return None
    if not all(shape.covers(Point(sample(center,s)[0][:2])) for s in (low+1.,(low+high)/2,high-1.)):return None
    cells=tuple(cell for cell in triangulate(ring) if area(cell)>1e-8)
    ids=[selected[side][0]['ID'] for side in ('left','right')]
    source='NGII_SURVEYED_LOWER_ROAD_PROTECTION:'+key+':'+':'.join(ids)
    road=VerifiedRoadCorridor(cells,source=source,link_ids=(key,),allowed_sides=('left','right'),exact_union_backend=True)
    return dict(link=key,low=low,high=high,left=li,right=ri,road=road,
        source=source,marking_id=ids[0],curb_id=ids[1],ring=ring)


def centered_path(boundaries,network,body,item,margin=.15):
    import numpy as np
    from scipy.interpolate import make_interp_spline
    points=network.links[item['link']]['points'];total=arc(points)[-1]
    start=item['low']+10.;end=item['high']-12.
    if end-start<50.:raise ValueError('insufficient observed centering support')
    count=math.ceil((end-start)/5);stations=[start+(end-start)*i/count for i in range(count+1)];targets=[]
    def smooth(u):
        u=max(0.,min(1.,u));return u*u*u*(10.-15.*u+6.*u*u)
    offsets=[]
    for s in stations:
        p,t=sample(points,s);a=item['left'].normal_intersection(p,t,'left');b=item['right'].normal_intersection(p,t,'right')
        if a is None or b is None:raise ValueError('missing measured cross section')
        middle=((a[1][0][0]+b[1][0][0])/2,(a[1][0][1]+b[1][0][1])/2)
        weight=smooth((s-start)/25.)*smooth((end-s)/25.)
        targets.append([p[k]+weight*(middle[k]-p[k]) for k in (0,1)])
        offsets.append(weight*math.dist(middle,p[:2]))
    conditions=[]
    for s in (start,end):
        _,t=sample(points,s);k=curvature_at(points,s)
        conditions.append([(1,list(t)),(2,[-k*t[1],k*t[0]])])
    spline=make_interp_spline(stations,np.array(targets),k=5,bc_type=tuple(conditions))
    dense=np.linspace(start,end,math.ceil((end-start)/.25)+1)
    corrected=tuple((float(x),float(y),sample(points,float(s))[0][2]) for s,(x,y) in zip(dense,spline(dense)))
    if max(map(abs,curvatures(corrected)),default=math.inf)>.08:raise ValueError('centering curvature')
    previous=None
    for pose in poses(corrected):
        polygon=inflate(swept_body(body,previous or pose,pose),margin);previous=pose
        if boundary_contacts(boundaries,polygon):raise ValueError('centering forbidden boundary')
        if not item['road'].covers(polygon):raise ValueError('centering body outside measured road')
    path=_native_slice(points,0.,start)+corrected[1:]+_native_slice(points,end,total)[1:]
    return path,dict(link=item['link'],source=item['source'],marking_id=item['marking_id'],curb_id=item['curb_id'],
        changed_station_range_m=[start,end],max_center_shift_m=max(offsets),tracking_margin_m=margin,
        unchanged_caps_are_not_reclassified=True,
        authority='NGII 2019 pp101-104; Type2/3/4 surveyed lower guardrail/wall/curb, LowHigh2',
        manual_sha256='93958d4079dcdde311368753049f0df95238122ab6bde0ea241fed89499a6541')
