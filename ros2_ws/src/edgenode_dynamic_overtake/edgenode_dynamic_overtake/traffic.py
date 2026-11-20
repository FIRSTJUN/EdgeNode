"""Source-time track adaptation and covariance-aware timed collision checks."""
import math
import numpy as np
from shapely.geometry import Polygon
from edgenode_planning.avoidance_geometry import swept_body,inflate,convex_hull,polygon_distance
from edgenode_planning.lane_geometry import PolylineIndex,sample

def rectangle(center,yaw,length,width):
    f=np.array((math.cos(yaw),math.sin(yaw)));l=np.array((-f[1],f[0]));c=np.asarray(center)
    return tuple(tuple(c+a*f+b*l) for a,b in ((-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)))

def projected_piece(track,seconds,source_age,acceleration_std,lateral_std=.15):
    t=seconds+source_age;v=np.asarray(track['velocity_xy_mps']);center=np.asarray(track['center_xy'])+t*v
    H=np.array(((1.,0.,t,0.),(0.,1.,0.,t)));G=np.eye(2)*t*t/2
    yaw=track['heading_rad'];forward=np.array((math.cos(yaw),math.sin(yaw)));left=np.array((-forward[1],forward[0]))
    lateral_std=lateral_std if track.get('road_motion_conditioned',False) else acceleration_std
    process=acceleration_std**2*np.outer(forward,forward)+lateral_std**2*np.outer(left,left)
    covariance=H@np.asarray(track['covariance'])@H.T+G@process@G.T
    along=2*math.sqrt(max(0.,float(forward@covariance@forward)))+.1
    across=2*math.sqrt(max(0.,float(left@covariance@left)))+.1
    padding=[a*forward+b*left for a,b in ((-along,-across),(along,-across),(along,across),(-along,across))]
    shape=rectangle(center,yaw,track['vehicle_length_model_m'],track['vehicle_width_model_m'])
    return convex_hull([tuple(np.asarray(p)+offset) for p in shape for offset in padding])

def assess(points,pose,speed_kmh,tracks,body,*,now_ns,source_ns,acceleration_std=1.5,horizon_s=2.,margin_m=.25,minimum_speed_kmh=None):
    age=(now_ns-source_ns)/1e9
    if not -.05<=age<=.6:return dict(ready=False,reason='D_TRAFFIC_SOURCE_STALE')
    index=PolylineIndex(points);station=max(0.,index.project(pose[:2]));speed=speed_kmh/3.6
    lower=speed if minimum_speed_kmh is None else minimum_speed_kmh/3.6
    if not 0<=lower<=speed:return dict(ready=False,reason='D_INVALID_EGO_SPEED_ENVELOPE')
    horizon_s=max(horizon_s,.3+speed/(2*4.)+.6)
    if index.lengths[-1]-station<speed*horizon_s+1:return dict(ready=False,reason='D_SHORT_TIMED_PATH')
    previous={};minimum=math.inf;last_t=0.
    for t in np.arange(0.,horizon_s+.05,.1):
        # Include all intermediate progress values, not just the two speed
        # endpoints. The same hull encloses the preceding temporal interval.
        lo=station+lower*last_t;hi=station+speed*t;stations=[lo]
        stations.extend(s for s in index.lengths if lo<s<hi);stations.append(hi)
        first,tangent=sample(points,lo,index.lengths)
        prior=pose if last_t==0. else (*first[:2],math.atan2(tangent[1],tangent[0]));pieces=[]
        for s in stations:
            point,tangent=sample(points,s,index.lengths);current=(*point[:2],math.atan2(tangent[1],tangent[0]))
            pieces.extend(swept_body(body,prior,current));prior=current
        ego=convex_hull(pieces)
        for track in tracks:
            if (now_ns-track['observed_ns'])/1e9>.6:return dict(ready=False,reason='D_TRAFFIC_TRACK_STALE')
            shape=projected_piece(track,float(t),max(0.,age),acceleration_std);before=previous.get(track['id'],shape);gap=polygon_distance(ego,convex_hull(list(before)+list(shape)));minimum=min(minimum,gap)
            if gap<=margin_m:return dict(ready=False,reason='D_PREDICTED_TRAFFIC_CONFLICT',track=track['id'],time_s=float(t),gap_m=gap,horizon_s=horizon_s)
            previous[track['id']]=shape
        last_t=float(t)
    return dict(ready=True,reason='D_TIMED_WINDOW_CLEAR',minimum_gap_m=None if not math.isfinite(minimum) else minimum,horizon_s=horizon_s,source_age_s=age,ego_speed_envelope_kmh=[lower*3.6,speed*3.6])
