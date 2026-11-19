"""Road-traffic candidates and partial-face box observations, without truth.

The rectangle dimensions are an explicit passenger-car prior. They are not
measured Actor dimensions. Raw Perception collision solids are never changed.
"""
import math
import numpy as np
from edgenode_planning.map_matcher import closest_point_on_link
from .kinematics import Observation

def nearest_native(matcher,center):
    x,y=center;rows=[]
    for link in matcher.links:
        if matcher.bbox_candidate(x,y,link['_bbox']):
            distance,heading=closest_point_on_link(x,y,link);rows.append((distance,heading,link['idx']))
    return min(rows) if rows else None

def vehicle_observation(observation,ego,matcher,*,length_m=4.93,width_m=2.12,predicted_centers=()):
    nearby=nearest_native(matcher,observation.center_xy)
    if nearby is None or nearby[0]>2.2:return None,dict(reason='NO_NEAR_NATIVE_TRAFFIC_LINK',center_xy=observation.center_xy)
    _,heading,link=nearby;yaw=math.radians(heading);forward=np.array((math.cos(yaw),math.sin(yaw)));left=np.array((-forward[1],forward[0]))
    vertices=np.array([p for piece in observation.footprints for p in piece],dtype=float)
    along=vertices@forward;across=vertices@left;lo,hi=float(along.min()),float(along.max());right,le=float(across.min()),float(across.max())
    if max(hi-lo,le-right)<.5:return None,dict(reason='INSUFFICIENT_VEHICLE_SURFACE_SUPPORT',center_xy=observation.center_xy)
    length=max(length_m,hi-lo);width=max(width_m,le-right)
    eye=np.asarray(ego[:2]);eye_a=float(eye@forward);eye_b=float(eye@left)
    a=(lo+hi)/2 if hi-lo>=.8*length else lo+length/2 if eye_a<(lo+hi)/2 else hi-length/2
    b=(right+le)/2 if le-right>=.8*width else right+width/2 if eye_b<(right+le)/2 else le-width/2
    candidates_a=[a,lo+length/2,hi-length/2]
    candidates_b=[b,right+width/2,le-width/2]
    if hi-lo>=.6*length:candidates_a.append((lo+hi)/2)
    if le-right>=.6*width:candidates_b.append((right+le)/2)
    default=a*forward+b*left;prediction=None;distance=math.inf
    for center in predicted_centers:
        d=float(np.linalg.norm(np.asarray(center)-default))
        if d<distance:distance=d;prediction=np.asarray(center)
    if prediction is not None and distance<3.:
        # Earlier source-time geometry resolves a changing visible face.
        # It cannot snap a vehicle or authorize motion; only an observation
        # hypothesis is selected before the existing gated KF association.
        choices=[aa*forward+bb*left for aa in candidates_a for bb in candidates_b]
        chosen=min(choices,key=lambda p:float(np.linalg.norm(p-prediction)))
    else:chosen=default
    fitted=Observation(tuple(map(float,chosen)),observation.footprints,
        surface_center_xy=observation.center_xy,model_length_m=length,model_width_m=width)
    return fitted,dict(reason='PASSENGER_RECTANGLE_VISIBLE_FACE_MODEL',native_link=link,
        raw_center_xy=observation.center_xy,fitted_center_xy=fitted.center_xy,
        length_prior_m=length_m,width_prior_m=width_m,observed_span_m=[hi-lo,le-right])
