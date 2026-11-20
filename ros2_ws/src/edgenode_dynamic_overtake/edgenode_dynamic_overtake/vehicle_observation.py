"""D-only visible-face inference consistent with the actual sensor viewpoint.

Prediction may disambiguate occlusion, but cannot select the opposite visible
face of the same passenger rectangle. All original collision pieces survive.
"""
import numpy as np
import math
from dataclasses import dataclass
from edgenode_dynamic_tracking.vehicle_observation import vehicle_observation as original_observation,nearest_native
from edgenode_dynamic_tracking.kinematics import Observation

@dataclass(frozen=True)
class PartialVehicleObservation(Observation):
    point_axes: tuple = ()
    interval_axes: tuple = ()

def vehicle_observation(observation,ego,matcher,**kwargs):
    fitted,model=original_observation(observation,ego,matcher,**kwargs)
    if fitted is None:return fitted,model
    native=nearest_native(matcher,observation.center_xy);yaw=math.radians(native[1]);forward=np.array((math.cos(yaw),math.sin(yaw)));left=np.array((-forward[1],forward[0]))
    vertices=np.asarray([p for piece in observation.footprints for p in piece]);eye=np.asarray(ego[:2]);center=np.asarray(fitted.center_xy).copy();constraints=[]
    points=[];intervals=[];prediction=None
    candidates=kwargs.get('predicted_centers',())
    if candidates:
        candidate=min(candidates,key=lambda p:float(np.linalg.norm(np.asarray(p)-center)))
        if np.linalg.norm(np.asarray(candidate)-center)<3.:prediction=np.asarray(candidate)
    spans=[float(np.ptp(vertices@axis)) for axis in (forward,left)]
    for i,(name,axis,size) in enumerate([('longitudinal',forward,fitted.model_length_m),('lateral',left,fitted.model_width_m)]):
        values=vertices@axis;lo,hi=float(values.min()),float(values.max());span=hi-lo;view=float(eye@axis)
        # A long side patch does not measure the car's front/rear face.
        # Only a thin patch normal to this axis supports a visible-face point.
        thin=span<=.5*size and span<=spans[1-i]
        if span>=.8*size:chosen=(lo+hi)/2;kind='FULL_SPAN_MIDPOINT';points.append(tuple(axis))
        elif thin and view<lo-.25:chosen=lo+size/2;kind='LOW_VISIBLE_FACE';points.append(tuple(axis))
        elif thin and view>hi+.25:chosen=hi-size/2;kind='HIGH_VISIBLE_FACE';points.append(tuple(axis))
        else:
            low,high=hi-size/2,lo+size/2
            chosen=(low+high)/2 if prediction is None else max(low,min(high,float(prediction@axis)))
            kind='VISIBLE_SUPPORT_INTERVAL';intervals.append((tuple(axis),low,high))
        center+=(chosen-float(center@axis))*axis;constraints.append(dict(axis=name,kind=kind,observed_bounds=[lo,hi],ego_projection=view))
    corrected=PartialVehicleObservation(tuple(map(float,center)),fitted.footprints,fitted.surface_center_xy,fitted.model_length_m,fitted.model_width_m,tuple(points),tuple(intervals))
    model.update(base_reason=model['reason'],reason='D_EGO_VIEW_CONSISTENT_PASSENGER_FACE_PRIOR',view_constraints=constraints,view_correction_m=float(np.linalg.norm(center-np.asarray(fitted.center_xy))),fitted_center_xy=corrected.center_xy,raw_collision_solids_unchanged=True,observed_position_axes=len(points),unobserved_position_intervals=len(intervals))
    return corrected,model
