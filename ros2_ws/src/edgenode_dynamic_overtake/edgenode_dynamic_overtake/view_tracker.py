"""D observation adapter; unchanged C association/static-memory/row contracts.

The update body is preserved from completed C, using only D's physically
view-consistent observation factory. No truth or control interface is added.
"""
import math
import numpy as np
from edgenode_dynamic_tracking.tracker import TrafficTracker
from edgenode_dynamic_tracking.kinematics import Observation,observations_from_frame
from edgenode_dynamic_tracking.stationary_hypothesis import StationaryHypothesis
from .vehicle_observation import vehicle_observation
from edgenode_dynamic_tracking.vehicle_observation import nearest_native

def joint_surface_support(first,second,matcher,length,width):
    # Association to the SAME prior ID is checked by the caller. Raw support
    # must also fit one passenger body, so adjacent/nose-to-tail cars stay apart.
    center=(np.asarray(first.center_xy)+np.asarray(second.center_xy))/2
    native=nearest_native(matcher,center)
    if native is None:return False
    yaw=math.radians(native[1]);forward=np.array((math.cos(yaw),math.sin(yaw)));left=np.array((-forward[1],forward[0]))
    vertices=np.asarray([p for piece in first.footprints+second.footprints for p in piece])
    return float(np.ptp(vertices@forward))<=length+.6 and float(np.ptp(vertices@left))<=width+.6

class ViewConsistentTracker(TrafficTracker):
    @classmethod
    def from_existing(cls,tracker):
        result=cls.__new__(cls);result.__dict__.update(tracker.__dict__)
        return result
    def update(self,envelope,ego=None):
        self.observations=observations_from_frame(envelope)
        observations=self.observations;self.observation_models=[];self.nontraffic=[]
        if ego is not None:
            stamp=envelope['status']['source_stamp_ns'];dt=0. if self.filter.source_ns is None else (stamp-self.filter.source_ns)/1e9
            predicted=[t.state[:2]+dt*t.state[2:] for t in self.filter.tracks.values() if stamp-t.observed_ns<=self.filter.max_missed_ns]
            observations=[];owner_ids=[]
            for track in self.filter.tracks.values():
                if track.identifier not in self.stationary and track.motion=='STATIC' and track.observation.model_length_m is not None:
                    matched,yaw,source=self.lane(track)
                    if yaw is not None:self.stationary[track.identifier]=StationaryHypothesis(track,ego,yaw)
            for raw in self.observations:
                fitted,model=vehicle_observation(raw,ego,self.matcher,length_m=self.length_prior,width_m=self.width_prior,predicted_centers=predicted)
                if fitted is None:self.nontraffic.append(model)
                else:
                    nearest=min(self.filter.tracks.values(),key=lambda t:float(np.linalg.norm(t.state[:2]+dt*t.state[2:]-np.asarray(fitted.center_xy))),default=None)
                    owner=None
                    if nearest is not None and np.linalg.norm(nearest.state[:2]+dt*nearest.state[2:]-np.asarray(fitted.center_xy))<3.:
                        owner=nearest.identifier;hypothesis=self.stationary.get(owner)
                        if hypothesis is not None:
                            if hypothesis.consistent(raw,ego):
                                fitted=Observation(tuple(hypothesis.center),raw.footprints,raw.center_xy,hypothesis.length,hypothesis.width)
                                model.update(stationary_hypothesis=True,stationary_anchor_xy=hypothesis.center.tolist(),fitted_center_xy=fitted.center_xy)
                            else:
                                self.stationary.pop(owner,None);model['stationary_hypothesis_rejected']=True
                    # Multiple disconnected surfaces associated to one prior
                    # vehicle are one measurement, not two persistent IDs.
                    merge=next((i for i,old in enumerate(observations) if owner is not None and owner_ids[i]==owner and joint_surface_support(old,fitted,self.matcher,self.length_prior,self.width_prior)),None)
                    if merge is None:observations.append(fitted);owner_ids.append(owner)
                    else:
                        old=observations[merge];n=len(old.footprints);m=len(fitted.footprints)
                        surface=(n*np.asarray(old.surface_center_xy)+m*np.asarray(fitted.surface_center_xy))/(n+m)
                        combined=Observation(tuple(surface),old.footprints+fitted.footprints)
                        joint,joint_model=vehicle_observation(combined,ego,self.matcher,length_m=self.length_prior,width_m=self.width_prior,predicted_centers=predicted)
                        if joint is None:raise ValueError('source-owned joint surface lost native support')
                        hypothesis=self.stationary.get(owner)
                        if hypothesis is not None:
                            if hypothesis.consistent(combined,ego):joint=Observation(tuple(hypothesis.center),joint.footprints,joint.surface_center_xy,hypothesis.length,hypothesis.width)
                            else:self.stationary.pop(owner,None)
                        observations[merge]=joint
                        model.update(joint_model,merged_disconnected_surfaces_of_prior_track=owner,joint_surface_bounded_by_one_passenger_body=True)
                    self.observation_models.append(model)
        tracks=self.filter.update(envelope['status']['source_stamp_ns'],observations)
        # A reused static anchor is a correlated position hypothesis. New
        # partial surfaces must not collapse its position covariance below
        # the underlying observation variance by repetition alone.
        for track in tracks:
            if track.identifier in self.stationary:
                minimum=float(np.linalg.eigvalsh(track.covariance[:2,:2]).min())
                if minimum<self.filter.measurement_var:
                    track.covariance[:2,:2]+=np.eye(2)*(self.filter.measurement_var-minimum)
        self.stationary={k:v for k,v in self.stationary.items() if k in self.filter.tracks}
        self.lanes={k:v for k,v in self.lanes.items() if k in self.filter.tracks}
        return tracks
