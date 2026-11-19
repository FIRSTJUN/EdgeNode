"""C-owned lane/relative-motion/uncertainty/visibility outputs; no Actor API."""
import math
import numpy as np
from shapely.geometry import LineString,Polygon
from edgenode_planning.map_matcher import MapMatcher,closest_point_on_link
from .kinematics import ObstacleTracking,observations_from_frame
from .vehicle_observation import vehicle_observation
from .stationary_hypothesis import StationaryHypothesis
from .kinematics import Observation

class TrafficTracker:
    def __init__(self,mgeo_dir,*,measurement_std_m=.2,acceleration_std_mps2=.8,max_missed_s=1.2,vehicle_length_prior_m=4.93,vehicle_width_prior_m=2.12,hdmap_dir=None):
        self.filter=ObstacleTracking(measurement_std_m,acceleration_std_mps2,max_missed_s)
        self.matcher=MapMatcher(mgeo_dir,max_match_distance_m=2.2,max_heading_diff_deg=75.)
        self.native_reference_evidence='MGE0_ONLY_NOT_A_LANE_PERMISSION'
        if hdmap_dir is not None:
            from edgenode_planning.hdmap_boundaries import HDMapBoundaries
            boundaries=HDMapBoundaries(hdmap_dir,mgeo_dir)
            self.matcher.links=[link for link in self.matcher.links if link['idx'] in boundaries.aligned_links]
            if not self.matcher.links:raise ValueError('no surveyed native traffic references')
            self.native_reference_evidence='ORIGINAL_HD_MAP_A2_CENTERLINE_ALIGNMENT_WITHIN_0.05M'
        self.stationary={};self.lanes={};self.observations=[]
        self.length_prior=vehicle_length_prior_m;self.width_prior=vehicle_width_prior_m;self.observation_models=[];self.nontraffic=[]
    def reset(self):self.filter.reset();self.stationary={};self.lanes={};self.observations=[];self.observation_models=[];self.nontraffic=[]
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
                    merge=next((i for i,old in enumerate(observations) if owner is not None and owner_ids[i]==owner and np.linalg.norm(np.asarray(old.center_xy)-np.asarray(fitted.center_xy))<=1.),None)
                    if merge is None:observations.append(fitted);owner_ids.append(owner)
                    else:
                        old=observations[merge];n=len(old.footprints);m=len(fitted.footprints)
                        center=(n*np.asarray(old.center_xy)+m*np.asarray(fitted.center_xy))/(n+m)
                        surface=(n*np.asarray(old.surface_center_xy)+m*np.asarray(fitted.surface_center_xy))/(n+m)
                        observations[merge]=Observation(tuple(center),old.footprints+fitted.footprints,tuple(surface),max(old.model_length_m,fitted.model_length_m),max(old.model_width_m,fitted.model_width_m))
                        model['merged_disconnected_surfaces_of_prior_track']=owner
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
    def lane(self,track):
        x,y=map(float,track.state[:2]);speed=track.speed_mps
        if track.motion=='MOVING' and speed>.3:yaw=math.atan2(track.state[3],track.state[2]);source='LIDAR_WORLD_VELOCITY'
        else:
            choices=[closest_point_on_link(x,y,link) for link in self.matcher.links if self.matcher.bbox_candidate(x,y,link['_bbox'])]
            if not choices:return None,None,'UNAVAILABLE'
            distance,heading=min(choices);yaw=math.radians(heading);source='MAP_TANGENT_HEADING_UNOBSERVED'
        self.matcher.previous_link_id=self.lanes.get(track.identifier)
        matched=self.matcher.match(x,y,math.degrees(yaw))
        if matched:self.lanes[track.identifier]=matched['link_id']
        return matched,yaw,source
    def rows(self,now_ns,ego):
        epoch=self.filter.source_ns
        if epoch is None:return []
        pose,ego_speed=ego;forward=np.array((math.cos(pose[2]),math.sin(pose[2])));left=np.array((-forward[1],forward[0]));result=[]
        for track in self.filter.tracks.values():
            age=(now_ns-track.observed_ns)/1e9
            if age>self.filter.max_missed_ns/1e9:continue
            matched,yaw,heading_source=self.lane(track);relative=track.state[2:]-ego_speed*forward
            future=[]
            for seconds in (1.,2.,3.):
                H=np.array(((1.,0.,seconds,0.),(0.,1.,0.,seconds)))
                G=np.array(((seconds**2/2,0.),(0.,seconds**2/2)))
                covariance=H@track.covariance@H.T+self.filter.acceleration_var*(G@G.T)
                future.append(dict(seconds=seconds,center_xy=(track.state[:2]+seconds*track.state[2:]).tolist(),covariance_xy=covariance.tolist()))
            missed=track.observed_ns<epoch;visibility='OBSERVED'
            if missed:
                ray=LineString((pose[:2],track.state[:2]));visibility='MISSED'
                if any(Polygon(piece).intersects(ray) for obj in self.observations for piece in obj.footprints):visibility='LIKELY_OCCLUDED'
            shift=track.state[:2]-np.asarray(track.observation.center_xy)
            pieces=[[(float(p[0]+shift[0]),float(p[1]+shift[1])) for p in piece] for piece in track.observation.footprints]
            confidence=min(1.,track.frames/10)*math.exp(-max(0.,age)/(self.filter.max_missed_ns/1e9))/(1.+math.sqrt(max(0.,float(np.linalg.eigvalsh(track.covariance[:2,:2]).max()))))
            result.append(dict(id=track.identifier,center_xy=track.state[:2].tolist(),velocity_xy_mps=track.state[2:].tolist(),speed_mps=track.speed_mps,
                relative_velocity_xy_mps=relative.tolist(),relative_forward_mps=float(relative@forward),relative_left_mps=float(relative@left),
                heading_rad=yaw,heading_source=heading_source,lane=matched,lane_membership_verified=False,
                motion=track.motion,motion_sigma_mps=None if not math.isfinite(track.motion_sigma) else track.motion_sigma,
                position_reference='SOURCE_TIME_VISIBLE_FACE_PASSENGER_RECTANGLE_PRIOR',covariance=track.covariance.tolist(),footprints=pieces,
                observed_surface_center_xy=track.observation.surface_center_xy,
                vehicle_length_model_m=track.observation.model_length_m,vehicle_width_model_m=track.observation.model_width_m,
                vehicle_size_is_a_prior_not_actor_truth=True,stationary_partial_view_hypothesis=track.identifier in self.stationary,stationary_support_tolerance_m=.6 if track.identifier in self.stationary else None,velocity_valid=track.motion in ('MOVING','STATIC'),
                observed_ns=track.observed_ns,state_source_ns=epoch,age_s=age,missed=missed,visibility=visibility,
                frames=track.frames,confidence_score=confidence,confidence_is_calibrated_probability=False,
                usable=age<=.6 and track.frames>=5 and matched is not None and track.motion!='UNKNOWN',
                expires_ns=track.observed_ns+self.filter.max_missed_ns,future=future))
        return result
