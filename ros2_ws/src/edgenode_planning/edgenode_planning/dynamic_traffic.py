"""Source-time traffic prediction and continuous swept trajectory assessment.

Only LiDAR-associated tracks enter this module. Actor APIs are experiment-only.
"""
from dataclasses import dataclass
import math
import numpy as np

from .avoidance_geometry import convex_hull,polygon_distance,swept_body
from .lane_geometry import PolylineIndex,sample


@dataclass(frozen=True)
class TrafficAssessment:
    ready: bool
    reason: str
    minimum_gap_m: float
    completion_s: float
    conflict_track_id: int | None


def predicted_pieces(track,seconds,source_age_s=0.):
    """Translate measured pieces with anisotropic covariance and acceleration.

    Longitudinal acceleration uncertainty grows with time. The measured
    lateral state covariance is retained rather than inventing an actor lane.
    """
    if seconds<0 or source_age_s<0:raise ValueError('nonnegative prediction time required')
    t=seconds+source_age_s;state=track.state;covariance=track.covariance
    velocity=state[2:];speed=float(np.linalg.norm(velocity))
    forward=velocity/speed if speed>.1 else np.array((1.,0.))
    lateral=np.array((-forward[1],forward[0]))
    H=np.array(((1.,0.,t,0.),(0.,1.,0.,t)))
    position_covariance=H@covariance@H.T
    along=2*math.sqrt(max(0.,float(forward@position_covariance@forward)))+.25*t*t
    across=2*math.sqrt(max(0.,float(lateral@position_covariance@lateral)))
    padding=[forward*a+lateral*b for a,b in ((-along,-across),(along,-across),(along,across),(-along,across))]
    shift=state[:2]-np.asarray(track.observation.center_xy)+t*velocity
    return tuple(convex_hull([tuple(np.asarray(p)+shift+offset) for p in piece for offset in padding])
                 for piece in track.observation.footprints)


def assess_trajectory(points,pose,speed_kmh,tracks,body,*,completion_distance_m,
                      now_ns,source_ns,margin_m=.25,step_s=.2,maximum_prediction_s=10.):
    if (speed_kmh<=0 or step_s<=0 or not all(math.isfinite(v) for v in
            (speed_kmh,completion_distance_m,margin_m,step_s,maximum_prediction_s))):
        raise ValueError('finite positive trajectory timing required')
    age=(now_ns-source_ns)/1e9
    if not 0<=age<=.6:return TrafficAssessment(False,'TRAFFIC_SOURCE_STALE',0.,0.,None)
    index=PolylineIndex(points);start=max(0.,index.project(pose[:2]));velocity=speed_kmh/3.6
    duration=completion_distance_m/velocity
    if duration>maximum_prediction_s:
        return TrafficAssessment(False,'TRAFFIC_PREDICTION_HORIZON',0.,duration,None)
    if start+completion_distance_m>index.lengths[-1]:
        return TrafficAssessment(False,'SHORT_TIMED_PATH',0.,duration,None)
    closest=math.inf;previous_pose=pose;previous_pieces={}
    ticks=max(1,math.ceil(duration/step_s))
    for i in range(ticks+1):
        t=duration*i/ticks;point,tangent=sample(points,start+velocity*t)
        current=(*point[:2],math.atan2(tangent[1],tangent[0]));ego=swept_body(body,previous_pose,current)
        for track in tracks:
            if (now_ns-track.observed_ns)/1e9>.6:
                return TrafficAssessment(False,'TRAFFIC_TRACK_STALE',0.,duration,track.identifier)
            pieces=predicted_pieces(track,t,age)
            before=previous_pieces.get(track.identifier,pieces)
            for first,second in zip(before,pieces):
                other=convex_hull(list(first)+list(second));gap=polygon_distance(ego,other)
                closest=min(closest,gap)
                if gap<=margin_m:
                    return TrafficAssessment(False,'PREDICTED_TRAFFIC_CONFLICT',gap,duration,track.identifier)
            previous_pieces[track.identifier]=pieces
        previous_pose=current
    return TrafficAssessment(True,'PREDICTED_TRAFFIC_GAP_CLEAR',closest,duration,None)


def following_speed_kmh(gap_m,lead_speed_kmh,desired_kmh,*,standstill_gap_m=6.,time_headway_s=1.5):
    if not all(math.isfinite(v) for v in (gap_m,lead_speed_kmh,desired_kmh)):
        raise ValueError('finite following observations required')
    desired_gap=standstill_gap_m+max(0.,lead_speed_kmh)/3.6*time_headway_s
    return min(desired_kmh,max(0.,lead_speed_kmh+1.8*(gap_m-desired_gap)))


def longitudinal_kinematics(points,pose,ego_speed_kmh,tracks,*,now_ns,source_ns,horizons=(2.,4.)):
    """Measured route-relative diagnostics; never authorize a lane change.

    The body corridor identifies potential lead objects, not lane membership.
    Timed full-body trajectory assessment and map permission remain separate.
    """
    if (not math.isfinite(ego_speed_kmh) or len(pose)!=3 or
            not all(math.isfinite(v) for v in pose) or
            any(not math.isfinite(t) or t<=0 for t in horizons)):
        raise ValueError('finite measured kinematics and positive prediction times required')
    age=(now_ns-source_ns)/1e9
    if not 0<=age<=.6:return dict(ready=False,reason='TRAFFIC_SOURCE_STALE',tracks=[])
    from .vehicle_geometry import sonata_geometry
    body=sonata_geometry();index=PolylineIndex(points);ego=index.project(pose[:2]);rows=[]
    for track in tracks:
        if not 0<=(now_ns-track.observed_ns)/1e9<=.6:continue
        center=track.state[:2]+age*track.state[2:];station=index.project(center)
        sampled=sample(points,max(0.,min(index.lengths[-1],station)))
        tangent=np.asarray(sampled[1]);normal=np.array((-tangent[1],tangent[0]))
        lateral=float(normal@(center-np.asarray(sampled[0][:2])))
        vertices=np.array([p for piece in track.observation.footprints for p in piece])
        relative=vertices-np.asarray(track.observation.center_xy)
        along=relative@tangent;across=relative@normal+lateral
        ahead=station>=ego;other_speed=float(tangent@track.state[2:]);ego_speed=ego_speed_kmh/3.6
        gap=(station-ego+float(along.min())-body.wheelbase_m-body.front_overhang_m if ahead
             else ego-station-float(along.max())-body.rear_overhang_m)
        closing=ego_speed-other_speed if ahead else other_speed-ego_speed
        future=[]
        for t in horizons:
            elapsed=age+t;H=np.array(((1.,0.,elapsed,0.),(0.,1.,0.,elapsed)))
            covariance=H@track.covariance@H.T
            uncertainty=2*math.sqrt(max(0.,float(tangent@covariance@tangent)))+.25*elapsed*elapsed
            future.append(dict(seconds=t,nominal_gap_m=gap-closing*t,lower_bound_m=gap-closing*t-uncertainty))
        rows.append(dict(id=track.identifier,relation='AHEAD' if ahead else 'BEHIND',
            longitudinal_gap_m=gap,lateral_offset_m=lateral,
            intersects_body_corridor=bool(across.min()<=body.width_m/2+.25 and across.max()>=-body.width_m/2-.25),
            forward_speed_mps=other_speed,closing_speed_mps=closing,motion=track.motion,
            future_gaps=future,lane_membership_verified=False))
    leads=[r for r in rows if r['relation']=='AHEAD' and r['motion']=='MOVING' and
           r['forward_speed_mps']>=0 and r['intersects_body_corridor'] and r['longitudinal_gap_m']>0]
    lead=min(leads,key=lambda r:r['longitudinal_gap_m']) if leads else None
    return dict(ready=True,source='LIDAR_SOURCE_TIME_KINEMATICS',source_stamp_ns=source_ns,
        ego_speed_mps=ego_speed_kmh/3.6,lead_candidate_id=lead['id'] if lead else None,
        driving_policy_enabled=False,tracks=rows)
