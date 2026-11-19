from types import SimpleNamespace
import numpy as np
import pytest
from edgenode_dynamic_tracking.kinematics import Observation,ObstacleTracking
from edgenode_dynamic_tracking.vehicle_observation import vehicle_observation

MATCHER=SimpleNamespace(links=[dict(idx='native',points=[[-100.,0.,0.],[100.,0.,0.]],_bbox=(-100,100,0,0))],bbox_candidate=lambda *args:True)

def box(x0,x1,y0,y1):
    piece=((x0,y0),((x0+x1)/2,y0),(x1,y0),(x1,(y0+y1)/2),(x1,y1),((x0+x1)/2,y1),(x0,y1),(x0,(y0+y1)/2))
    return Observation(((x0+x1)/2,(y0+y1)/2),(piece,))

def test_rear_side_and_front_views_fit_the_same_stationary_vehicle_without_actor_truth():
    rear=box(27.535,27.535,-1.06,1.06);side=box(27.535,32.465,1.06,1.06);front=box(32.465,32.465,-1.06,1.06)
    for raw,eye in [(rear,(0,0,0)),(side,(30,4,0)),(front,(50,0,0))]:
        fitted,model=vehicle_observation(raw,eye,MATCHER,predicted_centers=[(30,0)])
        assert fitted.center_xy==pytest.approx((30,0),abs=1e-8)
        assert fitted.footprints is raw.footprints and fitted.surface_center_xy==raw.center_xy

def test_rigid_moving_rear_surface_keeps_velocity_after_vehicle_center_fit():
    tracker=ObstacleTracking()
    for i in range(31):
        x=30+i*.2777778;raw=box(x-2.465,x-2.465,-1.06,1.06)
        predicted=[t.state[:2]+.1*t.state[2:] for t in tracker.tracks.values()]
        fitted,_=vehicle_observation(raw,(0,0,0),MATCHER,predicted_centers=predicted)
        tracks=tracker.update(1_000_000_000+i*100_000_000,[fitted])
    assert tracks[0].motion=='MOVING' and tracks[0].speed_mps==pytest.approx(10/3.6,abs=.1)

def test_static_vehicle_does_not_move_when_visible_centroid_changes_by_vehicle_length():
    tracker=ObstacleTracking()
    for i in range(41):
        raw=box(27.535,27.535,-1.06,1.06) if i<15 else box(27.535,32.465,1.06,1.06) if i<25 else box(32.465,32.465,-1.06,1.06)
        eye=(0,0,0) if i<15 else (30,4,0) if i<25 else (50,0,0)
        fitted,_=vehicle_observation(raw,eye,MATCHER,predicted_centers=[t.state[:2] for t in tracker.tracks.values()])
        tracks=tracker.update(1_000_000_000+i*100_000_000,[fitted])
    assert len(tracks)==1 and tracks[0].motion=='STATIC' and tracks[0].speed_mps<.01

def test_off_lane_surface_and_insufficient_vehicle_support_remain_explicit_nontraffic():
    assert vehicle_observation(box(10,15,10,10.1),(0,0,0),MATCHER)[0] is None
    assert vehicle_observation(box(10,10.1,0,.1),(0,0,0),MATCHER)[0] is None

def test_recorded_real_startup_keeps_one_id_with_visible_face_measurement_uncertainty():
    import json,pathlib
    from edgenode_dynamic_tracking.tracker import TrafficTracker
    fixture=json.loads((pathlib.Path(__file__).parent/'data/actual-startup-observations.json').read_text())
    tracker=TrafficTracker('/workspace/local_data/kcity_mgeo',measurement_std_m=.3,acceleration_std_mps2=1.5)
    births=set()
    for row in fixture['frames']:
        births.update(t.identifier for t in tracker.update(row['envelope'],tuple(row['ego'])))
    assert births=={1}
    assert next(iter(tracker.filter.tracks.values())).speed_mps==pytest.approx(10/3.6,abs=.5)

def test_same_visible_end_translation_rejects_a_previously_stationary_hypothesis():
    from edgenode_dynamic_tracking.kinematics import Track
    from edgenode_dynamic_tracking.stationary_hypothesis import StationaryHypothesis
    raw=box(27.535,27.535,-1.06,1.06)
    fitted=Observation((30.,0.),raw.footprints,raw.center_xy,4.93,2.12)
    track=Track(1,np.array((30.,0.,0.,0.)),np.eye(4),1,1,20,fitted)
    hypothesis=StationaryHypothesis(track,(0.,0.,0.),0.)
    assert hypothesis.consistent(raw,(0.,0.,0.))
    # The translated rear surface still fits inside the old full-body box.
    # Its matching visible end nevertheless refutes the static hypothesis.
    assert not hypothesis.consistent(box(28.535,28.535,-1.06,1.06),(0.,0.,0.))
    assert hypothesis.consistent(box(27.535,32.465,1.06,1.06),(30.,4.,0.))

def test_recorded_parked_car_survives_partial_views_and_split_surfaces_without_false_motion():
    import gzip,json,pathlib
    from edgenode_dynamic_tracking.tracker import TrafficTracker
    with gzip.open(pathlib.Path(__file__).parent/'data/actual-parked-source-observations.json.gz','rt') as stream:fixture=json.load(stream)
    assert fixture['scope']=='SENSOR_AND_SOURCE_EGO_ONLY_NO_ACTOR_TRUTH'
    tracker=TrafficTracker('/workspace/local_data/kcity_mgeo',measurement_std_m=.3,acceleration_std_mps2=1.5,hdmap_dir='/workspace/local_data/kcity_hdmap');births=set();states=[]
    for i,row in enumerate(fixture['frames']):
        tracks=tracker.update(row['envelope'],tuple(row['ego']));births.update(t.identifier for t in tracks)
        if i>=20:states.extend((t.motion,t.speed_mps) for t in tracks)
        for track in tracks:
            if track.identifier in tracker.stationary:assert np.linalg.eigvalsh(track.covariance[:2,:2]).min()>=.09-1e-9
    assert births=={1} and len(states)>150
    assert all(motion=='STATIC' and speed<.3 for motion,speed in states)
    assert not tracker.filter.tracks and not tracker.stationary
