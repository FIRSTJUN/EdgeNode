import math
import numpy as np
import pytest
from edgenode_dynamic_tracking.kinematics import ObstacleTracking,Observation,observations_from_frame
from edgenode_dynamic_tracking.ego_history import EgoHistory

def observed(x,y):
    return Observation((x,y),(((x-1,y-.5),(x+1,y-.5),(x+1,y+.5),(x-1,y+.5)),))

def test_moving_ego_transform_does_not_turn_a_static_world_object_into_a_moving_track():
    tracker=ObstacleTracking()
    for i in range(31):
        ego_x=i*.2;relative_x=40.-ego_x
        tracks=tracker.update(1_000_000_000+i*100_000_000,[observed(ego_x+relative_x,0.)])
    assert tracks[0].motion=='STATIC' and tracks[0].speed_mps<.01

def test_two_vehicle_ids_velocity_and_static_background_are_distinct():
    tracker=ObstacleTracking()
    for i in range(31):
        tracks=tracker.update(1_000_000_000+i*100_000_000,[observed(10+i*.2777778,0),observed(25+i*.4,3.5),observed(5,-3)])
    a,b,c=sorted(tracks,key=lambda t:t.identifier)
    assert a.motion==b.motion=='MOVING' and c.motion=='STATIC'
    assert a.speed_mps==pytest.approx(10/3.6,abs=.1) and b.speed_mps==pytest.approx(4.,abs=.1)
    assert all(np.linalg.eigvalsh(t.covariance).min()>=0 for t in tracks)

def test_occlusion_retains_id_but_does_not_renew_observed_time_and_expiration_is_bounded():
    tracker=ObstacleTracking(max_missed_s=1.2)
    for i in range(21):tracks=tracker.update(1_000_000_000+i*100_000_000,[observed(i*.28,0)])
    key=tracks[0].identifier;seen=tracks[0].observed_ns
    for i in range(21,26):tracks=tracker.update(1_000_000_000+i*100_000_000,[])
    assert tracks[0].identifier==key and tracks[0].observed_ns==seen
    tracks=tracker.update(3_600_000_000,[observed(26*.28,0)]);assert tracks[0].identifier==key
    assert tracker.update(4_900_000_000,[])==()

def test_duplicate_or_backward_source_cannot_refresh_a_track():
    tracker=ObstacleTracking();tracker.update(1_000_000_000,[observed(0,0)])
    for ns in (1_000_000_000,900_000_000):
        with pytest.raises(ValueError):tracker.update(ns,[observed(10,0)])
    assert tracker.tracks[1].frames==1 and tracker.tracks[1].observed_ns==1_000_000_000

def test_source_ego_interpolation_rejects_extrapolation_duplicates_and_relocation():
    h=EgoHistory();h.add(1_000_000_000,(0,0,math.radians(179)),2.)
    h.add(1_100_000_000,(.2,0,math.radians(-179)),4.)
    p,v=h.sample(1_050_000_000);assert p[0]==pytest.approx(.1) and abs(abs(p[2])-math.pi)<1e-9 and v==3.
    assert h.sample(900_000_000) is None and h.sample(1_200_000_000) is None
    assert not h.add(1_100_000_000,(20,0,0),20)
    assert h.add(1_200_000_000,(100,0,0),0.) and h.sample(1_050_000_000) is None

def test_component_membership_cannot_drop_a_collision_tile_silently():
    piece=[(-1.,-1.),(0.,-1.),(1.,-1.),(1.,0.),(1.,1.),(0.,1.),(-1.,1.),(-1.,0.)]
    frame=dict(points=piece+[(x+2,y) for x,y in piece],objects=[dict(center_xy=(1,0),footprint_indices=[0,1])])
    assert len(observations_from_frame(frame))==1
    frame['objects'][0]['footprint_indices']=[0]
    with pytest.raises(ValueError):observations_from_frame(frame)

def test_lane_relative_velocity_and_future_covariance_use_measured_world_state():
    from edgenode_dynamic_tracking.tracker import TrafficTracker
    from edgenode_planning.lane_geometry import sample
    from edgenode_planning.dijkstra_planner import DijkstraPlanner
    tracker=TrafficTracker('/workspace/local_data/kcity_mgeo');points=DijkstraPlanner('/workspace/local_data/kcity_mgeo').links_by_id['A219BS010301']['points']
    for i in range(31):
        p,_=sample(points,30+i*.27777778);tracker.filter.update(1_000_000_000+i*100_000_000,[observed(*p[:2])])
    p,t=sample(points,20);rows=tracker.rows(4_000_000_000,((p[0],p[1],math.atan2(t[1],t[0])),5/3.6))
    assert len(rows)==1 and rows[0]['motion']=='MOVING' and rows[0]['lane']['link_id']=='A219BS010301'
    assert rows[0]['relative_forward_mps']==pytest.approx(5/3.6,abs=.15)
    assert len(rows[0]['future'])==3 and rows[0]['observed_ns']==4_000_000_000
    assert rows[0]['future'][-1]['covariance_xy'][0][0]>rows[0]['future'][0]['covariance_xy'][0][0]
