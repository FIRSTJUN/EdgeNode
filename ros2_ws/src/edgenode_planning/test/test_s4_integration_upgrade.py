import math
from dataclasses import replace
from pathlib import Path
import yaml
import pytest

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.route_resume import mission_suffix
from edgenode_planning.static_obstacle_planner import AvoidanceConfig,StaticObstaclePlanner
from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.morai_speed_safety import stopping_speed_cap,required_deceleration_distance
from avoidance_world import world


def test_actual_kcity_resume_and_immutable_80m_path():
    root=Path(__file__).resolve().parents[4]
    p=DijkstraPlanner(root/'local_data/kcity_mgeo')
    w=yaml.safe_load((root/'local_data/kcity_routes/kcity_demo_route.yaml').read_text())['waypoint_node_ids']
    full=p.plan_route('A219BS010380',w);r=mission_suffix(p,'A219BS010380',w,'A219BS010469')
    assert len(w)==18 and len(r['remaining_waypoints'])==15
    assert r['link_ids']==full['link_ids'][10:] and r['link_ids'].count('A219BS010469')==1
    local=LocalPathPlanner();prepared=local.prepare(r['points']);x,y=r['points'][0][:2]
    result=local.extract(prepared,x,y,horizon_m=80.)
    assert math.isclose(result['path_length_m'],80.) and result['nearest_distance_m']==0.
    assert result==local.extract(r['points'],x,y,horizon_m=80.)
    # Mutating the source does not invalidate the frozen prepared mission.
    r['points'][0][0]+=1000.
    assert local.extract(prepared,x,y,horizon_m=80.)==result


def test_roadside_adjacent_and_passed_objects_do_not_request_avoidance():
    network,point,obstacle=world()
    for group in ([],[obstacle(15.,19.,5.,6.)],[obstacle(15.,19.,-6.,-5.)],
                  [obstacle(15.,19.,3.,4.)],[obstacle(-10.,-7.)]):
        planner=StaticObstaclePlanner(network,config=AvoidanceConfig(normal_obstacle_confirmations=2))
        reference=[point(i,0.) for i in range(81)]
        r=planner.plan(reference,group,(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=1)
        assert r['state']=='NORMAL' and r['speed_cap_kmh'] is None and r['path_obstacle_count']==0
        assert r['detected_object_count']==len(group)
        assert not planner.needs_validation_stop(reference,group)


def test_distinct_observations_confirm_but_one_false_frame_retires():
    network,point,obstacle=world();reference=[point(i,0.) for i in range(81)]
    planner=StaticObstaclePlanner(network,config=AvoidanceConfig(normal_obstacle_confirmations=2))
    assert planner.plan(reference,[obstacle()],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=1)['state']=='NORMAL'
    assert planner.plan(reference,[obstacle()],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=1)['state']=='NORMAL'
    assert planner.plan(reference,[obstacle()],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=2)['state']=='DETECT'
    assert not planner.candidates
    planner.reset()
    planner.plan(reference,[obstacle()],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=1)
    for stamp in (2,3,4):
        r=planner.plan(reference,[],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=stamp)
        assert r['state']=='NORMAL' and r['speed_cap_kmh'] is None
    assert not planner._hazards
    # Confirmation never delays an immediate braking/collision rejection.
    assert planner.plan(reference,[obstacle(5.,8.)],(0.,0.,0.),speed_kmh=10.,observation_stamp_ns=5)['blocked']


def test_curved_body_sweep_detects_collision_outside_vehicle_forward_axis():
    network,_,_=world();planner=StaticObstaclePlanner(network)
    reference=[(20.*math.sin(i*.04),20.*(1-math.cos(i*.04)),0.) for i in range(101)]
    theta=3.4;center=(20.*math.sin(theta),20.*(1-math.cos(theta)))
    polygon=[(center[0]+x,center[1]+y) for x,y in ((-1.,-1.),(1.,-1.),(1.,1.),(-1.,1.))]
    assert planner._normal_hazards(reference,[polygon])


def test_braking_preview_reduces_before_future_curve_and_does_not_slow_whole_80m():
    p=SpeedPlanner(cruise_speed_kmh=50.,curve_speed_kmh=25.,sharp_curve_speed_kmh=15.,
        preview_distance_m=80.,anticipatory=True)
    def road(straight):
        return [(i*.5,0.,0.) for i in range(int(straight*2)+1)]+[
            (straight+15.*math.sin(i*.03),15.*(1-math.cos(i*.03)),0.) for i in range(1,101)]
    far=p.plan(road(70.),measured_speed_kmh=50.)
    near=p.plan(road(15.),measured_speed_kmh=50.)
    assert far['target_speed_kmh']==50.
    assert 15.<near['target_speed_kmh']<40.
    assert near['braking_constraint_distance_m']>10.
    distance=required_deceleration_distance(50.,10.)
    assert distance>60. and stopping_speed_cap(20.)<50.


def test_actual_loop_keeps_future_revisit_and_requires_occurrence():
    root=Path(__file__).resolve().parents[4]
    p=DijkstraPlanner(root/'local_data/kcity_mgeo')
    w=yaml.safe_load((root/'local_data/kcity_routes/kcity_demo_route.yaml').read_text())['waypoint_node_ids']
    with pytest.raises(ValueError,match='AMBIGUOUS_ROUTE_OCCURRENCE'):
        mission_suffix(p,'A219BS010469',w,'A219BS010469')
    first=mission_suffix(p,'A219BS010469',w,'A219BS010469',0)
    second=mission_suffix(p,'A219BS010469',w,'A219BS010469',1)
    assert first['link_ids'].count('A219BS010469')==2
    assert second['link_ids'].count('A219BS010469')==1
    assert first['link_ids'][second['mission_link_index']:]==second['link_ids']


def test_sparse_straight_does_not_inflate_longitudinal_travel_sideways():
    network,point,obstacle=world();planner=StaticObstaclePlanner(network)
    reference=[point(i*2.,0.) for i in range(41)]
    r=planner.plan(reference,[obstacle(20.,24.,1.6,2.)],(0.,0.,0.),speed_kmh=5.)
    assert r['state']=='NORMAL' and r['path_obstacle_count']==0
    # A footprint entering the unchanged 0.25m body margin still stops/detects.
    planner.reset()
    assert planner.plan(reference,[obstacle(20.,24.,1.1,1.4)],(0.,0.,0.),speed_kmh=5.)['state']=='DETECT'


def test_exact_straight_sweep_contains_every_intermediate_body_pose():
    from edgenode_planning.avoidance_geometry import swept_body,convex_covers
    network,_,_=world();body=StaticObstaclePlanner(network).body
    for yaw in (0.,.6,math.pi/2):
        end=(20.*math.cos(yaw),20.*math.sin(yaw),yaw)
        sweep=swept_body(body,(0.,0.,yaw),end)
        for i in range(101):assert convex_covers(sweep,body.polygon(end[0]*i/100,end[1]*i/100,yaw))


def test_speed_aware_detection_retains_confirmed_far_hazard_across_empty_frames():
    network,point,obstacle=world();reference=[point(i,0.) for i in range(81)]
    planner=StaticObstaclePlanner(network,config=AvoidanceConfig(normal_obstacle_confirmations=2,
        normal_detection_latency_s=1.6))
    for stamp in (1,2):
        r=planner.plan(reference,[obstacle(60.,64.)],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=stamp)
        assert r['state']=='NORMAL' and r['speed_cap_kmh'] is None
    assert r['confirmed_path_obstacles']==1 and 26.<r['detection_trigger_distance_m']<29.
    for stamp in (3,4,5):
        assert planner.plan(reference,[],(20.,0.,0.),speed_kmh=5.,observation_stamp_ns=stamp)['state']=='NORMAL'
    assert planner.plan(reference,[],(30.,0.,0.),speed_kmh=5.,observation_stamp_ns=6)['state']=='DETECT'


def test_new_noise_next_to_a_separate_confirmed_hazard_needs_its_own_frames():
    network,point,obstacle=world();reference=[point(i,0.) for i in range(81)]
    planner=StaticObstaclePlanner(network,config=AvoidanceConfig(normal_obstacle_confirmations=2,
        normal_detection_latency_s=1.6))
    for stamp in (1,2):planner.plan(reference,[obstacle(60.,64.)],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=stamp)
    result=planner.plan(reference,[obstacle(60.,64.),obstacle(25.,29.)],(0.,0.,0.),speed_kmh=5.,observation_stamp_ns=3)
    assert result['state']=='NORMAL' and result['confirmed_path_obstacles']==1
