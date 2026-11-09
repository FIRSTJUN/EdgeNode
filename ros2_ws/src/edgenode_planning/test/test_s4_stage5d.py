"""Frame coalescing, exact clearance proof and continuous committed paths."""
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from rclpy.parameter import Parameter
from rclpy.time import Time

from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner, _axle_bounds_separated
from edgenode_planning.avoidance_geometry import inflate, polygon_distance
from avoidance_world import world

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'tools'))
from s4_timing_audit import NodeCase, load, make_case


@pytest.fixture
def adapter(tmp_path):
    loop,modules=load(Path(__file__).resolve().parents[4])
    value=NodeCase(make_case(loop,modules,dict(name='stage5d_tick',radius=0.)),modules,tmp_path)
    try:yield value
    finally:value.close()


def test_duplicate_frame_yields_without_command_or_stamp_refresh(adapter):
    node=adapter.node;adapter.feed(.01)
    node.planning_tick();assert node._planning_execution_id==1
    before=list(adapter.events);matched=node.sensor_guard.matched
    node.planning_tick()
    assert node._planning_skipped and node._planning_execution_id==1
    assert adapter.events==before and node.sensor_guard.matched is matched
    assert node.get_parameter('enable_drive').value is False


@pytest.mark.parametrize('change',['frame','pose','route','speed','boundary','reset'])
def test_new_input_or_configuration_cannot_be_hidden_by_coalescing(adapter,change):
    node=adapter.node;adapter.feed(.01);node.planning_tick()
    if change=='frame':adapter.feed(.01,reset_guard=False)
    elif change=='pose':node.latest_pose_stamp_ns+=1
    elif change=='route':node.set_parameters([Parameter('goal_node_id',value='no_such_goal')])
    elif change=='speed':node.set_parameters([Parameter('cruise_speed_kmh',value=2.)])
    elif change=='boundary':node.avoidance_planner.drivable_area=world()[1]
    else:
        response=node.reset_avoidance(None,SimpleNamespace())
        assert response.success
    node.planning_tick()
    assert not node._planning_skipped and node._planning_execution_id==2
    assert node.target_speed_pub.last.data==0.


@pytest.mark.parametrize('rewind',[False,True])
def test_skipped_work_never_skips_stale_or_clock_regression_stop(adapter,rewind):
    node=adapter.node;adapter.feed(.01);node.planning_tick()
    now=node.get_clock().now().nanoseconds
    clock=SimpleNamespace(now=lambda:Time(nanoseconds=now+(-1_000_000_000 if rewind else 700_000_000)))
    with patch.object(node,'get_clock',return_value=clock):node.planning_tick()
    assert node.avoidance_planner.state=='BLOCKED'
    assert not node.local_path_pub.last.poses and node.target_speed_pub.last.data==0.


def test_expiring_drive_lease_changes_key_even_without_new_frames():
    # No ROS node/drive session exists. Exercise the scheduling decision only.
    from edgenode_planning.planning_node import PlanningNode
    network,area,_,_=world();planner=StaticObstaclePlanner(network,drivable_area=area)
    now=[1_000_000_000];calls=[]
    value=SimpleNamespace(sensor_guard=SimpleNamespace(matched=({'frame_seq':1},),session_id='x',fault_seq=0),
        _drive_session_start_ns=now[0],_drive_session_duration_sec=1.,_route_result=None,
        _last_planning_input=None,_planning_execution_id=0,latest_pose_stamp_ns=1,latest_pose=(0.,0.,0.),
        latest_speed_kmh=0.,latest_turn_curvature=0.,avoidance_planner=planner,distance_safety=None,
        get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=now[0])),
        get_parameter=lambda name:SimpleNamespace(value=False),refresh_sensor=lambda stamp:None,
        refresh_distance=lambda start,route:None,plan=lambda:calls.append(now[0]))
    PlanningNode.planning_tick(value);PlanningNode.planning_tick(value)
    assert len(calls)==1
    now[0]+=1_000_000_000;PlanningNode.planning_tick(value)
    assert len(calls)==2


@pytest.mark.parametrize('scale',[1.,1e6,1e12])
def test_axle_broad_phase_proves_distance_and_preserves_exact_body_results(scale):
    rng=random.Random(5112026);network,area,_,_=world()
    planner=StaticObstaclePlanner(network,drivable_area=area)
    for i in range(300):
        origin=scale if scale>1 else 0.
        first=(origin+rng.uniform(-10,10),origin+rng.uniform(-10,10),rng.uniform(-3,3))
        second=(first[0]+rng.uniform(0,.3),first[1]+rng.uniform(-.2,.2),first[2]+rng.uniform(-.03,.03))
        x,y=origin+rng.uniform(-15,15),origin+rng.uniform(-15,15)
        obstacle=((x,y),(x+1,y),(x+1,y+1),(x,y+1));sagitta=rng.uniform(0,.03)
        if _axle_bounds_separated(first,second,sagitta,obstacle,1.2):
            assert polygon_distance(inflate((first[:2],second[:2]),sagitta),obstacle)>=1.2
        planner._min_clearance=math.inf
        actual=planner._body_safety(first,second,(obstacle,),False);clearance=planner._min_clearance
        planner._min_clearance=math.inf
        with patch('edgenode_planning.static_obstacle_planner._axle_bounds_separated',return_value=False):
            expected=planner._body_safety(first,second,(obstacle,),False)
        assert actual==expected and clearance==planner._min_clearance


@pytest.mark.parametrize('offset',[-1e-8,-1e-12,0.,1e-12,1e-8])
def test_axle_threshold_near_reference_floor_uses_exact_branch(offset):
    obstacle=((0.,1.2+offset),(1.,1.2+offset),(1.,2.),(0.,2.))
    if offset<=1e-12:
        assert not _axle_bounds_separated((0.,0.,0.),(1.,0.,0.),0.,obstacle,1.2)


def selected():
    network,area,point,obstacle=world();planner=StaticObstaclePlanner(network,drivable_area=area)
    normal=[point(float(x),0.) for x in range(21)];preview=[point(float(x),0.) for x in range(81)]
    kwargs=dict(speed_kmh=3.,current_link_id='O0',route_link_ids=['O0','O1'],hazard_reference=preview)
    result=planner.plan(normal,[obstacle()],(0.,0.,0.),**kwargs)
    assert result['state']=='AVOID_ENTRY'
    return planner,normal,obstacle,kwargs


def test_hold_finishes_same_committed_entry_without_centerline_target_jump():
    planner,normal,obstacle,kwargs=selected();candidate=planner.selected
    row=planner.pair.at(candidate.entry_end_s)['adjacent'];point,tangent=row
    pose=(*point[:2],math.atan2(tangent[1],tangent[0]));planner.confirmations=2
    result=planner.plan(normal,[obstacle()],pose,**kwargs)
    assert result['state']=='AVOID_HOLD' and result['points'] is candidate.entry_path
    assert planner._hold_path is candidate.entry_path
    assert (id(candidate.return_path),planner.remembered) in planner._validated_paths


def test_future_return_obstruction_invalidates_prevalidated_representation():
    planner,normal,obstacle,kwargs=selected()
    x,y,_=planner.selected.return_path[len(planner.selected.return_path)*3//4]
    occupied=((x-1,y-1),(x+1,y-1),(x+1,y+1),(x-1,y+1))
    result=planner.plan(normal,[obstacle(),occupied],(0.,0.,0.),**kwargs)
    assert result['blocked'] and result['reason']=='RETURN_RESERVATION_BODY_COLLISION'
