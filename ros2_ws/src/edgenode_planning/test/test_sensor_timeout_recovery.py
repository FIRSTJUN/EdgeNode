"""Three focused checks: DDS pairing, safe recovery, and parked route edits."""
import json
from unittest.mock import patch

import pytest
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from edgenode_planning.sensor_readiness import SensorReadiness
from test_sensor_readiness import NOW,payload,send
from test_static_obstacle_node import node as node_fixture,output,arm_isolated_node
from test_s4_sensor_node import clock as clock_fixture,feed

node=node_fixture
clock=clock_fixture


def test_continuous_eight_hz_crossed_streams_keep_newest_complete_frame():
    steady=[NOW]
    guard=SensorReadiness(steady_time_ns=lambda:steady[0])
    guard.observation(NOW,[],NOW);send(guard,payload())
    # Every status is one frame ahead of the array callback. Neither stream
    # has a gap; matching only the newest status used to expire frame zero.
    for i in range(1,13):
        stamp=NOW+i*125_000_000;steady[0]=stamp+5_000_000
        send(guard,payload(stamp=stamp,seq=i+1,frame=i+1),steady[0])
        steady[0]+=1_000_000
        guard.observation(stamp-125_000_000,[],steady[0])
        assert guard.reason(steady[0])==''
        assert steady[0]-guard.matched[0]['source_stamp_ns']<140_000_000
    # A heartbeat and duplicate array must still not refresh an old frame.
    later=stamp+610_000_000;steady[0]=later
    send(guard,payload(stamp=stamp,seq=14,frame=13,published=later),later)
    guard.observation(stamp,[],later)
    assert guard.reason(later) in ('OBSTACLE_FRAME_TIMEOUT','SENSOR_INPUT_TIMEOUT','SENSOR_SOURCE_STALE')


def update(node,clock,seq,points=(),state=None):
    clock.ns+=125_000_000
    node.latest_pose_received_ns=clock.ns
    feed(node,clock,points=points,seq=seq,frame=seq,state=state)


@pytest.mark.parametrize('node',[True],indirect=True)
def test_brief_gap_freezes_entry_then_revalidates_but_real_faults_stay_latched(node,clock,monkeypatch):
    monkeypatch.setattr('edgenode_planning.planning_node.time.monotonic_ns',lambda:clock.ns)
    node.sensor_guard.steady_time_ns=lambda:clock.ns
    node.distance_safety=None;node.latest_speed_kmh=0.
    node.set_parameters([Parameter('local_path_length_m',value=50.)])
    points=node.test_obstacle(35.,39.)
    with patch('edgenode_planning.planning_node.time.perf_counter',return_value=0.):
        feed(node,clock,points=points);node.plan();node.plan()
        assert node.avoidance_planner.state=='AVOID_ENTRY'
        selected=node.avoidance_planner.selected
        generated=node.avoidance_planner.generated_transition_count
        arm_isolated_node(node);node.plan()
        clock.ns+=601_000_000
        node.sensor_watchdog();node.plan()
        assert node.avoidance_planner.state=='AVOID_ENTRY'
        assert output(node,'avoidance_state').data=='SENSOR_HOLD'
        assert not output(node,'local_path').poses and output(node,'target_speed').data==0.
        for seq in (2,3,4):
            update(node,clock,seq,points);node.plan()
            assert output(node,'target_speed').data==0.
        assert node._sensor_pause is None
        for seq in (5,6,7):
            update(node,clock,seq,points);node.plan()
        assert node.avoidance_planner.selected is selected
        assert node.avoidance_planner.generated_transition_count==generated
        assert output(node,'target_speed').data>0.
        # Real body contact in a new valid LiDAR frame must prevent restart.
        update(node,clock,8,node.test_obstacle(1.,3.));node.plan()
        assert node.avoidance_planner.reason=='BODY_COLLISION'
        update(node,clock,9);node.plan()
        assert node.avoidance_planner.state=='BLOCKED' and output(node,'target_speed').data==0.
        node.set_parameters([Parameter('enable_drive',value=False)])
        assert node.reset_avoidance(Trigger.Request(),Trigger.Response()).success
        # Producer startup/localization delivery gaps follow the same hold rule.
        update(node,clock,10,state='LOCALIZATION_UNAVAILABLE');node.plan()
        assert node._sensor_pause is not None and node.avoidance_planner.state=='NORMAL'
        for seq in (11,12,13):
            update(node,clock,seq);node.plan()
        assert node._sensor_pause is None
        assert node.reset_avoidance(Trigger.Request(),Trigger.Response()).success
        update(node,clock,14,state='INVALID_INPUT');node.plan()
        assert node.avoidance_planner.reason=='S4_SENSOR_INVALID_INPUT'
        update(node,clock,15);node.plan()
        assert node.avoidance_planner.state=='BLOCKED'
        assert node.avoidance_planner.reason=='S4_SENSOR_INVALID_INPUT'


@pytest.mark.parametrize('node',[True],indirect=True)
def test_route_editor_change_is_allowed_only_when_disarmed_and_stationary(node,clock,monkeypatch):
    monkeypatch.setattr('edgenode_planning.planning_node.time.monotonic_ns',lambda:clock.ns)
    node.sensor_guard.steady_time_ns=lambda:clock.ns
    node.distance_safety=None;node.latest_speed_kmh=0.
    feed(node,clock);node.plan()
    cached=output(node,'global_path');node.plan()
    assert output(node,'global_path') is cached
    assert cached.poses[0].header is cached.header
    node.avoidance_planner.state='AVOID_ENTRY'
    node.set_parameters([Parameter('goal_node_id',value='E')]);node.plan()
    assert node.avoidance_planner.state=='NORMAL'
    assert output(node,'global_path') is not cached
    assert output(node,'global_path').poses and output(node,'target_speed').data==0.
    # Loading a parked route may not erase an existing collision/error latch.
    node.avoidance_planner.stop('BODY_COLLISION')
    node.set_parameters([Parameter('goal_node_id',value='D')]);node.plan()
    assert node.avoidance_planner.reason=='BODY_COLLISION'
    node.avoidance_planner.reset();node.avoidance_planner.state='AVOID_ENTRY'
    node.latest_speed_kmh=1.;arm_isolated_node(node)
    node.set_parameters([Parameter('goal_node_id',value='E')]);node.plan()
    assert node.avoidance_planner.reason=='ROUTE_CHANGED_DURING_AVOIDANCE'
    assert output(node,'target_speed').data==0. and not output(node,'local_path').poses
