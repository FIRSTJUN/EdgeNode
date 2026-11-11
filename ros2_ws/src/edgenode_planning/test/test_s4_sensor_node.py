"""Real Planning adapter with isolated subscriptions and mocked motion outputs."""
import json
from types import SimpleNamespace

import pytest
from rclpy.parameter import Parameter
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from test_static_obstacle_node import node as node_fixture, detect, output, arm_isolated_node
from test_sensor_readiness import payload


pytestmark = pytest.mark.parametrize('node', [True], indirect=True)
node = node_fixture


@pytest.fixture
def clock(node, monkeypatch):
    value = SimpleNamespace(ns=10_000_000_000)
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(now=lambda: Time(nanoseconds=value.ns)))
    node.latest_pose_received_ns = value.ns
    node.latest_pose_stamp_ns = value.ns
    node._s4_pose_valid = True
    return value


def feed(node, clock, *, points=(), seq=1, frame=1, session='a' * 32, state=None):
    node.latest_pose_stamp_ns = clock.ns
    detect(node, points, stamp=clock.ns / 1e9)
    data = payload(points, stamp=clock.ns, seq=seq, frame=frame, session=session, state=state)
    node.sensor_status_callback(String(data=json.dumps(data)))


def assert_stop(node):
    assert output(node, 'target_speed').data == 0.
    assert not output(node, 'local_path').poses
    assert node.avoidance_planner.state == 'BLOCKED'


def assert_hold(node):
    assert output(node, 'target_speed').data == 0.
    assert not output(node, 'local_path').poses
    assert node.avoidance_planner.state != 'BLOCKED'
    assert node._sensor_pause is not None


def test_never_received_holds_then_latches_if_wait_exceeds_recovery_window(node, clock, monkeypatch):
    steady = [clock.ns]
    monkeypatch.setattr('edgenode_planning.planning_node.time.monotonic_ns', lambda: steady[0])
    node.plan(); assert_hold(node)
    steady[0] += 2_001_000_000
    node.sensor_watchdog(); node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_SENSOR_RECOVERY_TIMEOUT'
    assert output(node, 'fsm_state').data == 'EMERGENCY_STOP'
    assert not node.get_parameter('enable_drive').value


def test_valid_empty_preserves_global_and_normal_local_geometry(node, clock):
    feed(node, clock)
    node.plan()
    assert node.avoidance_planner.state == 'NORMAL'
    assert output(node, 'local_path').poses and output(node, 'global_path').poses
    assert output(node, 'target_speed').data == 0.  # Explicitly disarmed.
    assert json.loads(output(node, 's4_sensor_state').data)['ready']


def test_valid_obstacle_reaches_detection_stage(node, clock):
    points = node.test_obstacle(21., 25.)
    feed(node, clock, points=points)
    assert not node.sensor_guard.reason(clock.ns)
    node.plan()
    assert node.avoidance_planner.state=='DETECT'
    assert not node.avoidance_planner.candidates


@pytest.mark.parametrize('state', ['INVALID_INPUT', 'PROCESSING_ERROR',
                                  'DUPLICATE_FRAME', 'OUT_OF_ORDER', 'FUTURE_STAMP'])
def test_sensor_failure_stops_before_next_planning_cycle_and_stays_latched(node, clock, state):
    feed(node, clock); node.plan()
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2, state=state)
    assert_stop(node)
    original_reason = node.avoidance_planner.reason
    clock.ns += 1_000_000
    feed(node, clock, seq=3, frame=3)
    assert not node.sensor_guard.reason(clock.ns)
    node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == original_reason


def test_watchdog_holds_without_new_data_and_disarmed_reset_requires_fresh_data(node, clock):
    feed(node, clock); node.plan()
    clock.ns += 601_000_000
    node.sensor_watchdog(); assert_hold(node)
    response = node.reset_avoidance(Trigger.Request(), Trigger.Response())
    assert not response.success
    clock.ns += 1_000_000
    node.latest_pose_received_ns = clock.ns
    feed(node, clock, seq=2, frame=2)
    node.plan(); assert_hold(node)
    response = node.reset_avoidance(Trigger.Request(), Trigger.Response())
    assert response.success
    node.plan()
    assert node.avoidance_planner.state == 'NORMAL'
    assert output(node, 'local_path').poses


def test_restarted_producer_cannot_erase_latched_stop_or_reuse_old_pair(node, clock):
    feed(node, clock); node.plan()
    node.sensor_status_callback(String(data=json.dumps(payload(session='b' * 32))))
    assert_stop(node)
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2, session='b' * 32)
    assert not node.sensor_guard.reason(clock.ns)
    node.plan(); assert_stop(node)


def test_malformed_status_and_unpaired_obstacles_cannot_look_like_empty_road(node, clock):
    detect(node, [], stamp=10.)
    node.sensor_status_callback(String(data='{broken json'))
    assert_stop(node)


def test_s4_gate_cannot_be_disabled_at_runtime(node, clock):
    result = node.set_parameters([Parameter('s4_sensor_validation_enabled', value=False)])
    assert not result[0].successful
    node.plan(); assert_hold(node)


def test_armed_mock_outputs_stop_on_failure_and_reset_is_rejected(node, clock):
    feed(node, clock)
    arm_isolated_node(node)  # Only mocked publishers; never a live Control.
    node.plan()
    assert output(node, 'target_speed').data == 3.  # S4 prospective speed admission cap.
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2, state='INVALID_INPUT')
    assert_stop(node)
    clock.ns += 1_000_000
    feed(node, clock, seq=3, frame=3)
    response = node.reset_avoidance(Trigger.Request(), Trigger.Response())
    assert not response.success
    node.plan(); assert_stop(node)


def test_expiry_during_speed_planning_is_checked_again_before_motion_gate(node, clock, monkeypatch):
    feed(node, clock)
    original = node.speed_planner.plan
    def slow(points, **kwargs):
        result = original(points, **kwargs)
        clock.ns += 601_000_000
        return result
    monkeypatch.setattr(node.speed_planner, 'plan', slow)
    node.plan(); assert_hold(node)
    assert output(node, 'fsm_state').data == 'EMERGENCY_STOP'


def test_fresh_lidar_cannot_authorize_stale_or_replayed_planning_pose(node, clock):
    feed(node, clock); node.plan()
    pose = Odometry(); pose.header.frame_id = 'map'; pose.pose.pose.orientation.w = 1.
    pose.header.stamp.sec = 10  # Same stamp as the previous pose.
    node.localization_callback(pose)
    assert_stop(node)
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2)
    assert node.sensor_reason(clock.ns) == 'PLANNING_LOCALIZATION_INVALID'


def test_failure_lost_before_delivery_still_latches_from_recovered_snapshot(node, clock):
    feed(node, clock); node.plan()
    clock.ns += 1_000_000
    node.latest_pose_stamp_ns = clock.ns
    detect(node, [], stamp=clock.ns / 1e9)
    data = payload(stamp=clock.ns, seq=3, frame=3)
    data['fault_seq'] = 1
    node.sensor_status_callback(String(data=json.dumps(data)))
    assert not node.sensor_guard.reason(clock.ns)
    assert_stop(node)
