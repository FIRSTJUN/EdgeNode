"""Enabled global speed planning via real ROS APIs with captured outputs."""
import json
import time
from types import SimpleNamespace
from unittest.mock import patch
from unittest.mock import Mock

import pytest
import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from morai_ros2_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy

from test_speed_parameters_ros import make_node, change, output, set_geometry
from test_speed_planner import curvature_profile_path


def status(node,speed=50.,age=0.):
    msg = EgoVehicleStatus()
    now = node.get_clock().now().nanoseconds
    msg.header.stamp = Time(nanoseconds=now-int(age*1e9)).to_msg()
    msg.velocity.x = speed/3.6
    node.ego_status_callback(msg)


def diagnostic(node):return json.loads(output(node,'long_speed_plan'))


@pytest.mark.parametrize('stream', ['ego', 'localization'])
def test_current_inputs_drop_backlog_while_executor_is_busy(make_node, stream):
    """A blocked planning timer must resume with the latest queued input."""
    node = make_node()
    sub = node.ego_status_sub if stream == 'ego' else node.localization_sub
    assert sub.qos_profile.history == HistoryPolicy.KEEP_LAST
    assert sub.qos_profile.depth == 1
    message_type = EgoVehicleStatus if stream == 'ego' else Odometry
    publisher = node.create_publisher(message_type, sub.topic_name, 10)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 3.
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert publisher.get_subscription_count() == 1
        for value in range(1, 11):
            msg = message_type()
            msg.header.stamp = node.get_clock().now().to_msg()
            if stream == 'ego':
                msg.velocity.x = float(value)
            else:
                msg.pose.pose.position.x = float(value)
                msg.pose.pose.orientation.w = 1.
            publisher.publish(msg)
        # Transport receives the burst while no application callbacks run.
        time.sleep(.15)
        executor.spin_once(timeout_sec=1.)
        if stream == 'ego':
            assert node._ego_speed[0] == 36.
        else:
            assert node.latest_pose == (10., 0., 0.)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_publisher(publisher)


def make_long(make_node,**overrides):
    settings = dict(speed_long_preview_enabled=True,enable_drive=False,
                    curve_curvature_threshold=.025,sharp_curvature_threshold=.07,
                    sharp_curve_speed_kmh=20.)
    settings.update(overrides)
    node = make_node(**settings)
    points = curvature_profile_path([(0.,80.,.5),(.12,12.,.5),(0.,200.,.5)])
    node.dijkstra_planner.plan.return_value = {'points':points}
    return node


def warm(node):
    for _ in range(200):
        status(node)
        node.plan()
        if diagnostic(node)['ready']:return node
    pytest.fail(str(diagnostic(node)))


def atomic_service(node,**values):
    client = node.create_client(SetParametersAtomically,
        node.get_fully_qualified_name()+'/set_parameters_atomically')
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        assert client.wait_for_service(timeout_sec=1.)
        request = SetParametersAtomically.Request(parameters=[
            Parameter(k,value=v).to_parameter_msg() for k,v in values.items()])
        future = client.call_async(request)
        executor.spin_until_future_complete(future,timeout_sec=3.)
        assert future.done()
        return future.result().result
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_client(client)


def test_building_stops_then_ready_reuses_profile_and_drive_gate(make_node):
    node = make_long(make_node)
    status(node)
    node.plan()
    assert diagnostic(node)['status'] == 'BUILDING'
    assert output(node,'target_speed') == output(node,'recommended_speed') == 0.
    warm(node)
    planner,profile = node._long_planner,node._long_planner.profile
    assert output(node,'recommended_speed') == 50.
    assert output(node,'target_speed') == 0.
    for _ in range(3):
        status(node);node.plan()
        assert node._long_planner is planner and node._long_planner.profile is profile
    assert change(node,enable_drive=True).successful
    status(node);node.plan()
    assert output(node,'target_speed') == 30. # unchanged independent drive cap


@pytest.mark.parametrize('age,expected',[(.21,'STALE_SPEED'),(-1.,'STALE_SPEED')])
def test_header_age_rejects_old_or_future_speed(make_node,age,expected):
    node = warm(make_long(make_node))
    profile = node._long_planner.profile
    status(node,age=age);node.plan()
    assert diagnostic(node)['status'] == expected
    assert output(node,'recommended_speed') == output(node,'target_speed') == 0.
    assert node._long_planner.profile is profile and not node._long_planner._urgent
    status(node);node.plan()
    assert diagnostic(node)['ready']


def test_missing_nan_and_old_receive_time_stop_without_rebuilding(make_node):
    node = warm(make_long(make_node))
    profile = node._long_planner.profile
    for value in [None,(50.,node.get_clock().now().nanoseconds,0)]:
        node._ego_speed = value;node.plan()
        assert output(node,'target_speed') == output(node,'recommended_speed') == 0.
        assert node._long_planner.profile is profile
    status(node,float('nan'));node.plan()
    assert diagnostic(node)['status'] == 'MISSING_SPEED'


def test_speed_and_lateral_batch_apply_next_cycle_without_geometry_rebuild(make_node):
    node = warm(make_long(make_node))
    node.local_path_planner.extract.return_value['progress_arc_m'] = 90.
    status(node,speed=10.);node.plan()
    assert output(node,'recommended_speed') > 6.
    old,profile = node._long_planner,node._long_planner.profile
    result = atomic_service(node,sharp_curve_speed_kmh=6.,speed_plan_lateral_accel_mps2=.8)
    assert result.successful
    assert node._long_planner is old
    status(node,speed=6.);node.plan()
    assert node._long_planner is not old and node._long_planner.profile is profile
    assert output(node,'recommended_speed') == 6.
    assert node._long_planner.settings.lateral_accel_mps2 == .8
    assert output(node,'speed_mode') == 'CRUISE' # local mode is not polluted by global approach


@pytest.mark.parametrize('name,value',[
    ('speed_plan_deceleration_mps2',0.),('speed_plan_lateral_accel_mps2',-1.),
    ('speed_plan_reaction_time_sec',float('nan')),('speed_long_preview_distance_m',0.),
    ('speed_plan_acceleration_mps2',0.),
    ('speed_plan_recovery_acceleration_mps2',0.),('speed_plan_recovery_clear_sec',0.),
])
def test_invalid_atomic_batch_changes_no_planner_parameter_or_route(make_node,name,value):
    node = warm(make_long(make_node))
    old = node._long_planner
    speed = node.get_parameter('sharp_curve_speed_kmh').value
    result = atomic_service(node,**{name:value,'sharp_curve_speed_kmh':8.,'goal_node_id':'wrong'})
    assert not result.successful
    assert node._long_planner is old
    assert node.get_parameter('sharp_curve_speed_kmh').value == speed
    assert node.get_parameter('goal_node_id').value != 'wrong'


def test_later_rejection_does_not_mutate_active_long_planner(make_node):
    from rcl_interfaces.msg import SetParametersResult
    node = warm(make_long(make_node))
    old = node._long_planner
    node.add_on_set_parameters_callback(lambda _:SetParametersResult(successful=False,reason='test'))
    assert not change(node,speed_plan_lateral_accel_mps2=.8).successful
    assert node._long_planner is old
    assert node.get_parameter('speed_plan_lateral_accel_mps2').value == 1.5


def test_geometry_change_invalidates_profile_without_extending_lease(make_node):
    node = warm(make_long(make_node,enable_drive=True))
    route,start = node._route_result,node._drive_session_start_ns
    assert change(node,curve_curvature_threshold=.03).successful
    status(node);node.plan()
    assert diagnostic(node)['status'] == 'BUILDING'
    assert node._long_planner is None
    assert node._route_result is route and node._drive_session_start_ns == start
    assert output(node,'target_speed') == 0.


def test_route_editor_transaction_drops_old_cache_and_cannot_drive_while_building(make_node):
    node = warm(make_long(make_node,enable_drive=True))
    old = node._long_planner.profile
    replacement = curvature_profile_path([(0.,70.,.5),(-.1,20.,.5),(0.,100.,.5)])
    node.dijkstra_planner.plan_route.return_value = {'points':replacement}
    # This is the same atomic interface and disable+route transaction used by Route Editor.
    assert atomic_service(node,enable_drive=False,route_node_ids=['new_a','new_b']).successful
    status(node);node.plan()
    assert node._long_planner is None and node._long_builder is not None
    assert diagnostic(node)['status'] == 'BUILDING'
    assert output(node,'target_speed') == 0.
    warm(node)
    assert node._long_planner.profile is not old
    assert any(s.direction < 0 for s in node._long_planner.profile.spans)


def test_match_loss_resets_query_state_but_recovers_same_route_profile(make_node):
    node = warm(make_long(make_node))
    planner = node._long_planner
    planner._urgent.add(0)
    old_match = node.map_matcher.match.return_value
    node.map_matcher.match.return_value = None
    node.plan()
    assert not planner._urgent and planner._progress is None
    assert output(node,'target_speed') == 0.
    node.map_matcher.match.return_value = old_match
    status(node);node.plan()
    assert diagnostic(node)['ready'] and node._long_planner is planner


def test_enabled_invalid_startup_does_not_publish_motion_recommendation(make_node):
    node = make_long(make_node,speed_plan_deceleration_mps2=0.)
    status(node);node.plan()
    assert diagnostic(node)['status'] == 'INVALID_CONFIG'
    assert output(node,'target_speed') == output(node,'recommended_speed') == 0.


def test_unchanged_batch_preserves_geometry_cache_and_query_state(make_node):
    node = warm(make_long(make_node))
    old = node._long_planner
    assert change(node,speed_plan_lateral_accel_mps2=1.5).successful
    status(node);node.plan()
    assert node._long_planner is old


def test_clock_rewind_invalidates_speed_and_preserves_expired_lease(make_node):
    node = warm(make_long(make_node,enable_drive=True))
    planner = node._long_planner
    rewind = node.get_clock().now().nanoseconds-1_000_000_000
    clock = SimpleNamespace(now=lambda:Time(nanoseconds=rewind))
    with patch.object(node,'get_clock',return_value=clock):
        node.plan()
    assert node._ego_speed is None and node._long_planner is planner
    assert planner._progress is None and not planner._urgent
    assert output(node,'target_speed') == 0.
    assert output(node,'drive_status') == 'EXPIRED'
    status(node);node.plan()
    assert output(node,'target_speed') == 0. and output(node,'drive_status') == 'EXPIRED'


def test_speed_freshness_is_checked_at_end_of_profile_work(make_node):
    node = warm(make_long(make_node))
    status(node)
    before = node.get_clock().now().nanoseconds
    clock = SimpleNamespace(now=lambda:Time(nanoseconds=before+300_000_000))
    with patch.object(node,'get_clock',return_value=clock):
        result,diag = node._long_speed_recommendation(0.,50.,before)
    assert result is None and diag['status'] == 'STALE_SPEED'


def test_runtime_speed_increase_respects_release_rate_on_next_cycle(make_node):
    node = warm(make_long(make_node))
    base = node.get_clock().now().nanoseconds
    tick = [base]
    clock = SimpleNamespace(now=lambda:Time(nanoseconds=tick[0]))
    with patch.object(node,'get_clock',return_value=clock):
        node.local_path_planner.extract.return_value['progress_arc_m'] = 90.
        assert change(node,sharp_curve_speed_kmh=6.).successful
        status(node,speed=6.);node.plan()
        assert output(node,'recommended_speed') == 6.
        profile = node._long_planner.profile
        assert change(node,sharp_curve_speed_kmh=20.,speed_plan_acceleration_mps2=1.).successful
        tick[0] += 100_000_000
        status(node,speed=6.);node.plan()
        assert node._long_planner.profile is profile
        assert diagnostic(node)['uncapped_target_speed_kmh'] > 6.36
        assert output(node,'recommended_speed') == pytest.approx(6.36,abs=1e-5)


def test_recovery_mode_and_atomic_settings_reuse_profile_with_new_clear_confirmation(make_node):
    node=warm(make_long(make_node))
    base=node.get_clock().now().nanoseconds;tick=[base]
    clock=SimpleNamespace(now=lambda:Time(nanoseconds=tick[0]))
    with patch.object(node,'get_clock',return_value=clock):
        set_geometry(node,.1);status(node,speed=8.);node.plan()
        assert output(node,'recommended_speed')==20.
        set_geometry(node,0.)
        for _ in range(15):
            tick[0]+=100_000_000;status(node,speed=8.);node.plan()
        assert diagnostic(node)['recovery_active']
        previous=output(node,'recommended_speed');profile=node._long_planner.profile
        assert atomic_service(node,speed_plan_recovery_acceleration_mps2=1.25,
                              speed_plan_recovery_clear_sec=1.5).successful
        tick[0]+=100_000_000;status(node,speed=8.);node.plan()
        assert node._long_planner.profile is profile
        assert node._long_settings.recovery_acceleration_mps2==1.25
        assert node._long_settings.recovery_clear_sec==1.5
        assert not diagnostic(node)['recovery_active']
        assert output(node,'recommended_speed')==pytest.approx(previous+.36,abs=1e-5)
