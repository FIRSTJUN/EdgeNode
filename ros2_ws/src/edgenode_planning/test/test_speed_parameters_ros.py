"""Runtime speed configuration through real ROS parameter APIs, isolated from MORAI."""

import math
import uuid
from unittest.mock import Mock, patch

import pytest
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from edgenode_planning.planning_node import PlanningNode


SPEED_SETTINGS = {
    'cruise_speed_kmh': 50.0,
    'curve_speed_kmh': 25.0,
    'sharp_curve_speed_kmh': 3.5,
    'speed_preview_distance_m': 10.0,
    'curve_curvature_threshold': 0.04,
    'sharp_curvature_threshold': 0.10,
    'speed_curvature_smoothing_window_m': 2.0,
    'speed_curvature_persistence_m': 2.0,
    'speed_curvature_hysteresis': 0.005,
}
PUBLISHERS = (
    'target_error', 'target_speed', 'state', 'current_link', 'map_match_status',
    'map_match_distance', 'map_match_heading_diff', 'global_path', 'local_path',
    'local_path_index', 'local_path_progress', 'recommended_speed', 'speed_mode',
    'speed_curvature', 'speed_max_curvature', 'drive_status', 'fsm_state',
)


def output(node, name):
    return getattr(node, name + '_pub').publish.call_args.args[0].data


def set_geometry(node, curvature):
    if curvature == 0:
        points = [[i * 0.5, 0.0, 0.0] for i in range(41)]
    else:
        points = [[math.sin(curvature * i * 0.5) / curvature,
                   (1.0 - math.cos(curvature * i * 0.5)) / curvature, 0.0]
                  for i in range(41)]
    node.local_path_planner.extract.return_value = {
        'points': points, 'nearest_index': 0, 'progress_arc_m': 0.0,
    }


def change(node, **values):
    return node.set_parameters_atomically([
        Parameter(name, value=value) for name, value in values.items()
    ])


@pytest.fixture
def make_node():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=193)
    nodes = []
    original_init = Node.__init__

    def make(**overrides):
        prefix = '/speed_parameter_test_' + uuid.uuid4().hex[:10]
        settings = dict(SPEED_SETTINGS, enable_drive=True, max_drive_speed_kmh=30.0,
                        drive_duration_sec=300.0)
        # Every planning topic is isolated even if another participant uses this domain.
        settings.update({name + '_topic': prefix + '/' + name for name in PUBLISHERS})
        settings['localization_topic'] = prefix + '/odometry'
        settings.update(overrides)

        def init(self, *args, **kwargs):
            original_init(self, *args, context=context, namespace=prefix,
                          use_global_arguments=False,
                          parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()],
                          **kwargs)

        with patch.object(Node, '__init__', init), \
                patch('edgenode_planning.planning_node.MapMatcher'), \
                patch('edgenode_planning.planning_node.DijkstraPlanner'):
            node = PlanningNode()
        nodes.append(node)
        for timer in node.timers:
            timer.cancel()
        # plan() outputs are captured locally, never sent as motion targets.
        for name in PUBLISHERS:
            setattr(node, name + '_pub', Mock())
        node.latest_pose = (0.0, 0.0, 0.0)
        node.map_matcher.match.return_value = {
            'link_id': 'AB', 'distance': 0.0, 'heading_diff': 0.0,
        }
        node.dijkstra_planner.plan.return_value = {
            'points': [[i * 0.5, 0.0, 0.0] for i in range(41)],
        }
        node.local_path_planner = Mock()
        set_geometry(node, 0.0)
        return node

    yield make
    for node in reversed(nodes):
        node.destroy_node()
    rclpy.shutdown(context=context)


@pytest.mark.parametrize('curvature, mode, old_speed, new_speed', [
    (0.0, 'CRUISE', 50.0, 30.0),
    (0.05, 'CURVE', 25.0, 15.0),
    (0.2, 'SHARP_CURVE', 3.5, 8.0),
])
def test_runtime_speeds_apply_on_next_cycle(make_node, curvature, mode, old_speed, new_speed):
    node = make_node()
    set_geometry(node, curvature)
    node.plan()
    assert output(node, 'speed_mode') == mode
    assert output(node, 'recommended_speed') == old_speed
    assert output(node, 'target_speed') == min(old_speed, 30.0)
    planner = node.speed_planner
    result = change(node, cruise_speed_kmh=30.0, curve_speed_kmh=15.0,
                    sharp_curve_speed_kmh=8.0)
    assert result.successful, result.reason
    # Validation must not replace the object before the parameter commit.
    assert node.speed_planner is planner
    node.plan()
    assert output(node, 'speed_mode') == mode
    assert output(node, 'recommended_speed') == new_speed
    assert output(node, 'target_speed') == new_speed
    assert node.speed_planner is not planner


def test_all_nine_settings_apply_as_one_snapshot_via_ros_service(make_node):
    node = make_node()
    set_geometry(node, 0.09)
    node.plan()
    planner = node.speed_planner
    values = dict(cruise_speed_kmh=30.0, curve_speed_kmh=15.0, sharp_curve_speed_kmh=8.0,
                  speed_preview_distance_m=6.0, curve_curvature_threshold=0.08,
                  sharp_curvature_threshold=0.15, speed_curvature_smoothing_window_m=1.5,
                  speed_curvature_persistence_m=1.5, speed_curvature_hysteresis=0.003)
    probe = Node('parameter_probe', context=node.context, namespace=node.get_namespace(),
                 use_global_arguments=False)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    executor.add_node(probe)
    try:
        client = probe.create_client(
            SetParametersAtomically, node.get_fully_qualified_name() + '/set_parameters_atomically')
        assert client.wait_for_service(timeout_sec=5.0)
        request = SetParametersAtomically.Request(parameters=[
            Parameter(k, value=v).to_parameter_msg() for k, v in values.items()
        ])
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        assert future.done()
        assert future.result().result.successful
        assert node.speed_planner is planner
        node.plan()
        for name, attribute in (
                ('cruise_speed_kmh', 'cruise_speed_kmh'),
                ('curve_speed_kmh', 'curve_speed_kmh'),
                ('sharp_curve_speed_kmh', 'sharp_curve_speed_kmh'),
                ('speed_preview_distance_m', 'preview_distance_m'),
                ('curve_curvature_threshold', 'curve_curvature_threshold'),
                ('sharp_curvature_threshold', 'sharp_curvature_threshold'),
                ('speed_curvature_smoothing_window_m', 'curvature_smoothing_window_m'),
                ('speed_curvature_persistence_m', 'curvature_persistence_m'),
                ('speed_curvature_hysteresis', 'curvature_hysteresis')):
            assert getattr(node.speed_planner, attribute) == values[name]
        assert output(node, 'speed_mode') == 'CURVE'
        assert output(node, 'recommended_speed') == 15.0
        assert output(node, 'target_speed') == 15.0
    finally:
        executor.shutdown(timeout_sec=0)
        probe.destroy_node()


@pytest.mark.parametrize('invalid', [
    {'cruise_speed_kmh': -1.0}, {'curve_speed_kmh': -1.0},
    {'sharp_curve_speed_kmh': -1.0}, {'speed_preview_distance_m': 0.0},
    {'speed_curvature_smoothing_window_m': 0.0}, {'speed_curvature_persistence_m': -1.0},
    {'speed_curvature_hysteresis': -0.01}, {'curve_curvature_threshold': -0.01},
    {'curve_curvature_threshold': 0.1}, {'sharp_curvature_threshold': 0.03},
    {'cruise_speed_kmh': math.nan}, {'speed_preview_distance_m': math.inf},
])
def test_invalid_batch_changes_no_parameter_or_planner_state(make_node, invalid):
    node = make_node()
    set_geometry(node, 0.2)
    node.plan()
    before = {p.name: p.value for p in node.get_parameters(list(SPEED_SETTINGS))}
    planner = node.speed_planner
    previous = node._previous_speed_mode
    # The unrelated cap must also remain unchanged when the atomic request fails.
    result = change(node, max_drive_speed_kmh=1.0, **invalid)
    assert not result.successful
    assert result.reason
    assert {p.name: p.value for p in node.get_parameters(list(SPEED_SETTINGS))} == before
    assert node.get_parameter('max_drive_speed_kmh').value == 30.0
    assert node.speed_planner is planner
    assert node._previous_speed_mode == previous
    node.plan()
    assert node.speed_planner is planner
    assert output(node, 'speed_mode') == 'SHARP_CURVE'
    assert output(node, 'recommended_speed') == 3.5
    assert output(node, 'target_speed') == 3.5


def test_threshold_pair_accepts_valid_atomic_change_but_rejects_invalid_intermediate(make_node):
    node = make_node()
    assert not change(node, curve_curvature_threshold=0.12).successful
    assert change(node, curve_curvature_threshold=0.12, sharp_curvature_threshold=0.2).successful
    set_geometry(node, 0.15)
    node.plan()
    assert output(node, 'speed_mode') == 'CURVE'
    assert output(node, 'recommended_speed') == 25.0
    assert output(node, 'target_speed') == 25.0


def test_unchanged_settings_keep_object_hysteresis_and_route_cache(make_node):
    node = make_node()
    set_geometry(node, 0.041)
    node.plan()
    assert output(node, 'speed_mode') == 'CURVE'
    planner = node.speed_planner
    assert change(node, curve_speed_kmh=25.0).successful
    set_geometry(node, 0.039)
    with patch('edgenode_planning.planning_node.SpeedPlanner', side_effect=AssertionError('rebuilt')):
        node.plan()
    assert node.speed_planner is planner
    assert output(node, 'speed_mode') == 'CURVE'
    assert output(node, 'recommended_speed') == 25.0
    node.dijkstra_planner.plan.assert_called_once()


def test_config_change_resets_old_hysteresis_without_resetting_drive_or_route(make_node):
    node = make_node()
    set_geometry(node, 0.041)
    node.plan()
    assert output(node, 'speed_mode') == 'CURVE'
    lease_start = node._drive_session_start_ns
    assert change(node, cruise_speed_kmh=30.0).successful
    set_geometry(node, 0.039)
    node.plan()
    assert output(node, 'speed_mode') == 'CRUISE'
    assert output(node, 'recommended_speed') == 30.0
    assert output(node, 'target_speed') == 30.0
    assert node._drive_session_start_ns == lease_start
    node.dijkstra_planner.plan.assert_called_once()


def test_changed_threshold_does_not_retain_old_sharp_mode(make_node):
    node = make_node()
    set_geometry(node, 0.105)
    node.plan()
    assert output(node, 'speed_mode') == 'SHARP_CURVE'
    assert change(node, sharp_curvature_threshold=0.11).successful
    set_geometry(node, 0.106)
    node.plan()
    assert output(node, 'speed_mode') == 'CURVE'
    assert output(node, 'recommended_speed') == 25.0
    assert output(node, 'target_speed') == 25.0


def test_later_callback_rejection_leaves_parameter_and_hysteresis_unchanged(make_node):
    node = make_node()
    set_geometry(node, 0.041)
    node.plan()
    planner = node.speed_planner
    snapshot = node._speed_planner_snapshot
    # Humble runs newest callbacks first: accept ours, then reject elsewhere.
    node.remove_on_set_parameters_callback(node._validate_speed_parameters)
    node.add_on_set_parameters_callback(
        lambda parameters: SetParametersResult(successful=False, reason='test rejection'))
    node.add_on_set_parameters_callback(node._validate_speed_parameters)
    result = change(node, curve_speed_kmh=15.0)
    assert not result.successful
    assert result.reason == 'test rejection'
    assert node.get_parameter('curve_speed_kmh').value == 25.0
    assert node.speed_planner is planner
    assert node._speed_planner_snapshot == snapshot
    assert node._previous_speed_mode == 'CURVE'
    set_geometry(node, 0.039)
    node.plan()
    assert node.speed_planner is planner
    assert output(node, 'speed_mode') == 'CURVE'
    assert output(node, 'recommended_speed') == 25.0
    assert output(node, 'target_speed') == 25.0


def test_settings_refresh_even_while_localization_is_missing(make_node):
    node = make_node()
    node.latest_pose = None
    assert change(node, cruise_speed_kmh=30.0, curve_speed_kmh=15.0,
                  sharp_curve_speed_kmh=8.0).successful
    node.plan()
    assert node.speed_planner.sharp_curve_speed_kmh == 8.0
    assert node._previous_speed_mode is None
    assert output(node, 'speed_mode') == 'INVALID'
    assert output(node, 'target_speed') == 0.0
    node.latest_pose = (0.0, 0.0, 0.0)
    set_geometry(node, 0.2)
    node.plan()
    assert output(node, 'speed_mode') == 'SHARP_CURVE'
    assert output(node, 'recommended_speed') == 8.0
    assert output(node, 'target_speed') == 8.0


def test_unrelated_live_cap_does_not_rebuild_or_reset_hysteresis(make_node):
    node = make_node()
    set_geometry(node, 0.041)
    node.plan()
    planner = node.speed_planner
    assert change(node, max_drive_speed_kmh=10.0).successful
    set_geometry(node, 0.039)
    node.plan()
    assert node.speed_planner is planner
    assert output(node, 'speed_mode') == 'CURVE'
    assert output(node, 'recommended_speed') == 25.0
    assert output(node, 'target_speed') == 10.0


def test_zero_speed_and_drive_gate_remain_supported(make_node):
    node = make_node()
    set_geometry(node, 0.2)
    assert change(node, sharp_curve_speed_kmh=0.0).successful
    node.plan()
    assert output(node, 'speed_mode') == 'SHARP_CURVE'
    assert output(node, 'recommended_speed') == 0.0
    assert output(node, 'target_speed') == 0.0
    assert change(node, sharp_curve_speed_kmh=8.0, enable_drive=False).successful
    node.plan()
    assert output(node, 'speed_mode') == 'SHARP_CURVE'
    assert output(node, 'recommended_speed') == 8.0
    assert output(node, 'target_speed') == 0.0


def test_invalid_startup_is_safe_and_valid_runtime_snapshot_recovers(make_node):
    node = make_node(curve_curvature_threshold=0.2)
    node.plan()
    assert output(node, 'speed_mode') == 'INVALID'
    assert output(node, 'recommended_speed') == 0.0
    assert output(node, 'target_speed') == 0.0
    assert change(node, curve_curvature_threshold=0.04).successful
    node.plan()
    assert output(node, 'speed_mode') == 'CRUISE'
    assert output(node, 'recommended_speed') == 50.0
    assert output(node, 'target_speed') == 30.0
