"""ROS message/service boundary tests with mocked transport and the real editor."""

import asyncio
from copy import deepcopy
import math
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from edgenode_planning.route_editor_node import RouteEditorNode, _preview_display_points


@pytest.fixture
def node(monkeypatch, route_editor_map, tmp_path):
    parameters = {}
    publishers = {}

    def declare(self, name, value, descriptor):
        parameters[name] = value
        if name == 'mgeo_dir':
            parameters[name] = str(route_editor_map)
        if name == 'route_save_dir':
            parameters[name] = str(tmp_path / 'routes')

    def publisher(self, message_type, topic, qos):
        pub = Mock()
        pub.qos = qos
        publishers[topic] = pub
        return pub

    monkeypatch.setattr(Node, '__init__', lambda self, *args, **kwargs: None)
    monkeypatch.setattr(Node, 'declare_parameter', declare)
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_subscription', Mock())
    monkeypatch.setattr(Node, 'create_service', Mock())
    monkeypatch.setattr(Node, 'create_client', lambda *args, **kwargs: Mock())
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(now=lambda: Time(seconds=1)))
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    result = RouteEditorNode()
    result.test_parameters = parameters
    result.test_publishers = publishers
    return result


def click(node, x, frame='map'):
    msg = PointStamped()
    msg.header.frame_id = frame
    msg.point.x = float(x)
    node.clicked_point_callback(msg)


def ready(node):
    node.current_link_callback(String(data='AB'))
    click(node, 20)


def localize(node, x, y=0.0, frame='map'):
    msg = Odometry()
    msg.header.frame_id = frame
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.w = 1.0
    node.odometry_callback(msg)


def displayed_points(node):
    return [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in node.preview_pub.publish.call_args.args[0].poses]


def service_values(node, drive=False, frame='map', mgeo_dir=None):
    return GetParameters.Response(values=[
        Parameter('enable_drive', value=drive).get_parameter_value(),
        Parameter('mgeo_dir', value=mgeo_dir or node.editor.mgeo_dir).get_parameter_value(),
        Parameter('global_path_frame_id', value=frame).get_parameter_value(),
    ])


def activate(node):
    return asyncio.run(node.activate_callback(Trigger.Request(), Trigger.Response()))


def test_static_map_is_latched_and_not_rebuilt_on_click_or_repeated_link(node):
    assert node.map_pub.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    markers = node.map_pub.publish.call_args.args[0].markers
    assert markers[0].action == Marker.DELETEALL
    assert len([m for m in markers if m.action == Marker.ADD]) == 3
    ready(node)
    before = node.preview_pub.publish.call_count
    node.current_link_callback(String(data='AB'))
    assert node.preview_pub.publish.call_count == before
    node.map_pub.publish.assert_called_once()


def test_numbered_waypoints_and_clear_delete_stale_markers_and_preview(node):
    ready(node)
    click(node, 30)
    markers = node.waypoints_pub.publish.call_args.args[0].markers
    assert [m.text for m in markers if m.type == Marker.TEXT_VIEW_FACING] == ['1', '2']
    assert markers[0].action == Marker.DELETEALL
    assert node.preview_pub.publish.call_args.args[0].header.frame_id == 'map'
    response = node.clear_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    assert node.preview_pub.publish.call_args.args[0].poses == []
    assert len(node.waypoints_pub.publish.call_args.args[0].markers) == 1
    node.set_parameters_client.call_async.assert_not_called()


def test_non_map_click_and_too_far_click_are_rejected(node):
    click(node, 20, frame='odom')
    assert node.editor.waypoint_node_ids == []
    assert 'FRAME_REJECTED' in node.status_pub.publish.call_args.args[0].data
    click(node, 100)
    assert 'SNAP_TOO_FAR' in node.status_pub.publish.call_args.args[0].data


def test_current_link_has_priority_when_capturing_then_stays_locked(node):
    node._fallback_link = 'AB'
    node.current_link_callback(String(data='BC'))
    click(node, 30)
    assert node.editor.preview['start_link_id'] == 'BC'
    node.current_link_callback(String(data=''))
    assert node.editor.preview['start_link_id'] == 'BC'
    assert node.clear_callback(None, Trigger.Response()).success
    click(node, 30)
    assert node.editor.preview['start_link_id'] == 'AB'


def test_unreachable_preview_publishes_empty_path(node):
    ready(node)
    click(node, 50)
    assert node.preview_pub.publish.call_args.args[0].poses == []
    assert 'ROUTE_NOT_FOUND' in node.status_pub.publish.call_args.args[0].data


def test_vehicle_marker_uses_map_odometry_and_deletes_invalid_pose(node):
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 12.0
    msg.pose.pose.orientation.w = 2.0
    node.odometry_callback(msg)
    marker = node.vehicle_pub.publish.call_args.args[0]
    assert marker.action == Marker.ADD
    assert marker.pose.position.x == 12
    assert marker.pose.orientation.w == 1
    msg.header.frame_id = 'odom'
    node.odometry_callback(msg)
    assert node.vehicle_pub.publish.call_args.args[0].action == Marker.DELETE


def test_save_load_and_undo_services_only_change_draft(node):
    ready(node)
    click(node, 30)
    assert node.save_callback(None, Trigger.Response()).success
    assert node.undo_callback(None, Trigger.Response()).success
    assert node.editor.waypoint_node_ids == ['C']
    assert node.load_callback(None, Trigger.Response()).success
    assert node.editor.waypoint_node_ids == ['C', 'D']
    node.set_parameters_client.call_async.assert_not_called()


def test_valid_activation_applies_ordered_string_array_atomically_with_drive_false(node):
    ready(node)
    node._call_parameter_service = AsyncMock(side_effect=[
        service_values(node), SetParametersAtomically.Response(result=SetParametersResult(successful=True)),
    ])
    response = activate(node)
    assert response.success
    assert 'ROUTE_ACTIVATED' in response.message
    request = node._call_parameter_service.call_args_list[1].args[1]
    parameters = {p.name: Parameter.from_parameter_msg(p).value for p in request.parameters}
    assert parameters == {'enable_drive': False, 'route_node_ids': ['C']}
    assert not node._activation_pending


@pytest.mark.parametrize('mode', ['empty', 'unreachable', 'no_start'])
def test_invalid_draft_does_not_call_planning(node, mode):
    if mode == 'no_start':
        click(node, 20)
    elif mode != 'empty':
        ready(node)
    if mode == 'unreachable':
        click(node, 50)
    node._call_parameter_service = AsyncMock()
    assert not activate(node).success
    node._call_parameter_service.assert_not_called()


@pytest.mark.parametrize('case', ['drive', 'map', 'frame', 'uninitialized'])
def test_activation_refuses_armed_drive_or_mismatched_planning(node, case):
    ready(node)
    response = service_values(node, drive=case == 'drive', frame='odom' if case == 'frame' else 'map',
                               mgeo_dir='/other/map' if case == 'map' else None)
    if case == 'uninitialized':
        response.values[0] = ParameterValue()
    node._call_parameter_service = AsyncMock(return_value=response)
    assert not activate(node).success
    assert node._call_parameter_service.call_count == 1


def test_activation_unavailable_rejected_and_timeout_never_report_success(node):
    ready(node)
    node.get_parameters_client.service_is_ready.return_value = False
    assert not activate(node).success
    node.get_parameters_client.service_is_ready.return_value = True
    node._call_parameter_service = AsyncMock(side_effect=[service_values(node),
        SetParametersAtomically.Response(result=SetParametersResult(successful=False, reason='rejected'))])
    assert not activate(node).success
    node._call_parameter_service = AsyncMock(side_effect=TimeoutError('timeout'))
    assert not activate(node).success
    assert not node._activation_pending


def test_live_link_changes_during_activation_keep_captured_preview_and_succeed(node):
    ready(node)
    original = deepcopy(node.editor.preview)

    async def changed(client, request):
        for link_id in ('BC', 'AB', 'CD', 'BC'):
            node.current_link_callback(String(data=link_id))
        if isinstance(request, GetParameters.Request):
            return service_values(node)
        return SetParametersAtomically.Response(result=SetParametersResult(successful=True))

    node._call_parameter_service = AsyncMock(side_effect=changed)
    response = activate(node)
    assert response.success
    assert node.preview_start_link_id == 'AB'
    assert node.editor.preview == original
    assert node._call_parameter_service.call_count == 2


def test_pending_activation_blocks_draft_mutations(node):
    ready(node)
    node._activation_pending = True
    click(node, 30)
    assert not node.clear_callback(None, Trigger.Response()).success
    assert not node.load_callback(None, Trigger.Response()).success
    assert not activate(node).success
    assert node.editor.waypoint_node_ids == ['C']


@pytest.mark.parametrize('x,start_index', [
    (-1.0, 0), (0.0, 0), (0.1, 1), (5.0, 5), (5.2, 6),
    (19.8, 20), (20.0, 20), (22.0, 20),
])
def test_preview_display_trims_behind_vehicle_and_preserves_forward_route(node, x, start_index):
    node.editor.planner.plan_route = Mock(wraps=node.editor.planner.plan_route)
    ready(node)
    route = node.editor.preview
    original = deepcopy(route)
    revision = node.editor.revision
    marker_calls = node.waypoints_pub.publish.call_count
    localize(node, x)
    assert displayed_points(node) == original['points'][start_index:]
    assert node.editor.preview is route
    assert route == original
    assert node.editor.waypoint_node_ids == ['C']
    assert node.editor.revision == revision
    node.editor.planner.plan_route.assert_called_once_with('AB', ['C'])
    assert node.waypoints_pub.publish.call_count == marker_calls
    node.map_pub.publish.assert_called_once()
    node.set_parameters_client.call_async.assert_not_called()


def test_preview_without_localization_displays_full_route(node):
    ready(node)
    assert displayed_points(node) == node.editor.preview['points']
    assert node._vehicle_xy is None


def test_pose_received_before_preview_is_used_when_draft_is_created(node):
    localize(node, 5.2)
    ready(node)
    assert displayed_points(node) == node.editor.preview['points'][6:]


@pytest.mark.parametrize('x,y,frame', [(math.nan, 0, 'map'), (0, math.inf, 'map'), (1, 0, 'odom')])
def test_invalid_localization_keeps_last_valid_preview_without_crashing(node, x, y, frame):
    ready(node)
    localize(node, 5.2)
    expected = displayed_points(node)
    localize(node, x, y, frame)
    # A later edit/republication also retains the last usable localization.
    node._publish_preview()
    assert displayed_points(node) == expected
    assert node._vehicle_xy == (5.2, 0.0)


def test_trimmed_preview_does_not_change_save_load_or_activation(node):
    ready(node)
    click(node, 30)
    original = deepcopy(node.editor.preview)
    localize(node, 5.2)
    assert displayed_points(node) == original['points'][6:]
    assert node.save_callback(None, Trigger.Response()).success
    assert node.clear_callback(None, Trigger.Response()).success
    assert displayed_points(node) == []
    assert node.load_callback(None, Trigger.Response()).success
    assert node.editor.preview == original
    assert displayed_points(node) == original['points'][6:]
    node._call_parameter_service = AsyncMock(side_effect=[
        service_values(node), SetParametersAtomically.Response(result=SetParametersResult(successful=True)),
    ])
    assert activate(node).success
    request = node._call_parameter_service.call_args_list[1].args[1]
    parameters = {p.name: Parameter.from_parameter_msg(p).value for p in request.parameters}
    assert parameters['route_node_ids'] == ['C', 'D']
    assert node.editor.preview == original


@pytest.mark.parametrize('points,xy,expected', [
    ([[10, 0, 1], [5, 0, 2], [0, 0, 3]], (7, 0), [[5, 0, 2], [0, 0, 3]]),
    ([[0, 0, 1], [0, 5, 2], [0, 10, 3]], (0, 5.2), [[0, 10, 3]]),
    ([[0, 0, 1], [10, 0, 2], [10, 10, 3]], (10, 5), [[10, 10, 3]]),
    ([[0, 0, 1], [0, 0, 2], [5, 0, 3]], (2, 0), [[5, 0, 3]]),
    ([[1, 2, 3]], (2, 2), [[1, 2, 3]]),
    ([], (2, 2), []),
])
def test_display_trim_follows_directed_geometry_and_handles_degenerate_paths(points, xy, expected):
    original = deepcopy(points)
    assert _preview_display_points(points, xy) == expected
    assert points == original


def test_display_trim_chooses_first_segment_at_a_crossing():
    points = [[0, 0, 0], [10, 0, 0], [10, 10, 0], [5, 0, 0], [5, -10, 0]]
    assert _preview_display_points(points, (5, 0)) == points[1:]


def test_first_waypoint_captures_latest_link_not_an_earlier_match(node):
    node.current_link_callback(String(data='AB'))
    node.current_link_callback(String(data='BC'))
    assert node.preview_start_link_id == ''
    assert node.editor.waypoint_node_ids == []
    click(node, 30)
    assert node.preview_start_link_id == 'BC'
    assert node.editor.preview['start_link_id'] == 'BC'


def test_locked_preview_points_cost_revision_and_publish_stay_stable_under_link_noise(node):
    node.editor.planner.plan_route = Mock(wraps=node.editor.planner.plan_route)
    ready(node)
    original = node.editor.preview
    snapshot = deepcopy(original)
    message = node.preview_pub.publish.call_args.args[0]
    revision = node.editor.revision
    marker_calls = node.waypoints_pub.publish.call_count
    preview_calls = node.preview_pub.publish.call_count
    for link_id in ('BC', 'AB', 'CD', 'BC', '', 'unknown'):
        node.current_link_callback(String(data=link_id))
        assert node.preview_start_link_id == 'AB'
        assert node.editor.preview is original
        assert node.editor.preview == snapshot
        assert node.editor.revision == revision
        assert node.preview_pub.publish.call_args.args[0] is message
    node.editor.planner.plan_route.assert_called_once_with('AB', ['C'])
    assert node.preview_pub.publish.call_count == preview_calls
    assert node.waypoints_pub.publish.call_count == marker_calls
    node.map_pub.publish.assert_called_once()


def test_additional_waypoints_always_plan_from_the_locked_link(node):
    node.editor.planner.plan_route = Mock(wraps=node.editor.planner.plan_route)
    node.current_link_callback(String(data='AB'))
    for x, next_link in ((10, 'BC'), (20, 'CD'), (30, 'BC')):
        click(node, x)
        node.current_link_callback(String(data=next_link))
        assert node.preview_start_link_id == 'AB'
        assert node.editor.preview['start_link_id'] == 'AB'
    assert node.editor.waypoint_node_ids == ['B', 'C', 'D']
    assert node.editor.planner.plan_route.call_count == 3
    assert all(call.args[0] == 'AB' for call in node.editor.planner.plan_route.call_args_list)
    assert node.editor.preview['total_cost_m'] == 30.0


def test_undo_keeps_lock_until_last_waypoint_is_removed(node):
    ready(node)
    click(node, 30)
    node.current_link_callback(String(data='BC'))
    assert node.undo_callback(None, Trigger.Response()).success
    assert node.editor.waypoint_node_ids == ['C']
    assert node.preview_start_link_id == 'AB'
    assert node.editor.preview['start_link_id'] == 'AB'
    assert node.undo_callback(None, Trigger.Response()).success
    assert node.preview_start_link_id == ''
    assert node.editor.preview is None
    click(node, 30)
    assert node.preview_start_link_id == 'BC'


def test_clear_releases_lock_and_new_draft_captures_latest_link(node):
    ready(node)
    assert node.clear_callback(None, Trigger.Response()).success
    assert node.preview_start_link_id == ''
    assert node.editor.preview is None
    assert displayed_points(node) == []
    node.current_link_callback(String(data='CD'))
    assert node.preview_start_link_id == ''
    click(node, 30)
    assert node.preview_start_link_id == 'CD'
    assert node.editor.preview['start_link_id'] == 'CD'


def test_waiting_draft_captures_first_valid_link_only_once(node):
    node.editor.planner.plan_route = Mock(wraps=node.editor.planner.plan_route)
    click(node, 30)
    assert node.editor.waypoint_node_ids == ['D']
    assert node.editor.preview_status == 'WAITING_FOR_CURRENT_LINK'
    node.current_link_callback(String(data='unknown'))
    assert node.preview_start_link_id == ''
    node.current_link_callback(String(data='AB'))
    original = deepcopy(node.editor.preview)
    for link_id in ('BC', 'CD', 'AB'):
        node.current_link_callback(String(data=link_id))
        assert node.preview_start_link_id == 'AB'
        assert node.editor.preview == original
    node.editor.planner.plan_route.assert_called_once_with('AB', ['D'])


def test_load_recaptures_current_link_once_and_locks_the_new_session(node):
    ready(node)
    click(node, 30)
    assert node.save_callback(None, Trigger.Response()).success
    node.current_link_callback(String(data='BC'))
    assert node.preview_start_link_id == 'AB'
    node.editor.planner.plan_route = Mock(wraps=node.editor.planner.plan_route)
    assert node.load_callback(None, Trigger.Response()).success
    assert node.preview_start_link_id == 'BC'
    assert node.editor.waypoint_node_ids == ['C', 'D']
    assert node.editor.preview['link_ids'] == ['BC', 'CD']
    original = deepcopy(node.editor.preview)
    for link_id in ('AB', 'CD', 'BC'):
        node.current_link_callback(String(data=link_id))
        assert node.preview_start_link_id == 'BC'
        assert node.editor.preview == original
    node.editor.planner.plan_route.assert_called_once_with('BC', ['C', 'D'])


def test_load_without_current_link_waits_for_a_new_capture(node):
    ready(node)
    assert node.save_callback(None, Trigger.Response()).success
    node.current_link_callback(String(data=''))
    assert node.load_callback(None, Trigger.Response()).success
    assert node.preview_start_link_id == ''
    assert node.editor.waypoint_node_ids == ['C']
    assert node.editor.preview_status == 'WAITING_FOR_CURRENT_LINK'
    node.current_link_callback(String(data='BC'))
    assert node.preview_start_link_id == 'BC'
    node.current_link_callback(String(data='AB'))
    assert node.preview_start_link_id == 'BC'


def test_offline_fallback_is_captured_and_not_replaced_by_later_live_links(node):
    node._fallback_link = 'AB'
    click(node, 30)
    assert node.preview_start_link_id == 'AB'
    node.current_link_callback(String(data='BC'))
    assert node.preview_start_link_id == 'AB'
    assert node.editor.preview['start_link_id'] == 'AB'


def test_rejected_first_click_does_not_capture_a_start_link(node):
    node.current_link_callback(String(data='AB'))
    click(node, 100)
    assert node.preview_start_link_id == ''
    node.current_link_callback(String(data='BC'))
    click(node, 30)
    assert node.preview_start_link_id == 'BC'


def test_pose_based_trim_still_updates_without_unlocking_or_replanning(node):
    ready(node)
    original = deepcopy(node.editor.preview)
    node.editor.planner.plan_route = Mock(wraps=node.editor.planner.plan_route)
    node.current_link_callback(String(data='BC'))
    localize(node, 5.2)
    assert displayed_points(node) == original['points'][6:]
    node.current_link_callback(String(data='CD'))
    localize(node, 7.2)
    assert displayed_points(node) == original['points'][8:]
    assert node.editor.preview == original
    assert node.preview_start_link_id == 'AB'
    node.editor.planner.plan_route.assert_not_called()
