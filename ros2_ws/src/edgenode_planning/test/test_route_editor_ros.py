"""Isolated real ROS transport smoke test: no MORAI or driving nodes are started."""

import time
import uuid
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from edgenode_planning.route_editor_node import RouteEditorNode


def test_real_ros_late_join_click_preview_and_async_activation(route_editor_map, tmp_path):
    prefix = '/route_editor_test_' + uuid.uuid4().hex[:8]
    args = ['--ros-args', '-r', '__ns:=' + prefix]
    overrides = {
        'mgeo_dir': str(route_editor_map),
        'planning_node_name': prefix + '/planning_stub',
        'clicked_point_topic': prefix + '/clicked_point',
        'current_link_topic': prefix + '/current_link',
        'localization_topic': prefix + '/odometry',
        'preview_path_topic': prefix + '/preview_path',
        'status_topic': prefix + '/status',
        'route_save_dir': str(tmp_path / 'routes'),
        'parameter_timeout_sec': '0.5',
    }
    for name, value in overrides.items():
        args.extend(['-p', f'{name}:={value}'])
    # Remap every absolute editor output/service, leaving live project topics untouched.
    for name in ('mgeo_links', 'mgeo_nodes', 'waypoint_markers', 'vehicle_marker',
                 'undo', 'clear', 'save', 'load', 'activate'):
        args.extend(['-r', f'/route_editor/{name}:={prefix}/{name}'])
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = []

    def until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'ROS operation timed out'

    try:
        stub = Node('planning_stub')
        nodes.append(stub)
        stub.declare_parameter('enable_drive', False)
        stub.declare_parameter('mgeo_dir', str(route_editor_map))
        stub.declare_parameter('global_path_frame_id', 'map')
        stub.declare_parameter('route_node_ids', [], ParameterDescriptor(dynamic_typing=True))
        editor = RouteEditorNode()
        nodes.append(editor)
        probe = Node('probe')
        nodes.append(probe)
        for node in nodes:
            executor.add_node(node)

        received_map = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        probe.create_subscription(MarkerArray, prefix + '/mgeo_links', received_map.append, qos)
        until(lambda: bool(received_map))
        assert len([m for m in received_map[0].markers if m.action == Marker.ADD]) == 3
        links = probe.create_publisher(String, prefix + '/current_link', 10)
        clicks = probe.create_publisher(PointStamped, prefix + '/clicked_point', 10)
        until(lambda: links.get_subscription_count() > 0 and clicks.get_subscription_count() > 0)
        links.publish(String(data='AB'))
        until(lambda: editor._latest_current_link == 'AB')
        assert editor.preview_start_link_id == ''
        click = PointStamped()
        click.header.frame_id = 'map'
        click.point.x = 20.0
        clicks.publish(click)
        until(lambda: editor.editor.preview is not None)
        assert editor.editor.preview['link_ids'] == ['AB', 'BC']

        # Odometry only trims the published visualization, never the cached
        # route that is validated and later activated.
        previews = []
        probe.create_subscription(Path, prefix + '/preview_path', previews.append, qos)
        odometry = probe.create_publisher(Odometry, prefix + '/odometry', 10)
        until(lambda: odometry.get_subscription_count() > 0)
        pose = Odometry()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = 5.2
        pose.pose.pose.orientation.w = 1.0
        odometry.publish(pose)
        until(lambda: bool(previews) and bool(previews[-1].poses)
              and previews[-1].poses[0].pose.position.x == 6.0)
        assert [p.pose.position.x for p in previews[-1].poses] == list(range(6, 21))
        assert editor.editor.preview['points'] == [[x, 0, 0] for x in range(21)]
        assert editor.editor.preview['node_ids'] == ['A', 'B', 'C']
        assert editor.editor.preview['total_cost_m'] == 20.0

        original_route = deepcopy(editor.editor.preview)
        preview_count = len(previews)
        for link_id in ('BC', 'AB', 'CD', 'BC'):
            links.publish(String(data=link_id))
            until(lambda: editor._latest_current_link == link_id)
            assert editor.preview_start_link_id == 'AB'
            assert editor.editor.preview == original_route
        assert len(previews) == preview_count

        client = probe.create_client(Trigger, prefix + '/activate')
        until(lambda: client.service_is_ready() and editor.get_parameters_client.service_is_ready()
              and editor.set_parameters_client.service_is_ready())
        future = client.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success, future.result().message
        assert stub.get_parameter('route_node_ids').value == ['C']
        assert stub.get_parameter('enable_drive').value is False

        # Also exercise the real steady-clock timeout, with the remote request
        # deliberately unanswered. This must return, not deadlock the executor.
        get_service = next(service for service in stub._services
                           if service.srv_name.endswith('/get_parameters'))
        original_callback = get_service.callback
        from rclpy.task import Future
        release = Future()

        async def unanswered(request, response):
            await release
            return original_callback(request, response)

        get_service.callback = unanswered
        timed = client.call_async(Trigger.Request())
        until(timed.done)
        assert not timed.result().success
        assert 'timed out' in timed.result().message
        release.set_result(None)
        executor.spin_once(timeout_sec=0.02)
        get_service.callback = original_callback
        assert stub.get_parameter('route_node_ids').value == ['C']
    finally:
        executor.shutdown(timeout_sec=0)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
