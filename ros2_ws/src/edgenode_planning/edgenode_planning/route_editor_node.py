"""RViz Publish Point, persistent markers and services for editing MGeo routes."""

import math
from pathlib import Path as FilePath

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from edgenode_planning.route_editor import RouteEditor, RouteEditorError


def _preview_display_points(points, vehicle_xy):
    """Return a display-only forward slice; never modify the stored route.

    Project onto the nearest XY segment, retaining the first segment on ties.
    Start at its forward endpoint if the projection is past the segment start,
    so rounding to a nearest point cannot put a point behind the vehicle back
    into the display. XYZ values and all following points remain unchanged.
    """
    if vehicle_xy is None or len(points) < 2:
        return points
    x, y = vehicle_xy
    nearest_index, nearest_distance = 0, math.inf
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length == 0 or not math.isfinite(length):
            continue
        ux, uy = dx / length, dy / length
        along = (x - a[0]) * ux + (y - a[1]) * uy
        if not math.isfinite(along):
            continue
        along = max(0.0, min(length, along))
        distance = math.hypot(x - (a[0] + along * ux), y - (a[1] + along * uy))
        if distance < nearest_distance - 1e-9:
            nearest_distance = distance
            nearest_index = index + 1 if along > 0 else index
    return points[nearest_index:]


class RouteEditorNode(Node):
    """Keep draft edits separate from the live Planning route until Activate.

    The static map is constructed/published once with transient-local QoS.
    Each draft captures its start link once, unaffected by later live matches.
    The displayed preview follows localization without recalculating the route.
    Parameter service calls are asynchronous, including their steady-clock
    timeout, so the executor can process responses without nested spinning.
    """

    def __init__(self):
        super().__init__('route_editor_node')
        defaults = {
            'mgeo_dir': '/workspace/local_data/kcity_mgeo',
            'clicked_point_topic': '/clicked_point',
            'current_link_topic': '/planning/current_link',
            'localization_topic': '/localization/odometry',
            'preview_path_topic': '/route_editor/preview_path',
            'status_topic': '/route_editor/status',
            'waypoint_snap_max_distance_m': 5.0,
            'route_name': 'kcity_demo_route',
            'route_save_dir': '/workspace/local_data/kcity_routes',
            'planning_node_name': '/planning_node',
            'start_link_id': '',
            'show_mgeo_nodes': True,
            'show_waypoint_node_ids': False,
            'parameter_timeout_sec': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(
                read_only=name not in ('route_name', 'waypoint_snap_max_distance_m'),
            ))
        self.editor = RouteEditor(self._value('mgeo_dir'))
        self._vehicle_xy = None
        self._activation_pending = False
        self._warned_frames = set()
        self._fallback_link = self._value('start_link_id')
        self._latest_current_link = ''
        timeout = self._value('parameter_timeout_sec')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('parameter_timeout_sec must be finite and positive')

        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(MarkerArray, '/route_editor/mgeo_links', retained)
        self.nodes_pub = self.create_publisher(MarkerArray, '/route_editor/mgeo_nodes', retained)
        self.waypoints_pub = self.create_publisher(MarkerArray, '/route_editor/waypoint_markers', retained)
        self.preview_pub = self.create_publisher(Path, self._value('preview_path_topic'), retained)
        self.status_pub = self.create_publisher(String, self._value('status_topic'), retained)
        self.vehicle_pub = self.create_publisher(Marker, '/route_editor/vehicle_marker', retained)
        self.create_subscription(PointStamped, self._value('clicked_point_topic'), self.clicked_point_callback, 10)
        self.create_subscription(String, self._value('current_link_topic'), self.current_link_callback, 10)
        self.create_subscription(Odometry, self._value('localization_topic'), self.odometry_callback, 10)

        # Humble in this workspace has no rclpy.parameter_client module.
        # Use the same standard services directly, with no Planning changes.
        self._parameter_group = ReentrantCallbackGroup()
        target = self._value('planning_node_name').rstrip('/')
        self.get_parameters_client = self.create_client(
            GetParameters, target + '/get_parameters', callback_group=self._parameter_group,
        )
        self.set_parameters_client = self.create_client(
            SetParametersAtomically, target + '/set_parameters_atomically',
            callback_group=self._parameter_group,
        )
        for name in ('undo', 'clear', 'save', 'load'):
            self.create_service(Trigger, '/route_editor/' + name, getattr(self, name + '_callback'))
        self.create_service(Trigger, '/route_editor/activate', self.activate_callback,
                            callback_group=self._parameter_group)
        self._publish_map()
        self._publish_draft('READY')
        if not self.editor.node_points or not self.editor.valid_link_ids:
            self._status('MAP_UNAVAILABLE: check mgeo_dir and MGeo JSON files', warning=True)
        self.get_logger().info(
            f'Route editor ready: {len(self.editor.node_points)} nodes, '
            f'{len(self.editor.valid_link_ids)} directed links; Publish Point in map frame. '
            'Preview edits do not change the active route. Activation requires drive disabled.'
        )

    def _value(self, name):
        return self.get_parameter(name).value

    @property
    def preview_start_link_id(self):
        """The draft's locked start; empty until a waypoint and valid link exist."""
        return self.editor.start_link_id

    def _start_link_candidate(self):
        if self._latest_current_link:
            return self._latest_current_link
        return self._fallback_link if self._fallback_link in self.editor.valid_link_ids else ''

    def _capture_preview_start_link(self):
        if self.editor.waypoint_node_ids and not self.preview_start_link_id:
            return self.editor.set_start_link(self._start_link_candidate())
        return False

    def _status(self, message, warning=False):
        self.status_pub.publish(String(data=message))
        if warning:
            self.get_logger().warning(message)

    def _marker(self, namespace, marker_id, kind, color, scale):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = namespace, marker_id, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        return marker

    @staticmethod
    def _point(xyz, lift=0.0):
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]) + lift)

    def _publish_map(self):
        links = MarkerArray()
        clear = self._marker('mgeo_links', 0, Marker.LINE_STRIP,
                             (1.0, 1.0, 1.0, 1.0), (1.0, 1.0, 1.0))
        clear.action = Marker.DELETEALL
        links.markers.append(clear)
        for index, link_id in enumerate(sorted(self.editor.valid_link_ids)):
            marker = self._marker('mgeo_links', index, Marker.LINE_STRIP,
                                  (0.6, 0.7, 0.8, 1.0), (0.35, 0.0, 0.0))
            marker.points = [self._point(p) for p in self.editor.planner.links_by_id[link_id]['points']]
            links.markers.append(marker)
        self.map_pub.publish(links)
        nodes = MarkerArray()
        nodes.markers.append(clear)
        if self._value('show_mgeo_nodes'):
            marker = self._marker('mgeo_nodes', 0, Marker.SPHERE_LIST,
                                  (0.2, 0.65, 1.0, 0.9), (1.0, 1.0, 1.0))
            marker.points = [self._point(p, 0.3) for p in self.editor.node_points.values()]
            nodes.markers.append(marker)
        self.nodes_pub.publish(nodes)

    def _publish_draft(self, reason):
        markers = MarkerArray()
        clear = self._marker('waypoints', 0, Marker.SPHERE, (1.0, 1.0, 0.0, 1.0), (1.0, 1.0, 1.0))
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, node_id in enumerate(self.editor.waypoint_node_ids):
            xyz = self.editor.node_points[node_id]
            point = self._marker('waypoints', index, Marker.SPHERE,
                                 (1.0, 0.8, 0.1, 1.0), (2.0, 2.0, 2.0))
            point.pose.position = self._point(xyz, 0.8)
            text = self._marker('waypoint_numbers', index, Marker.TEXT_VIEW_FACING,
                                (1.0, 1.0, 0.4, 1.0), (0.0, 0.0, 3.0))
            text.pose.position = self._point(xyz, 4.0)
            text.text = str(index + 1)
            if self._value('show_waypoint_node_ids'):
                text.text += '\n' + node_id
            markers.markers.extend((point, text))
        self.waypoints_pub.publish(markers)
        self._publish_preview()
        detail = self.editor.preview_status
        if self.editor.preview is not None:
            detail += (f": {self.editor.preview['total_cost_m']:.2f} m, "
                       f"{len(self.editor.preview['link_ids'])} links")
        self._status(f'{reason}; {detail}; waypoints={len(self.editor.waypoint_node_ids)}; '
                     f'start_link={self.editor.start_link_id or "none"}')

    def _publish_preview(self):
        """Publish only the visual slice, keeping validation/activation intact."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        if self.editor.preview is not None:
            for xyz in _preview_display_points(self.editor.preview['points'], self._vehicle_xy):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position = self._point(xyz)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.preview_pub.publish(path)

    def _require_idle(self):
        if self._activation_pending:
            raise RouteEditorError('ACTIVATION_IN_PROGRESS', 'wait for the activation response')

    def clicked_point_callback(self, msg):
        try:
            self._require_idle()
            if msg.header.frame_id != 'map':
                raise RouteEditorError('FRAME_REJECTED', f'expected map, received {msg.header.frame_id!r}; no TF conversion')
            result = self.editor.append_click(msg.point.x, msg.point.y,
                                              self._value('waypoint_snap_max_distance_m'))
            self._capture_preview_start_link()
            self._publish_draft(f"WAYPOINT_ADDED: {result['node_id']}, snap={result['distance_m']:.2f} m")
        except RouteEditorError as error:
            self._status(str(error), warning=True)

    def current_link_callback(self, msg):
        self._latest_current_link = msg.data if msg.data in self.editor.valid_link_ids else ''
        # Only a waiting draft may capture its first start. A live match change
        # must never replace a locked draft's route, cost or activation snapshot.
        if self._capture_preview_start_link():
            self._publish_draft('PREVIEW_START_LINK_LOCKED')

    def odometry_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.header.frame_id == 'map' and math.isfinite(p.x) and math.isfinite(p.y):
            vehicle_xy = (p.x, p.y)
            if vehicle_xy != self._vehicle_xy:
                self._vehicle_xy = vehicle_xy
                if self.editor.preview is not None:
                    self._publish_preview()
        # Missing/invalid localization keeps the last valid XY for display;
        # before the first valid pose, _vehicle_xy=None preserves the full path.
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        valid = msg.header.frame_id == 'map' and all(math.isfinite(v) for v in values)
        norm = math.hypot(q.x, q.y, q.z, q.w) if valid else 0.0
        marker = self._marker('vehicle', 0, Marker.ARROW,
                              (1.0, 0.3, 0.2, 1.0), (4.0, 1.5, 1.5))
        if not valid or not math.isfinite(norm) or norm <= 1e-9:
            marker.action = Marker.DELETE
            frame = msg.header.frame_id
            if frame not in self._warned_frames:
                self._warned_frames.add(frame)
                self.get_logger().warning(f'Vehicle marker rejected invalid pose or frame {frame!r}; map required')
        else:
            marker.pose.position = self._point((p.x, p.y, p.z), 1.0)
            marker.pose.orientation.x, marker.pose.orientation.y = q.x / norm, q.y / norm
            marker.pose.orientation.z, marker.pose.orientation.w = q.z / norm, q.w / norm
        self.vehicle_pub.publish(marker)

    def _edit_service(self, response, operation, success):
        try:
            self._require_idle()
            result = operation()
            if not self.editor.waypoint_node_ids:
                self.editor.set_start_link('')
            self._publish_draft(success)
            response.success = True
            response.message = success + (f': {result}' if result is not None else '')
        except (RouteEditorError, OSError, ValueError, TypeError) as error:
            response.success = False
            response.message = str(error)
            self._status(response.message, warning=True)
        return response

    def undo_callback(self, request, response):
        return self._edit_service(response, self.editor.undo, 'WAYPOINT_REMOVED')

    def clear_callback(self, request, response):
        return self._edit_service(response, self.editor.clear, 'ROUTE_CLEARED')

    def save_callback(self, request, response):
        return self._edit_service(response, lambda: self.editor.save(
            self._value('route_save_dir'), self._value('route_name')), 'ROUTE_SAVED')

    def load_callback(self, request, response):
        return self._edit_service(response, lambda: self.editor.load(
            self._value('route_save_dir'), self._value('route_name'),
            start_link_id=self._start_link_candidate()), 'ROUTE_LOADED')

    async def _call_parameter_service(self, client, request):
        future = client.call_async(request)
        completed = Future()

        def receive(done):
            if completed.done():
                return
            try:
                completed.set_result(done.result())
            except Exception as error:
                completed.set_exception(error)

        def expire():
            if not completed.done():
                completed.set_exception(TimeoutError('Planning parameter response timed out; check Planning parameters'))

        timer = self.create_timer(self._value('parameter_timeout_sec'), expire,
                                  callback_group=self._parameter_group,
                                  clock=Clock(clock_type=ClockType.STEADY_TIME))
        future.add_done_callback(receive)
        try:
            return await completed
        finally:
            self.destroy_timer(timer)
            if not future.done():
                client.remove_pending_request(future)

    async def activate_callback(self, request, response):
        if self._activation_pending:
            response.success, response.message = False, 'ACTIVATION_IN_PROGRESS'
            return response
        self._activation_pending = True
        try:
            node_ids = self.editor.activation_node_ids()
            revision = self.editor.revision
            if not (self.get_parameters_client.service_is_ready()
                    and self.set_parameters_client.service_is_ready()):
                raise RouteEditorError('PLANNING_UNAVAILABLE', 'Planning parameter services are not ready')
            values = (await self._call_parameter_service(self.get_parameters_client, GetParameters.Request(
                names=['enable_drive', 'mgeo_dir', 'global_path_frame_id'],
            ))).values
            if (len(values) != 3 or values[0].type != ParameterType.PARAMETER_BOOL
                    or values[0].bool_value):
                raise RouteEditorError('DRIVE_MUST_BE_DISABLED', 'set /planning_node enable_drive false first')
            if (values[1].type != ParameterType.PARAMETER_STRING
                    or str(FilePath(values[1].string_value).resolve()) != self.editor.mgeo_dir
                    or values[2].type != ParameterType.PARAMETER_STRING or values[2].string_value != 'map'):
                raise RouteEditorError('MAP_MISMATCH', 'Planning must use the same MGeo directory and map frame')
            if self.editor.revision != revision:
                raise RouteEditorError('PREVIEW_CHANGED', 'draft changed; inspect the updated preview and retry')
            # Atomic with drive=false: even a late response or racing enable
            # cannot apply a new route while preserving an armed drive state.
            parameters = [Parameter('enable_drive', value=False),
                          Parameter('route_node_ids', Parameter.Type.STRING_ARRAY, node_ids)]
            result = (await self._call_parameter_service(self.set_parameters_client,
                SetParametersAtomically.Request(parameters=[p.to_parameter_msg() for p in parameters]))).result
            if not result.successful:
                raise RouteEditorError('ACTIVATION_REJECTED', result.reason)
            response.success = True
            response.message = ('ROUTE_ACTIVATED: route_node_ids applied with enable_drive=false; '
                                'verify /planning/global_path before enabling drive')
            self._status(response.message)
        except (RouteEditorError, TimeoutError, RuntimeError, ValueError, TypeError) as error:
            response.success = False
            response.message = str(error)
            self._status('ACTIVATION_FAILED: ' + response.message, warning=True)
        finally:
            self._activation_pending = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RouteEditorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
