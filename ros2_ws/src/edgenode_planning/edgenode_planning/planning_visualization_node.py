"""Display-only, stable marker updates; never publish Planning inputs."""
import copy
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .dijkstra_planner import DijkstraPlanner


# Marker widths are world metres. Keep the final Control path widest;
# phase colors render as a narrower stripe over its white band.
PATH_STYLES = {
    'global_path': ('global_path_marker', 'global_path', (.20,1.,.35,1.), .46),
    'local_path': ('local_path_marker', 'local_path_debug', (1.,1.,1.,1.), .70),
    'original_lane_center': ('original_lane_marker', 'original_lane', (.20,.80,.95,1.), .24),
    'left_lane_center': ('left_lane_marker', 'left_lane', (1.,.88,.08,1.), .26),
    'right_lane_center': ('right_lane_marker', 'right_lane', (1.,.60,.08,1.), .26),
    'avoidance_selected_path': ('avoidance_selected_marker', 'avoidance_selected', (.05,1.,1.,1.), .34),
    'avoidance_return_path': ('avoidance_return_marker', 'avoidance_return', (1.,.28,.70,1.), .34),
}

# Rendering layers only. Point arrays stay exact copies of the source messages;
# canonical Global/Local mirrors retain the identity marker pose (no lift).
DISPLAY_Z_OFFSETS_M = {
    'original_lane_center': .02,
    'left_lane_center': .04,
    'right_lane_center': .04,
    'avoidance_selected_path': .08,
    'avoidance_return_path': .08,
}


class PlanningVisualizationNode(Node):
    """Cache display geometry; expire lost sources independently of rendering."""

    _PHASE_PATHS = ('avoidance_selected_path', 'avoidance_return_path')
    _REFRESH_SEC = 1.0  # Volatile RViz late joiners receive a complete cached view.

    def __init__(self):
        super().__init__('planning_visualization_node')
        self.declare_parameter('mgeo_dir', '/workspace/local_data/kcity_mgeo')
        self.declare_parameter('freshness_sec', 1.0)
        self.declare_parameter('source_timeout_sec', 3.0)
        self.declare_parameter('obstacle_points_per_cluster', 8)
        self.freshness = float(self.get_parameter('freshness_sec').value)
        self.source_timeout = float(self.get_parameter('source_timeout_sec').value)
        for name, value in (('freshness_sec', self.freshness),
                            ('source_timeout_sec', self.source_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.map = DijkstraPlanner(self.get_parameter('mgeo_dir').value)
        self._valid_link_ids = {edge['link_id'] for edges in self.map.graph.values()
                                for edge in edges}
        self._seen = {}
        self._markers = {}
        self._signatures = {}
        self._last_sent = {}
        self._input_stamp_ns = {}
        self._deleted = set()
        self._obstacle_ids = set()
        self._route_geometry = None
        self._route_epoch_ns = 0
        self._planning_ready = None
        self._phase = None
        self._last_ros_ns = self.get_clock().now().nanoseconds
        self._path_publishers = {}
        for source, (topic, namespace, color, width) in PATH_STYLES.items():
            self._path_publishers[source] = self.create_publisher(Marker, '/planning/'+topic, 10)
            # Visualization needs the newest sample, not a backlog of old cycles.
            self.create_subscription(Path, '/planning/'+source,
                lambda msg, key=source: self.path_callback(key, msg), 1)
        self.current_pub = self.create_publisher(Marker, '/planning/current_lane_marker', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/planning/obstacle_footprint_markers', 10)
        self.vehicle_pub = self.create_publisher(Marker, '/planning/ego_vehicle_marker', 10)
        self.create_subscription(String, '/planning/current_link', self.current_link_callback, 1)
        self.create_subscription(String, '/planning/state', self.state_callback, 1)
        self.create_subscription(String, '/planning/avoidance_state', self.phase_callback, 1)
        self.create_subscription(PoseArray, '/planning/obstacles', self.obstacles_callback, 1)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Marker, '/route_editor/vehicle_marker', self.vehicle_callback, retained)
        # One startup cleanup also removes infinite markers from an older process.
        for key in PATH_STYLES:
            self._delete_path(key)
        self._delete_single('current')
        self._delete_single('vehicle')
        clear = self._marker('obstacle_footprint')
        clear.action = Marker.DELETEALL  # Only here: previous process IDs are unknown.
        self.obstacle_pub.publish(MarkerArray(markers=[clear]))
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.expire, clock=self._steady_clock)

    def _marker(self, namespace, color=(1., 1., 1., 1.), width=.1):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.scale.x = width
        # Rendering never races a per-message deadline. The steady watchdog
        # explicitly deletes stale sources; input age only controls admission.
        marker.lifetime = Duration(seconds=0).to_msg()
        return marker

    def _publisher(self, key):
        if key in self._path_publishers:
            return self._path_publishers[key]
        return {'current': self.current_pub, 'vehicle': self.vehicle_pub,
                'obstacles': self.obstacle_pub}[key]

    def _send_cached(self, key):
        msg = self._markers[key]
        stamp = self.get_clock().now().to_msg()
        for marker in msg.markers if key == 'obstacles' else (msg,):
            marker.header.stamp = stamp
        self._publisher(key).publish(msg)
        self._last_sent[key] = time.monotonic()

    def _remember(self, key, msg, signature):
        self._markers[key] = msg
        self._signatures[key] = signature
        self._deleted.discard(key)
        self._send_cached(key)

    def _touch(self, key):
        self._seen[key] = time.monotonic() + self.source_timeout

    def _forget(self, key):
        for cache in (self._markers, self._signatures, self._last_sent, self._seen):
            cache.pop(key, None)

    def _delete_single(self, key):
        if key in self._deleted:
            return
        namespace = PATH_STYLES[key][1] if key in PATH_STYLES else {
            'current': 'current_lane', 'vehicle': 'ego_vehicle'}[key]
        marker = self._marker(namespace)
        marker.action = Marker.DELETE
        self._publisher(key).publish(marker)
        self._forget(key)
        self._deleted.add(key)

    def _delete_path(self, key):
        self._delete_single(key)

    def _clear_obstacles(self):
        if self._obstacle_ids:
            markers = []
            for marker_id in sorted(self._obstacle_ids):
                marker = self._marker('obstacle_footprint')
                marker.id = marker_id
                marker.action = Marker.DELETE
                markers.append(marker)
            self.obstacle_pub.publish(MarkerArray(markers=markers))
        self._obstacle_ids.clear()
        self._forget('obstacles')

    def _check_clock(self):
        now = self.get_clock().now().nanoseconds
        if now < self._last_ros_ns:
            for key in PATH_STYLES:
                self._delete_path(key)
            self._delete_single('current')
            self._delete_single('vehicle')
            self._clear_obstacles()
            self._route_geometry = None
            self._route_epoch_ns = 0
            self._input_stamp_ns.clear()
        self._last_ros_ns = now
        return now

    def _accept_stamp(self, key, header):
        now = self._check_clock()
        stamp_ns = header.stamp.sec*10**9 + header.stamp.nanosec
        age = (now-stamp_ns)/1e9
        if not -.05 <= age < self.freshness:
            return False
        if stamp_ns < self._input_stamp_ns.get(key, 0):
            return False
        if key in PATH_STYLES and key != 'global_path' and stamp_ns < self._route_epoch_ns:
            return False
        self._input_stamp_ns[key] = stamp_ns
        return True

    def path_callback(self, key, msg):
        self._check_clock()
        # Late or reordered data must not erase a newer valid display.
        if not self._accept_stamp(key, msg.header):
            return
        points = tuple((p.pose.position.x, p.pose.position.y, p.pose.position.z)
                       for p in msg.poses)
        allowed = (self._planning_ready is not False and
                   not (key in self._PHASE_PATHS and self._phase in
                        ('BLOCKED', 'NORMAL')) and
                   not (key in ('left_lane_center','right_lane_center') and self._phase == 'NORMAL'))
        if (not allowed or msg.header.frame_id != 'map' or not points or
                not all(math.isfinite(v) for point in points for v in point)):
            self._delete_path(key)
            return
        if key == 'global_path' and points != self._route_geometry:
            self._route_geometry = points
            self._route_epoch_ns = self._input_stamp_ns[key]
            for old_key in PATH_STYLES:
                if old_key != 'global_path':
                    self._delete_path(old_key)
        self._touch(key)
        if self._signatures.get(key) == points:
            return
        topic, namespace, color, width = PATH_STYLES[key]
        marker = self._marker(namespace, color, width)
        marker.pose.position.z = DISPLAY_Z_OFFSETS_M.get(key, 0.)
        # Exact copies: no routing, trimming, resampling or XYZ changes.
        marker.points = [copy.deepcopy(p.pose.position) for p in msg.poses]
        self._remember(key, marker, points)

    def current_link_callback(self, msg):
        self._check_clock()
        if msg.data not in self._valid_link_ids:
            self._delete_single('current')
            return
        self._touch('current')
        if self._signatures.get('current') == msg.data:
            return
        marker = self._marker('current_lane', (.20, .65, 1., 1.), .30)
        marker.pose.position.z = .05
        marker.points = [Point(x=float(x), y=float(y), z=float(z))
                         for x, y, z in self.map.links_by_id[msg.data]['points']]
        self._remember('current', marker, msg.data)

    def state_callback(self, msg):
        self._planning_ready = msg.data == 'GLOBAL_PATH_READY'
        if not self._planning_ready:
            for key in PATH_STYLES:
                self._delete_path(key)

    def phase_callback(self, msg):
        self._phase = msg.data
        if msg.data in ('BLOCKED', 'NORMAL'):
            for key in self._PHASE_PATHS:
                self._delete_path(key)
            if msg.data == 'NORMAL':
                self._delete_path('left_lane_center')
                self._delete_path('right_lane_center')

    def obstacles_callback(self, msg):
        if not self._accept_stamp('obstacles', msg.header):
            return
        points = tuple((p.position.x, p.position.y, p.position.z) for p in msg.poses)
        if (msg.header.frame_id != 'map' or not points or
                not all(math.isfinite(v) for point in points for v in point)):
            self._clear_obstacles()
            return
        self._touch('obstacles')
        size = self.get_parameter('obstacle_points_per_cluster').value
        grouped = size == 8 and len(points) % 8 == 0
        signature = (grouped, points)
        if self._signatures.get('obstacles') == signature:
            return
        size = 8 if grouped else len(points)
        active = []
        for i in range(0, len(points), size):
            marker = self._marker('obstacle_footprint', (1., .40, .08, 1.), .22)
            marker.id = i//size
            marker.points = [copy.deepcopy(p.position) for p in msg.poses[i:i+size]]
            if grouped:
                marker.points.append(copy.deepcopy(marker.points[0]))
            else:
                marker.type = Marker.POINTS
                marker.scale.y = .22
            active.append(marker)
        new_ids = {marker.id for marker in active}
        removed = []
        for marker_id in sorted(self._obstacle_ids-new_ids):
            marker = self._marker('obstacle_footprint')
            marker.id = marker_id
            marker.action = Marker.DELETE
            removed.append(marker)
        # Only disappeared IDs are deleted. Retained IDs are updated by ADD.
        self.obstacle_pub.publish(MarkerArray(markers=removed+active))
        self._obstacle_ids = new_ids
        self._markers['obstacles'] = MarkerArray(markers=active)
        self._signatures['obstacles'] = signature
        self._last_sent['obstacles'] = time.monotonic()

    def vehicle_callback(self, msg):
        if not self._accept_stamp('vehicle', msg.header):
            return
        if msg.header.frame_id != 'map' or msg.action != Marker.ADD:
            self._delete_single('vehicle')
            return
        self._touch('vehicle')
        # Source marker has stable ID 0; preserve its actual pose/style exactly.
        signature = (msg.pose, msg.scale, msg.color, msg.type, msg.points)
        if self._signatures.get('vehicle') == signature:
            return
        marker = copy.deepcopy(msg)
        marker.ns = 'ego_vehicle'
        marker.id = 0
        marker.lifetime = Duration(seconds=0).to_msg()
        self._remember('vehicle', marker, copy.deepcopy(signature))

    def expire(self):
        self._check_clock()
        now = time.monotonic()
        for key, deadline in list(self._seen.items()):
            if now >= deadline:
                if key == 'obstacles':
                    self._clear_obstacles()
                else:
                    self._delete_single(key)
        # No DELETE/ADD refresh cycle. Cache refresh supports volatile RViz
        # subscribers without copying/resampling unchanged source geometry.
        for key in list(self._markers):
            if now-self._last_sent[key] >= self._REFRESH_SEC:
                self._send_cached(key)


def main(args=None):
    rclpy.init(args=args)
    node=PlanningVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node();rclpy.shutdown()
