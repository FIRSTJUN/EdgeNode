import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.map_matcher import MapMatcher


class PlanningNode(Node):
    """
    Match localization and publish a cached global route at 20 Hz.

    Current stage:
      - Perception is intentionally disconnected.
      - Localization odometry is the only pose input.
      - Global routes are recalculated only when the matched link or goal changes.
      - Vehicle target speed remains 0 km/h for safety.

    Future flow:
      GPS + IMU
          ↓
         EKF
          ↓
      Current Pose
          ↓
         MGeo
          ↓
      Global / Local Path
          ↓
       Planning
          ↓
      target_error
      target_speed
          ↓
       Control
    """

    def __init__(self):
        super().__init__('planning_node')

        # Output topics
        self.declare_parameter(
            'target_error_topic',
            '/planning/target_error',
        )

        self.declare_parameter(
            'target_speed_topic',
            '/planning/target_speed',
        )

        self.declare_parameter(
            'state_topic',
            '/planning/state',
        )

        # Retained for compatibility; this stage always publishes zero speed.
        self.declare_parameter(
            'cruise_speed_kmh',
            0.0,
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('localization_topic', '/localization/odometry'),
                ('current_link_topic', '/planning/current_link'),
                ('map_match_status_topic', '/planning/map_match_status'),
                ('map_match_distance_topic', '/planning/map_match_distance'),
                ('map_match_heading_diff_topic', '/planning/map_match_heading_diff'),
                ('global_path_topic', '/planning/global_path'),
                ('goal_node_id', 'A122CC001096'),
                ('global_path_frame_id', 'map'),
                ('mgeo_dir', '/workspace/local_data/c_track_mgeo'),
                ('map_match_search_radius_m', 6.0),
                ('map_match_max_distance_m', 4.0),
                ('map_match_max_heading_diff_deg', 80.0),
                ('map_match_heading_weight', 0.025),
                ('map_match_same_link_bonus', 0.4),
                ('map_match_connected_link_bonus', 0.7),
                ('map_match_unrelated_link_penalty', 1.0),
            ],
        )

        mgeo_dir = self.get_parameter('mgeo_dir').value
        self.map_matcher = MapMatcher(
            mgeo_dir=mgeo_dir,
            search_radius_m=self.get_parameter('map_match_search_radius_m').value,
            max_match_distance_m=self.get_parameter('map_match_max_distance_m').value,
            max_heading_diff_deg=self.get_parameter('map_match_max_heading_diff_deg').value,
            heading_weight=self.get_parameter('map_match_heading_weight').value,
            same_link_bonus=self.get_parameter('map_match_same_link_bonus').value,
            connected_link_bonus=self.get_parameter('map_match_connected_link_bonus').value,
            unrelated_link_penalty=self.get_parameter('map_match_unrelated_link_penalty').value,
        )
        self.dijkstra_planner = DijkstraPlanner(mgeo_dir=mgeo_dir)
        self._route_key = None
        self._route_result = None
        self.latest_pose = None

        self.target_error_pub = self.create_publisher(
            Float32,
            self.get_parameter('target_error_topic').value,
            10,
        )

        self.target_speed_pub = self.create_publisher(
            Float32,
            self.get_parameter('target_speed_topic').value,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
            10,
        )

        self.current_link_pub = self.create_publisher(
            String,
            self.get_parameter('current_link_topic').value,
            10,
        )
        self.map_match_status_pub = self.create_publisher(
            String,
            self.get_parameter('map_match_status_topic').value,
            10,
        )
        self.map_match_distance_pub = self.create_publisher(
            Float32,
            self.get_parameter('map_match_distance_topic').value,
            10,
        )
        self.map_match_heading_diff_pub = self.create_publisher(
            Float32,
            self.get_parameter('map_match_heading_diff_topic').value,
            10,
        )
        self.global_path_pub = self.create_publisher(
            Path,
            self.get_parameter('global_path_topic').value,
            10,
        )
        self.localization_sub = self.create_subscription(
            Odometry,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )

        # Planning loop: 20 Hz
        self.create_timer(
            0.05,
            self.plan,
        )

        self.get_logger().info(
            f'Planning node ready - loaded {len(self.map_matcher.links)} MGeo links; '
            'waiting for localization (target speed: 0.0 km/h)'
        )

    def localization_callback(self, msg):
        """Store the latest pose; map matching runs only in the planning timer."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (position.x, position.y, yaw)

    def plan(self):
        """Publish diagnostics and valid cached routes with zero commands.

        Cache failures too, so an unreachable goal does not trigger a search
        every tick. During match loss retain the cache but publish no Path;
        a recovered match must validate the link/goal key before reuse.
        """
        state_value = 'WAIT_FOR_LOCALIZATION'
        status_value = 'WAIT_FOR_LOCALIZATION'
        current_link = ''
        distance = float('nan')
        heading_diff = float('nan')

        if self.latest_pose is not None:
            x, y, yaw = self.latest_pose
            result = self.map_matcher.match(x, y, math.degrees(yaw))
            if result is None:
                state_value = 'MAP_MATCH_LOST'
                status_value = 'NO_MATCH'
            else:
                status_value = 'MATCHED'
                current_link = result['link_id']
                distance = result['distance']
                heading_diff = result['heading_diff']

                goal_node_id = self.get_parameter('goal_node_id').value
                route_key = (current_link, goal_node_id)
                if route_key != self._route_key:
                    self._route_result = self.dijkstra_planner.plan(
                        current_link, goal_node_id,
                    )
                    self._route_key = route_key

                if self._route_result is None:
                    state_value = 'GLOBAL_PATH_NOT_FOUND'
                else:
                    state_value = 'GLOBAL_PATH_READY'
                    path = Path()
                    path.header.frame_id = self.get_parameter('global_path_frame_id').value
                    path.header.stamp = self.get_clock().now().to_msg()
                    # Keep the entire start link; trimming belongs to local planning.
                    for point in self._route_result['points']:
                        pose = PoseStamped()
                        pose.header = path.header
                        pose.pose.position.x = float(point[0])
                        pose.pose.position.y = float(point[1])
                        pose.pose.position.z = float(point[2])
                        pose.pose.orientation.w = 1.0
                        path.poses.append(pose)
                    self.global_path_pub.publish(path)

        target_error = Float32()
        target_error.data = 0.0

        target_speed = Float32()
        target_speed.data = 0.0

        state = String()
        state.data = state_value

        self.target_error_pub.publish(target_error)
        self.target_speed_pub.publish(target_speed)
        self.state_pub.publish(state)
        self.current_link_pub.publish(String(data=current_link))
        self.map_match_status_pub.publish(String(data=status_value))
        self.map_match_distance_pub.publish(Float32(data=distance))
        self.map_match_heading_diff_pub.publish(Float32(data=heading_diff))


def main(args=None):
    rclpy.init(args=args)

    node = PlanningNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
