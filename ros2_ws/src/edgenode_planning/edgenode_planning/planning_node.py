import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PlanningNode(Node):
    """
    Baseline behavior + local decision planner.

    FSM:
      LANE_FOLLOW
      AVOID_LEFT
      AVOID_RIGHT
      STOP
      LANE_LOST

    The "lattice" part is intentionally lightweight for the first baseline:
    three candidate lateral offsets (left / center / right) are scored against
    the nearest LiDAR cluster and the lowest-cost candidate is selected.
    """

    def __init__(self):
        super().__init__('planning_node')

        # Topics
        for name, default in [
            ('lane_error_topic', '/perception/lane_error'),
            ('lane_confidence_topic', '/perception/lane_confidence'),
            ('obstacle_topic', '/perception/obstacle'),
            ('target_error_topic', '/planning/target_error'),
            ('target_speed_topic', '/planning/target_speed'),
            ('state_topic', '/planning/state'),
        ]:
            self.declare_parameter(name, default)

        # Numeric parameters
        for name, default in [
            ('cruise_speed_kmh', 8.0),
            ('avoid_speed_kmh', 5.0),
            ('lane_lost_speed_kmh', 0.0),
            ('min_lane_confidence', 0.12),
            ('lane_timeout_sec', 0.6),
            ('obstacle_timeout_sec', 0.6),
            ('emergency_stop_x_m', 2.5),
            ('emergency_corridor_half_width_m', 1.0),
            ('avoid_trigger_x_m', 8.0),
            ('obstacle_consider_abs_y_m', 2.2),
            ('collision_half_width_m', 0.9),
            ('candidate_offset_m', 1.0),
            ('avoid_error_offset_normalized', 0.32),
        ]:
            self.declare_parameter(name, default)

        self.lane_error = 0.0
        self.lane_confidence = 0.0
        self.lane_stamp_ns = 0
        self.obstacle = None
        self.obstacle_stamp_ns = 0
        self.previous_offset_m = 0.0

        self.create_subscription(
            Float32, self.get_parameter('lane_error_topic').value,
            self.lane_error_cb, 10)
        self.create_subscription(
            Float32, self.get_parameter('lane_confidence_topic').value,
            self.lane_conf_cb, 10)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('obstacle_topic').value,
            self.obstacle_cb, 10)

        self.target_error_pub = self.create_publisher(
            Float32, self.get_parameter('target_error_topic').value, 10)
        self.target_speed_pub = self.create_publisher(
            Float32, self.get_parameter('target_speed_topic').value, 10)
        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10)

        self.create_timer(0.05, self.plan)  # 20 Hz
        self.get_logger().info('Planning node ready')

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def lane_error_cb(self, msg: Float32):
        self.lane_error = float(msg.data)
        self.lane_stamp_ns = self.now_ns()

    def lane_conf_cb(self, msg: Float32):
        self.lane_confidence = float(msg.data)
        # Update freshness on both lane messages.
        self.lane_stamp_ns = self.now_ns()

    def obstacle_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self.obstacle = tuple(float(v) for v in msg.data[:5])
            self.obstacle_stamp_ns = self.now_ns()

    def plan(self):
        now = self.now_ns()
        lane_timeout = float(self.get_parameter('lane_timeout_sec').value)
        lane_age = (now - self.lane_stamp_ns) / 1e9 if self.lane_stamp_ns else float('inf')

        min_conf = float(self.get_parameter('min_lane_confidence').value)
        if lane_age > lane_timeout or self.lane_confidence < min_conf:
            self.publish_plan(
                'LANE_LOST',
                0.0,
                float(self.get_parameter('lane_lost_speed_kmh').value),
            )
            self.previous_offset_m = 0.0
            return

        obstacle = self.fresh_obstacle(now)
        if obstacle is None:
            self.previous_offset_m = 0.0
            self.publish_plan(
                'LANE_FOLLOW',
                self.lane_error,
                float(self.get_parameter('cruise_speed_kmh').value),
            )
            return

        ox, oy, _oz, _dist, _count = obstacle

        emergency_x = float(self.get_parameter('emergency_stop_x_m').value)
        emergency_y = float(self.get_parameter('emergency_corridor_half_width_m').value)
        if 0.0 < ox <= emergency_x and abs(oy) <= emergency_y:
            self.publish_plan('STOP', self.lane_error, 0.0)
            return

        trigger_x = float(self.get_parameter('avoid_trigger_x_m').value)
        consider_y = float(self.get_parameter('obstacle_consider_abs_y_m').value)

        if 0.0 < ox <= trigger_x and abs(oy) <= consider_y:
            selected_offset = self.select_candidate_offset(oy, ox)
            self.previous_offset_m = selected_offset

            offset_m = float(self.get_parameter('candidate_offset_m').value)
            error_offset = float(self.get_parameter('avoid_error_offset_normalized').value)

            # LiDAR +y is assumed LEFT.
            # Camera lane_error + means target is RIGHT in image.
            # Therefore left candidate -> negative image error offset.
            if abs(offset_m) > 1e-6:
                target_error = self.lane_error - (selected_offset / offset_m) * error_offset
            else:
                target_error = self.lane_error

            if selected_offset > 0.05:
                state = 'AVOID_LEFT'
            elif selected_offset < -0.05:
                state = 'AVOID_RIGHT'
            else:
                state = 'LANE_FOLLOW'

            self.publish_plan(
                state,
                clamp(target_error, -1.0, 1.0),
                float(self.get_parameter('avoid_speed_kmh').value),
            )
            return

        self.previous_offset_m = 0.0
        self.publish_plan(
            'LANE_FOLLOW',
            self.lane_error,
            float(self.get_parameter('cruise_speed_kmh').value),
        )

    def fresh_obstacle(self, now_ns):
        if self.obstacle is None or not self.obstacle_stamp_ns:
            return None

        timeout = float(self.get_parameter('obstacle_timeout_sec').value)
        age = (now_ns - self.obstacle_stamp_ns) / 1e9
        if age > timeout:
            return None

        x, y, z, dist, count = self.obstacle
        if count <= 0.0 or not math.isfinite(x) or not math.isfinite(y):
            return None
        return self.obstacle

    def select_candidate_offset(self, obstacle_y: float, obstacle_x: float) -> float:
        d = float(self.get_parameter('candidate_offset_m').value)
        collision_half = float(self.get_parameter('collision_half_width_m').value)
        candidates = [d, 0.0, -d]  # left, center, right

        best_offset = 0.0
        best_score = float('inf')

        for candidate in candidates:
            # Prefer lane center and smooth transitions.
            center_cost = 0.35 * abs(candidate)
            smooth_cost = 0.45 * abs(candidate - self.previous_offset_m)

            lateral_clearance = abs(obstacle_y - candidate)
            obstacle_cost = 0.0
            if lateral_clearance < collision_half:
                obstacle_cost = 100.0 + 20.0 * (collision_half - lateral_clearance)

            # A closer obstacle makes collision penalty even more important.
            proximity_scale = max(0.0, 1.0 - obstacle_x /
                                  max(0.1, float(self.get_parameter('avoid_trigger_x_m').value)))
            obstacle_cost *= (1.0 + proximity_scale)

            score = center_cost + smooth_cost + obstacle_cost
            if score < best_score:
                best_score = score
                best_offset = candidate

        return best_offset

    def publish_plan(self, state: str, target_error: float, target_speed_kmh: float):
        e = Float32()
        e.data = float(clamp(target_error, -1.0, 1.0))
        v = Float32()
        v.data = float(max(0.0, target_speed_kmh))
        s = String()
        s.data = state

        self.target_error_pub.publish(e)
        self.target_speed_pub.publish(v)
        self.state_pub.publish(s)


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
