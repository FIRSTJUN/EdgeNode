import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String


class PlanningNode(Node):
    """
    Planning skeleton for the new map-based architecture.

    Current stage:
      - Perception is intentionally disconnected.
      - MGeo/localization integration is not implemented yet.
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

        # Keep the vehicle stopped until map/localization is ready.
        self.declare_parameter(
            'cruise_speed_kmh',
            0.0,
        )

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

        # Planning loop: 20 Hz
        self.create_timer(
            0.05,
            self.plan,
        )

        self.get_logger().info(
            'Planning node ready - waiting for MGeo/localization integration'
        )

    def plan(self):
        """
        Temporary safe planning output.

        This will later be replaced by:
          EKF pose
              +
          MGeo path
              ↓
          path tracking error
        """

        target_error = Float32()
        target_error.data = 0.0

        target_speed = Float32()
        target_speed.data = 0.0

        state = String()
        state.data = 'WAIT_FOR_MAP'

        self.target_error_pub.publish(target_error)
        self.target_speed_pub.publish(target_speed)
        self.state_pub.publish(state)


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