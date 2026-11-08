"""Detect a RED -> sustained GRAY pavement transition from a ROS 2 color label.

Observation only: does NOT decide a legal school-zone boundary or control speed.
The school-zone manager must gate this event on a validated entry sign and
other applicable road/map evidence before lifting any speed limit.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RoadTransitionDetector(Node):
    def __init__(self):
        super().__init__('road_transition_detector')
        self.declare_parameter('color_topic', '/perception/road_color/state')
        self.declare_parameter('gray_confirm_sec', 1.5)
        self.declare_parameter('red_confirm_messages', 3)
        self.declare_parameter('max_message_gap_sec', 0.7)

        self.gray_confirm_sec = float(self.get_parameter('gray_confirm_sec').value)
        self.red_confirm_messages = int(self.get_parameter('red_confirm_messages').value)
        self.max_message_gap_sec = float(self.get_parameter('max_message_gap_sec').value)
        if self.gray_confirm_sec <= 0 or self.red_confirm_messages < 1 or self.max_message_gap_sec <= 0:
            raise ValueError('All detector thresholds must be positive')

        self.red_count = 0
        self.armed = False
        self.gray_since = None
        self.last_received_at = None
        self.status = 'WAITING_FOR_RED'

        self.create_subscription(
            String, self.get_parameter('color_topic').value, self.on_color, 10
        )
        self.status_pub = self.create_publisher(
            String, '/perception/road_transition/state', 10
        )
        self.event_pub = self.create_publisher(
            String, '/perception/road_transition/event', 10
        )
        self.get_logger().info('Road transition detector ready (observation only)')

    def on_color(self, msg):
        now = time.monotonic()
        if self.last_received_at is not None and now - self.last_received_at > self.max_message_gap_sec:
            # Stale images must not satisfy a continuous-gray condition.
            self.gray_since = None
            self.red_count = 0
        self.last_received_at = now
        label = msg.data.strip().upper()

        if label == 'RED':
            self.gray_since = None
            self.red_count += 1
            if self.red_count >= self.red_confirm_messages:
                self.armed = True
                self.status = 'RED_CONFIRMED'
        elif label == 'GRAY':
            self.red_count = 0
            if self.armed:
                if self.gray_since is None:
                    self.gray_since = now
                self.status = 'GRAY_CONFIRMING'
                if now - self.gray_since >= self.gray_confirm_sec:
                    self.armed = False
                    self.gray_since = None
                    self.status = 'EXIT_CANDIDATE'
                    self.event_pub.publish(String(data='RED_TO_GRAY'))
                    self.get_logger().info('RED_TO_GRAY candidate observed; not a legal zone exit')
        else:
            # UNKNOWN interrupts continuous gray; retain previously confirmed RED.
            self.red_count = 0
            self.gray_since = None
            if self.armed:
                self.status = 'RED_CONFIRMED'

        self.status_pub.publish(String(data=self.status))


def main(args=None):
    rclpy.init(args=args)
    node = RoadTransitionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
