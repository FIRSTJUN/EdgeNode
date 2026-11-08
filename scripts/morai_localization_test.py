import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from morai_ros2_msgs.msg import EgoVehicleStatus, GPSMessage


class MoraiLocalization(Node):
    def __init__(self):
        super().__init__('morai_localization_test')

        self.east = None
        self.north = None

        self.pub = self.create_publisher(
            Odometry, '/localization/odometry', 10
        )

        self.create_subscription(
            GPSMessage, '/gps', self.gps_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            EgoVehicleStatus,
            '/ego_vehicle_status',
            self.ego_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('Waiting for MORAI GPS and Ego Status')

    def gps_callback(self, msg):
        self.east = msg.east_offset
        self.north = msg.north_offset

    def ego_callback(self, msg):
        if self.east is None or self.north is None:
            return

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = msg.position.x + self.east
        odom.pose.pose.position.y = msg.position.y + self.north
        odom.pose.pose.position.z = msg.position.z

        yaw = math.radians(msg.heading)

        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)

        self.pub.publish(odom)


rclpy.init()
node = MoraiLocalization()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
