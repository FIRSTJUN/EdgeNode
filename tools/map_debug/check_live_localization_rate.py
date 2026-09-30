import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


DURATION_SEC = 10.0


class RateChecker(Node):

    def __init__(self):
        super().__init__('live_localization_rate_checker')

        self.imu_count = 0
        self.odom_count = 0
        self.gps_speed_count = 0

        self.imu_header_stamps = set()
        self.odom_header_stamps = set()

        self.create_subscription(
            Imu,
            '/Imu',
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Float32,
            '/localization/gps_speed',
            self.gps_speed_callback,
            qos_profile_sensor_data,
        )

    @staticmethod
    def stamp_ns(msg):
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def imu_callback(self, msg):
        self.imu_count += 1
        self.imu_header_stamps.add(self.stamp_ns(msg))

    def odom_callback(self, msg):
        self.odom_count += 1
        self.odom_header_stamps.add(self.stamp_ns(msg))

    def gps_speed_callback(self, msg):
        self.gps_speed_count += 1


def main():
    rclpy.init()

    node = RateChecker()
    start = time.monotonic()

    while rclpy.ok() and time.monotonic() - start < DURATION_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)

    elapsed = time.monotonic() - start

    print()
    print('===== LIVE LOCALIZATION RATE CHECK =====')
    print(f'Duration              : {elapsed:.3f} s')

    print()
    print(f'IMU received           : {node.imu_count}')
    print(f'IMU receive rate       : {node.imu_count / elapsed:.2f} Hz')
    print(f'IMU unique timestamps  : {len(node.imu_header_stamps)}')
    print(
        f'IMU unique stamp rate  : '
        f'{len(node.imu_header_stamps) / elapsed:.2f} Hz'
    )

    print()
    print(f'Odom received          : {node.odom_count}')
    print(f'Odom receive rate      : {node.odom_count / elapsed:.2f} Hz')
    print(f'Odom unique timestamps : {len(node.odom_header_stamps)}')
    print(
        f'Odom unique stamp rate : '
        f'{len(node.odom_header_stamps) / elapsed:.2f} Hz'
    )

    print()
    print(f'GPS speed received     : {node.gps_speed_count}')
    print(f'GPS speed rate         : {node.gps_speed_count / elapsed:.2f} Hz')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
