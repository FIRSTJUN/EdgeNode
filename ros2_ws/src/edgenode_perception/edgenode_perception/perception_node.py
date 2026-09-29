import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


class PerceptionNode(Node):
    """
    Minimal perception development node.

    Current role:
      MORAI / rosbag camera
              ↓
      sensor_msgs/CompressedImage
              ↓
      OpenCV BGR image
              ↓
      future perception algorithm

    No lane detection, LiDAR processing, DBSCAN,
    semantic diagnostics, or control dependency is included.
    """

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter(
            'camera_topic',
            '/camera/image/compressed',
        )

        camera_topic = self.get_parameter(
            'camera_topic'
        ).value

        self.frame_count = 0

        self.camera_sub = self.create_subscription(
            CompressedImage,
            camera_topic,
            self.camera_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Perception development node ready: {camera_topic}'
        )

    def camera_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        frame = cv2.imdecode(
            np_arr,
            cv2.IMREAD_COLOR,
        )

        if frame is None:
            self.get_logger().warning(
                'Failed to decode compressed camera image'
            )
            return

        self.frame_count += 1

        # Temporary lightweight heartbeat.
        # Print only once every 100 frames so logging stays cheap.
        if self.frame_count % 100 == 0:
            height, width = frame.shape[:2]

            self.get_logger().info(
                f'Camera OK | '
                f'frames={self.frame_count} '
                f'size={width}x{height}'
            )

        # --------------------------------------------------------
        # Future perception algorithm starts here.
        #
        # Example:
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # bird = ...
        # lane detection = ...
        #
        # Keep this node minimal until an algorithm is validated.
        # --------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()