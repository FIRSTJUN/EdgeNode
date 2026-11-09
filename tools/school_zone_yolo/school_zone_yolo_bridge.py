import time
from collections import deque

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from ultralytics import YOLO


class SchoolZoneYOLOBridge(Node):

    def __init__(self):
        super().__init__('school_zone_yolo_bridge')

        self.model = YOLO(
            '/workspace/local_data/models/school_zone/best.pt'
        )

        self.camera_sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.callback,
            qos_profile_sensor_data
        )

        self.sign_pub = self.create_publisher(
            Bool,
            '/perception/school_zone/sign_detected',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/perception/school_zone/yolo_debug',
            qos_profile_sensor_data
        )

        self.history = deque(maxlen=5)
        self.event_sent = False
        self.missing_frames = 0
        self.last_run = 0.0

        self.get_logger().info(
            'School Zone YOLO Bridge Ready!'
        )

    def callback(self, msg):

        # 최대 2FPS로 추론
        now = time.monotonic()

        if now - self.last_run < 0.5:
            return

        self.last_run = now

        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8),
            cv2.IMREAD_COLOR
        )

        if frame is None:
            return

        # YOLO 추론
        result = self.model.predict(
            frame,
            conf=0.5,
            imgsz=640,
            device=0,
            verbose=False
        )[0]

        detected = len(result.boxes) > 0

        # 최근 5프레임 인식 결과 저장
        self.history.append(detected)

        if detected:
            self.missing_frames = 0
        else:
            self.missing_frames += 1

        # 최근 5프레임 중 3회 이상 탐지 시 진입 이벤트
        stable = (
            len(self.history) == 5
            and sum(self.history) >= 3
        )

        if stable and not self.event_sent:

            self.sign_pub.publish(Bool(data=True))

            self.get_logger().info(
                'SCHOOL ZONE CONFIRMED -> True published!'
            )

            self.event_sent = True

        # 표지판이 충분히 오래 사라지면 재탐지 허용
        if self.missing_frames >= 10:
            self.event_sent = False
            self.history.clear()

        # YOLO 결과 이미지 발행
        annotated = result.plot()

        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = annotated.tobytes()

        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = SchoolZoneYOLOBridge()

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
