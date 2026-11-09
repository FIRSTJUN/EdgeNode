import time
from pathlib import Path
from collections import deque

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from ultralytics import YOLO


class SchoolZoneYOLONode(Node):

    def __init__(self):
        super().__init__('school_zone_yolo_node')

        self.declare_parameter('model_path', '/workspace/local_data/models/school_zone/best.pt')
        self.declare_parameter('device', '0')
        model_path = Path(str(self.get_parameter('model_path').value)).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO weights not found: {model_path}')
        self.model = YOLO(str(model_path))
        self.device = str(self.get_parameter('device').value)

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

        # 최대 5FPS로 추론
        now = time.monotonic()

        if now - self.last_run < 0.2:
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
            device=self.device,
            verbose=False
        )[0]

        detected = len(result.boxes) > 0

        # 최근 5프레임 인식 결과 저장
        self.history.append(float(result.boxes.conf.max()) if detected else 0.0)

        if detected:
            self.missing_frames = 0
        else:
            self.missing_frames += 1

        # 최근 5프레임 중 3회 이상 탐지 + 최소 1회 confidence >= 0.80
        stable = (
            len(self.history) == 5
            and sum(conf >= 0.5 for conf in self.history) >= 3
            and max(self.history) >= 0.80
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


def main(args=None):
    rclpy.init(args=args)
    node = SchoolZoneYOLONode()

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
