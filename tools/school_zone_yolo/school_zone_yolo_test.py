import time
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO


class SchoolZoneYOLOTest(Node):

    def __init__(self):
        super().__init__('school_zone_yolo_test')

        # 학습한 어린이보호구역 YOLO 모델
        self.model = YOLO(
            '/workspace/local_data/models/school_zone/best.pt'
        )

        # MORAI 카메라 영상 구독
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.callback,
            qos_profile_sensor_data
        )

        # YOLO 탐지 결과 이미지 발행
        self.pub = self.create_publisher(
            Image,
            '/perception/school_zone/yolo_debug',
            qos_profile_sensor_data
        )

        self.last_run = 0.0
        self.frames = 0

        self.get_logger().info(
            'School Zone YOLO ready. Waiting for camera...'
        )

    def callback(self, msg):

        # GPU 부담을 줄이기 위해 초당 최대 2회 추론
        now = time.monotonic()
        if now - self.last_run < 0.5:
            return
        self.last_run = now

        # 압축된 카메라 영상 복원
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8),
            cv2.IMREAD_COLOR
        )

        if frame is None:
            self.get_logger().warning('Camera decode failed')
            return

        # YOLO 추론
        result = self.model.predict(
            source=frame,
            conf=0.4,
            imgsz=640,
            device=0,
            verbose=False
        )[0]

        boxes = result.boxes
        count = len(boxes)

        self.frames += 1

        if count > 0:
            confidence = float(boxes.conf.max())

            self.get_logger().info(
                f'SCHOOL ZONE DETECTED! '
                f'count={count}, confidence={confidence:.2f}'
            )

        elif self.frames % 10 == 0:
            self.get_logger().info(
                'Camera OK / No school zone detected'
            )

        # 표지판에 탐지 박스 그리기
        annotated = result.plot()

        # ROS 2 이미지 메시지로 변환
        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = annotated.tobytes()

        self.pub.publish(out)


def main():
    rclpy.init()

    node = SchoolZoneYOLOTest()

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
