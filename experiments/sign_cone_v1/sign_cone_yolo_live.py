#!/usr/bin/env python3
"""Safe ROS 2 live preview for the EdgeNode three-class YOLO26n model.

Inputs:
  /camera/image/compressed (sensor_msgs/CompressedImage)
Outputs:
  /perception/sign_cone/detections (std_msgs/String, JSON)
  /perception/sign_cone/debug (sensor_msgs/Image, bgr8)

This preview DOES NOT send commands to planning or control. It also never
publishes /perception/school_zone/sign_detected.
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from ultralytics import YOLO


EXPECTED_CLASSES = ("sign_30", "sign_50", "traffic_cone")


class SignConeYoloLive(Node):
    def __init__(self):
        super().__init__("sign_cone_yolo_live")
        self.declare_parameter(
            "model_path", "/workspace/local_data/runs/sign_cone_v1/weights/best.pt"
        )
        self.declare_parameter("camera_topic", "/camera/image/compressed")
        self.declare_parameter("device", "0")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("max_fps", 5.0)

        model_path = Path(str(self.get_parameter("model_path").value))
        camera_topic = str(self.get_parameter("camera_topic").value)
        self.device = str(self.get_parameter("device").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.image_size = int(self.get_parameter("image_size").value)
        max_fps = float(self.get_parameter("max_fps").value)

        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")
        if max_fps <= 0:
            raise ValueError("max_fps must be greater than zero")

        self.model = YOLO(str(model_path))
        names = self.model.names
        actual_classes = tuple(names[index] for index in range(len(names)))
        if actual_classes != EXPECTED_CLASSES:
            raise ValueError(
                f"Incorrect YOLO classes {actual_classes}; expected {EXPECTED_CLASSES}"
            )

        self.min_interval_sec = 1.0 / max_fps
        self.last_run = 0.0
        self.processed_frames = 0
        self.detection_pub = self.create_publisher(
            String, "/perception/sign_cone/detections", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "/perception/sign_cone/debug", 10
        )
        self.camera_sub = self.create_subscription(
            CompressedImage, camera_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Sign/cone YOLO live preview ready | input={camera_topic} "
            f"| device={self.device} | classes={actual_classes} | max_fps={max_fps}"
        )

    def on_image(self, msg: CompressedImage):
        now = time.monotonic()
        if now - self.last_run < self.min_interval_sec:
            return
        self.last_run = now

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Could not decode camera image")
            return

        result = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )[0]

        detections = []
        for box in result.boxes:
            cls_id = int(box.cls.item())
            x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
            detections.append(
                {
                    "class_id": cls_id,
                    "class_name": result.names[cls_id],
                    "confidence": round(float(box.conf.item()), 4),
                    "bbox_xyxy_px": [round(value, 1) for value in (x1, y1, x2, y2)],
                }
            )

        height, width = frame.shape[:2]
        payload = {
            "timestamp_sec": msg.header.stamp.sec,
            "timestamp_nanosec": msg.header.stamp.nanosec,
            "image_width": width,
            "image_height": height,
            "detections": detections,
        }
        self.detection_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        annotated = np.ascontiguousarray(result.plot())
        debug = Image()
        debug.header = msg.header
        debug.height, debug.width = annotated.shape[:2]
        debug.encoding = "bgr8"
        debug.is_bigendian = 0
        debug.step = debug.width * 3
        debug.data = annotated.tobytes()
        self.debug_pub.publish(debug)

        self.processed_frames += 1
        if self.processed_frames % 50 == 0:
            self.get_logger().info(
                f"Processed {self.processed_frames} frames; "
                f"current frame detections={len(detections)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SignConeYoloLive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
