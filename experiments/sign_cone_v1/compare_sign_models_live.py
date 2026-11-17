#!/usr/bin/env python3
"""Compare original sign model and combined YOLO model on the SAME ROS frame.

Read-only: subscribes to MORAI camera and prints predictions to terminal.
Does not publish ROS topics and does not command the vehicle.
"""

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


DATA_ROOT = Path("/workspace/local_data/datasets/edgenode_sign_cone_v1")
ORIGINAL_MODEL = DATA_ROOT / "pretrained/sign_30_50_original.pt"
COMBINED_MODEL = Path("/workspace/local_data/runs/sign_cone_v1/weights/best.pt")


class CompareSignModels(Node):
    def __init__(self):
        super().__init__("compare_sign_models_live")
        for path in (ORIGINAL_MODEL, COMBINED_MODEL):
            if not path.is_file():
                raise FileNotFoundError(f"Cannot find model: {path}")
        self.original = YOLO(str(ORIGINAL_MODEL))
        self.combined = YOLO(str(COMBINED_MODEL))
        self.last_sample = 0.0
        self.camera_sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.on_camera,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "READY: Comparing original sign model vs combined model every 2 seconds. "
            "Place the 50 km/h sign on the RIGHT in the camera view."
        )

    @staticmethod
    def predict_signs(model, image):
        result = model.predict(
            image, device="0", conf=0.20, imgsz=640, verbose=False
        )[0]
        signs = []
        for box in result.boxes:
            class_id = int(box.cls.item())
            class_name = result.names[class_id]
            if class_name not in ("sign_30", "sign_50"):
                continue
            confidence = float(box.conf.item())
            x1, _, x2, _ = (float(v) for v in box.xyxy[0].tolist())
            signs.append((confidence, class_name, (x1 + x2) / 2))
        signs.sort(reverse=True)
        if not signs:
            return "NONE (no sign above 0.20 confidence)"
        return ", ".join(
            f"{label} {confidence:.2f} at x={center_x:.0f}px"
            for confidence, label, center_x in signs[:4]
        )

    def on_camera(self, msg):
        now = time.monotonic()
        if now - self.last_sample < 2.0:
            return
        self.last_sample = now
        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            self.get_logger().warning("Camera image could not be decoded")
            return
        original = self.predict_signs(self.original, image)
        combined = self.predict_signs(self.combined, image)
        self.get_logger().info(f"ORIGINAL: {original}")
        self.get_logger().info(f"COMBINED: {combined}")
        self.get_logger().info("--")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CompareSignModels()
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
