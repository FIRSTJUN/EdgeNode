"""Show live MORAI camera and lane debug images. Press Q or Esc to close."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


WINDOW = 'EdgeNode | MORAI camera + lane perception | Q: close'


class LaneViewer(Node):
    def __init__(self):
        super().__init__('lane_debug_viewer')
        self.frames = {}
        self.received = {}
        self.create_subscription(
            CompressedImage, '/camera/image/compressed',
            self.camera_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, '/perception/debug/lane_image',
            self.debug_callback, qos_profile_sensor_data,
        )
        self.get_logger().info('Viewer ready. Press Q or Esc to close.')

    def camera_callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            if 'camera' not in self.frames:
                self.get_logger().info('Receiving live MORAI camera images')
            self.frames['camera'] = frame
            self.received['camera'] = time.monotonic()

    def debug_callback(self, msg):
        if msg.encoding != 'bgr8':
            self.get_logger().warning('Expected bgr8 lane image', throttle_duration_sec=5.0)
            return
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        if 'lanes' not in self.frames:
            self.get_logger().info('Receiving live lane perception images')
        self.frames['lanes'] = data[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        self.received['lanes'] = time.monotonic()

    def draw(self):
        canvas = np.zeros((510, 1280, 3), dtype=np.uint8)
        now = time.monotonic()
        for index, (key, label) in enumerate((('camera', 'MORAI camera'), ('lanes', 'Lane perception'))):
            offset = index * 640
            frame = self.frames.get(key)
            if frame is not None:
                scale = min(640 / frame.shape[1], 480 / frame.shape[0])
                width, height = round(frame.shape[1] * scale), round(frame.shape[0] * scale)
                resized = cv2.resize(frame, (width, height))
                canvas[30:30 + height, offset:offset + width] = resized
                age = now - self.received[key]
                if age > 1.0:
                    label += f' | STALE: {age:.1f}s'
            else:
                label += ' | waiting for images'
            cv2.putText(canvas, label, (offset + 12, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, canvas)


def main():
    rclpy.init()
    node = LaneViewer()
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 510)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.draw()
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                break
            # GTK does not implement WND_PROP_VISIBLE; AUTOSIZE is supported.
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0:
                    break
            except cv2.error:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
