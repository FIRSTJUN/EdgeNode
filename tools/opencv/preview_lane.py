#!/usr/bin/env python3
"""Preview lane debug images; standard ROS --ros-args remaps are supported."""

import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_array(message, encoding):
    """Decode a ROS Image, including row padding, without cv_bridge."""
    if message.encoding != encoding:
        raise ValueError(f'expected {encoding}, received {message.encoding!r}')
    channels = {'bgr8': 3, 'mono8': 1}[encoding]
    height, width, step = message.height, message.width, message.step
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError(f'invalid dimensions/step: {width}x{height}, step={step}')
    if len(message.data) != height * step:
        raise ValueError('image data length does not match height * step')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :width * channels]
    shape = (height, width, channels) if channels == 3 else (height, width)
    return np.ascontiguousarray(pixels.reshape(shape))


class LanePreview(Node):
    def __init__(self):
        super().__init__('lane_opencv_preview')
        self.frames = {}
        self.windows = ('Lane overlay', 'Lane mask')
        for window, topic, encoding in (
            (self.windows[0], '/perception/debug/lane_image', 'bgr8'),
            (self.windows[1], '/perception/debug/lane_mask', 'mono8'),
        ):
            self.create_subscription(
                Image, topic,
                lambda message, name=window, fmt=encoding:
                    self.receive(message, name, fmt),
                qos_profile_sensor_data,
            )
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 800, 450)
            waiting = np.zeros((450, 800, 3), dtype=np.uint8)
            cv2.putText(waiting, 'Waiting for images... Q / ESC to close',
                        (30, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (220, 220, 220), 1, cv2.LINE_AA)
            cv2.imshow(window, waiting)
        self.get_logger().info('Waiting for lane debug images. Q / ESC closes the preview.')

    def receive(self, message, window, encoding):
        try:
            self.frames[window] = image_array(message, encoding)
        except ValueError as error:
            self.get_logger().warning(f'{window}: {error}', throttle_duration_sec=5.0)

    def display(self):
        # Poll GUI events even while ROS receives no images.
        if cv2.waitKey(1) & 0xFF in (27, ord('q'), ord('Q')):
            return False
        try:
            # GTK reports -1 when this property is unsupported.
            if any(cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) == 0
                   for name in self.windows):
                return False
        except cv2.error:
            return False
        for window, frame in self.frames.items():
            cv2.imshow(window, frame)
        return True


def main(args=None):
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise SystemExit('No GUI display is available; set DISPLAY to your desktop display.')
    rclpy.init(args=args)
    node = None
    try:
        node = LanePreview()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if not node.display():
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
