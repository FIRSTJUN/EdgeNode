"""Display the detector's annotated compressed stream using the existing OpenCV."""
import os
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class TrafficLightDebugViewer(Node):
    TITLE = 'Traffic Light Detection'

    def __init__(self):
        super().__init__('traffic_light_debug_viewer')
        self.declare_parameter('image_topic', '/perception/traffic_light/debug/compressed')
        self.declare_parameter('latest_camera', False)
        self.declare_parameter('camera_topic', '/camera/image/compressed')
        # Keep the previous spelling as a startup-compatible alias.
        self.declare_parameter('prediction_max_age_sec', 1.5)
        self.declare_parameter('max_prediction_age_sec',
                               self.get_parameter('prediction_max_age_sec').value)
        max_age = self.get_parameter('max_prediction_age_sec').value
        if not np.isfinite(max_age) or max_age <= 0:
            raise ValueError('max_prediction_age_sec must be positive and finite')
        self.live_camera = self.get_parameter('latest_camera').value
        self.predictions = None
        self.prediction_received = 0.0
        self.latest_header = None
        self.latest = None
        self.running = True
        self.displayed_frames = 0
        self.frame = None
        self.window_created = False
        self.window_confirmed = False
        self.closed_since = None
        self.subscription = self.create_subscription(
            CompressedImage, self.get_parameter('camera_topic' if self.live_camera else 'image_topic').value,
            self._receive, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.live_camera:
            self.prediction_sub = self.create_subscription(
                String, '/perception/traffic_light/detections', self._predictions, 1)
        # Keep GUI calls on the executor thread, including during camera silence.
        self.timer = self.create_timer(1.0 / 30.0, self._display)
        self.get_logger().info('Viewer ready; ESC/q or window close to exit')

    def _receive(self, msg):
        self.latest = msg  # Drop older frames rather than accumulating latency.

    def _predictions(self, msg):
        try:
            payload = json.loads(msg.data)
            age = float(payload.get('prediction_age_sec', float('inf')))
            if not np.isfinite(age) or age < 0:
                self.predictions = None
                return
            if not isinstance(payload.get('predictions'), list):
                raise ValueError('predictions must be a list')
            payload['prediction_age_sec'] = age
            self.predictions = payload
            self.prediction_received = time.monotonic()
        except (ValueError, TypeError) as exc:
            self.predictions = None
            self.get_logger().warning(f'Prediction decode exception: {exc}',
                                      throttle_duration_sec=5.0)

    def _overlay(self):
        frame = self.frame.copy()
        p = self.predictions
        if not p:
            return frame
        age = p['prediction_age_sec'] + time.monotonic() - self.prediction_received
        max_age = self.get_parameter('max_prediction_age_sec').value
        if age > max_age:
            if p['predictions']:
                self.get_logger().info(
                    f'prediction hidden: age={age:.2f}s > max_age={max_age:.2f}s',
                    throttle_duration_sec=2.0)
            return frame
        if (self.latest_header and p.get('frame_id') and self.latest_header.frame_id and
                p['frame_id'] != self.latest_header.frame_id):
            return frame
        if (p.get('image_width') != frame.shape[1] or p.get('image_height') != frame.shape[0]):
            return frame
        # Prediction boxes are from an older frame; no motion compensation is implied.
        colors = {'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0)}
        for d in p['predictions']:
            try:
                x, y, w, h = [float(d[k]) for k in ('x', 'y', 'w', 'h')]
                if d['label'] not in colors or not all(np.isfinite([x, y, w, h])):
                    continue
                x1 = max(0, min(frame.shape[1]-1, round(x-w/2)))
                y1 = max(0, min(frame.shape[0]-1, round(y-h/2)))
                x2 = max(0, min(frame.shape[1]-1, round(x+w/2)))
                y2 = max(0, min(frame.shape[0]-1, round(y+h/2)))
                color = colors[d['label']]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{d['label']} {float(d['confidence']):.2f}",
                            (x1, max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
            except (KeyError, ValueError, TypeError):
                continue
        cv2.putText(frame, f'prediction age {age*1000:.0f}ms', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
        return frame

    def _display(self):
        # GTK3 returns -1 for WND_PROP_VISIBLE even for a healthy window.
        # AUTOSIZE is supported here. Arm closure detection only after observing
        # a valid property, and debounce disappearance before accepting a close.
        if self.window_created:
            try:
                prop = cv2.getWindowProperty(self.TITLE, cv2.WND_PROP_AUTOSIZE)
            except cv2.error as exc:
                self.get_logger().warning(f'Window property exception: {exc}',
                                          throttle_duration_sec=5.0)
                prop = -1
            if prop >= 0:
                self.window_confirmed = True
                self.closed_since = None
            elif self.window_confirmed:
                if self.closed_since is None:
                    self.closed_since = time.monotonic()
                    self.get_logger().info('Window disappeared; confirming close event')
                if time.monotonic() - self.closed_since >= 0.3:
                    self.get_logger().info('Viewer exit: confirmed window close')
                    self.running = False
                return  # Do not recreate a window the user just closed.
        msg, self.latest = self.latest, None
        if msg is not None:
            try:
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('JPEG decode returned no image')
                self.frame = frame
                self.latest_header = msg.header
            except (cv2.error, ValueError) as exc:
                self.get_logger().warning(f'Compressed image decode exception: {exc}',
                                          throttle_duration_sec=5.0)
        if self.frame is None:
            return  # Wait indefinitely for the first frame, without a blank window.
        try:
            if not self.window_created:
                cv2.namedWindow(self.TITLE, cv2.WINDOW_AUTOSIZE)
                self.window_created = True
            cv2.imshow(self.TITLE, self._overlay() if self.live_camera else self.frame)
            if msg is not None:
                self.displayed_frames += 1
                if self.displayed_frames == 1:
                    self.get_logger().info(
                        f'Live image displayed: {self.frame.shape[1]}x{self.frame.shape[0]} '
                        f'mean={self.frame.mean():.2f} min={self.frame.min()} '
                        f'max={self.frame.max()}')
            raw_key = cv2.waitKey(1)
            if raw_key >= 0 and (raw_key & 0xff) in (27, ord('q'), ord('Q')):
                self.get_logger().info(f'Viewer exit: actual key event code={raw_key}')
                self.running = False
        except cv2.error as exc:
            self.get_logger().error(f'OpenCV display exception: {exc}')
            raise

    def destroy_node(self):
        self.timer.cancel()
        try:
            cv2.destroyAllWindows()
        except cv2.error as exc:
            self.get_logger().warning(f'Window cleanup exception: {exc}')
        return super().destroy_node()


def main(args=None):
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('Viewer needs a graphical session: set DISPLAY and expose the display socket.')
        return 1
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficLightDebugViewer()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().info('Viewer exit: interrupt / ROS shutdown')
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'Viewer exception before exit: {type(exc).__name__}: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
