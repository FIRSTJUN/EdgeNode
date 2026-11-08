"""Camera-only red/gray road prototype for MORAI K-City.

This is an experimental observation node. It does NOT send control commands,
change vehicle speed, or establish a legal school-zone boundary.
"""

from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, String


def classify_road(frame):
    """Return raw label, red fraction, gray fraction, and ROI rectangle."""
    h, w = frame.shape[:2]
    # Look at the center of the road ahead; avoid the sky, sign, and car hood.
    x1, x2 = int(w * 0.30), int(w * 0.70)
    y1, y2 = int(h * 0.46), int(h * 0.60)
    if x2 <= x1 or y2 <= y1:
        return 'UNKNOWN', 0.0, 0.0, (x1, y1, x2, y2)

    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red/brownish-red pavement. OpenCV Hue range is 0..179.
    red_low = cv2.inRange(hsv, (0, 70, 35), (18, 255, 255))
    red_high = cv2.inRange(hsv, (170, 70, 35), (179, 255, 255))
    red_mask = cv2.bitwise_or(red_low, red_high)
    # Neutral, reasonably illuminated pavement; these are initial values.
    gray_mask = cv2.inRange(hsv, (0, 0, 45), (179, 55, 240))

    red_ratio = float(np.count_nonzero(red_mask)) / red_mask.size
    gray_ratio = float(np.count_nonzero(gray_mask)) / gray_mask.size

    if red_ratio >= 0.25:
        raw_label = 'RED'
    elif red_ratio <= 0.02 and gray_ratio >= 0.35:
        raw_label = 'GRAY'
    else:
        raw_label = 'UNKNOWN'
    return raw_label, red_ratio, gray_ratio, (x1, y1, x2, y2)


class SchoolZoneRoadNode(Node):
    def __init__(self):
        super().__init__('school_zone_road_node')
        self.declare_parameter('camera_topic', '/camera/image/compressed')
        topic = self.get_parameter('camera_topic').value
        self.sub = self.create_subscription(
            CompressedImage, topic, self.on_image, qos_profile_sensor_data
        )
        self.state_pub = self.create_publisher(
            String, '/perception/road_color/state', 10
        )
        self.red_pub = self.create_publisher(
            Float32, '/perception/road_color/red_ratio', 10
        )
        self.debug_pub = self.create_publisher(
            Image, '/perception/road_color/debug', qos_profile_sensor_data
        )
        self.history = deque(maxlen=7)
        self.frames = 0
        self.last_state = None
        self.get_logger().info(f'Road color detector listening: {topic}')

    def on_image(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().warning('Camera image decoding failed')
            return

        raw_label, red_ratio, gray_ratio, (x1, y1, x2, y2) = classify_road(frame)
        self.history.append(raw_label)
        counts = Counter(self.history)
        # Stable decision only after at least five of the latest seven agree.
        state = ('UNKNOWN' if len(self.history) < 7 else
                 next((label for label, count in counts.items()
                       if count >= 5 and label != 'UNKNOWN'), 'UNKNOWN'))

        self.state_pub.publish(String(data=state))
        self.red_pub.publish(Float32(data=red_ratio))
        if state != self.last_state:
            self.get_logger().info(
                f'Road: {state} (raw={raw_label}, red={red_ratio:.2f}, gray={gray_ratio:.2f})'
            )
            self.last_state = state

        self.frames += 1
        if self.frames % 5 != 0:
            return

        # Draw the exact region being classified; this is a visualization only.
        annotated = frame.copy()
        color = {'RED': (0, 0, 255), 'GRAY': (150, 150, 150)}.get(state, (0, 255, 255))
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            annotated, f'{state}  red:{red_ratio:.2f} gray:{gray_ratio:.2f}',
            (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA
        )
        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = int(out.width * 3)
        out.data = annotated.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SchoolZoneRoadNode()
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
