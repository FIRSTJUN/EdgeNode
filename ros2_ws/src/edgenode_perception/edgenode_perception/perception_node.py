import math
from typing import Optional, Tuple

import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Float32, Float32MultiArray


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PerceptionNode(Node):
    """
    Baseline perception:
      - Camera: white/yellow lane mask -> sliding windows -> polynomial fit
      - LiDAR: front ROI filter -> DBSCAN -> nearest obstacle cluster

    Published conventions:
      /perception/lane_error:
        normalized image error in [-1, 1]
        + : desired lane center is to the RIGHT of camera center
        - : desired lane center is to the LEFT of camera center

      /perception/obstacle Float32MultiArray:
        [x_m, y_m, z_m, distance_m, cluster_points]
        Uses LiDAR coordinates. Empty detection is [nan, nan, nan, inf, 0].
    """

    def __init__(self):
        super().__init__('perception_node')

        # Topics
        self.declare_parameter('camera_topic', '/camera/image/compressed')
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('lane_error_topic', '/perception/lane_error')
        self.declare_parameter('lane_confidence_topic', '/perception/lane_confidence')
        self.declare_parameter('obstacle_topic', '/perception/obstacle')
        self.declare_parameter('debug_image_topic', '/perception/debug/lane_image/compressed')

        # Lane parameters
        for name, default in [
            ('roi_top_y_ratio', 0.55),
            ('roi_bottom_y_ratio', 0.98),
            ('roi_top_left_x_ratio', 0.34),
            ('roi_top_right_x_ratio', 0.66),
            ('roi_bottom_left_x_ratio', 0.04),
            ('roi_bottom_right_x_ratio', 0.96),
            ('default_lane_width_px_640', 280.0),
            ('lookahead_y_ratio', 0.80),
        ]:
            self.declare_parameter(name, default)
        self.declare_parameter('sliding_windows', 9)
        self.declare_parameter('sliding_margin_px', 55)
        self.declare_parameter('sliding_minpix', 30)
        self.declare_parameter('min_lane_pixels', 180)

        # LiDAR parameters
        for name, default in [
            ('lidar_x_min_m', 0.5),
            ('lidar_x_max_m', 20.0),
            ('lidar_abs_y_max_m', 4.0),
            ('lidar_z_min_m', -1.5),
            ('lidar_z_max_m', 1.5),
            ('dbscan_eps_m', 0.65),
        ]:
            self.declare_parameter(name, default)
        self.declare_parameter('lidar_max_points', 4500)
        self.declare_parameter('dbscan_min_samples', 6)
        self.declare_parameter('min_cluster_points', 8)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            CompressedImage, self.camera_topic, self.camera_callback, sensor_qos)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, sensor_qos)

        self.lane_error_pub = self.create_publisher(
            Float32, self.get_parameter('lane_error_topic').value, 10)
        self.lane_conf_pub = self.create_publisher(
            Float32, self.get_parameter('lane_confidence_topic').value, 10)
        self.obstacle_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('obstacle_topic').value, 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, self.get_parameter('debug_image_topic').value, 3)

        self.get_logger().info(
            f'Perception ready: camera={self.camera_topic}, lidar={self.lidar_topic}')

    # ---------------- Camera / lane ----------------

    def camera_callback(self, msg: CompressedImage):
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.publish_lane(0.0, 0.0)
            self.get_logger().warning('Failed to decode compressed camera image')
            return

        mask = self.make_lane_mask(image)
        result = self.sliding_window_lane(mask)

        debug = image.copy()
        h, w = image.shape[:2]
        cv2.line(debug, (w // 2, 0), (w // 2, h - 1), (0, 0, 255), 2)

        if result is None:
            self.publish_lane(0.0, 0.0)
            cv2.putText(
                debug, 'LANE LOST', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            lane_error, confidence, lane_center_x, lookahead_y = result
            self.publish_lane(lane_error, confidence)
            cv2.circle(
                debug, (int(lane_center_x), int(lookahead_y)),
                8, (0, 255, 0), -1)
            cv2.line(
                debug, (w // 2, int(lookahead_y)),
                (int(lane_center_x), int(lookahead_y)),
                (255, 0, 255), 3)
            cv2.putText(
                debug,
                f'error={lane_error:+.3f} conf={confidence:.2f}',
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 0), 2)

        # Show ROI mask in the lower-left corner for quick tuning.
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        small = cv2.resize(mask_bgr, (w // 3, h // 3))
        sh, sw = small.shape[:2]
        debug[h - sh:h, 0:sw] = small

        ok, encoded = cv2.imencode('.jpg', debug, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = encoded.tobytes()
            self.debug_pub.publish(out)

    def publish_lane(self, error: float, confidence: float):
        e = Float32()
        e.data = float(clamp(error, -1.0, 1.0))
        c = Float32()
        c.data = float(clamp(confidence, 0.0, 1.0))
        self.lane_error_pub.publish(e)
        self.lane_conf_pub.publish(c)

    def make_lane_mask(self, image: np.ndarray) -> np.ndarray:
        h, w = image.shape[:2]

        # White lane mask in HLS.
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        white = cv2.inRange(
            hls,
            np.array([0, 175, 0], dtype=np.uint8),
            np.array([255, 255, 130], dtype=np.uint8),
        )

        # Yellow lane mask in HSV.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(
            hsv,
            np.array([12, 70, 70], dtype=np.uint8),
            np.array([42, 255, 255], dtype=np.uint8),
        )

        combined = cv2.bitwise_or(white, yellow)
        combined = cv2.GaussianBlur(combined, (5, 5), 0)
        _, combined = cv2.threshold(combined, 120, 255, cv2.THRESH_BINARY)

        top_y = int(h * float(self.get_parameter('roi_top_y_ratio').value))
        bot_y = int(h * float(self.get_parameter('roi_bottom_y_ratio').value))
        tl = int(w * float(self.get_parameter('roi_top_left_x_ratio').value))
        tr = int(w * float(self.get_parameter('roi_top_right_x_ratio').value))
        bl = int(w * float(self.get_parameter('roi_bottom_left_x_ratio').value))
        br = int(w * float(self.get_parameter('roi_bottom_right_x_ratio').value))

        roi = np.zeros_like(combined)
        polygon = np.array([[(bl, bot_y), (tl, top_y), (tr, top_y), (br, bot_y)]],
                           dtype=np.int32)
        cv2.fillPoly(roi, polygon, 255)
        return cv2.bitwise_and(combined, roi)

    def sliding_window_lane(
        self, binary: np.ndarray
    ) -> Optional[Tuple[float, float, float, float]]:
        h, w = binary.shape
        histogram = np.sum(binary[h // 2:, :] > 0, axis=0).astype(np.float32)

        midpoint = w // 2
        left_base = int(np.argmax(histogram[:midpoint])) if midpoint > 0 else 0
        right_base = int(np.argmax(histogram[midpoint:]) + midpoint) if midpoint < w else w - 1

        # If one side is nearly empty, keep it as unavailable.
        peak_threshold = max(5.0, h * 0.015)
        left_available = histogram[left_base] > peak_threshold
        right_available = histogram[right_base] > peak_threshold

        nonzero_y, nonzero_x = binary.nonzero()
        if len(nonzero_x) == 0:
            return None

        nwindows = int(self.get_parameter('sliding_windows').value)
        margin = int(self.get_parameter('sliding_margin_px').value)
        minpix = int(self.get_parameter('sliding_minpix').value)
        min_lane_pixels = int(self.get_parameter('min_lane_pixels').value)
        window_height = max(1, h // nwindows)

        left_current = left_base
        right_current = right_base
        left_inds = []
        right_inds = []

        for window in range(nwindows):
            y_low = h - (window + 1) * window_height
            y_high = h - window * window_height

            if left_available:
                good_left = (
                    (nonzero_y >= y_low) & (nonzero_y < y_high) &
                    (nonzero_x >= left_current - margin) &
                    (nonzero_x < left_current + margin)
                ).nonzero()[0]
                left_inds.append(good_left)
                if len(good_left) > minpix:
                    left_current = int(np.mean(nonzero_x[good_left]))

            if right_available:
                good_right = (
                    (nonzero_y >= y_low) & (nonzero_y < y_high) &
                    (nonzero_x >= right_current - margin) &
                    (nonzero_x < right_current + margin)
                ).nonzero()[0]
                right_inds.append(good_right)
                if len(good_right) > minpix:
                    right_current = int(np.mean(nonzero_x[good_right]))

        left_inds = np.concatenate(left_inds) if left_inds else np.array([], dtype=np.int64)
        right_inds = np.concatenate(right_inds) if right_inds else np.array([], dtype=np.int64)

        left_fit = None
        right_fit = None
        if left_available and len(left_inds) >= min_lane_pixels:
            left_fit = np.polyfit(nonzero_y[left_inds], nonzero_x[left_inds], 2)
        if right_available and len(right_inds) >= min_lane_pixels:
            right_fit = np.polyfit(nonzero_y[right_inds], nonzero_x[right_inds], 2)

        if left_fit is None and right_fit is None:
            return None

        lookahead_y = int(h * float(self.get_parameter('lookahead_y_ratio').value))
        lane_width = (
            float(self.get_parameter('default_lane_width_px_640').value) *
            (w / 640.0)
        )

        left_x = np.polyval(left_fit, lookahead_y) if left_fit is not None else None
        right_x = np.polyval(right_fit, lookahead_y) if right_fit is not None else None

        if left_x is not None and right_x is not None:
            if right_x <= left_x:
                return None
            lane_center = 0.5 * (left_x + right_x)
            count_score = min(1.0, (len(left_inds) + len(right_inds)) / 3000.0)
            width_score = math.exp(-abs((right_x - left_x) - lane_width) /
                                   max(1.0, lane_width))
            confidence = clamp(0.55 * count_score + 0.45 * width_score, 0.0, 1.0)
        elif left_x is not None:
            lane_center = left_x + lane_width * 0.5
            confidence = clamp(len(left_inds) / 1800.0, 0.0, 0.55)
        else:
            lane_center = right_x - lane_width * 0.5
            confidence = clamp(len(right_inds) / 1800.0, 0.0, 0.55)

        error = (lane_center - (w * 0.5)) / max(1.0, w * 0.5)
        return clamp(float(error), -1.0, 1.0), float(confidence), float(lane_center), float(lookahead_y)

    # ---------------- LiDAR / DBSCAN ----------------

    def lidar_callback(self, msg: PointCloud2):
        points = self.pointcloud_xyz(msg)
        if points is None or len(points) == 0:
            self.publish_no_obstacle()
            return

        x_min = float(self.get_parameter('lidar_x_min_m').value)
        x_max = float(self.get_parameter('lidar_x_max_m').value)
        y_max = float(self.get_parameter('lidar_abs_y_max_m').value)
        z_min = float(self.get_parameter('lidar_z_min_m').value)
        z_max = float(self.get_parameter('lidar_z_max_m').value)

        valid = (
            np.isfinite(points).all(axis=1) &
            (points[:, 0] >= x_min) &
            (points[:, 0] <= x_max) &
            (np.abs(points[:, 1]) <= y_max) &
            (points[:, 2] >= z_min) &
            (points[:, 2] <= z_max)
        )
        points = points[valid]

        if len(points) == 0:
            self.publish_no_obstacle()
            return

        max_points = int(self.get_parameter('lidar_max_points').value)
        if len(points) > max_points:
            stride = int(math.ceil(len(points) / max_points))
            points = points[::stride]

        eps = float(self.get_parameter('dbscan_eps_m').value)
        min_samples = int(self.get_parameter('dbscan_min_samples').value)
        min_cluster_points = int(self.get_parameter('min_cluster_points').value)

        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points[:, :2])

        best = None
        for label in np.unique(labels):
            if label < 0:
                continue
            cluster = points[labels == label]
            if len(cluster) < min_cluster_points:
                continue

            centroid = np.mean(cluster, axis=0)
            # Prefer the closest cluster in front of the vehicle.
            distance = float(math.hypot(float(centroid[0]), float(centroid[1])))
            if best is None or distance < best[3]:
                best = (
                    float(centroid[0]),
                    float(centroid[1]),
                    float(centroid[2]),
                    distance,
                    float(len(cluster)),
                )

        if best is None:
            self.publish_no_obstacle()
            return

        out = Float32MultiArray()
        out.data = list(best)
        self.obstacle_pub.publish(out)

    def publish_no_obstacle(self):
        out = Float32MultiArray()
        out.data = [float('nan'), float('nan'), float('nan'), float('inf'), 0.0]
        self.obstacle_pub.publish(out)

    @staticmethod
    def pointcloud_xyz(msg: PointCloud2) -> Optional[np.ndarray]:
        fields = {field.name: field for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(fields):
            return None

        count = int(msg.width) * int(msg.height)
        if count <= 0 or msg.point_step <= 0:
            return None

        endian = '>' if msg.is_bigendian else '<'

        def extract(name: str):
            field = fields[name]
            if field.datatype == PointField.FLOAT32:
                dtype = np.dtype(endian + 'f4')
            elif field.datatype == PointField.FLOAT64:
                dtype = np.dtype(endian + 'f8')
            else:
                raise ValueError(f'Unsupported datatype for {name}: {field.datatype}')
            return np.ndarray(
                shape=(count,),
                dtype=dtype,
                buffer=msg.data,
                offset=int(field.offset),
                strides=(int(msg.point_step),),
            ).astype(np.float32, copy=False)

        try:
            x = extract('x')
            y = extract('y')
            z = extract('z')
        except (ValueError, TypeError, BufferError) as exc:
            return None

        return np.column_stack((x, y, z))


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
