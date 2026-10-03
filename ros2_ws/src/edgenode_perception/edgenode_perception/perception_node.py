import math
import time
from typing import Optional, Tuple

import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Float32, Float32MultiArray
from edgenode_perception.lidar_geometry import pointcloud_xyz, corridor_points


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PerceptionNode(Node):
    """
    Baseline perception:
      - Camera: grayscale brightness/gradient lane mask -> sliding windows -> polynomial fit
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
            ('roi_top_left_x_ratio', 0.0),
            ('roi_top_right_x_ratio', 1.0),
            ('roi_bottom_left_x_ratio', 0.0),
            ('roi_bottom_right_x_ratio', 1.0),
            ('default_lane_width_px_640', 280.0),
            ('lookahead_y_ratio', 0.60),
            ('clahe_clip_limit', 2.0),
            ('min_lane_span_ratio', 0.10),
            ('geometry_tolerance_ratio', 0.35),
            ('max_center_jump_ratio', 0.15),
            ('lane_min_contrast', 45.0),
            ('lane_min_paint_fraction', 0.6),
            ('lane_max_run_width_ratio', 0.11),
            ('lane_max_candidates', 12),
            ('lane_width_ema_alpha', 0.2),
            ('lane_width_max_age_frames', 30),
            ('lane_width_update_confidence', 0.7),
            ('lane_side_memory_frames', 90),
            ('camera_rate_hz', 15.0),
            ('lidar_corridor_half_width_m', 1.05),
        ]:
            self.declare_parameter(name, default)
        self.declare_parameter('sliding_windows', 9)
        self.declare_parameter('sliding_margin_px', 55)
        self.declare_parameter('sliding_minpix', 30)
        self.declare_parameter('min_lane_pixels', 180)

        for name, default in [('gray_threshold', 160), ('sobel_threshold', 40),
                              ('blur_kernel', 3), ('morph_kernel', 3)]:
            self.declare_parameter(name, default)
        self.curvature = 0.0
        self.curvature_stamp = 0.0
        self.last_camera = 0.0
        cv2.setNumThreads(1)
        self.previous_lane_center = None
        self.lane_debug = {}

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

        self.create_subscription(Float32, '/planning/curvature', self.curvature_cb, 10)

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

    def curvature_cb(self, msg):
        if math.isfinite(msg.data):
            self.curvature = float(msg.data)
            self.curvature_stamp = time.monotonic()

    def camera_callback(self, msg: CompressedImage):
        now = time.monotonic()
        if now-self.last_camera < 1/max(1., float(self.get_parameter('camera_rate_hz').value)):
            return
        self.last_camera = now
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.publish_lane(0.0, 0.0)
            self.get_logger().warning('Failed to decode compressed camera image')
            return

        mask = self.make_lane_mask(image)
        result = self.sliding_window_lane(mask)

        from edgenode_perception.lane_tracker import draw_lane_debug
        if result is None:
            self.publish_lane(0.0, 0.0)
        else:
            self.publish_lane(result[0], result[1])
        debug = draw_lane_debug(image, mask, self.roi_polygon,
                                self.lane_debug, result)

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
        self._lane_source_image = image
        h, w = image.shape[:2]

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.createCLAHE(
            clipLimit=max(0.01, float(self.get_parameter('clahe_clip_limit').value)),
            tileGridSize=(8, 8)).apply(gray)
        # Odd, positive kernels are required by OpenCV.
        blur = max(1, int(self.get_parameter('blur_kernel').value)) | 1
        enhanced = cv2.GaussianBlur(enhanced, (blur, blur), 0)
        threshold = float(self.get_parameter('gray_threshold').value)
        bright = enhanced >= threshold
        gradient = np.abs(cv2.Sobel(enhanced, cv2.CV_32F, 1, 0, ksize=3))
        # Edges must be adjacent to a bright candidate: road texture alone is not a lane.
        near_bright = cv2.dilate(bright.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        edges = (gradient >= float(self.get_parameter('sobel_threshold').value)) & near_bright
        combined = ((bright | edges).astype(np.uint8) * 255)
        size = max(1, int(self.get_parameter('morph_kernel').value)) | 1
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, np.ones((size, size), np.uint8))

        top_y = int(h * float(self.get_parameter('roi_top_y_ratio').value))
        bot_y = int(h * float(self.get_parameter('roi_bottom_y_ratio').value))
        tl = int(w * float(self.get_parameter('roi_top_left_x_ratio').value))
        tr = int(w * float(self.get_parameter('roi_top_right_x_ratio').value))
        bl = int(w * float(self.get_parameter('roi_bottom_left_x_ratio').value))
        br = int(w * float(self.get_parameter('roi_bottom_right_x_ratio').value))

        roi = np.zeros_like(combined)
        polygon = np.array([[(bl, bot_y), (tl, top_y), (tr, top_y), (br, bot_y)]],
                           dtype=np.int32)
        self.roi_polygon = polygon
        cv2.fillPoly(roi, polygon, 255)
        return cv2.bitwise_and(combined, roi)

    def sliding_window_lane(self, binary):
        from edgenode_perception.lane_tracker import LaneTracker
        if not hasattr(self, '_lane_tracker'):
            self._lane_tracker = LaneTracker()
        result = self._lane_tracker.detect(
            binary, lambda name: self.get_parameter(name).value,
            getattr(self, 'previous_lane_center', None),
            getattr(self, '_lane_source_image', None))
        self.lane_debug = self._lane_tracker.debug
        self.previous_lane_center = None if result is None else result[2]
        return result

    # ---------------- LiDAR / DBSCAN ----------------

    def lidar_callback(self, msg: PointCloud2):
        points = self.pointcloud_xyz(msg)
        if points is None or len(points) == 0:
            self.publish_invalid_obstacle()
            return

        x_min = float(self.get_parameter('lidar_x_min_m').value)
        x_max = float(self.get_parameter('lidar_x_max_m').value)
        z_min = float(self.get_parameter('lidar_z_min_m').value)
        z_max = float(self.get_parameter('lidar_z_max_m').value)

        if not np.isfinite(points).all(axis=1).any():
            self.publish_invalid_obstacle()
            return
        # The MORAI cloud header says /map, but captured coordinates are local
        # sensor coordinates. Ground was measured at z=-0.56m in this scene.
        curvature = self.curvature if time.monotonic()-self.curvature_stamp < .6 else 0.0
        points = corridor_points(points, curvature,
            float(self.get_parameter('lidar_corridor_half_width_m').value),
            x_min=x_min, x_max=x_max, z_min=z_min, z_max=z_max)
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
            distance = float(np.min(np.linalg.norm(cluster[:, :2], axis=1)))
            if best is None or distance < best[3]:
                best = (
                    float(centroid[0]),
                    float(centroid[1]),
                    float(centroid[2]),
                    distance,
                    float(len(cluster)),
                )

        # Dense nearby returns cannot be declared clear just because DBSCAN
        # failed to form a cluster (e.g. sparse pole / thin wall).
        near = points[np.linalg.norm(points[:, :2], axis=1) < 3.5]
        if len(near) >= 3:
            nearest = near[np.argmin(np.linalg.norm(near[:, :2], axis=1))]
            dist = float(np.linalg.norm(nearest[:2]))
            if best is None or dist < best[3]:
                best = (*map(float, nearest), dist, float(len(near)))
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

    def publish_invalid_obstacle(self):
        out = Float32MultiArray()
        out.data = [float('nan')]*5
        self.obstacle_pub.publish(out)

    pointcloud_xyz = staticmethod(pointcloud_xyz)


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


# --- Disabled EdgeNode camera-only implementation (preserved verbatim) ---
# import cv2
# import numpy as np
#
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data
#
# from sensor_msgs.msg import CompressedImage
#
#
# class PerceptionNode(Node):
#     """
#     Minimal perception development node.
#
#     Current role:
#       MORAI / rosbag camera
#               ↓
#       sensor_msgs/CompressedImage
#               ↓
#       OpenCV BGR image
#               ↓
#       future perception algorithm
#
#     No lane detection, LiDAR processing, DBSCAN,
#     semantic diagnostics, or control dependency is included.
#     """
#
#     def __init__(self):
#         super().__init__('perception_node')
#
#         self.declare_parameter(
#             'camera_topic',
#             '/camera/image/compressed',
#         )
#
#         camera_topic = self.get_parameter(
#             'camera_topic'
#         ).value
#
#         self.frame_count = 0
#
#         self.camera_sub = self.create_subscription(
#             CompressedImage,
#             camera_topic,
#             self.camera_callback,
#             qos_profile_sensor_data,
#         )
#
#         self.get_logger().info(
#             f'Perception development node ready: {camera_topic}'
#         )
#
#     def camera_callback(self, msg: CompressedImage):
#         np_arr = np.frombuffer(
#             msg.data,
#             dtype=np.uint8,
#         )
#
#         frame = cv2.imdecode(
#             np_arr,
#             cv2.IMREAD_COLOR,
#         )
#
#         if frame is None:
#             self.get_logger().warning(
#                 'Failed to decode compressed camera image'
#             )
#             return
#
#         self.frame_count += 1
#
#         # Temporary lightweight heartbeat.
#         # Print only once every 100 frames so logging stays cheap.
#         if self.frame_count % 100 == 0:
#             height, width = frame.shape[:2]
#
#             self.get_logger().info(
#                 f'Camera OK | '
#                 f'frames={self.frame_count} '
#                 f'size={width}x{height}'
#             )
#
#         # --------------------------------------------------------
#         # Future perception algorithm starts here.
#         #
#         # Example:
#         # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         # bird = ...
#         # lane detection = ...
#         #
#         # Keep this node minimal until an algorithm is validated.
#         # --------------------------------------------------------
#
#
# def main(args=None):
#     rclpy.init(args=args)
#
#     node = PerceptionNode()
#
#     try:
#         rclpy.spin(node)
#
#     except KeyboardInterrupt:
#         pass
#
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
