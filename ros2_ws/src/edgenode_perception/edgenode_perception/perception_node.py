import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Float32, Float32MultiArray, Int32, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class LaneDiagnostics:
    # 최소 픽셀 수 검사 전, 슬라이딩 윈도우에서 수집한 좌우 픽셀 수.
    left_pixel_count: int = 0
    right_pixel_count: int = 0
    mode: str = 'NONE'
    failure_reason: str = 'OK'
    # 영상 픽셀 좌표. 피팅이 없는 쪽과 계산할 수 없는 폭은 NaN으로 표시한다.
    left_x_eval: float = float('nan')
    right_x_eval: float = float('nan')
    lane_width_eval: float = float('nan')
    # 실제 피팅 입력의 y 범위와 평가 위치. 피팅이 없으면 NaN / NO_FIT.
    eval_y: float = float('nan')
    left_y_min: float = float('nan')
    left_y_max: float = float('nan')
    right_y_min: float = float('nan')
    right_y_max: float = float('nan')
    left_eval_in_range: str = 'NO_FIT'
    right_eval_in_range: str = 'NO_FIT'


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
            ('roi_top_left_x_ratio', 0.0),
            ('roi_top_right_x_ratio', 1.0),
            ('roi_bottom_left_x_ratio', 0.0),
            ('roi_bottom_right_x_ratio', 1.0),
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
        self.left_pixel_count_pub = self.create_publisher(
            Int32, '/perception/left_pixel_count', 10)
        self.right_pixel_count_pub = self.create_publisher(
            Int32, '/perception/right_pixel_count', 10)
        self.lane_detection_mode_pub = self.create_publisher(
            String, '/perception/lane_detection_mode', 10)
        self.lane_failure_reason_pub = self.create_publisher(
            String, '/perception/lane_failure_reason', 10)
        self.left_x_eval_pub = self.create_publisher(
            Float32, '/perception/left_x_eval', 10)
        self.right_x_eval_pub = self.create_publisher(
            Float32, '/perception/right_x_eval', 10)
        self.lane_width_eval_pub = self.create_publisher(
            Float32, '/perception/lane_width_eval', 10)
        self.eval_y_pub = self.create_publisher(
            Float32, '/perception/eval_y', 10)
        self.left_y_min_pub = self.create_publisher(
            Float32, '/perception/left_y_min', 10)
        self.left_y_max_pub = self.create_publisher(
            Float32, '/perception/left_y_max', 10)
        self.right_y_min_pub = self.create_publisher(
            Float32, '/perception/right_y_min', 10)
        self.right_y_max_pub = self.create_publisher(
            Float32, '/perception/right_y_max', 10)
        self.left_eval_in_range_pub = self.create_publisher(
            String, '/perception/left_eval_in_range', 10)
        self.right_eval_in_range_pub = self.create_publisher(
            String, '/perception/right_eval_in_range', 10)

        self.get_logger().info(
            f'Perception ready: camera={self.camera_topic}, lidar={self.lidar_topic}')

    # ---------------- Camera / lane ----------------

    def camera_callback(self, msg: CompressedImage):
        diagnostics = LaneDiagnostics()
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.publish_lane(0.0, 0.0)
            diagnostics.failure_reason = 'IMAGE_DECODE_FAILED'
            self.publish_lane_diagnostics(diagnostics)
            self.get_logger().warning('Failed to decode compressed camera image')
            return

        mask = self.make_lane_mask(image)
        result = self.sliding_window_lane(mask, diagnostics)

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

        self.publish_lane_diagnostics(diagnostics)

    def publish_lane_diagnostics(self, diagnostics: LaneDiagnostics):
        self.left_pixel_count_pub.publish(Int32(data=diagnostics.left_pixel_count))
        self.right_pixel_count_pub.publish(Int32(data=diagnostics.right_pixel_count))
        self.lane_detection_mode_pub.publish(String(data=diagnostics.mode))
        self.lane_failure_reason_pub.publish(String(data=diagnostics.failure_reason))
        self.left_x_eval_pub.publish(Float32(data=diagnostics.left_x_eval))
        self.right_x_eval_pub.publish(Float32(data=diagnostics.right_x_eval))
        self.lane_width_eval_pub.publish(Float32(data=diagnostics.lane_width_eval))
        self.eval_y_pub.publish(Float32(data=diagnostics.eval_y))
        self.left_y_min_pub.publish(Float32(data=diagnostics.left_y_min))
        self.left_y_max_pub.publish(Float32(data=diagnostics.left_y_max))
        self.right_y_min_pub.publish(Float32(data=diagnostics.right_y_min))
        self.right_y_max_pub.publish(Float32(data=diagnostics.right_y_max))
        self.left_eval_in_range_pub.publish(String(data=diagnostics.left_eval_in_range))
        self.right_eval_in_range_pub.publish(String(data=diagnostics.right_eval_in_range))

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
        self, binary: np.ndarray, diagnostics: Optional[LaneDiagnostics] = None
    ) -> Optional[Tuple[float, float, float, float]]:
        if diagnostics is None:
            diagnostics = LaneDiagnostics()
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
            diagnostics.failure_reason = 'EMPTY_MASK'
            return None

        nwindows = int(self.get_parameter('sliding_windows').value)
        margin = int(self.get_parameter('sliding_margin_px').value)
        minpix = int(self.get_parameter('sliding_minpix').value)
        min_lane_pixels = int(self.get_parameter('min_lane_pixels').value)
        window_height = max(1, h // nwindows)

        def collect_pixels(left_base, right_base, left_available, right_available):
            left_current = left_base
            right_current = right_base
            left_inds = []
            right_inds = []

            for window in range(nwindows):
                y_low = h - (window + 1) * window_height
                y_high = h - window * window_height

                in_window_y = (nonzero_y >= y_low) & (nonzero_y < y_high)
                left_mask = (
                    left_available & in_window_y &
                    (nonzero_x >= left_current - margin) &
                    (nonzero_x < left_current + margin)
                )
                right_mask = (
                    right_available & in_window_y &
                    (nonzero_x >= right_current - margin) &
                    (nonzero_x < right_current + margin)
                )

                # 중심을 갱신하기 전에 겹친 픽셀을 더 가까운 쪽에만 배정한다.
                # 거리가 같으면 양쪽 후보에서 제외한다.
                overlap = left_mask & right_mask
                if np.any(overlap):
                    left_distance = np.abs(nonzero_x[overlap] - left_current)
                    right_distance = np.abs(nonzero_x[overlap] - right_current)
                    left_mask[overlap] = left_distance < right_distance
                    right_mask[overlap] = right_distance < left_distance

                if left_available:
                    good_left = left_mask.nonzero()[0]
                    left_inds.append(good_left)
                    if len(good_left) > minpix:
                        left_current = int(np.mean(nonzero_x[good_left]))

                if right_available:
                    good_right = right_mask.nonzero()[0]
                    right_inds.append(good_right)
                    if len(good_right) > minpix:
                        right_current = int(np.mean(nonzero_x[good_right]))

            left_inds = np.concatenate(left_inds) if left_inds else np.array([], dtype=np.int64)
            right_inds = np.concatenate(right_inds) if right_inds else np.array([], dtype=np.int64)

            return left_inds, right_inds

        # 원래 half-max 탐색은 유효 pair가 없을 때의 fallback으로 유지한다.
        left_inds, right_inds = collect_pixels(
            left_base, right_base, left_available, right_available)

        # 평평한 꼭대기는 하나의 peak로 취급하며 영상 양 끝도 포함한다.
        starts = np.r_[0, np.flatnonzero(np.diff(histogram) != 0) + 1]
        ends = np.r_[starts[1:] - 1, w - 1]
        peaks = []
        for start, end in zip(starts, ends):
            height = histogram[start]
            if (height > peak_threshold and
                    (start == 0 or height > histogram[start - 1]) and
                    (end == w - 1 or height > histogram[end + 1])):
                peaks.append((int((start + end) // 2), float(height)))
        # 인접 seed 중복만 줄인다. 이 거리를 차로 폭 판정에 사용하지 않는다.
        seeds = []
        for peak, _ in sorted(peaks, key=lambda item: (-item[1], item[0])):
            if all(abs(peak - seed) >= max(1, margin // 2) for seed in seeds):
                seeds.append(peak)

        eval_y = int(h * float(self.get_parameter('lookahead_y_ratio').value))
        expected_width = (
            float(self.get_parameter('default_lane_width_px_640').value) * w / 640.0)

        def fit_candidate(indices):
            if len(indices) < min_lane_pixels:
                return None
            ys = nonzero_y[indices]
            fit = np.polyfit(ys, nonzero_x[indices], 2)
            return fit, int(np.min(ys)), int(np.max(ys))

        def evaluate_pair(left_candidate, right_candidate):
            left_fit, left_y_min, left_y_max = left_candidate
            right_fit, right_y_min, right_y_max = right_candidate
            common_y_min = max(left_y_min, right_y_min)
            common_y_max = min(left_y_max, right_y_max)

            # 위쪽 끝은 ROI 절단과 원근 증폭의 영향을 더 크게 받으므로 8px,
            # 아래쪽 끝은 5px을 비운다. 둘 다 sliding window 높이보다 충분히
            # 작으며, 절대 y가 아니라 각 pair의 실제 공통 범위에 적용한다.
            top_margin = 8
            bottom_margin = 5
            safe_y_min = common_y_min + top_margin
            safe_y_max = common_y_max - bottom_margin
            if safe_y_min > safe_y_max:
                return None

            sample_y = np.arange(safe_y_min, safe_y_max + 1, dtype=np.float64)
            left_x = np.polyval(left_fit, sample_y)
            right_x = np.polyval(right_fit, sample_y)
            widths = right_x - left_x
            valid = (
                np.isfinite(left_x) & np.isfinite(right_x) &
                (right_x > left_x) &
                (widths >= 0.6 * expected_width) &
                (widths <= 1.4 * expected_width)
            )
            if not np.any(valid):
                return None

            valid_indices = np.flatnonzero(valid)
            # 영상 좌우 끝 5px 이내는 가능한 경우 피하되, 유일한 유효 구간이면
            # 관측 범위 내 결과를 버리지 않고 폭 일치도를 기준으로 사용한다.
            edge_margin = 5.0
            away_from_edge = (
                (left_x[valid_indices] >= edge_margin) &
                (right_x[valid_indices] <= (w - 1) - edge_margin)
            )
            if np.any(away_from_edge):
                valid_indices = valid_indices[away_from_edge]

            best_index = min(
                valid_indices,
                key=lambda index: (
                    abs(widths[index] - expected_width),
                    abs(sample_y[index] - eval_y),
                ),
            )
            return (
                int(sample_y[best_index]),
                float(left_x[best_index]),
                float(right_x[best_index]),
            )

        candidates = []
        fixed_candidates = []
        for seed in seeds:
            indices, _ = collect_pixels(seed, 0, True, False)
            candidate = fit_candidate(indices)
            if candidate is None:
                continue
            candidates.append((seed, candidate))
            fit, y_min, y_max = candidate
            if y_min <= eval_y <= y_max:
                x = float(np.polyval(fit, eval_y))
                if np.isfinite(x):
                    fixed_candidates.append((x, seed))

        # 기존 고정-y 선택을 먼저 그대로 수행해 정상 장면의 회귀를 막는다.
        fixed_candidates.sort()
        best_score = None
        adaptive_eval_y = None
        for i, (left_eval, left_seed) in enumerate(fixed_candidates):
            for right_eval, right_seed in fixed_candidates[i + 1:]:
                if not 0.6 * expected_width <= right_eval - left_eval <= 1.4 * expected_width:
                    continue
                pair_left, pair_right = collect_pixels(left_seed, right_seed, True, True)
                left_candidate = fit_candidate(pair_left)
                right_candidate = fit_candidate(pair_right)
                if left_candidate is None or right_candidate is None:
                    continue
                left_fit, left_y_min, left_y_max = left_candidate
                right_fit, right_y_min, right_y_max = right_candidate
                if not (left_y_min <= eval_y <= left_y_max and
                        right_y_min <= eval_y <= right_y_max):
                    continue
                lx = float(np.polyval(left_fit, eval_y))
                rx = float(np.polyval(right_fit, eval_y))
                if not 0.6 * expected_width <= rx - lx <= 1.4 * expected_width:
                    continue
                score = (abs(rx - lx - expected_width),
                         -min(len(pair_left), len(pair_right)),
                         -(len(pair_left) + len(pair_right)))
                if best_score is None or score < best_score:
                    best_score = score
                    left_inds, right_inds = pair_left, pair_right
                    left_available = right_available = True
                    adaptive_eval_y = eval_y

        # 고정 위치에서 유효 pair가 없을 때만 공통 관측 범위를 탐색한다.
        if best_score is None:
            candidates.sort(key=lambda item: item[0])
            for i, (left_seed, _) in enumerate(candidates):
                for right_seed, _ in candidates[i + 1:]:
                    pair_left, pair_right = collect_pixels(
                        left_seed, right_seed, True, True)
                    left_candidate = fit_candidate(pair_left)
                    right_candidate = fit_candidate(pair_right)
                    if left_candidate is None or right_candidate is None:
                        continue
                    pair_eval = evaluate_pair(left_candidate, right_candidate)
                    if pair_eval is None:
                        continue
                    pair_eval_y, lx, rx = pair_eval
                    score = (abs(rx - lx - expected_width),
                             abs(pair_eval_y - eval_y),
                             -min(len(pair_left), len(pair_right)),
                             -(len(pair_left) + len(pair_right)))
                    if best_score is None or score < best_score:
                        best_score = score
                        left_inds, right_inds = pair_left, pair_right
                        left_available = right_available = True
                        adaptive_eval_y = pair_eval_y

        diagnostics.left_pixel_count = len(left_inds)
        diagnostics.right_pixel_count = len(right_inds)

        left_fit = None
        right_fit = None
        if left_available and len(left_inds) >= min_lane_pixels:
            left_fit = np.polyfit(nonzero_y[left_inds], nonzero_x[left_inds], 2)
            diagnostics.left_y_min = float(np.min(nonzero_y[left_inds]))
            diagnostics.left_y_max = float(np.max(nonzero_y[left_inds]))
        if right_available and len(right_inds) >= min_lane_pixels:
            right_fit = np.polyfit(nonzero_y[right_inds], nonzero_x[right_inds], 2)
            diagnostics.right_y_min = float(np.min(nonzero_y[right_inds]))
            diagnostics.right_y_max = float(np.max(nonzero_y[right_inds]))

        if left_fit is None and right_fit is None:
            # 탐색을 시작했지만 픽셀 수가 부족한 경우와 피크가 없는 경우를 구분한다.
            diagnostics.failure_reason = (
                'INSUFFICIENT_PIXELS' if left_available or right_available
                else 'BOTH_FIT_FAILED'
            )
            return None

        # 유효 pair가 있으면 공통 관측 범위에서 고른 위치를 사용한다. pair가
        # 없을 때의 single-lane/fallback 동작은 기존 고정 위치를 유지한다.
        lookahead_y = adaptive_eval_y if adaptive_eval_y is not None else eval_y
        diagnostics.eval_y = float(lookahead_y)
        if left_fit is not None:
            diagnostics.left_eval_in_range = (
                'INSIDE' if diagnostics.left_y_min <= lookahead_y <= diagnostics.left_y_max
                else 'OUTSIDE'
            )
        if right_fit is not None:
            diagnostics.right_eval_in_range = (
                'INSIDE' if diagnostics.right_y_min <= lookahead_y <= diagnostics.right_y_max
                else 'OUTSIDE'
            )
        lane_width = (
            float(self.get_parameter('default_lane_width_px_640').value) *
            (w / 640.0)
        )

        left_x = np.polyval(left_fit, lookahead_y) if left_fit is not None else None
        right_x = np.polyval(right_fit, lookahead_y) if right_fit is not None else None

        # 기하 검사로 반환하더라도 콜백에서 실제 평가 좌표를 발행할 수 있게 보존한다.
        diagnostics.left_x_eval = float(left_x) if left_x is not None else float('nan')
        diagnostics.right_x_eval = float(right_x) if right_x is not None else float('nan')
        diagnostics.lane_width_eval = diagnostics.right_x_eval - diagnostics.left_x_eval

        # 원래 평가 좌표는 진단에 남기되, 관측 y 범위 밖의 fit은 중앙 계산에서 제외한다.
        if left_fit is not None and not (
            diagnostics.left_y_min <= lookahead_y <= diagnostics.left_y_max
        ):
            left_x = None
        if right_fit is not None and not (
            diagnostics.right_y_min <= lookahead_y <= diagnostics.right_y_max
        ):
            right_x = None
        if left_x is None and right_x is None:
            diagnostics.failure_reason = 'EVAL_OUTSIDE_FIT_RANGE'
            return None

        if left_x is not None and right_x is not None:
            if right_x <= left_x:
                diagnostics.failure_reason = 'INVALID_LANE_GEOMETRY'
                return None
            # 영상 폭에 맞춘 기본 차로 폭의 50%를 최소 유효 폭으로 사용한다.
            # 640px 영상에서는 140px: 약 25px인 한 표시의 양쪽 경계를 병합한다.
            min_lane_width = lane_width * 0.5
            if right_x - left_x < min_lane_width:
                marker_x = 0.5 * (left_x + right_x)
                # 정확히 영상 중앙이면 왼쪽 표시로 처리한다.
                if marker_x <= w * 0.5:
                    left_x, right_x = marker_x, None
                else:
                    left_x, right_x = None, marker_x
            elif not 0.6 * lane_width <= right_x - left_x <= 1.4 * lane_width:
                # fallback의 두 fit도 허용 폭을 벗어나면 단일 경계로만 사용한다.
                if len(left_inds) >= len(right_inds):
                    right_x = None
                else:
                    left_x = None

        if left_x is not None and right_x is not None:
            diagnostics.mode = 'BOTH'
            lane_center = 0.5 * (left_x + right_x)
            count_score = min(1.0, (len(left_inds) + len(right_inds)) / 3000.0)
            width_score = math.exp(-abs((right_x - left_x) - lane_width) /
                                   max(1.0, lane_width))
            confidence = clamp(0.55 * count_score + 0.45 * width_score, 0.0, 1.0)
        elif left_x is not None:
            diagnostics.mode = 'LEFT_ONLY'
            lane_center = left_x + lane_width * 0.5
            confidence = clamp(len(left_inds) / 1800.0, 0.0, 0.55)
        else:
            diagnostics.mode = 'RIGHT_ONLY'
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
