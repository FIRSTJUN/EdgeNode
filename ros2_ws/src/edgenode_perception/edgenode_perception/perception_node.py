import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String


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


@dataclass
class ProductionFitSnapshot:
    """Read-only exposure of the fits already selected by the production path."""

    left_fit: Optional[np.ndarray] = None
    right_fit: Optional[np.ndarray] = None
    eval_y: float = float('nan')


@dataclass
class SemanticComponent:
    """One diagnostic-only connected component and its center-line fit."""

    fit: np.ndarray
    pixel_count: int
    y_min: int
    y_max: int
    x_eval: float = float('nan')
    component_id: int = -1
    area: int = 0
    median_h: float = float('nan')
    median_s: float = float('nan')
    median_v: float = float('nan')


@dataclass
class SemanticDiagnostics:
    """Semantic observations that are never consumed by lane selection."""

    eval_y: float = float('nan')
    yellow_candidate_x: float = float('nan')
    beige_candidate_x: float = float('nan')
    neutral_white_candidate_x: float = float('nan')
    yellow_pixel_count: int = 0
    beige_pixel_count: int = 0
    neutral_white_pixel_count: int = 0
    yellow_components: List[SemanticComponent] = field(default_factory=list)
    beige_components: List[SemanticComponent] = field(default_factory=list)
    neutral_white_components: List[SemanticComponent] = field(default_factory=list)
    right_identity: str = 'NO_RIGHT_FIT'
    beige_neutral_distance: float = float('nan')
    right_to_beige_distance: float = float('nan')
    right_to_neutral_white_distance: float = float('nan')


@dataclass
class ShadowCandidateEvaluation:
    """Explainable score for one beige component in the shadow path."""

    component: SemanticComponent
    score: float = float('-inf')
    status: str = 'REJECTED'
    rejection_reason: str = 'LOW_SCORE'
    eval_y: float = float('nan')
    x: float = float('nan')
    y_min: float = float('nan')
    y_max: float = float('nan')
    lane_width: float = float('nan')
    temporal_delta: float = float('nan')
    sample_y: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64))
    score_parts: Dict[str, float] = field(default_factory=dict)
    yellow_component: Optional[SemanticComponent] = None
    neutral_component: Optional[SemanticComponent] = None
    yellow_order_fraction: float = 0.0
    neutral_order_fraction: float = 0.0
    bounds_fraction: float = 0.0
    common_y_span: float = 0.0
    neutral_common_y_span: float = 0.0


@dataclass
class ShadowRightDiagnostics:
    """Shadow selection consumed only by the explicit experimental gate."""

    right_x: float = float('nan')
    eval_y: float = float('nan')
    score: float = float('nan')
    status: str = 'UNRESOLVED'
    rejection_reason: str = 'NO_BEIGE_COMPONENT'
    candidate_count: int = 0
    component_id: int = -1
    y_min: float = float('nan')
    y_max: float = float('nan')
    lane_width: float = float('nan')
    temporal_delta: float = float('nan')
    evaluations: List[ShadowCandidateEvaluation] = field(default_factory=list)


@dataclass
class ExperimentalShadowOverrideDiagnostics:
    """A/B-only result; the original LaneDiagnostics remain untouched."""

    active: bool = False
    reason: str = 'FEATURE_DISABLED'
    original_right_x: float = float('nan')
    selected_right_x: float = float('nan')
    right_shift_px: float = float('nan')
    original_lane_error: float = float('nan')
    selected_lane_error: float = float('nan')
    lane_error_delta: float = float('nan')
    original_lane_center: float = float('nan')
    selected_lane_center: float = float('nan')
    lane_center_shift_px: float = float('nan')
    original_lane_width: float = float('nan')
    selected_lane_width: float = float('nan')
    selected_right_fit: Optional[np.ndarray] = None
    selected_right_y_min: float = float('nan')
    selected_right_y_max: float = float('nan')


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
        self.declare_parameter(
            'identity_debug_image_topic', '/perception/debug/identity_image/compressed')
        self.declare_parameter(
            'shadow_debug_image_topic', '/perception/debug/shadow_identity_image/compressed')

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
            ('adaptive_max_lane_width_ratio', 1.55),
        ]:
            self.declare_parameter(name, default)
        self.declare_parameter('sliding_windows', 9)
        self.declare_parameter('sliding_margin_px', 55)
        self.declare_parameter('sliding_minpix', 30)
        self.declare_parameter('min_lane_pixels', 180)

        # Passive semantic diagnostics. These parameters never feed production fits.
        self.declare_parameter('semantic_diagnostics_enabled', True)
        self.declare_parameter('diag_neutral_white_max_saturation', 25)
        self.declare_parameter('diag_beige_hue_min', 5)
        self.declare_parameter('diag_beige_hue_max', 45)
        self.declare_parameter('diag_min_component_pixels', 40)
        self.declare_parameter('diag_min_component_y_span', 20)
        self.declare_parameter('diag_component_close_kernel', 3)
        self.declare_parameter('diag_identity_ambiguity_px', 5.0)
        self.declare_parameter('diag_identity_max_distance_px', 55.0)

        # Shadow selector; production can consume it only through the default-off gate.
        self.declare_parameter('shadow_right_selector_enabled', True)
        self.declare_parameter('shadow_debug_image_enabled', False)
        self.declare_parameter('shadow_min_common_y_span_px', 8)
        self.declare_parameter('shadow_sample_count', 5)
        self.declare_parameter('shadow_min_valid_sample_ratio', 0.8)
        self.declare_parameter('shadow_neutral_missing_score', 0.4)
        self.declare_parameter('shadow_neutral_contradiction_score', -0.5)
        self.declare_parameter('shadow_yellow_position_tiebreak_weight', 0.05)
        self.declare_parameter('shadow_min_score', 0.55)
        self.declare_parameter('shadow_expected_width_tolerance_ratio', 0.75)
        self.declare_parameter('shadow_max_temporal_jump_px', 120.0)
        self.declare_parameter('shadow_semantic_weight', 0.10)
        self.declare_parameter('shadow_yellow_order_weight', 0.25)
        self.declare_parameter('shadow_neutral_order_weight', 0.25)
        self.declare_parameter('shadow_geometry_weight', 0.15)
        self.declare_parameter('shadow_y_span_weight', 0.10)
        self.declare_parameter('shadow_bounds_weight', 0.10)
        self.declare_parameter('shadow_temporal_weight', 0.05)

        # Strict production A/B gate. It is intentionally disabled by default.
        self.declare_parameter(
            'experimental_shadow_right_override_enabled', False)
        self.declare_parameter('experimental_shadow_min_score', 0.85)
        self.declare_parameter(
            'experimental_shadow_require_neutral_order', True)
        self.declare_parameter(
            'experimental_shadow_max_temporal_delta_px', 35.0)
        self.declare_parameter(
            'experimental_shadow_min_common_y_span_px', 17)
        self.declare_parameter(
            'experimental_shadow_min_lane_width_ratio', 0.90)
        self.declare_parameter(
            'experimental_shadow_max_lane_width_ratio', 1.25)
        self.declare_parameter(
            'experimental_shadow_confirm_frames', 4)
        self.declare_parameter(
            'experimental_shadow_slew_rate_px_per_frame', 10.0)

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
        self.identity_debug_pub = self.create_publisher(
            CompressedImage,
            self.get_parameter('identity_debug_image_topic').value, 3)
        self.shadow_debug_pub = self.create_publisher(
            CompressedImage,
            self.get_parameter('shadow_debug_image_topic').value, 3)
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

        semantic_names = ('yellow', 'beige', 'neutral_white')
        self.semantic_candidate_x_pubs = {
            name: self.create_publisher(
                Float32, f'/perception/diag/{name}_candidate_x', 10)
            for name in semantic_names
        }
        self.semantic_pixel_count_pubs = {
            name: self.create_publisher(
                Int32, f'/perception/diag/{name}_pixel_count', 10)
            for name in semantic_names
        }
        self.semantic_component_count_pubs = {
            name: self.create_publisher(
                Int32, f'/perception/diag/{name}_component_count', 10)
            for name in semantic_names
        }
        self.semantic_component_x_pubs = {
            name: self.create_publisher(
                Float32MultiArray, f'/perception/diag/{name}_component_x', 10)
            for name in semantic_names
        }
        self.semantic_eval_y_pub = self.create_publisher(
            Float32, '/perception/diag/eval_y', 10)
        self.right_identity_pub = self.create_publisher(
            String, '/perception/diag/right_identity', 10)
        self.beige_neutral_distance_pub = self.create_publisher(
            Float32, '/perception/diag/beige_neutral_distance', 10)
        self.right_to_beige_distance_pub = self.create_publisher(
            Float32, '/perception/diag/right_to_beige_distance', 10)
        self.right_to_neutral_white_distance_pub = self.create_publisher(
            Float32, '/perception/diag/right_to_neutral_white_distance', 10)

        shadow_float_topics = {
            'right_x': '/perception/diag/shadow_right_x',
            'eval_y': '/perception/diag/shadow_right_eval_y',
            'score': '/perception/diag/shadow_right_score',
            'y_min': '/perception/diag/shadow_right_y_min',
            'y_max': '/perception/diag/shadow_right_y_max',
            'lane_width': '/perception/diag/shadow_right_lane_width',
            'temporal_delta': '/perception/diag/shadow_right_temporal_delta',
        }
        self.shadow_float_pubs = {
            name: self.create_publisher(Float32, topic, 10)
            for name, topic in shadow_float_topics.items()
        }
        self.shadow_status_pub = self.create_publisher(
            String, '/perception/diag/shadow_right_status', 10)
        self.shadow_rejection_reason_pub = self.create_publisher(
            String, '/perception/diag/shadow_right_rejection_reason', 10)
        self.shadow_candidate_count_pub = self.create_publisher(
            Int32, '/perception/diag/shadow_right_candidate_count', 10)
        self.shadow_component_id_pub = self.create_publisher(
            Int32, '/perception/diag/shadow_right_component_id', 10)

        experimental_float_topics = {
            'original_right_x':
                '/perception/diag/experimental_original_right_x',
            'selected_right_x':
                '/perception/diag/experimental_selected_right_x',
            'right_shift_px':
                '/perception/diag/experimental_right_shift_px',
            'original_lane_error':
                '/perception/diag/experimental_original_lane_error',
            'selected_lane_error':
                '/perception/diag/experimental_selected_lane_error',
            'lane_error_delta':
                '/perception/diag/experimental_lane_error_delta',
            'original_lane_center':
                '/perception/diag/experimental_original_lane_center',
            'selected_lane_center':
                '/perception/diag/experimental_selected_lane_center',
            'lane_center_shift_px':
                '/perception/diag/experimental_lane_center_shift_px',
            'original_lane_width':
                '/perception/diag/experimental_original_lane_width',
            'selected_lane_width':
                '/perception/diag/experimental_selected_lane_width',
        }
        self.experimental_float_pubs = {
            name: self.create_publisher(Float32, topic, 10)
            for name, topic in experimental_float_topics.items()
        }
        self.experimental_override_active_pub = self.create_publisher(
            Bool, '/perception/diag/experimental_shadow_override_active', 10)
        self.experimental_override_reason_pub = self.create_publisher(
            String, '/perception/diag/experimental_shadow_override_reason', 10)

        # Deliberately separate from every production temporal/fit value.
        self.shadow_previous_right_x = float('nan')
        self.shadow_previous_eval_y = float('nan')
        self.shadow_last_stamp_ns: Optional[int] = None

        # Experimental override state is deliberately separate from
        # production and diagnostic shadow continuity.
        self.experimental_shadow_confirm_count = 0
        self.experimental_shadow_override_active_state = False
        self.experimental_shadow_previous_output_right_x = float('nan')

        self.get_logger().info(
            f'Perception ready: camera={self.camera_topic}, lidar={self.lidar_topic}')

    # ---------------- Camera / lane ----------------

    def camera_callback(self, msg: CompressedImage):
        diagnostics = LaneDiagnostics()
        fit_snapshot = ProductionFitSnapshot()
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.publish_lane(0.0, 0.0)
            diagnostics.failure_reason = 'IMAGE_DECODE_FAILED'
            self.publish_lane_diagnostics(diagnostics)
            _, experimental = self.evaluate_experimental_shadow_override(
                None, diagnostics, ShadowRightDiagnostics(), 0)
            self.publish_experimental_shadow_override_diagnostics(experimental)
            if self.get_parameter('semantic_diagnostics_enabled').value:
                self.publish_semantic_diagnostics(SemanticDiagnostics())
                if self.get_parameter('shadow_right_selector_enabled').value:
                    self.publish_shadow_right_diagnostics(
                        ShadowRightDiagnostics())
                    self.clear_shadow_continuity()
            self.get_logger().warning('Failed to decode compressed camera image')
            return

        mask = self.make_lane_mask(image)
        original_result = self.sliding_window_lane(
            mask, diagnostics, fit_snapshot=fit_snapshot)

        h, w = image.shape[:2]
        semantic = SemanticDiagnostics()
        shadow = ShadowRightDiagnostics()
        if self.get_parameter('semantic_diagnostics_enabled').value:
            try:
                production_right_x = (
                    diagnostics.right_x_eval
                    if diagnostics.mode in ('BOTH', 'RIGHT_ONLY')
                    else float('nan'))
                semantic = self.analyze_semantic_candidates(
                    image, diagnostics.eval_y, production_right_x)
                self.publish_semantic_diagnostics(semantic)
                self.publish_identity_debug_image(
                    msg, image, semantic, fit_snapshot, diagnostics)
                if self.get_parameter('shadow_right_selector_enabled').value:
                    self.update_shadow_sequence_stamp(msg)
                    shadow = self.select_shadow_right(semantic, w)
                    self.publish_shadow_right_diagnostics(shadow)
                    if self.get_parameter('shadow_debug_image_enabled').value:
                        self.publish_shadow_debug_image(
                            msg, image, semantic, shadow, fit_snapshot,
                            diagnostics)
            except Exception as exc:  # Diagnostics must not suppress production output.
                self.get_logger().error(
                    f'Passive semantic diagnostics failed: {exc}')

        selected_result, experimental = (
            self.evaluate_experimental_shadow_override(
                original_result, diagnostics, shadow, w))
        self.publish_experimental_shadow_override_diagnostics(experimental)

        debug = image.copy()
        cv2.line(debug, (w // 2, 0), (w // 2, h - 1), (0, 0, 255), 2)

        if selected_result is None:
            self.publish_lane(0.0, 0.0)
            cv2.putText(
                debug, 'LANE LOST', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            lane_error, confidence, lane_center_x, lookahead_y = selected_result
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

        # These retain their original production-only meaning even when overridden.
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

    def publish_semantic_diagnostics(
        self, diagnostics: SemanticDiagnostics
    ):
        groups = {
            'yellow': (
                diagnostics.yellow_candidate_x,
                diagnostics.yellow_pixel_count,
                diagnostics.yellow_components,
            ),
            'beige': (
                diagnostics.beige_candidate_x,
                diagnostics.beige_pixel_count,
                diagnostics.beige_components,
            ),
            'neutral_white': (
                diagnostics.neutral_white_candidate_x,
                diagnostics.neutral_white_pixel_count,
                diagnostics.neutral_white_components,
            ),
        }
        for name, (candidate_x, pixel_count, components) in groups.items():
            self.semantic_candidate_x_pubs[name].publish(
                Float32(data=float(candidate_x)))
            self.semantic_pixel_count_pubs[name].publish(
                Int32(data=int(pixel_count)))
            self.semantic_component_count_pubs[name].publish(
                Int32(data=len(components)))
            component_x = sorted(
                float(component.x_eval)
                for component in components
                if np.isfinite(component.x_eval)
            )
            self.semantic_component_x_pubs[name].publish(
                Float32MultiArray(data=component_x))

        self.semantic_eval_y_pub.publish(
            Float32(data=float(diagnostics.eval_y)))
        self.right_identity_pub.publish(
            String(data=diagnostics.right_identity))
        self.beige_neutral_distance_pub.publish(
            Float32(data=float(diagnostics.beige_neutral_distance)))
        self.right_to_beige_distance_pub.publish(
            Float32(data=float(diagnostics.right_to_beige_distance)))
        self.right_to_neutral_white_distance_pub.publish(
            Float32(data=float(diagnostics.right_to_neutral_white_distance)))

    def analyze_semantic_candidates(
        self, image: np.ndarray, production_eval_y: float,
        production_right_x: float
    ) -> SemanticDiagnostics:
        """Observe color/component identities without feeding production selection."""
        h, w = image.shape[:2]
        if np.isfinite(production_eval_y):
            eval_y = float(production_eval_y)
        else:
            eval_y = float(
                int(h * float(self.get_parameter('lookahead_y_ratio').value)))

        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hls,
            np.array([0, 175, 0], dtype=np.uint8),
            np.array([255, 255, 130], dtype=np.uint8),
        )
        yellow = cv2.inRange(
            hsv,
            np.array([12, 70, 70], dtype=np.uint8),
            np.array([42, 255, 255], dtype=np.uint8),
        )

        roi = self.make_diagnostic_roi(h, w)
        white = cv2.bitwise_and(white, roi)
        yellow = cv2.bitwise_and(yellow, roi)

        close_kernel = int(
            self.get_parameter('diag_component_close_kernel').value)
        if close_kernel > 1:
            if close_kernel % 2 == 0:
                close_kernel += 1
            kernel = np.ones((close_kernel, close_kernel), dtype=np.uint8)
            white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel)
            yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kernel)
            white = cv2.bitwise_and(white, roi)
            yellow = cv2.bitwise_and(yellow, roi)

        yellow_components = self.fit_semantic_components(yellow, hsv, eval_y)
        white_components = self.classify_white_components(
            white, hsv, eval_y)

        diagnostics = SemanticDiagnostics(
            eval_y=eval_y,
            yellow_components=yellow_components,
            beige_components=white_components['beige'],
            neutral_white_components=white_components['neutral_white'],
        )
        diagnostics.yellow_pixel_count = sum(
            component.pixel_count for component in yellow_components)
        diagnostics.beige_pixel_count = sum(
            component.pixel_count
            for component in diagnostics.beige_components)
        diagnostics.neutral_white_pixel_count = sum(
            component.pixel_count
            for component in diagnostics.neutral_white_components)

        diagnostics.yellow_candidate_x = self.primary_component_x(
            diagnostics.yellow_components)
        diagnostics.beige_candidate_x = self.primary_component_x(
            diagnostics.beige_components)
        diagnostics.neutral_white_candidate_x = self.primary_component_x(
            diagnostics.neutral_white_components)

        beige_x = diagnostics.beige_candidate_x
        neutral_x = diagnostics.neutral_white_candidate_x
        if np.isfinite(beige_x) and np.isfinite(neutral_x):
            diagnostics.beige_neutral_distance = float(neutral_x - beige_x)

        self.assign_right_identity(
            diagnostics, float(production_right_x))
        return diagnostics

    def make_diagnostic_roi(self, h: int, w: int) -> np.ndarray:
        """Build an ROI equivalent to production without reusing its binary mask."""
        top_y = int(h * float(self.get_parameter('roi_top_y_ratio').value))
        bot_y = int(h * float(self.get_parameter('roi_bottom_y_ratio').value))
        tl = int(w * float(self.get_parameter('roi_top_left_x_ratio').value))
        tr = int(w * float(self.get_parameter('roi_top_right_x_ratio').value))
        bl = int(w * float(self.get_parameter('roi_bottom_left_x_ratio').value))
        br = int(w * float(self.get_parameter('roi_bottom_right_x_ratio').value))
        roi = np.zeros((h, w), dtype=np.uint8)
        polygon = np.array(
            [[(bl, bot_y), (tl, top_y), (tr, top_y), (br, bot_y)]],
            dtype=np.int32,
        )
        cv2.fillPoly(roi, polygon, 255)
        return roi

    def component_geometry(
        self, labels: np.ndarray, stats: np.ndarray, label: int,
        eval_y: float
    ) -> Optional[Tuple[SemanticComponent, np.ndarray]]:
        x, y, width, height, area = stats[label]
        min_pixels = int(
            self.get_parameter('diag_min_component_pixels').value)
        min_y_span = int(
            self.get_parameter('diag_min_component_y_span').value)
        if area < min_pixels or height - 1 < min_y_span:
            return None

        local_labels = labels[y:y + height, x:x + width]
        local_y, local_x = np.nonzero(local_labels == label)
        pixel_y = local_y + y
        pixel_x = local_x + x
        if len(pixel_x) < min_pixels:
            return None

        fit = np.polyfit(pixel_y, pixel_x, 2)
        y_min = int(np.min(pixel_y))
        y_max = int(np.max(pixel_y))
        x_eval = float('nan')
        if y_min <= eval_y <= y_max:
            evaluated = float(np.polyval(fit, eval_y))
            if np.isfinite(evaluated):
                x_eval = evaluated
        return (
            SemanticComponent(
                fit=fit,
                pixel_count=int(len(pixel_x)),
                y_min=y_min,
                y_max=y_max,
                x_eval=x_eval,
                component_id=int(label),
                area=int(area),
            ),
            (local_labels == label),
        )

    def fit_semantic_components(
        self, mask: np.ndarray, hsv: np.ndarray, eval_y: float
    ) -> List[SemanticComponent]:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        components = []
        for label in range(1, count):
            geometry = self.component_geometry(
                labels, stats, label, eval_y)
            if geometry is not None:
                component, local_component = geometry
                x, y, width, height, _ = stats[label]
                local_hsv = hsv[y:y + height, x:x + width]
                values = local_hsv[local_component]
                component.median_h = float(np.median(values[:, 0]))
                component.median_s = float(np.median(values[:, 1]))
                component.median_v = float(np.median(values[:, 2]))
                components.append(component)
        return components

    def classify_white_components(
        self, mask: np.ndarray, hsv: np.ndarray, eval_y: float
    ):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        classified = {'beige': [], 'neutral_white': []}
        neutral_max_s = int(
            self.get_parameter('diag_neutral_white_max_saturation').value)
        beige_hue_min = int(
            self.get_parameter('diag_beige_hue_min').value)
        beige_hue_max = int(
            self.get_parameter('diag_beige_hue_max').value)

        for label in range(1, count):
            geometry = self.component_geometry(
                labels, stats, label, eval_y)
            if geometry is None:
                continue
            component, local_component = geometry
            x, y, width, height, _ = stats[label]
            local_hsv = hsv[y:y + height, x:x + width]
            median_s = float(np.median(local_hsv[:, :, 1][local_component]))
            median_h = float(np.median(local_hsv[:, :, 0][local_component]))
            component.median_h = median_h
            component.median_s = median_s
            component.median_v = float(np.median(local_hsv[:, :, 2][local_component]))
            if median_s <= neutral_max_s:
                classified['neutral_white'].append(component)
            elif beige_hue_min <= median_h <= beige_hue_max:
                classified['beige'].append(component)
        return classified

    @staticmethod
    def primary_component_x(
        components: List[SemanticComponent]
    ) -> float:
        valid = [
            component for component in components
            if np.isfinite(component.x_eval)
        ]
        if not valid:
            return float('nan')
        primary = max(
            valid,
            key=lambda component: (
                component.y_max - component.y_min,
                component.pixel_count,
            ),
        )
        return float(primary.x_eval)

    def assign_right_identity(
        self, diagnostics: SemanticDiagnostics, production_right_x: float
    ):
        if not np.isfinite(production_right_x):
            diagnostics.right_identity = 'NO_RIGHT_FIT'
            return

        def nearest_distance(components):
            values = [
                abs(float(component.x_eval) - production_right_x)
                for component in components
                if np.isfinite(component.x_eval)
            ]
            return min(values) if values else float('nan')

        beige_distance = nearest_distance(diagnostics.beige_components)
        neutral_distance = nearest_distance(
            diagnostics.neutral_white_components)
        diagnostics.right_to_beige_distance = float(beige_distance)
        diagnostics.right_to_neutral_white_distance = float(neutral_distance)

        candidates = []
        if np.isfinite(beige_distance):
            candidates.append(('BEIGE', beige_distance))
        if np.isfinite(neutral_distance):
            candidates.append(('NEUTRAL_WHITE', neutral_distance))
        if not candidates:
            diagnostics.right_identity = 'UNRESOLVED'
            return

        candidates.sort(key=lambda item: item[1])
        max_distance = float(
            self.get_parameter('diag_identity_max_distance_px').value)
        if candidates[0][1] > max_distance:
            diagnostics.right_identity = 'UNRESOLVED'
            return

        ambiguity = float(
            self.get_parameter('diag_identity_ambiguity_px').value)
        if (len(candidates) > 1 and
                abs(candidates[0][1] - candidates[1][1]) <= ambiguity):
            diagnostics.right_identity = 'AMBIGUOUS'
        else:
            diagnostics.right_identity = candidates[0][0]

    def clear_shadow_continuity(self):
        self.shadow_previous_right_x = float('nan')
        self.shadow_previous_eval_y = float('nan')

    def reset_experimental_shadow_override_state(self):
        """Reset only the default-off experimental production override state."""
        self.experimental_shadow_confirm_count = 0
        self.experimental_shadow_override_active_state = False
        self.experimental_shadow_previous_output_right_x = float('nan')

    def reset_shadow_state(self):
        """Reset shadow diagnostic and experimental state between sequences."""
        self.clear_shadow_continuity()
        self.reset_experimental_shadow_override_state()
        self.shadow_last_stamp_ns = None

    def update_shadow_sequence_stamp(self, msg: CompressedImage):
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000 +
            int(msg.header.stamp.nanosec)
        )
        if (self.shadow_last_stamp_ns is not None and
                (stamp_ns <= self.shadow_last_stamp_ns or
                 stamp_ns - self.shadow_last_stamp_ns > 2_000_000_000)):
            self.reset_shadow_state()
        self.shadow_last_stamp_ns = stamp_ns

    def publish_shadow_right_diagnostics(
        self, diagnostics: ShadowRightDiagnostics
    ):
        values = {
            'right_x': diagnostics.right_x,
            'eval_y': diagnostics.eval_y,
            'score': diagnostics.score,
            'y_min': diagnostics.y_min,
            'y_max': diagnostics.y_max,
            'lane_width': diagnostics.lane_width,
            'temporal_delta': diagnostics.temporal_delta,
        }
        for name, value in values.items():
            self.shadow_float_pubs[name].publish(Float32(data=float(value)))
        self.shadow_status_pub.publish(String(data=diagnostics.status))
        self.shadow_rejection_reason_pub.publish(
            String(data=diagnostics.rejection_reason))
        self.shadow_candidate_count_pub.publish(
            Int32(data=diagnostics.candidate_count))
        self.shadow_component_id_pub.publish(
            Int32(data=diagnostics.component_id))

    def publish_experimental_shadow_override_diagnostics(
        self, diagnostics: ExperimentalShadowOverrideDiagnostics
    ):
        values = {
            'original_right_x': diagnostics.original_right_x,
            'selected_right_x': diagnostics.selected_right_x,
            'right_shift_px': diagnostics.right_shift_px,
            'original_lane_error': diagnostics.original_lane_error,
            'selected_lane_error': diagnostics.selected_lane_error,
            'lane_error_delta': diagnostics.lane_error_delta,
            'original_lane_center': diagnostics.original_lane_center,
            'selected_lane_center': diagnostics.selected_lane_center,
            'lane_center_shift_px': diagnostics.lane_center_shift_px,
            'original_lane_width': diagnostics.original_lane_width,
            'selected_lane_width': diagnostics.selected_lane_width,
        }
        for name, value in values.items():
            self.experimental_float_pubs[name].publish(
                Float32(data=float(value)))
        self.experimental_override_active_pub.publish(
            Bool(data=diagnostics.active))
        self.experimental_override_reason_pub.publish(
            String(data=diagnostics.reason))

    def evaluate_experimental_shadow_override(
        self, original_result: Optional[Tuple[float, float, float, float]],
        production: LaneDiagnostics, shadow: ShadowRightDiagnostics,
        image_width: int,
    ) -> Tuple[
        Optional[Tuple[float, float, float, float]],
        ExperimentalShadowOverrideDiagnostics,
    ]:
        """Apply a strict, fail-closed A/B gate without mutating production data."""
        diagnostics = ExperimentalShadowOverrideDiagnostics(
            original_right_x=float(production.right_x_eval),
            selected_right_x=float(production.right_x_eval),
            right_shift_px=0.0 if np.isfinite(production.right_x_eval)
            else float('nan'),
            original_lane_width=float(production.lane_width_eval),
            selected_lane_width=float(production.lane_width_eval),
        )
        if original_result is not None:
            original_error, _, original_center, _ = original_result
            diagnostics.original_lane_error = float(original_error)
            diagnostics.selected_lane_error = float(original_error)
            diagnostics.lane_error_delta = 0.0
            diagnostics.original_lane_center = float(original_center)
            diagnostics.selected_lane_center = float(original_center)
            diagnostics.lane_center_shift_px = 0.0

        def fallback(reason: str, reset_state: bool = True):
            diagnostics.reason = reason

            if reset_state:
                self.experimental_shadow_confirm_count = 0
                self.experimental_shadow_override_active_state = False

            if np.isfinite(production.right_x_eval):
                self.experimental_shadow_previous_output_right_x = float(
                    production.right_x_eval)
            else:
                self.experimental_shadow_previous_output_right_x = float('nan')

            return original_result, diagnostics

        if not self.get_parameter(
                'experimental_shadow_right_override_enabled').value:
            return fallback('FEATURE_DISABLED')

        if not self.get_parameter('semantic_diagnostics_enabled').value:
            return fallback('SEMANTIC_DIAGNOSTICS_DISABLED')

        if not self.get_parameter('shadow_right_selector_enabled').value:
            return fallback('SHADOW_SELECTOR_DISABLED')

        if original_result is None:
            return fallback('NO_PRODUCTION_RESULT')

        if production.mode != 'BOTH':
            return fallback('PRODUCTION_NOT_BOTH')

        if shadow.status != 'VALID' or shadow.rejection_reason != 'OK':
            return fallback('SHADOW_NOT_VALID')

        if not np.isfinite(shadow.score) or shadow.score < float(
                self.get_parameter('experimental_shadow_min_score').value):
            return fallback('SHADOW_SCORE_TOO_LOW')

        selected = next((
            evaluation for evaluation in shadow.evaluations
            if evaluation.status == 'SELECTED' and
            evaluation.component.component_id == shadow.component_id
        ), None)

        if selected is None:
            return fallback('SELECTED_COMPONENT_MISSING')

        eval_y = float(production.eval_y)
        beige = selected.component

        if (not np.isfinite(eval_y) or
                not beige.y_min <= eval_y <= beige.y_max):
            return fallback('PRODUCTION_EVAL_OUTSIDE_BEIGE_RANGE')

        shadow_target_right_x = float(np.polyval(beige.fit, eval_y))

        if (not np.isfinite(shadow_target_right_x) or
                not 0.0 <= shadow_target_right_x < image_width):
            return fallback('BEIGE_OUT_OF_IMAGE_BOUNDS')

        min_span = float(self.get_parameter(
            'experimental_shadow_min_common_y_span_px').value)

        if selected.common_y_span < min_span:
            return fallback('YELLOW_COMMON_SPAN_TOO_SHORT')

        yellow = selected.yellow_component

        if yellow is None:
            return fallback('NO_YELLOW_REFERENCE')

        if not yellow.y_min <= eval_y <= yellow.y_max:
            return fallback('PRODUCTION_EVAL_OUTSIDE_YELLOW_RANGE')

        yellow_x = float(np.polyval(yellow.fit, eval_y))

        if (not np.isfinite(yellow_x) or
                not 0.0 <= yellow_x < image_width or
                not yellow_x < shadow_target_right_x):
            return fallback('YELLOW_BEIGE_ORDER_INVALID')

        require_neutral = bool(self.get_parameter(
            'experimental_shadow_require_neutral_order').value)

        neutral = selected.neutral_component

        if require_neutral:
            if neutral is None:
                return fallback('NO_ORDERED_NEUTRAL_REFERENCE')

            if selected.neutral_common_y_span < min_span:
                return fallback('NEUTRAL_COMMON_SPAN_TOO_SHORT')

            if not neutral.y_min <= eval_y <= neutral.y_max:
                return fallback('PRODUCTION_EVAL_OUTSIDE_NEUTRAL_RANGE')

            neutral_x = float(np.polyval(neutral.fit, eval_y))

            if (not np.isfinite(neutral_x) or
                    not 0.0 <= neutral_x < image_width or
                    not shadow_target_right_x < neutral_x):
                return fallback('BEIGE_NEUTRAL_ORDER_INVALID')

        max_temporal_delta = float(self.get_parameter(
            'experimental_shadow_max_temporal_delta_px').value)

        if not np.isfinite(selected.temporal_delta):
            return fallback('NO_TEMPORAL_REFERENCE')

        if selected.temporal_delta > max_temporal_delta:
            return fallback('TEMPORAL_DISCONTINUITY')

        left_x = float(production.left_x_eval)
        target_width = shadow_target_right_x - left_x

        expected_width = (
            float(self.get_parameter('default_lane_width_px_640').value) *
            image_width / 640.0
        )

        min_width = expected_width * float(self.get_parameter(
            'experimental_shadow_min_lane_width_ratio').value)

        max_width = expected_width * float(self.get_parameter(
            'experimental_shadow_max_lane_width_ratio').value)

        if (not np.isfinite(left_x) or
                not 0.0 <= left_x < image_width or
                shadow_target_right_x <= left_x or
                target_width <= 0.0 or
                not min_width <= target_width <= max_width):
            return fallback('LANE_GEOMETRY_INVALID')

        # --------------------------------------------------------
        # Stability stage 1:
        # Require N consecutive strict-valid frames before entry.
        # Any strict failure above immediately returns to production.
        # --------------------------------------------------------

        confirm_frames = max(
            1,
            int(self.get_parameter(
                'experimental_shadow_confirm_frames').value),
        )

        if not self.experimental_shadow_override_active_state:
            self.experimental_shadow_confirm_count += 1

            if self.experimental_shadow_confirm_count < confirm_frames:
                reason = (
                    'CONFIRMING_SHADOW_OVERRIDE_'
                    f'{self.experimental_shadow_confirm_count}_OF_'
                    f'{confirm_frames}'
                )
                return fallback(reason, reset_state=False)

            self.experimental_shadow_override_active_state = True
        else:
            self.experimental_shadow_confirm_count = confirm_frames

        # --------------------------------------------------------
        # Stability stage 2:
        # Move toward the strict-valid beige target by at most
        # experimental_shadow_slew_rate_px_per_frame each frame.
        # This applies only while strict conditions remain valid.
        # --------------------------------------------------------

        previous_output = self.experimental_shadow_previous_output_right_x

        if not np.isfinite(previous_output):
            previous_output = float(production.right_x_eval)

        slew_rate = max(
            0.0,
            float(self.get_parameter(
                'experimental_shadow_slew_rate_px_per_frame').value),
        )

        if slew_rate > 0.0 and np.isfinite(previous_output):
            delta = shadow_target_right_x - previous_output
            delta = clamp(delta, -slew_rate, slew_rate)
            selected_right_x = previous_output + delta
        else:
            selected_right_x = shadow_target_right_x

        selected_width = selected_right_x - left_x

        # Re-check the actually emitted right boundary after slew limiting.
        if (not np.isfinite(selected_right_x) or
                not 0.0 <= selected_right_x < image_width or
                selected_right_x <= left_x or
                selected_width <= 0.0):
            return fallback('SLEW_OUTPUT_GEOMETRY_INVALID')

        _, confidence, _, lookahead_y = original_result

        selected_center = 0.5 * (left_x + selected_right_x)

        selected_error = clamp(
            (selected_center - image_width / 2.0) /
            (image_width / 2.0),
            -1.0, 1.0)

        diagnostics.active = True
        diagnostics.reason = 'OK'
        diagnostics.selected_right_x = selected_right_x
        diagnostics.right_shift_px = (
            selected_right_x - diagnostics.original_right_x)
        diagnostics.selected_lane_error = selected_error
        diagnostics.lane_error_delta = (
            selected_error - diagnostics.original_lane_error)
        diagnostics.selected_lane_center = selected_center
        diagnostics.lane_center_shift_px = (
            selected_center - diagnostics.original_lane_center)
        diagnostics.selected_lane_width = selected_width

        # Keep the stored experimental fit consistent with the
        # slew-limited x at production eval_y by lateral translation only.
        selected_fit = beige.fit.copy()
        selected_fit[-1] += (
            selected_right_x - shadow_target_right_x)

        diagnostics.selected_right_fit = selected_fit
        diagnostics.selected_right_y_min = float(beige.y_min)
        diagnostics.selected_right_y_max = float(beige.y_max)

        self.experimental_shadow_previous_output_right_x = (
            selected_right_x)

        return (
            (selected_error, confidence, selected_center, lookahead_y),
            diagnostics,
        )

    def select_shadow_right(
        self, semantic: SemanticDiagnostics, image_width: int
    ) -> ShadowRightDiagnostics:
        """Select beige without reading or mutating production state."""
        result = ShadowRightDiagnostics(
            candidate_count=len(semantic.beige_components))
        if not semantic.beige_components:
            self.clear_shadow_continuity()
            return result
        if not semantic.yellow_components:
            result.rejection_reason = 'NO_YELLOW_REFERENCE'
            result.evaluations = [
                ShadowCandidateEvaluation(
                    component=component,
                    rejection_reason='NO_YELLOW_REFERENCE')
                for component in semantic.beige_components
            ]
            self.clear_shadow_continuity()
            return result

        evaluations = [
            self.evaluate_shadow_beige(
                beige, semantic.yellow_components,
                semantic.neutral_white_components, image_width)
            for beige in semantic.beige_components
        ]
        result.evaluations = evaluations
        eligible = [
            evaluation for evaluation in evaluations
            if evaluation.status == 'CANDIDATE' and np.isfinite(evaluation.score)
        ]
        if not eligible:
            reasons = []
            for evaluation in evaluations:
                if evaluation.rejection_reason not in reasons:
                    reasons.append(evaluation.rejection_reason)
            result.rejection_reason = '+'.join(reasons) if reasons else 'LOW_SCORE'
            self.clear_shadow_continuity()
            return result

        selected = max(
            eligible,
            key=lambda evaluation: (
                evaluation.score,
                evaluation.y_max - evaluation.y_min,
                evaluation.component.pixel_count,
            ),
        )
        result.score = float(selected.score)
        min_score = float(self.get_parameter('shadow_min_score').value)
        if selected.score < min_score:
            selected.status = 'REJECTED'
            selected.rejection_reason = 'LOW_SCORE'
            result.rejection_reason = 'LOW_SCORE'
            self.clear_shadow_continuity()
            return result

        selected.status = 'SELECTED'
        selected.rejection_reason = 'OK'
        result.right_x = float(selected.x)
        result.eval_y = float(selected.eval_y)
        result.status = 'VALID'
        result.rejection_reason = 'OK'
        result.component_id = int(selected.component.component_id)
        result.y_min = float(selected.y_min)
        result.y_max = float(selected.y_max)
        result.lane_width = float(selected.lane_width)
        result.temporal_delta = float(selected.temporal_delta)
        self.shadow_previous_right_x = result.right_x
        self.shadow_previous_eval_y = result.eval_y
        return result

    def evaluate_shadow_beige(
        self, beige: SemanticComponent,
        yellow_components: List[SemanticComponent],
        neutral_components: List[SemanticComponent], image_width: int
    ) -> ShadowCandidateEvaluation:
        evaluation = ShadowCandidateEvaluation(component=beige)
        min_span = int(
            self.get_parameter('shadow_min_common_y_span_px').value)
        sample_count = max(
            3, int(self.get_parameter('shadow_sample_count').value))
        min_valid_ratio = float(
            self.get_parameter('shadow_min_valid_sample_ratio').value)
        pair_options = []
        saw_common_range = False
        saw_in_bounds = False
        saw_right_order = False

        for yellow in yellow_components:
            common_y_min = max(beige.y_min, yellow.y_min)
            common_y_max = min(beige.y_max, yellow.y_max)
            common_span = common_y_max - common_y_min
            if common_span < min_span:
                continue
            saw_common_range = True
            sample_y = np.linspace(
                common_y_min, common_y_max, sample_count, dtype=np.float64)
            beige_x = np.polyval(beige.fit, sample_y)
            yellow_x = np.polyval(yellow.fit, sample_y)
            finite = np.isfinite(beige_x) & np.isfinite(yellow_x)
            in_bounds = (
                finite & (beige_x >= 0.0) & (beige_x < image_width) &
                (yellow_x >= 0.0) & (yellow_x < image_width)
            )
            bounds_fraction = float(np.mean(in_bounds))
            if bounds_fraction < min_valid_ratio:
                continue
            saw_in_bounds = True
            ordered = in_bounds & (beige_x > yellow_x)
            order_fraction = float(np.mean(ordered))
            if order_fraction < min_valid_ratio:
                continue
            saw_right_order = True
            widths = beige_x[ordered] - yellow_x[ordered]
            lane_width = float(np.median(widths))
            expected_width = (
                float(self.get_parameter('default_lane_width_px_640').value) *
                image_width / 640.0
            )
            tolerance = max(
                1.0, expected_width * float(self.get_parameter(
                    'shadow_expected_width_tolerance_ratio').value))
            geometry_score = math.exp(
                -abs(lane_width - expected_width) / tolerance)
            yellow_position = float(np.median(yellow_x[ordered]))
            pair_options.append((
                order_fraction + 0.5 * geometry_score +
                float(self.get_parameter(
                    'shadow_yellow_position_tiebreak_weight').value) *
                yellow_position / max(1.0, image_width),
                yellow, sample_y, beige_x, yellow_x, bounds_fraction,
                order_fraction, lane_width, geometry_score,
                common_y_min, common_y_max,
            ))

        if not pair_options:
            if not saw_common_range:
                evaluation.rejection_reason = 'NO_COMMON_Y_RANGE'
            elif not saw_in_bounds:
                evaluation.rejection_reason = 'OUT_OF_IMAGE_BOUNDS'
            elif not saw_right_order:
                evaluation.rejection_reason = 'LEFT_OF_YELLOW'
            else:
                evaluation.rejection_reason = 'LANE_GEOMETRY_INVALID'
            return evaluation

        (_, yellow, sample_y, beige_x, yellow_x, bounds_fraction,
         order_fraction, lane_width, geometry_score, common_y_min,
         common_y_max) = max(pair_options, key=lambda item: item[0])
        eval_y = float(np.median(sample_y))
        selected_x = float(np.polyval(beige.fit, eval_y))
        if not np.isfinite(selected_x) or not 0.0 <= selected_x < image_width:
            evaluation.rejection_reason = 'OUT_OF_IMAGE_BOUNDS'
            return evaluation

        neutral_score = float(self.get_parameter(
            'shadow_neutral_missing_score').value)
        neutral_order_found = False
        neutral_comparison_found = False
        best_neutral = None
        best_neutral_fraction = 0.0
        best_neutral_span = 0.0
        for neutral in neutral_components:
            triple_y_min = max(common_y_min, neutral.y_min)
            triple_y_max = min(common_y_max, neutral.y_max)
            if triple_y_max - triple_y_min < min_span:
                continue
            triple_y = np.linspace(
                triple_y_min, triple_y_max, sample_count, dtype=np.float64)
            triple_yellow_x = np.polyval(yellow.fit, triple_y)
            triple_beige_x = np.polyval(beige.fit, triple_y)
            neutral_x = np.polyval(neutral.fit, triple_y)
            valid = (
                np.isfinite(triple_yellow_x) & np.isfinite(triple_beige_x) &
                np.isfinite(neutral_x) &
                (triple_yellow_x >= 0.0) & (triple_yellow_x < image_width) &
                (triple_beige_x >= 0.0) & (triple_beige_x < image_width) &
                (neutral_x >= 0.0) & (neutral_x < image_width)
            )
            if float(np.mean(valid)) < min_valid_ratio:
                continue
            neutral_comparison_found = True
            ordered = (
                valid & (triple_yellow_x < triple_beige_x) &
                (triple_beige_x < neutral_x)
            )
            ordered_fraction = float(np.mean(ordered))
            if ordered_fraction >= min_valid_ratio:
                neutral_order_found = True
                neutral_score = max(neutral_score, ordered_fraction)
                if ordered_fraction > best_neutral_fraction:
                    best_neutral = neutral
                    best_neutral_fraction = ordered_fraction
                    best_neutral_span = float(triple_y_max - triple_y_min)

        if neutral_comparison_found and not neutral_order_found:
            neutral_score = float(self.get_parameter(
                'shadow_neutral_contradiction_score').value)

        span_score = min(
            1.0, (common_y_max - common_y_min) / max(1.0, 3.0 * min_span))
        semantic_score = min(
            1.0, beige.pixel_count / max(
                1.0, 4.0 * float(self.get_parameter(
                    'diag_min_component_pixels').value)))
        temporal_delta = float('nan')
        temporal_score = 1.0
        if np.isfinite(self.shadow_previous_right_x):
            temporal_delta = abs(selected_x - self.shadow_previous_right_x)
            temporal_scale = max(
                1.0, float(self.get_parameter(
                    'shadow_max_temporal_jump_px').value))
            temporal_score = math.exp(-temporal_delta / temporal_scale)

        normalized_parts = {
            'semantic': semantic_score,
            'yellow_order': order_fraction,
            'neutral_order': neutral_score,
            'geometry': geometry_score,
            'y_span': span_score,
            'bounds': bounds_fraction,
            'temporal': temporal_score,
        }
        weight_names = {
            'semantic': 'shadow_semantic_weight',
            'yellow_order': 'shadow_yellow_order_weight',
            'neutral_order': 'shadow_neutral_order_weight',
            'geometry': 'shadow_geometry_weight',
            'y_span': 'shadow_y_span_weight',
            'bounds': 'shadow_bounds_weight',
            'temporal': 'shadow_temporal_weight',
        }
        weighted_parts = {
            name: float(self.get_parameter(weight_names[name]).value) * value
            for name, value in normalized_parts.items()
        }
        total_weight = sum(
            float(self.get_parameter(parameter).value)
            for parameter in weight_names.values())
        evaluation.score = sum(weighted_parts.values()) / max(1e-6, total_weight)
        evaluation.status = 'CANDIDATE'
        evaluation.rejection_reason = (
            'RIGHT_OF_NEUTRAL_WHITE'
            if neutral_comparison_found and not neutral_order_found else 'OK')
        evaluation.eval_y = eval_y
        evaluation.x = selected_x
        evaluation.y_min = float(common_y_min)
        evaluation.y_max = float(common_y_max)
        evaluation.lane_width = lane_width
        evaluation.temporal_delta = temporal_delta
        evaluation.sample_y = sample_y
        evaluation.score_parts = weighted_parts
        evaluation.yellow_component = yellow
        evaluation.neutral_component = best_neutral
        evaluation.yellow_order_fraction = order_fraction
        evaluation.neutral_order_fraction = best_neutral_fraction
        evaluation.bounds_fraction = bounds_fraction
        evaluation.common_y_span = float(common_y_max - common_y_min)
        evaluation.neutral_common_y_span = best_neutral_span
        return evaluation

    @staticmethod
    def draw_fit(
        image: np.ndarray, component: SemanticComponent,
        color: Tuple[int, int, int], thickness: int
    ):
        sample_y = np.arange(
            component.y_min, component.y_max + 1, dtype=np.float64)
        sample_x = np.polyval(component.fit, sample_y)
        valid = (
            np.isfinite(sample_x) &
            (sample_x >= 0) &
            (sample_x < image.shape[1])
        )
        if not np.any(valid):
            return
        points = np.column_stack(
            (sample_x[valid].astype(np.int32),
             sample_y[valid].astype(np.int32)))
        cv2.polylines(image, [points], False, color, thickness)

    def publish_identity_debug_image(
        self, source_msg: CompressedImage, image: np.ndarray,
        semantic: SemanticDiagnostics, fit_snapshot: ProductionFitSnapshot,
        production: LaneDiagnostics
    ):
        debug = image.copy()
        colors = {
            'yellow': (0, 255, 255),
            'beige': (0, 150, 255),
            'neutral_white': (255, 255, 0),
        }
        for component in semantic.yellow_components:
            self.draw_fit(debug, component, colors['yellow'], 3)
        for component in semantic.beige_components:
            self.draw_fit(debug, component, colors['beige'], 3)
        for component in semantic.neutral_white_components:
            self.draw_fit(debug, component, colors['neutral_white'], 3)

        def draw_production_fit(fit, y_min, y_max, color):
            if fit is None or not np.isfinite(y_min) or not np.isfinite(y_max):
                return
            component = SemanticComponent(
                fit=fit,
                pixel_count=0,
                y_min=int(y_min),
                y_max=int(y_max),
            )
            self.draw_fit(debug, component, color, 4)

        draw_production_fit(
            fit_snapshot.left_fit,
            production.left_y_min,
            production.left_y_max,
            (0, 255, 0),
        )
        draw_production_fit(
            fit_snapshot.right_fit,
            production.right_y_min,
            production.right_y_max,
            (0, 0, 255),
        )

        eval_y = int(round(semantic.eval_y))
        if 0 <= eval_y < debug.shape[0]:
            cv2.line(
                debug, (0, eval_y), (debug.shape[1] - 1, eval_y),
                (255, 0, 255), 1)
            candidates = (
                (semantic.yellow_candidate_x, colors['yellow']),
                (semantic.beige_candidate_x, colors['beige']),
                (semantic.neutral_white_candidate_x,
                 colors['neutral_white']),
            )
            for candidate_x, color in candidates:
                if np.isfinite(candidate_x):
                    cv2.circle(
                        debug, (int(round(candidate_x)), eval_y),
                        7, color, -1)

        legend = [
            ('YELLOW candidate', colors['yellow']),
            ('BEIGE candidate', colors['beige']),
            ('NEUTRAL WHITE candidate', colors['neutral_white']),
            ('production LEFT fit', (0, 255, 0)),
            ('production RIGHT fit', (0, 0, 255)),
            (f'eval_y={semantic.eval_y:.0f} '
             f'right={semantic.right_identity}', (255, 0, 255)),
        ]
        for index, (label, color) in enumerate(legend):
            y = 24 + index * 23
            cv2.putText(
                debug, label, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 0, 0), 4)
            cv2.putText(
                debug, label, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 2)

        ok, encoded = cv2.imencode(
            '.jpg', debug, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            out = CompressedImage()
            out.header = source_msg.header
            out.format = 'jpeg'
            out.data = encoded.tobytes()
            self.identity_debug_pub.publish(out)

    def publish_shadow_debug_image(
        self, source_msg: CompressedImage, image: np.ndarray,
        semantic: SemanticDiagnostics, shadow: ShadowRightDiagnostics,
        fit_snapshot: ProductionFitSnapshot, production: LaneDiagnostics
    ):
        debug = image.copy()

        for component in semantic.yellow_components:
            self.draw_fit(debug, component, (0, 255, 255), 2)
        for component in semantic.neutral_white_components:
            self.draw_fit(debug, component, (255, 255, 0), 2)

        evaluated_ids = {
            evaluation.component.component_id: evaluation
            for evaluation in shadow.evaluations
        }
        for component in semantic.beige_components:
            evaluation = evaluated_ids.get(component.component_id)
            selected = evaluation is not None and evaluation.status == 'SELECTED'
            color = (0, 165, 255) if selected else (180, 0, 180)
            self.draw_fit(debug, component, color, 5 if selected else 2)
            if evaluation is not None and np.isfinite(evaluation.x):
                point = (int(round(evaluation.x)), int(round(evaluation.eval_y)))
                if 0 <= point[0] < debug.shape[1] and 0 <= point[1] < debug.shape[0]:
                    cv2.circle(debug, point, 7 if selected else 5, color, -1)
                    cv2.putText(
                        debug,
                        f'id={component.component_id} s={evaluation.score:.2f}',
                        (max(0, point[0] - 55), max(18, point[1] - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
            if evaluation is not None:
                for sampled_y in evaluation.sample_y:
                    y = int(round(float(sampled_y)))
                    if 0 <= y < debug.shape[0]:
                        cv2.line(debug, (0, y), (debug.shape[1] - 1, y),
                                 (80, 80, 80), 1)

        def draw_production_fit(fit, y_min, y_max, color):
            if fit is None or not np.isfinite(y_min) or not np.isfinite(y_max):
                return
            component = SemanticComponent(
                fit=fit, pixel_count=0, y_min=int(y_min), y_max=int(y_max))
            self.draw_fit(debug, component, color, 4)

        draw_production_fit(
            fit_snapshot.left_fit, production.left_y_min,
            production.left_y_max, (0, 255, 0))
        draw_production_fit(
            fit_snapshot.right_fit, production.right_y_min,
            production.right_y_max, (0, 0, 255))

        score_text = (
            f'{shadow.score:.3f}' if np.isfinite(shadow.score) else 'NaN')
        legend = [
            ('production LEFT', (0, 255, 0)),
            ('production RIGHT', (0, 0, 255)),
            ('yellow candidates', (0, 255, 255)),
            ('neutral-white candidates', (255, 255, 0)),
            ('rejected beige', (180, 0, 180)),
            ('selected shadow right', (0, 165, 255)),
            (f'{shadow.status} score={score_text} '
             f'reason={shadow.rejection_reason}', (255, 255, 255)),
        ]
        for index, (label, color) in enumerate(legend):
            y = 24 + index * 23
            cv2.putText(debug, label, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (0, 0, 0), 4)
            cv2.putText(debug, label, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, color, 2)

        ok, encoded = cv2.imencode(
            '.jpg', debug, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            out = CompressedImage()
            out.header = source_msg.header
            out.format = 'jpeg'
            out.data = encoded.tobytes()
            self.shadow_debug_pub.publish(out)

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
        self, binary: np.ndarray, diagnostics: Optional[LaneDiagnostics] = None,
        fit_snapshot: Optional[ProductionFitSnapshot] = None,
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

                # 영상 끝에서 시작한 급경사 경계는 한 window 안에서 수평으로
                # margin보다 멀리 이동할 수 있다. 끝 15px에 실제 픽셀이 있는
                # 후보만 같은 y 구간에서 안쪽으로 재중심화해 조각을 이어 간다.
                def refine_edge_candidate(mask, current, other_available, other_current):
                    if (not np.any(mask) or
                            not np.any(nonzero_x[mask] >= w - 15)):
                        return mask, current

                    refined = mask.copy()
                    for _ in range(8):
                        good = refined.nonzero()[0]
                        if len(good) <= minpix:
                            break
                        new_current = int(np.mean(nonzero_x[good]))
                        expanded = (
                            in_window_y &
                            (nonzero_x >= new_current - margin) &
                            (nonzero_x < new_current + margin)
                        )
                        if other_available:
                            expanded &= (
                                np.abs(nonzero_x - new_current) <
                                np.abs(nonzero_x - other_current)
                            )
                        updated = refined | expanded
                        current = new_current
                        if np.array_equal(updated, refined):
                            break
                        refined = updated
                    return refined, current

                if left_available:
                    left_mask, left_current = refine_edge_candidate(
                        left_mask, left_current, right_available, right_current)
                if right_available:
                    right_mask, right_current = refine_edge_candidate(
                        right_mask, right_current, left_available, left_current)

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

        # 화면 오른쪽 끝으로 나가는 경계는 관측 y 길이가 짧아 histogram
        # peak가 일반 threshold 바로 아래로 내려갈 수 있다. 마지막 15px에
        # 실제 지지가 있는 peak만 더 약한 seed threshold로 시작한다.
        # 이후에는 기존 min_lane_pixels, pair geometry, observed y-range
        # 검사를 모두 통과해야 하므로 이 단계만으로 차선이 채택되지는 않는다.
        edge_start = max(midpoint, w - 15)
        if edge_start < w:
            edge_peak = int(np.argmax(histogram[edge_start:]) + edge_start)
            edge_peak_threshold = max(3.0, 0.4 * peak_threshold)
            if (histogram[edge_peak] >= edge_peak_threshold and
                    all(abs(edge_peak - seed) >= max(1, margin // 2)
                        for seed in seeds)):
                seeds.append(edge_peak)

        eval_y = int(h * float(self.get_parameter('lookahead_y_ratio').value))
        expected_width = (
            float(self.get_parameter('default_lane_width_px_640').value) * w / 640.0)
        adaptive_max_width = (
            float(self.get_parameter('adaptive_max_lane_width_ratio').value) *
            expected_width)

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
                (widths <= adaptive_max_width)
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
                    width_error = abs(rx - lx - expected_width) / max(
                        1.0, expected_width)
                    center_error = abs(
                        0.5 * (lx + rx) - midpoint) / max(1.0, midpoint)
                    score = (width_error + 0.5 * center_error,
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
        if fit_snapshot is not None:
            fit_snapshot.left_fit = left_fit
            fit_snapshot.right_fit = right_fit

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
        if fit_snapshot is not None:
            fit_snapshot.eval_y = float(lookahead_y)
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
            # Pair selection을 통과한 adaptive 결과에는 공통 관측 범위에서
            # 검증한 상한을 유지한다. pair가 없던 기존 fallback은 1.4배를
            # 그대로 사용한다.
            pair_max_width = (
                adaptive_max_width if best_score is not None
                else 1.4 * lane_width)
            if right_x - left_x < min_lane_width:
                marker_x = 0.5 * (left_x + right_x)
                # 정확히 영상 중앙이면 왼쪽 표시로 처리한다.
                if marker_x <= w * 0.5:
                    left_x, right_x = marker_x, None
                else:
                    left_x, right_x = None, marker_x
            elif not 0.6 * lane_width <= right_x - left_x <= pair_max_width:
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
