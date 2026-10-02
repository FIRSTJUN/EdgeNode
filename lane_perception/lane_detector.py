import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # ============================================================
        # Camera topic
        # ============================================================

        self.declare_parameter(
            'camera_topic',
            '/camera/image/compressed'
        )

        camera_topic = self.get_parameter(
            'camera_topic'
        ).value

        self.frame_count = 0

        # ============================================================
        # Subscriber
        # ============================================================

        self.camera_sub = self.create_subscription(
            CompressedImage,
            camera_topic,
            self.camera_callback,
            qos_profile_sensor_data,
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.lane_error_pub = self.create_publisher(
            Float32,
            '/perception/lane_error',
            10,
        )

        self.lane_confidence_pub = self.create_publisher(
            Float32,
            '/perception/lane_confidence',
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/perception/debug/lane_image',
            10,
        )

        self.debug_mask_pub = self.create_publisher(
            Image,
            '/perception/debug/lane_mask',
            10,
        )

        self.get_logger().info(
            f'Perception node ready: {camera_topic}'
        )

    # ================================================================
    # Camera callback
    # ================================================================

    def camera_callback(
        self,
        msg: CompressedImage,
    ):

        np_arr = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        frame = cv2.imdecode(
            np_arr,
            cv2.IMREAD_COLOR,
        )

        if frame is None:
            self.get_logger().warning(
                'Failed to decode compressed camera image'
            )
            return

        self.frame_count += 1

        if self.frame_count % 100 == 0:

            height, width = frame.shape[:2]

            self.get_logger().info(
                f'Camera OK | '
                f'frames={self.frame_count} '
                f'size={width}x{height}'
            )

        # ============================================================
        # Lane perception
        # ============================================================

        lane_mask = self.make_lane_mask(
            frame
        )

        lane_result = self.detect_lane(
            lane_mask
        )

        debug_frame = self.make_debug_image(
            frame,
            lane_mask,
            lane_result,
        )

        # ============================================================
        # Publish lane result
        # ============================================================

        if lane_result is None:

            self.publish_lane_result(
                0.0,
                0.0,
            )

        else:

            self.publish_lane_result(
                lane_result['error'],
                lane_result['confidence'],
            )

        # ============================================================
        # Publish debug images
        # ============================================================

        self.publish_debug_image(
            debug_frame,
            msg,
        )

        self.publish_debug_mask(
            lane_mask,
            msg,
        )

    # ================================================================
    # Lane mask
    # ================================================================

    def make_lane_mask(
        self,
        frame,
    ):

        height, width = frame.shape[:2]

        # ------------------------------------------------------------
        # White lane
        # ------------------------------------------------------------

        hls = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HLS,
        )

        white_lower = np.array(
            [0, 170, 0],
            dtype=np.uint8,
        )

        white_upper = np.array(
            [255, 255, 140],
            dtype=np.uint8,
        )

        white_mask = cv2.inRange(
            hls,
            white_lower,
            white_upper,
        )

        # ------------------------------------------------------------
        # Yellow lane
        # ------------------------------------------------------------

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )

        yellow_lower = np.array(
            [12, 70, 70],
            dtype=np.uint8,
        )

        yellow_upper = np.array(
            [42, 255, 255],
            dtype=np.uint8,
        )

        yellow_mask = cv2.inRange(
            hsv,
            yellow_lower,
            yellow_upper,
        )

        # ------------------------------------------------------------
        # White + Yellow
        # ------------------------------------------------------------

        lane_mask = cv2.bitwise_or(
            white_mask,
            yellow_mask,
        )

        # ------------------------------------------------------------
        # Noise removal
        # ------------------------------------------------------------

        kernel = np.ones(
            (5, 5),
            dtype=np.uint8,
        )

        lane_mask = cv2.morphologyEx(
            lane_mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        lane_mask = cv2.morphologyEx(
            lane_mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        # ============================================================
        # ROI
        # ============================================================

        roi_mask = np.zeros_like(
            lane_mask
        )

        polygon = np.array(
            [[
                (
                    int(width * 0.02),
                    int(height * 0.98),
                ),
                (
                    int(width * 0.15),
                    int(height * 0.30),
                ),
                (
                    int(width * 0.90),
                    int(height * 0.30),
                ),
                (
                    int(width * 0.98),
                    int(height * 0.98),
                ),
            ]],
            dtype=np.int32,
        )

        cv2.fillPoly(
            roi_mask,
            polygon,
            255,
        )

        lane_mask = cv2.bitwise_and(
            lane_mask,
            roi_mask,
        )

        return lane_mask

    # ================================================================
    # Sliding Window
    # ================================================================

    def detect_lane(
        self,
        binary,
    ):

        height, width = binary.shape

        histogram = np.sum(
            binary[height // 2:, :] > 0,
            axis=0,
        ).astype(
            np.float32
        )

        midpoint = width // 2

        left_base = int(
            np.argmax(
                histogram[:midpoint]
            )
        )

        right_base = int(
            np.argmax(
                histogram[midpoint:]
            )
            + midpoint
        )

        minimum_peak = 5.0

        left_available = (
            histogram[left_base]
            > minimum_peak
        )

        right_available = (
            histogram[right_base]
            > minimum_peak
        )

        if (
            not left_available
            and not right_available
        ):
            return None

        nonzero_y, nonzero_x = (
            binary.nonzero()
        )

        if len(nonzero_x) == 0:
            return None

        # ============================================================
        # Sliding window settings
        # ============================================================

        number_of_windows = 9

        window_height = max(
            1,
            height // number_of_windows,
        )

        margin = max(
            40,
            int(width * 0.08),
        )

        minimum_pixels = 30

        left_current = left_base
        right_current = right_base

        left_lane_indices = []
        right_lane_indices = []

        # ============================================================
        # Search from bottom to top
        # ============================================================

        for window in range(
            number_of_windows
        ):

            y_low = (
                height
                - (window + 1)
                * window_height
            )

            y_high = (
                height
                - window
                * window_height
            )

            # --------------------------------------------------------
            # Left lane
            # --------------------------------------------------------

            if left_available:

                left_indices = np.where(
                    (
                        (nonzero_y >= y_low)
                        &
                        (nonzero_y < y_high)
                        &
                        (
                            nonzero_x
                            >= left_current - margin
                        )
                        &
                        (
                            nonzero_x
                            < left_current + margin
                        )
                    )
                )[0]

                left_lane_indices.append(
                    left_indices
                )

                if (
                    len(left_indices)
                    > minimum_pixels
                ):

                    left_current = int(
                        np.mean(
                            nonzero_x[
                                left_indices
                            ]
                        )
                    )

            # --------------------------------------------------------
            # Right lane
            # --------------------------------------------------------

            if right_available:

                right_indices = np.where(
                    (
                        (nonzero_y >= y_low)
                        &
                        (nonzero_y < y_high)
                        &
                        (
                            nonzero_x
                            >= right_current - margin
                        )
                        &
                        (
                            nonzero_x
                            < right_current + margin
                        )
                    )
                )[0]

                right_lane_indices.append(
                    right_indices
                )

                if (
                    len(right_indices)
                    > minimum_pixels
                ):

                    right_current = int(
                        np.mean(
                            nonzero_x[
                                right_indices
                            ]
                        )
                    )

        # ============================================================
        # Merge indices
        # ============================================================

        if left_lane_indices:

            left_lane_indices = np.concatenate(
                left_lane_indices
            )

        else:

            left_lane_indices = np.array(
                [],
                dtype=np.int64,
            )

        if right_lane_indices:

            right_lane_indices = np.concatenate(
                right_lane_indices
            )

        else:

            right_lane_indices = np.array(
                [],
                dtype=np.int64,
            )

        # ============================================================
        # Polynomial fitting
        # ============================================================

        minimum_lane_pixels = 150

        left_fit = None
        right_fit = None

        if (
            left_available
            and len(left_lane_indices)
            >= minimum_lane_pixels
        ):

            left_fit = np.polyfit(
                nonzero_y[
                    left_lane_indices
                ],
                nonzero_x[
                    left_lane_indices
                ],
                1,
            )

        if (
            right_available
            and len(right_lane_indices)
            >= minimum_lane_pixels
        ):

            right_fit = np.polyfit(
                nonzero_y[
                    right_lane_indices
                ],
                nonzero_x[
                    right_lane_indices
                ],
                1,
            )

        if (
            left_fit is None
            and right_fit is None
        ):
            return None

        # ============================================================
        # Lane center
        # ============================================================

        lookahead_y = int(
            height * 0.80
        )

        left_x = None
        right_x = None

        if left_fit is not None:

            left_x = float(
                np.polyval(
                    left_fit,
                    lookahead_y,
                )
            )

        if right_fit is not None:

            right_x = float(
                np.polyval(
                    right_fit,
                    lookahead_y,
                )
            )

        expected_lane_width = (
            width * 0.4375
        )

        # ------------------------------------------------------------
        # Both lanes detected
        # ------------------------------------------------------------

        if (
            left_x is not None
            and right_x is not None
        ):

            if right_x <= left_x:
                return None

            lane_center = (
                left_x + right_x
            ) / 2.0

            pixel_count = (
                len(left_lane_indices)
                +
                len(right_lane_indices)
            )

            confidence = min(
                1.0,
                pixel_count / 2500.0,
            )

        # ------------------------------------------------------------
        # Only left lane
        # ------------------------------------------------------------

        elif left_x is not None:

            lane_center = (
                left_x
                +
                expected_lane_width / 2.0
            )

            confidence = min(
                0.5,
                len(left_lane_indices)
                / 1800.0,
            )

        # ------------------------------------------------------------
        # Only right lane
        # ------------------------------------------------------------

        else:

            lane_center = (
                right_x
                -
                expected_lane_width / 2.0
            )

            confidence = min(
                0.5,
                len(right_lane_indices)
                / 1800.0,
            )

        image_center = (
            width / 2.0
        )

        lane_error = (
            lane_center
            -
            image_center
        ) / image_center

        lane_error = float(
            np.clip(
                lane_error,
                -1.0,
                1.0,
            )
        )

        return {
            'error': lane_error,
            'confidence': float(
                confidence
            ),
            'lane_center': float(
                lane_center
            ),
            'lookahead_y': int(
                lookahead_y
            ),
            'left_fit': left_fit,
            'right_fit': right_fit,
        }

    # ================================================================
    # Debug image
    # ================================================================

    def make_debug_image(
        self,
        frame,
        lane_mask,
        lane_result,
    ):

        debug = frame.copy()

        height, width = (
            frame.shape[:2]
        )

        # ------------------------------------------------------------
        # Green overlay
        # ------------------------------------------------------------

        overlay = np.zeros_like(
            frame
        )

        overlay[
            lane_mask > 0
        ] = (
            0,
            180,
            0,
        )

        debug = cv2.addWeighted(
            debug,
            1.0,
            overlay,
            0.35,
            0,
        )

        # ------------------------------------------------------------
        # ROI outline
        # ------------------------------------------------------------

        roi_polygon = np.array(
            [[
                (
                    int(width * 0.02),
                    int(height * 0.98),
                ),
                (
                    int(width * 0.15),
                    int(height * 0.30),
                ),
                (
                    int(width * 0.90),
                    int(height * 0.30),
                ),
                (
                    int(width * 0.98),
                    int(height * 0.98),
                ),
            ]],
            dtype=np.int32,
        )

        cv2.polylines(
            debug,
            roi_polygon,
            True,
            (255, 255, 0),
            2,
        )

        # ------------------------------------------------------------
        # Camera center
        # ------------------------------------------------------------

        cv2.line(
            debug,
            (
                width // 2,
                int(height * 0.55),
            ),
            (
                width // 2,
                height - 1,
            ),
            (0, 0, 255),
            2,
        )

        # ============================================================
        # Lane mask preview
        # ============================================================

        preview_width = max(
            1,
            width // 3,
        )

        preview_height = max(
            1,
            height // 3,
        )

        mask_small = cv2.resize(
            lane_mask,
            (
                preview_width,
                preview_height,
            ),
            interpolation=cv2.INTER_NEAREST,
        )

        mask_small = cv2.cvtColor(
            mask_small,
            cv2.COLOR_GRAY2BGR,
        )

        small_h, small_w = (
            mask_small.shape[:2]
        )

        y0 = (
            height - small_h
        )

        y1 = height

        x0 = 0

        x1 = small_w

        debug[
            y0:y1,
            x0:x1,
        ] = mask_small

        cv2.rectangle(
            debug,
            (
                x0,
                y0,
            ),
            (
                x1 - 1,
                y1 - 1,
            ),
            (0, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            'LANE MASK',
            (
                10,
                y0 + 25,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # ============================================================
        # Lane lost
        # ============================================================

        if lane_result is None:

            cv2.putText(
                debug,
                'LANE LOST',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

            return debug

        # ============================================================
        # Draw polynomial lanes
        # ============================================================

        plot_y = np.linspace(
            int(height * 0.55),
            height - 1,
            80,
        )

        left_fit = lane_result[
            'left_fit'
        ]

        right_fit = lane_result[
            'right_fit'
        ]

        if left_fit is not None:

            left_x = np.polyval(
                left_fit,
                plot_y,
            )

            valid = (
                (left_x >= 0)
                &
                (left_x < width)
            )

            left_points = np.column_stack(
                (
                    left_x[valid],
                    plot_y[valid],
                )
            ).astype(
                np.int32
            )

            if len(left_points) > 1:

                cv2.polylines(
                    debug,
                    [left_points],
                    False,
                    (255, 0, 0),
                    5,
                )

        if right_fit is not None:

            right_x = np.polyval(
                right_fit,
                plot_y,
            )

            valid = (
                (right_x >= 0)
                &
                (right_x < width)
            )

            right_points = np.column_stack(
                (
                    right_x[valid],
                    plot_y[valid],
                )
            ).astype(
                np.int32
            )

            if len(right_points) > 1:

                cv2.polylines(
                    debug,
                    [right_points],
                    False,
                    (255, 0, 0),
                    5,
                )

        # ============================================================
        # Lane center
        # ============================================================

        lane_center = int(
            lane_result[
                'lane_center'
            ]
        )

        lookahead_y = int(
            lane_result[
                'lookahead_y'
            ]
        )

        cv2.circle(
            debug,
            (
                lane_center,
                lookahead_y,
            ),
            8,
            (0, 255, 0),
            -1,
        )

        cv2.line(
            debug,
            (
                width // 2,
                lookahead_y,
            ),
            (
                lane_center,
                lookahead_y,
            ),
            (255, 0, 255),
            4,
        )

        # ============================================================
        # Text
        # ============================================================

        lane_error = lane_result[
            'error'
        ]

        confidence = lane_result[
            'confidence'
        ]

        cv2.putText(
            debug,
            f'error={lane_error:+.3f}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            debug,
            f'confidence={confidence:.2f}',
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        return debug

    # ================================================================
    # Lane result publisher
    # ================================================================

    def publish_lane_result(
        self,
        error,
        confidence,
    ):

        error_msg = Float32()

        error_msg.data = float(
            error
        )

        confidence_msg = Float32()

        confidence_msg.data = float(
            confidence
        )

        self.lane_error_pub.publish(
            error_msg
        )

        self.lane_confidence_pub.publish(
            confidence_msg
        )

    # ================================================================
    # Debug image publisher
    # ================================================================

    def publish_debug_image(
        self,
        frame,
        source_msg,
    ):

        frame = np.ascontiguousarray(
            frame
        )

        msg = Image()

        msg.header = (
            source_msg.header
        )

        msg.height = (
            frame.shape[0]
        )

        msg.width = (
            frame.shape[1]
        )

        msg.encoding = (
            'bgr8'
        )

        msg.is_bigendian = 0

        msg.step = (
            frame.shape[1] * 3
        )

        msg.data = (
            frame.tobytes()
        )

        self.debug_image_pub.publish(
            msg
        )

    # ================================================================
    # Debug mask publisher
    # ================================================================

    def publish_debug_mask(
        self,
        mask,
        source_msg,
    ):

        mask = np.ascontiguousarray(
            mask
        )

        msg = Image()

        msg.header = (
            source_msg.header
        )

        msg.height = (
            mask.shape[0]
        )

        msg.width = (
            mask.shape[1]
        )

        msg.encoding = (
            'mono8'
        )

        msg.is_bigendian = 0

        msg.step = (
            mask.shape[1]
        )

        msg.data = (
            mask.tobytes()
        )

        self.debug_mask_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = PerceptionNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()