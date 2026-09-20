import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, String
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus


class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        # 가장 최근에 받은 값 저장
        self.lane_error = float('nan')
        self.lane_confidence = float('nan')
        # 미수신 상태를 실제 검출 실패(NONE, 픽셀 수 0)와 구분한다.
        self.left_pixel_count = -1
        self.right_pixel_count = -1
        self.lane_detection_mode = 'UNKNOWN'
        self.lane_failure_reason = 'UNKNOWN'
        self.left_x_eval = float('nan')
        self.right_x_eval = float('nan')
        self.lane_width_eval = float('nan')
        self.eval_y = float('nan')
        self.left_y_min = float('nan')
        self.left_y_max = float('nan')
        self.right_y_min = float('nan')
        self.right_y_max = float('nan')
        self.left_eval_in_range = 'UNKNOWN'
        self.right_eval_in_range = 'UNKNOWN'

        self.target_error = float('nan')
        self.target_speed = float('nan')
        self.planning_state = 'UNKNOWN'

        self.current_speed_kmh = float('nan')

        self.front_steer = float('nan')
        self.accel = float('nan')
        self.brake = float('nan')
        self.longl_cmd_type = -1

        # 로그 저장 폴더
        log_dir = '/workspace/logs'
        os.makedirs(log_dir, exist_ok=True)

        # 실행할 때마다 새 CSV 생성
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(
            log_dir,
            f'run_{timestamp}.csv'
        )

        self.csv_file = open(
            self.log_path,
            'w',
            newline='',
            encoding='utf-8'
        )

        self.writer = csv.writer(self.csv_file)

        # CSV 헤더
        self.writer.writerow([
            'time_sec',
            'lane_error',
            'lane_confidence',
            'target_error',
            'target_speed_kmh',
            'planning_state',
            'current_speed_kmh',
            'front_steer',
            'accel',
            'brake',
            'longl_cmd_type',
            'left_pixel_count',
            'right_pixel_count',
            'lane_detection_mode',
            'lane_failure_reason',
            'left_x_eval',
            'right_x_eval',
            'lane_width_eval',
            'eval_y',
            'left_y_min',
            'left_y_max',
            'right_y_min',
            'right_y_max',
            'left_eval_in_range',
            'right_eval_in_range',
        ])

        # 시작 시간
        self.start_time = self.get_clock().now()

        # ---------------------------
        # Perception
        # ---------------------------
        self.create_subscription(
            Float32,
            '/perception/lane_error',
            self.lane_error_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/perception/lane_confidence',
            self.lane_confidence_callback,
            10
        )

        self.create_subscription(
            Int32, '/perception/left_pixel_count', self.left_pixel_count_callback, 10)
        self.create_subscription(
            Int32, '/perception/right_pixel_count', self.right_pixel_count_callback, 10)
        self.create_subscription(
            String, '/perception/lane_detection_mode', self.lane_detection_mode_callback, 10)
        self.create_subscription(
            String, '/perception/lane_failure_reason', self.lane_failure_reason_callback, 10)
        self.create_subscription(
            Float32, '/perception/left_x_eval', self.left_x_eval_callback, 10)
        self.create_subscription(
            Float32, '/perception/right_x_eval', self.right_x_eval_callback, 10)
        self.create_subscription(
            Float32, '/perception/lane_width_eval', self.lane_width_eval_callback, 10)
        self.create_subscription(
            Float32, '/perception/eval_y', self.eval_y_callback, 10)
        self.create_subscription(
            Float32, '/perception/left_y_min', self.left_y_min_callback, 10)
        self.create_subscription(
            Float32, '/perception/left_y_max', self.left_y_max_callback, 10)
        self.create_subscription(
            Float32, '/perception/right_y_min', self.right_y_min_callback, 10)
        self.create_subscription(
            Float32, '/perception/right_y_max', self.right_y_max_callback, 10)
        self.create_subscription(
            String, '/perception/left_eval_in_range', self.left_eval_in_range_callback, 10)
        self.create_subscription(
            String, '/perception/right_eval_in_range', self.right_eval_in_range_callback, 10)

        # ---------------------------
        # Planning
        # ---------------------------
        self.create_subscription(
            Float32,
            '/planning/target_error',
            self.target_error_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/planning/target_speed',
            self.target_speed_callback,
            10
        )

        self.create_subscription(
            String,
            '/planning/state',
            self.planning_state_callback,
            10
        )

        # ---------------------------
        # Vehicle status
        # ---------------------------
        self.create_subscription(
            EgoVehicleStatus,
            '/ego_vehicle_status',
            self.ego_status_callback,
            10
        )

        # ---------------------------
        # Control
        # ---------------------------
        self.create_subscription(
            CtrlCmd,
            '/ctrl_cmd',
            self.ctrl_cmd_callback,
            10
        )

        # 20 Hz = 0.05초마다 한 줄 저장
        self.create_timer(
            0.05,
            self.write_log
        )

        self.get_logger().info(
            f'Logger started: {self.log_path}'
        )

    # =========================
    # Callbacks
    # =========================

    def lane_error_callback(self, msg):
        self.lane_error = float(msg.data)

    def lane_confidence_callback(self, msg):
        self.lane_confidence = float(msg.data)

    def left_pixel_count_callback(self, msg):
        self.left_pixel_count = int(msg.data)

    def right_pixel_count_callback(self, msg):
        self.right_pixel_count = int(msg.data)

    def lane_detection_mode_callback(self, msg):
        self.lane_detection_mode = msg.data

    def lane_failure_reason_callback(self, msg):
        self.lane_failure_reason = msg.data

    def left_x_eval_callback(self, msg):
        self.left_x_eval = float(msg.data)

    def right_x_eval_callback(self, msg):
        self.right_x_eval = float(msg.data)

    def lane_width_eval_callback(self, msg):
        self.lane_width_eval = float(msg.data)

    def eval_y_callback(self, msg):
        self.eval_y = float(msg.data)

    def left_y_min_callback(self, msg):
        self.left_y_min = float(msg.data)

    def left_y_max_callback(self, msg):
        self.left_y_max = float(msg.data)

    def right_y_min_callback(self, msg):
        self.right_y_min = float(msg.data)

    def right_y_max_callback(self, msg):
        self.right_y_max = float(msg.data)

    def left_eval_in_range_callback(self, msg):
        self.left_eval_in_range = msg.data

    def right_eval_in_range_callback(self, msg):
        self.right_eval_in_range = msg.data

    def target_error_callback(self, msg):
        self.target_error = float(msg.data)

    def target_speed_callback(self, msg):
        self.target_speed = float(msg.data)

    def planning_state_callback(self, msg):
        self.planning_state = msg.data

    def ego_status_callback(self, msg):

        vx = float(msg.velocity.x)
        vy = float(msg.velocity.y)
        vz = float(msg.velocity.z)

        speed_mps = math.sqrt(
            vx * vx +
            vy * vy +
            vz * vz
        )

        self.current_speed_kmh = speed_mps * 3.6

    def ctrl_cmd_callback(self, msg):

        self.front_steer = float(msg.front_steer)
        self.accel = float(msg.accel)
        self.brake = float(msg.brake)
        self.longl_cmd_type = int(msg.longl_cmd_type)

    # =========================
    # CSV Logger
    # =========================

    def write_log(self):

        now = self.get_clock().now()

        elapsed = (
            now - self.start_time
        ).nanoseconds / 1e9

        self.writer.writerow([
            round(elapsed, 3),
            self.lane_error,
            self.lane_confidence,
            self.target_error,
            self.target_speed,
            self.planning_state,
            self.current_speed_kmh,
            self.front_steer,
            self.accel,
            self.brake,
            self.longl_cmd_type,
            self.left_pixel_count,
            self.right_pixel_count,
            self.lane_detection_mode,
            self.lane_failure_reason,
            self.left_x_eval,
            self.right_x_eval,
            self.lane_width_eval,
            self.eval_y,
            self.left_y_min,
            self.left_y_max,
            self.right_y_min,
            self.right_y_max,
            self.left_eval_in_range,
            self.right_eval_in_range,
        ])

        # 주행 중 강제 종료돼도 로그 손실을 줄이기 위해 flush
        self.csv_file.flush()

    def close_log(self):

        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

            self.get_logger().info(
                f'Logger saved: {self.log_path}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = LoggerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
