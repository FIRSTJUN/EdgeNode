"""Save sampled camera JPEG payloads without decoding or re-encoding them."""

from datetime import datetime
import math
from pathlib import Path
import time

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class TrafficLightImageCollector(Node):
    """Sample incoming frames using a monotonic clock; never repeat a stale frame."""

    def __init__(self):
        super().__init__('traffic_light_image_collector')
        self.saved_count = 0
        self._sequence = 0
        self._last_saved_at = None
        self._warned_invalid = False

        fixed = ParameterDescriptor(read_only=True)
        self.declare_parameter('camera_topic', '/camera/image/compressed', fixed)
        self.declare_parameter(
            'output_dir', '/workspace/dataset/traffic_light/raw/images', fixed)
        self.declare_parameter('save_interval_sec', 0.5)
        interval = self.get_parameter('save_interval_sec').value
        if not self._valid_interval(interval):
            raise ValueError('save_interval_sec must be finite and greater than zero')
        self.add_on_set_parameters_callback(self._validate_parameters)

        self.output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        topic = self.get_parameter('camera_topic').value
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            CompressedImage, topic, self._on_image, qos)
        self.get_logger().info(
            f'Collecting {topic} -> {self.output_dir.resolve()} | '
            f'interval={interval}s | JPEG payloads are saved unchanged')

    @staticmethod
    def _valid_interval(value):
        return isinstance(value, float) and math.isfinite(value) and value > 0.0

    def _validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'save_interval_sec':
                if not self._valid_interval(parameter.value):
                    return SetParametersResult(
                        successful=False,
                        reason='save_interval_sec must be a finite positive double')
        return SetParametersResult(successful=True)

    def _on_image(self, message):
        now = time.monotonic()
        interval = self.get_parameter('save_interval_sec').value
        if self._last_saved_at is not None and now - self._last_saved_at < interval:
            return

        payload = bytes(message.data)
        image_format = message.format.lower()
        format_is_jpeg = not image_format or 'jpeg' in image_format or 'jpg' in image_format
        if not format_is_jpeg or len(payload) < 4 or not payload.startswith(b'\xff\xd8'):
            if not self._warned_invalid:
                self.get_logger().warning(
                    'Skipping non-JPEG or empty payloads; no conversion is performed')
                self._warned_invalid = True
            return

        # Wall-clock reception time in the container's local timezone.
        # MORAI header stamps can be simulation time, so they are not used as dates.
        received_at = datetime.now().astimezone()
        timestamp = received_at.strftime('%Y%m%d_%H%M%S_')
        timestamp += f'{received_at.microsecond // 1000:03d}'
        while True:
            self._sequence += 1
            path = self.output_dir / f'{timestamp}_{self._sequence:06d}.jpg'
            try:
                stream = path.open('xb')
                break
            except FileExistsError:
                continue

        try:
            with stream:
                stream.write(payload)
        except BaseException:
            # Only this invocation's newly created incomplete file is removed.
            path.unlink(missing_ok=True)
            raise
        self.saved_count += 1
        self._last_saved_at = now
        self.get_logger().info(f'Saved #{self.saved_count}: {path.name} ({len(payload)} bytes)')


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = TrafficLightImageCollector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        count = node.saved_count if node is not None else 0
        print(f'Total images saved this run: {count}', flush=True)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
