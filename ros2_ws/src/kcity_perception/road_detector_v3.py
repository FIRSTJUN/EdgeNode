"""K-City experimental OpenCV road-region segmentation (no neural network).

Run: source /opt/ros/humble/setup.bash && python3 road_detector_v3.py
V3 uses GrabCut with a near-field road seed, optional previous-frame guidance,
connected component selection, and cautious temporal smoothing.
This is a visual research prototype, NOT a verified drivable-area detector.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class RoadDetectorV3(Node):
    def __init__(self):
        super().__init__('road_detector_v3')
        self.declare_parameter('camera_topic', '/camera/image/compressed')
        self.declare_parameter('top_ignore_ratio', 0.38)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('grabcut_iterations', 2)
        self.declare_parameter('temporal_alpha', 0.55)
        self.declare_parameter('min_component_pixels', 180)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.frames = 0

        self.previous_probability = None
        self.previous_mask = None
        topic = self.get_parameter('camera_topic').value
        self.sub = self.create_subscription(
            CompressedImage, topic, self.on_image, qos_profile_sensor_data
        )
        self.mask_pub = self.create_publisher(Image, '/perception/v3/road_mask', 1)
        self.image_pub = self.create_publisher(Image, '/perception/v3/road_image', 1)
        self.raw_pub = self.create_publisher(Image, '/perception/v3/raw_mask', 1)
        self.get_logger().info(f'V3 road detector ready | {topic}')

    @staticmethod
    def send(publisher, image, header, encoding):
        image = np.ascontiguousarray(image)
        msg = Image()
        msg.header = header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(image.strides[0])
        msg.data = image.tobytes()
        publisher.publish(msg)

    @staticmethod
    def connected_from_vehicle(mask, min_area):
        """Keep the component most represented in a bottom-center seed band."""
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        h, w = mask.shape
        seed = labels[int(h * .86):h, int(w * .39):int(w * .61)]
        ids, counts = np.unique(seed[seed > 0], return_counts=True)
        valid = [(int(i), int(c)) for i, c in zip(ids, counts)
                 if stats[int(i), cv2.CC_STAT_AREA] >= min_area]
        if not valid:
            return np.zeros_like(mask)
        chosen = max(valid, key=lambda pair: pair[1])[0]
        return np.where(labels == chosen, 255, 0).astype(np.uint8)

    def segment(self, frame):
        src_h, src_w = frame.shape[:2]
        w = max(160, int(self.get_parameter('width').value))
        h = max(120, int(self.get_parameter('height').value))
        small = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
        ratio = float(np.clip(self.get_parameter('top_ignore_ratio').value, .05, .70))
        cut = int(h * ratio)

        # GrabCut: 0 sure background, 1 sure foreground, 2 likely background,
        # 3 likely foreground. Do NOT force red/gray colors; let GrabCut model them.
        gc = np.full((h, w), cv2.GC_PR_BGD, dtype=np.uint8)
        gc[:cut, :] = cv2.GC_BGD

        # Broad road hypothesis. This is not a hard trapezoidal ROI: it only
        # initializes GrabCut, which may expand outside this polygon.
        polygon = np.array([[
            (int(w * .27), int(h * max(ratio + .08, .53))),
            (int(w * .73), int(h * max(ratio + .08, .53))),
            (int(w * .97), h - 1),
            (int(w * .03), h - 1),
        ]], dtype=np.int32)
        cv2.fillPoly(gc, polygon, cv2.GC_PR_FGD)

        # Previous detection is *weak* guidance only, never ground truth.
        prior = self.previous_mask
        if prior is not None and prior.shape == gc.shape:
            expanded = cv2.dilate(prior, np.ones((9, 9), np.uint8)) > 0
            gc[(expanded) & (gc != cv2.GC_BGD)] = cv2.GC_PR_FGD

        # Near-camera patch is assumed road while the vehicle stays on road.
        # Keep it small to avoid forcing sidewalks or red material as background.

        # HSV 색상 공간으로 변환
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # 초록색 잔디 후보 검출
        grass_lower = np.array([35, 40, 30], dtype=np.uint8)
        grass_upper = np.array([90, 255, 255], dtype=np.uint8)

        grass_mask = cv2.inRange(
            hsv,
            grass_lower,
            grass_upper
        )

        # GrabCut 초기 마스크에서 잔디를 배경 후보로 지정
        gc[grass_mask > 0] = cv2.GC_PR_BGD

        gc[:cut, :] = cv2.GC_BGD

        seed_y = int(h * .94)
        gc[seed_y:h, int(w * .46):int(w * .54)] = cv2.GC_FGD

        bg = np.zeros((1, 65), np.float64)
        fg = np.zeros((1, 65), np.float64)
        iterations = int(np.clip(self.get_parameter('grabcut_iterations').value, 1, 5))
        cv2.grabCut(small, gc, None, bg, fg, iterations, cv2.GC_INIT_WITH_MASK)
        candidate = np.where((gc == cv2.GC_FGD) | (gc == cv2.GC_PR_FGD),
                             255, 0).astype(np.uint8)
        candidate[:cut, :] = 0
        # HSV 잔디 후보를 최종 도로 후보에서 제외
        candidate[grass_mask > 0] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, kernel)
        raw = candidate.copy()
        current = self.connected_from_vehicle(
            candidate, max(50, int(self.get_parameter('min_component_pixels').value)))

        # Only gently smooth detection over time. Large transitions reset
        # instead of preserving a ghost mask across turns / new scenes.
        if np.count_nonzero(current) == 0:
            self.previous_mask = None
            self.previous_probability = None
            smooth = current
        else:
            signal = (current > 0).astype(np.float32)
            if self.previous_probability is not None and self.previous_probability.shape == signal.shape:
                overlap = np.count_nonzero((current > 0) & (self.previous_mask > 0))
                union = np.count_nonzero((current > 0) | (self.previous_mask > 0))
                iou = overlap / max(1, union)
                if iou > 0.45:
                    alpha = float(np.clip(self.get_parameter('temporal_alpha').value, .1, 1.0))
                    signal = alpha * signal + (1.0 - alpha) * self.previous_probability
            self.previous_probability = signal
            smooth = np.where(signal > .50, 255, 0).astype(np.uint8)
            smooth[:cut, :] = 0
            smooth = self.connected_from_vehicle(smooth, 50)
            self.previous_mask = smooth.copy()

        mask = cv2.resize(smooth, (src_w, src_h), interpolation=cv2.INTER_NEAREST)
        raw = cv2.resize(raw, (src_w, src_h), interpolation=cv2.INTER_NEAREST)
        overlay = frame.copy()
        color = np.zeros_like(frame)
        color[:] = (0, 200, 0)
        blended = cv2.addWeighted(frame, .65, color, .35, 0)
        overlay[mask > 0] = blended[mask > 0]
        cy = int(src_h * ratio)
        cv2.line(overlay, (0, cy), (src_w-1, cy), (0, 160, 255), 2)
        cv2.putText(overlay, 'V3 EXPERIMENT - not validated', (16, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
        return mask, overlay, raw

    def on_image(self, msg):
        self.frames += 1
        every = max(1, int(self.get_parameter('process_every_n_frames').value))
        if self.frames % every:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Could not decode input frame')
            return
        try:
            mask, display, raw = self.segment(frame)
        except cv2.error as exc:
            self.get_logger().error(f'OpenCV GrabCut failed: {exc}')
            self.previous_mask = None
            self.previous_probability = None
            return
        self.send(self.mask_pub, mask, msg.header, 'mono8')
        self.send(self.image_pub, display, msg.header, 'bgr8')
        self.send(self.raw_pub, raw, msg.header, 'mono8')

def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectorV3()
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
