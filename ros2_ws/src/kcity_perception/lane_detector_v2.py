"""K-City experimental lane detector guided by the V3 road mask.

Run (after sourcing ROS 2 Humble): python3 lane_detector_v2.py
This is a visual experiment, NOT a validated safety-critical perception system.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32


class LaneDetectorV2(Node):
    def __init__(self):
        super().__init__('lane_detector_v2')
        self.declare_parameter('camera_topic', '/camera/image/compressed')
        self.declare_parameter('road_mask_topic', '/perception/v3/road_mask')
        self.declare_parameter('top_ignore_ratio', 0.38)
        self.declare_parameter('road_margin_px', 18)
        self.declare_parameter('max_mask_age_sec', 0.25)
        self.declare_parameter('process_every_n_frames', 3)
        self.frame_count = 0
        self.road_mask = None
        self.road_stamp_ns = None
        self.last_width_fraction = None
        self.last_left = None
        self.last_right = None
        self.sub_cam = self.create_subscription(
            CompressedImage, self.get_parameter('camera_topic').value,
            self.on_camera, qos_profile_sensor_data)
        self.sub_road = self.create_subscription(
            Image, self.get_parameter('road_mask_topic').value,
            self.on_road, qos_profile_sensor_data)
        base = '/perception/v2'
        self.pub_error = self.create_publisher(Float32, base + '/lane_error', 10)
        self.pub_conf = self.create_publisher(Float32, base + '/lane_confidence', 10)
        self.pub_valid = self.create_publisher(Bool, base + '/lane_valid', 10)
        self.pub_debug = self.create_publisher(Image, base + '/lane_image', 1)
        self.pub_candidates = self.create_publisher(Image, base + '/lane_mask', 1)
        self.get_logger().info('Lane V2 ready | camera + V3 road mask | separate /perception/v2 topics')

    @staticmethod
    def timestamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    @staticmethod
    def send_image(pub, image, header, encoding):
        image = np.ascontiguousarray(image)
        msg = Image()
        msg.header = header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(image.strides[0])
        msg.data = image.tobytes()
        pub.publish(msg)

    def on_road(self, msg):
        if msg.encoding not in ('mono8', '8UC1') or msg.height == 0 or msg.width == 0:
            return
        expected = msg.height * msg.step
        if len(msg.data) < expected or msg.step < msg.width:
            return
        array = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        self.road_mask = array[:expected].reshape(msg.height, msg.step)[:, :msg.width].copy()
        self.road_stamp_ns = self.timestamp_ns(msg.header)

    def candidate_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

        white = cv2.inRange(
            hls, (0, 155, 0), (179, 255, 130)
        )

        yellow = cv2.inRange(
            hsv, (12, 65, 65), (43, 255, 255)
        )

        mask = cv2.bitwise_or(white, yellow)

        h, w = mask.shape

        cut = int(
            h * float(np.clip(
                self.get_parameter('top_ignore_ratio').value,
                0.0, 0.75
            ))
        )

        mask[:cut, :] = 0

        # 너무 넓은 가로 영역을 차선 후보에서 제외
        filtered = np.zeros_like(mask)

        for y in range(cut, h):
            row = mask[y] > 0

            padded = np.pad(
                row.astype(np.int8),
                (1, 1)
            )

            changes = np.diff(padded)

            starts = np.where(changes == 1)[0]
            ends = np.where(changes == -1)[0]

            # 원근감을 고려한 허용 두께 (초기 실험값)
            max_width = int(
                w * (0.012 + 0.038 * y / h)
            )

            max_width = max(5, max_width)

            for start, end in zip(starts, ends):
                width = end - start

                if 2 <= width <= max_width:
                    filtered[y, start:end] = 255
        # 작은 흰색 점 노이즈 제거
        #kernel = cv2.getStructuringElement(
        #    cv2.MORPH_ELLIPSE,
        #    (3, 3)
        #)

        #filtered = cv2.morphologyEx(
        #    filtered,
        #    cv2.MORPH_OPEN,
        #    kernel
        #)
        return filtered, cut

    @staticmethod
    def track_side(mask, base_x, side, width):
        h = mask.shape[0]
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return None
        n = 10
        win_h = max(1, h // n)
        margin = max(25, int(width * 0.075))
        center = int(base_x)
        indices = []
        for i in range(n):
            low = max(0, h - (i + 1) * win_h)
            high = h - i * win_h
            valid = np.where((ys >= low) & (ys < high) &
                             (xs >= center - margin) & (xs <= center + margin))[0]
            if len(valid) > 0:
                indices.append(valid)
                if len(valid) >= 22:
                    center = int(np.median(xs[valid]))
        if not indices:
            return None
        idx = np.concatenate(indices)
        pixel_count = len(idx)

        vertical_sections = len(
            np.unique(ys[idx] // max(1, h // 8))
        )

        if pixel_count < 120 or vertical_sections < 3:
            print(
                f'TRACK REJECT | '
                f'pixels={pixel_count} | '
                f'vertical_sections={vertical_sections}'
            )
            return None
        xx, yy = xs[idx].astype(float), ys[idx].astype(float)
        # 2nd-order curve fit, with outlier trimming to suppress wide markings.
        for _ in range(2):
            if len(xx) < 80:
                return None
            fit = np.polyfit(yy / h, xx / width, 1)
            residual = np.abs(np.polyval(fit, yy / h) * width - xx)
            keep = residual < max(12, width * 0.022)
            xx, yy = xx[keep], yy[keep]
        if len(xx) < 95 or np.ptp(yy) < h * .22:
            print(
                f'FIT REJECT | '
                f'pixels={len(xx)} | '
                f'vertical_span={np.ptp(yy):.1f} | '
                f'required_span={h * .22:.1f}'
            )
            return None
        return fit, len(xx)

    def on_camera(self, msg):
        self.frame_count += 1
        if self.frame_count % max(1, int(self.get_parameter('process_every_n_frames').value)):
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]
        raw, cut = self.candidate_mask(frame)
        mask = raw.copy()
        road_state = 'NO ROAD MASK'
        current_stamp = self.timestamp_ns(msg.header)
        max_age = float(self.get_parameter('max_mask_age_sec').value)
        # Reject stale/future road masks, e.g. after rosbag loop/reset.
        if self.road_mask is not None and self.road_stamp_ns is not None:
            age = (current_stamp - self.road_stamp_ns) / 1e9
            if 0 <= age <= max_age:
                road = cv2.resize(self.road_mask, (w, h), interpolation=cv2.INTER_NEAREST)
                dilation = max(0, int(self.get_parameter('road_margin_px').value))
                if dilation:
                    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*dilation+1, 2*dilation+1))
                    road = cv2.dilate(road, k)
                # Use V3 only as a SOFT preference: do not delete lane pixels
                # where road_mask is wrong (red roads, boundaries, etc.).
                road_state = 'ROAD MASK OK'
            else:
                road = None
                road_state = 'ROAD MASK STALE'
        else:
            road = None
        # Spatial preference, only in candidate ranking; raw mask preserved.
        histogram = np.count_nonzero(mask[int(h * .68):], axis=0).astype(float)
        midpoint = w // 2
        separation = max(18, int(w * .09))
        candidates = []
        for side, begin, end in [('L', 0, midpoint), ('R', midpoint, w)]:
            profile = histogram[begin:end].copy()
            # Candidates near the camera center are less likely to be outer borders.
            for _ in range(5):
                if profile.size == 0:
                    break
                pos = int(np.argmax(profile))
                val = float(profile[pos])
                if val < 7:
                    break
                x = pos + begin
                fit_result = self.track_side(mask, x, side, w)
                if fit_result is not None:
                    fit, count = fit_result
                    y_look = .82
                    x_look = float(np.polyval(fit, y_look) * w)
                    if 0 <= x_look < w and ((side == 'L' and x_look < w*.60) or
                                            (side == 'R' and x_look > w*.40)):
                        road_bonus = 0.0
                        if road is not None:
                            ix = int(np.clip(x_look, 0, w - 1))
                            iy = int(h * y_look)
                            road_bonus = 0.35 if road[iy, ix] > 0 else 0.0
                        proximity = 1.0 - min(1.0, abs(x_look - w/2)/(w/2))
                        score = np.log1p(count) + 0.9*proximity + road_bonus
                        candidates.append((side, fit, count, score, x_look))
                profile[max(0, pos-separation):min(len(profile), pos+separation)] = 0
        left = sorted((x for x in candidates if x[0]=='L'), key=lambda a:a[3], reverse=True)[:4]
        right = sorted((x for x in candidates if x[0]=='R'), key=lambda a:a[3], reverse=True)[:4]
        pair = None
        best = -1e9
        for l in left:
            for r in right:
                lx, rx = l[4], r[4]
                width_fraction = (rx-lx)/w
                if not 0.18 <= width_fraction <= 0.80:
                    continue
                # Reject intersecting / implausibly diverging fits.
                y_levels = np.array([.60, .72, .82, .94])
                widths = (np.polyval(r[1], y_levels) - np.polyval(l[1], y_levels))
                if np.min(widths) < .06 or np.max(widths) > .90:
                    continue
                consistency = (0.0 if self.last_width_fraction is None else
                               -3.0 * abs(width_fraction-self.last_width_fraction))
                score = l[3]+r[3]+consistency
                if score > best:
                    best = score
                    pair = l, r, width_fraction
        valid = pair is not None
        if valid:
            l, r, width_fraction = pair
            self.last_width_fraction = width_fraction
            center = (l[4]+r[4])/2
            error = float(np.clip((center-w/2)/(w/2), -1, 1))
            conf = float(np.clip((l[2]+r[2])/2300, 0, 0.85))
            selected = [l, r]
        else:
            # Do not manufacture a center error from a single uncertain edge.
            error, conf, selected = 0.0, 0.0, []
        debug = frame.copy()
        for candidate in left+right:
            self.draw_curve(debug, candidate[1], (0, 210, 255), 2, cut)
        for candidate in selected:
            self.draw_curve(debug, candidate[1], (255, 0, 0), 5, cut)
        if valid:
            cv2.circle(debug, (int(center), int(h*.82)), 8, (0,255,0), -1)
        cv2.line(debug, (w//2, int(h*.6)), (w//2, h-1), (0,0,255), 2)
        cv2.line(debug, (0,cut), (w-1,cut), (0,160,255), 2)
        status = 'BOTH' if valid else 'LOST'
        cv2.putText(debug, f'{status} | valid={valid} | error={error:+.3f}', (15, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, .75, (0,255,0) if valid else (0,0,255), 2)
        cv2.putText(debug, f'L={len(left)} R={len(right)} | {road_state}', (15, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0,255,255), 2)
        cv2.putText(debug, 'EXPERIMENT - verify against actual lane edges', (15, 96),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0,255,255), 2)
        self.pub_error.publish(Float32(data=error))
        self.pub_conf.publish(Float32(data=conf))
        self.pub_valid.publish(Bool(data=valid))
        self.send_image(self.pub_debug, debug, msg.header, 'bgr8')
        self.send_image(self.pub_candidates, raw, msg.header, 'mono8')

    @staticmethod
    def draw_curve(frame, fit, color, thickness, cut):
        h,w = frame.shape[:2]
        yy = np.linspace(max(cut, int(h*.48)), h-1, 90)
        xx = np.polyval(fit, yy/h)*w
        good = (xx>=0)&(xx<w)
        pts = np.column_stack((xx[good], yy[good])).astype(np.int32)
        if len(pts)>1:
            cv2.polylines(frame, [pts], False, color, thickness)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorV2()
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
