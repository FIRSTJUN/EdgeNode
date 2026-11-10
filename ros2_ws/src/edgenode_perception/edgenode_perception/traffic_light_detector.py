"""Latest-frame ROS camera inference with selectable local GPU / remote backends."""
import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import re
import time
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def parse_predictions(response, threshold):
    """Reject invalid detections; x/y are bbox centers in input image pixels."""
    if not isinstance(response, dict) or not isinstance(response.get('predictions'), list):
        raise ValueError('Invalid detection response')
    detections = []
    for item in response['predictions']:
        try:
            label = str(item['class']).strip().lower()
            values = [float(item[k]) for k in ('confidence', 'x', 'y', 'width', 'height')]
            confidence, x, y, width, height = values
            if (label not in ('red', 'yellow', 'green') or
                    not all(math.isfinite(v) for v in values) or
                    not threshold <= confidence <= 1.0 or width <= 0 or height <= 0):
                continue
            detections.append(dict(label=label, confidence=confidence,
                                   x=x, y=y, w=width, h=height))
        except (KeyError, TypeError, ValueError):
            continue
    return sorted(detections, key=lambda d: d['confidence'], reverse=True)


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        defaults = {
            'camera_topic': '/camera/image/compressed',
            'debug_topic': '/perception/traffic_light/debug/compressed',
            'state_topic': '/perception/traffic_light/state',
            'detections_topic': '/perception/traffic_light/detections',
            'backend': os.environ.get('ROBOFLOW_BACKEND', 'local'),
            'local_model_path': '/workspace/models/traffic_light',
            'yolo_model_path': '/workspace/models/traffic_light_yolo/weights.pt',
            'model_id': os.environ.get('ROBOFLOW_MODEL_ID', ''),
            'project_id': os.environ.get('ROBOFLOW_PROJECT_ID', ''),
            'version': os.environ.get('ROBOFLOW_VERSION', ''),
            'api_url': 'https://serverless.roboflow.com',
            'inference_fps': 10.0, 'confidence_threshold': 0.5,
            'request_timeout_sec': 5.0, 'camera_timeout_sec': 1.0,
            'result_max_age_sec': 3.0, 'error_backoff_sec': 2.0,
            'publish_debug': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(
                read_only=name != 'confidence_threshold'))
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if self.p['backend'] not in ('local', 'remote', 'yolo'):
            raise ValueError('backend must be local, remote or yolo')
        self.local_model = None
        self.yolo_model = None
        for name in ('inference_fps', 'request_timeout_sec', 'camera_timeout_sec',
                     'result_max_age_sec', 'error_backoff_sec'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not 0 <= self.p['confidence_threshold'] <= 1:
            raise ValueError('confidence_threshold must be between 0 and 1')
        if not self.p['api_url'].startswith('https://'):
            raise ValueError('api_url must use HTTPS')
        self.api_key = (os.environ.get('ROBOFLOW_API_KEY', '')
                        if self.p['backend'] == 'remote' else '')
        self.model_id = self.p['model_id'] or (
            f"{self.p['project_id']}/{self.p['version']}"
            if self.p['project_id'] and self.p['version'] else
            's-workspace-kcda0/edgenode-1-rfdetr-nas-t1--59de31')
        if (self.p['backend'] == 'remote' and self.model_id and
                not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+', self.model_id)):
            raise ValueError('model_id must be workspace/model-slug or project/version')
        self.ready = (
            self.p['backend'] in ('local', 'yolo')
            or bool(self.api_key and self.model_id)
        )
        self.state_pub = self.create_publisher(String, self.p['state_topic'], 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.p['debug_topic'], 1)
        self.detections_pub = self.create_publisher(String, self.p['detections_topic'], 10)
        self.metrics_pub = self.create_publisher(String, '/perception/traffic_light/metrics', 10)
        self.camera_sub = self.create_subscription(CompressedImage, self.p['camera_topic'],
                                                   self._camera, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                                           durability=DurabilityPolicy.VOLATILE))
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.latest = None
        self.sequence = 0
        self.dropped_frames = 0
        self.completions = deque(maxlen=100)
        self.requests_started = 0
        self.submitted = 0
        self.next_request = 0.0
        self.last_result = 0.0
        self.state = 'UNKNOWN'
        self.timer = self.create_timer(0.005, self._tick)
        self.heartbeat = self.create_timer(0.5, self._heartbeat)
        self.get_logger().info(f"Subscribed: {self.p['camera_topic']} | "
                               f"inference cap={self.p['inference_fps']:.1f} FPS")
        if not self.ready:
            self.get_logger().warning('Remote inference disabled: set ROBOFLOW_API_KEY, '
                                      'ROBOFLOW_MODEL_ID (or project/version); state=UNKNOWN')

    @staticmethod
    def _validate_parameters(params):
        for param in params:
            if param.name == 'confidence_threshold':
                if (type(param.value) is not float or
                        not math.isfinite(param.value) or not 0 <= param.value <= 1):
                    return SetParametersResult(successful=False,
                                               reason='threshold must be a float in [0, 1]')
        return SetParametersResult(successful=True)

    def _camera(self, msg):
        if self.latest is not None and self.latest[2] != self.submitted:
            self.dropped_frames += 1
        self.sequence += 1
        self.latest = (msg, time.monotonic(), self.sequence)

    def _infer(self, sample, threshold):
        msg, received, sequence = sample
        started = time.monotonic()
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('Camera decode failed')
        if self.p['backend'] == 'yolo':
            return self._infer_yolo(frame, msg, received, sequence, started, threshold)
        if self.p['backend'] == 'local':
            if self.local_model is None:
                # Load pip-provided CUDA 12/cuDNN libraries before creating ORT sessions.
                import onnxruntime as ort
                ort.preload_dlls(directory="")
                if 'CUDAExecutionProvider' not in ort.get_available_providers():
                    raise RuntimeError('CUDAExecutionProvider is unavailable')
                from inference_models import AutoModel
                model_path = Path(self.p['local_model_path'])
                # Fail locally rather than treating a missing path as a remote model ID.
                for name in ('model_config.json', 'inference_config.json',
                             'class_names.txt', 'weights.onnx'):
                    if not (model_path / name).is_file():
                        raise FileNotFoundError(f'Local model package missing: {model_path / name}')
                model = AutoModel.from_pretrained(
                    str(model_path), weights_provider='local',
                    use_auto_resolution_cache=False,
                    onnx_execution_providers=['CUDAExecutionProvider'])
                session = getattr(model, '_session', None)
                if session is None or 'CUDAExecutionProvider' not in session.get_providers():
                    raise RuntimeError('Local model did not activate CUDAExecutionProvider')
                self.get_logger().info(f'Local model providers: {session.get_providers()}')
                self.local_model = model
            inference_started = time.monotonic()
            result = self.local_model.infer(frame, confidence=threshold)[0]
            # Native inference-models returns xyxy tensors and remapped class IDs.
            payload = {'predictions': [
                {'class': self.local_model.class_names[int(class_id)],
                 'confidence': float(confidence),
                 'x': (box[0] + box[2]) / 2, 'y': (box[1] + box[3]) / 2,
                 'width': box[2] - box[0], 'height': box[3] - box[1]}
                for box, class_id, confidence in zip(
                    result.xyxy.cpu().tolist(), result.class_id.cpu().tolist(),
                    result.confidence.cpu().tolist())]}
            completed = time.monotonic()
            timing = dict(camera_received_monotonic_sec=received,
                          inference_started_monotonic_sec=started,
                          inference_completed_monotonic_sec=completed,
                          sequence=sequence, queue_ms=(started-received)*1000,
                          preprocessing_ms=(inference_started-started)*1000,
                          http_ms=0.0, inference_ms=(completed-started)*1000,
                          local_inference_ms=(completed-inference_started)*1000,
                          backend='local')
            return frame, payload, msg.header, received, timing
        # The camera is already JPEG: avoid a lossy encode and extra CPU work.
        if bytes(msg.data[:2]) == b'\xff\xd8':
            jpeg = bytes(msg.data)
        else:
            ok, encoded = cv2.imencode('.jpg', frame)
            if not ok:
                raise ValueError('JPEG encode failed')
            jpeg = encoded.tobytes()
        query = urlencode({'confidence': threshold * 100,
                           'format': 'json'})
        url = f"{self.p['api_url'].rstrip('/')}/{self.model_id}?{query}"
        request = Request(url, data=base64.b64encode(jpeg),
                          headers={'Content-Type': 'application/x-www-form-urlencoded',
                                   'Authorization': f'Bearer {self.api_key}'},
                          method='POST')
        http_started = time.monotonic()
        with urlopen(request, timeout=self.p['request_timeout_sec']) as response:
            payload = json.load(response)
        completed = time.monotonic()
        timing = dict(camera_received_monotonic_sec=received,
                      inference_started_monotonic_sec=started,
                      inference_completed_monotonic_sec=completed,
                      sequence=sequence, queue_ms=(started-received)*1000,
                      preprocessing_ms=(http_started-started)*1000,
                      http_ms=(completed-http_started)*1000,
                      inference_ms=(completed-started)*1000)
        return frame, payload, msg.header, received, timing

    def _infer_yolo(self, frame, msg, received, sequence, started, threshold):
        if self.yolo_model is None:
            model_path = Path(self.p['yolo_model_path'])
            if not model_path.is_file():
                raise FileNotFoundError(f'YOLO model missing: {model_path}')
            import torch
            if not torch.cuda.is_available():
                raise RuntimeError('YOLO requires CUDA device cuda:0')
            from ultralytics import YOLO
            model = YOLO(str(model_path))
            model.to('cuda:0')
            if str(model.device) != 'cuda:0':
                raise RuntimeError('YOLO model did not activate cuda:0')
            self.get_logger().info(
                f'YOLO model loaded: path={model_path} device={model.device} '
                f'class names={model.names}')
            self.yolo_model = model
        inference_started = time.monotonic()
        result = self.yolo_model.predict(
            source=frame, device='cuda:0', imgsz=640, conf=threshold,
            half=False, verbose=False, save=False)[0]
        payload = {'predictions': [
            {'class': self.yolo_model.names[int(class_id)],
             'confidence': float(confidence),
             'x': (box[0] + box[2]) / 2, 'y': (box[1] + box[3]) / 2,
             'width': box[2] - box[0], 'height': box[3] - box[1]}
            for box, class_id, confidence in zip(
                result.boxes.xyxy.cpu().tolist(), result.boxes.cls.cpu().tolist(),
                result.boxes.conf.cpu().tolist())]}
        completed = time.monotonic()
        timing = dict(camera_received_monotonic_sec=received,
                      inference_started_monotonic_sec=started,
                      inference_completed_monotonic_sec=completed,
                      sequence=sequence, queue_ms=(started-received)*1000,
                      preprocessing_ms=(inference_started-started)*1000 + result.speed['preprocess'],
                      http_ms=0.0, inference_ms=(completed-started)*1000,
                      local_inference_ms=(completed-inference_started)*1000,
                      yolo_forward_ms=result.speed['inference'],
                      postprocessing_ms=result.speed['postprocess'], backend='yolo')
        return frame, payload, msg.header, received, timing

    def _unknown(self):
        self.state = 'UNKNOWN'
        self.state_pub.publish(String(data=self.state))
        self.detections_pub.publish(String(data='{"state":"UNKNOWN","predictions":[]}'))

    def _heartbeat(self):
        now = time.monotonic()
        if (self.latest is None or now - self.latest[1] > self.p['camera_timeout_sec'] or
                now - self.last_result > self.p['result_max_age_sec']):
            self._unknown()
        else:
            self.state_pub.publish(String(data=self.state))

    def _tick(self):
        now = time.monotonic()
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                frame, payload, header, received, timing = future.result()
                self.completions.append(timing['inference_completed_monotonic_sec'])
                if (now - received > self.p['result_max_age_sec'] or
                        now - self.latest[1] > self.p['camera_timeout_sec']):
                    self._unknown()
                else:
                    detections = parse_predictions(
                        payload, self.get_parameter('confidence_threshold').value)
                    self.last_result = received
                    self.state = detections[0]['label'].upper() if detections else 'UNKNOWN'
                    self.state_pub.publish(String(data=self.state))
                    self.detections_pub.publish(String(data=json.dumps({
                        'state': self.state, 'stamp': {'sec': header.stamp.sec,
                        'nanosec': header.stamp.nanosec}, 'frame_id': header.frame_id,
                        'image_width': frame.shape[1], 'image_height': frame.shape[0],
                        'prediction_age_sec': time.monotonic() - received,
                        'predictions': detections})))
                    self.get_logger().info(f'Roboflow response: {len(detections)} accepted detections')
                    for d in detections:
                        self.get_logger().info(
                            f"{d['label']} confidence={d['confidence']:.2f} "
                            f"x={d['x']:.1f} y={d['y']:.1f} w={d['w']:.1f} h={d['h']:.1f}")
                    if self.p['publish_debug']:
                        self._debug(frame, detections, header)
                published = time.monotonic()
                elapsed = (self.completions[-1] - self.completions[0]
                           if len(self.completions) > 1 else 0)
                fps = (len(self.completions)-1)/elapsed if elapsed > 0 else 0.0
                timing.update(total_latency_ms=(published-received)*1000,
                              publish_delay_ms=(published-timing['inference_completed_monotonic_sec'])*1000,
                              fps=fps, received_frames=self.sequence,
                              dropped_frames=self.dropped_frames,
                              requests_started=self.requests_started,
                              stale=(now-received > self.p['result_max_age_sec']))
                self.metrics_pub.publish(String(data=json.dumps(timing)))
                self.get_logger().info(
                    f"inference={timing['inference_ms']:.0f}ms "
                    f"http={timing['http_ms']:.0f}ms queue={timing['queue_ms']:.0f}ms "
                    f"total_latency={timing['total_latency_ms']:.0f}ms fps={fps:.2f} "
                    f"dropped={self.dropped_frames} received={self.sequence} "
                    f"receive_t={received:.6f} start_t={timing['inference_started_monotonic_sec']:.6f} "
                    f"done_t={timing['inference_completed_monotonic_sec']:.6f}")
            except Exception as exc:
                # Never log exception text, URL or response body: these can contain credentials.
                detail = f'HTTP {exc.code}' if isinstance(exc, HTTPError) else type(exc).__name__
                self.get_logger().warning(f'Inference failed ({detail}); state=UNKNOWN')
                self._unknown()
                self.next_request = now + self.p['error_backoff_sec']
        if (self.ready and self.future is None and self.latest is not None and
                self.latest[2] != self.submitted and now >= self.next_request and
                now - self.latest[1] <= self.p['camera_timeout_sec']):
            self.requests_started += 1
            self.submitted = self.latest[2]
            self.next_request = now + 1.0 / self.p['inference_fps']
            self.future = self.pool.submit(
                self._infer, self.latest, self.get_parameter('confidence_threshold').value)

    def _debug(self, frame, detections, header):
        # Preserve the decoded MORAI frame; an empty detection list is passthrough.
        frame = frame.copy()
        height, width = frame.shape[:2]
        colors = {'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0)}
        for d in detections:
            x1 = max(0, min(width - 1, round(d['x'] - d['w'] / 2)))
            y1 = max(0, min(height - 1, round(d['y'] - d['h'] / 2)))
            x2 = max(0, min(width - 1, round(d['x'] + d['w'] / 2)))
            y2 = max(0, min(height - 1, round(d['y'] + d['h'] / 2)))
            color = colors[d['label']]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{d['label']} {d['confidence']:.2f}",
                        (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        ok, encoded = cv2.imencode('.jpg', frame)
        if ok:
            msg = CompressedImage(header=header, format='jpeg', data=encoded.tobytes())
            self.debug_pub.publish(msg)

    def destroy_node(self):
        self.timer.cancel()
        self.heartbeat.cancel()
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficLightDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
