#!/usr/bin/env python3
"""Offline-only semantic component lane fitting with pixel-coordinate RANSAC.

This script deliberately does not feed results back into PerceptionNode.  It
reuses the production mask only for the production baseline and independently
recreates the passive semantic masks/ROI so that every qualifying connected
component retains its real pixel coordinates for RANSAC.
"""

import argparse
import gzip
import inspect
import json
import math
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rosbag2_py
import sklearn
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sklearn.linear_model import LinearRegression, RANSACRegressor

from edgenode_perception.perception_node import (
    LaneDiagnostics,
    PerceptionNode,
    ProductionFitSnapshot,
)


CAMERA_TOPIC = '/camera/image/compressed'
DEFAULT_BAGS = [
    Path('/workspace/bags/2026-09-21/centered_start_lane_none'),
    Path('/workspace/bags/2026-09-21/yellow_line_false_both'),
    Path('/workspace/bags/2026-09-22/live_lane_none'),
    Path('/workspace/bags/2026-09-22/live_right_lane_missing'),
    Path('/workspace/bags/2026-09-23/beige_boundary_vs_parking_white'),
    Path('/workspace/bags/2026-09-23/both_to_none_transition'),
    Path('/workspace/bags/2026-09-24/lateral_semantic_sweep'),
    Path('/workspace/bags/2026-09-25/live_adaptive_pair_failure'),
]
DEFAULT_CONFIG = Path(
    '/workspace/ros2_ws/src/edgenode_perception/config/perception.yaml')
DEFAULT_TRUSTED = Path(
    '/workspace/debug_frames/lateral_semantic_sweep_analysis/'
    'component_geometry.json.gz')
DEFAULT_SUMMARY = Path('/workspace/debug_frames/ransac_lane_analysis_summary.json')
DEFAULT_RECORDS = Path(
    '/workspace/debug_frames/ransac_lane_analysis_records.json.gz')

# Deliberately fixed experiment settings.  Geometry bounds mirror production's
# adaptive philosophy; semantic and temporal terms cannot make invalid geometry
# valid.
RANSAC_RESIDUAL_THRESHOLD_PX = 7.0
RANSAC_MAX_TRIALS = 30
# Three points are mathematically sufficient for a quadratic but let one random
# minimal set dictate extreme curvature inside a thick marking. A 30-point
# sample remains below the 40-pixel component gate while stabilizing the fit.
RANSAC_MIN_SAMPLES = 30
PRIOR_POINT_BAND_PX = 14.0
PAIR_MIN_COMMON_SPAN_PX = 20
PAIR_EVAL_FRACTION = 0.70
WIDTH_MIN_RATIO = 0.60
WIDTH_MAX_RATIO = 1.55
TEMPORAL_SCORE_SCALE_PX = 45.0
EXTREME_TEMPORAL_JUMP_PX = 75.0
LARGE_CENTER_DEVIATION_PX = 50.0
LARGE_BOUNDARY_DEVIATION_PX = 75.0


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def coefficients(model):
    estimator = getattr(model, 'estimator_', None)
    if estimator is None:
        estimator = getattr(model, 'base_estimator_', None)
    if estimator is None:
        return None
    coef = np.asarray(estimator.coef_, dtype=np.float64).reshape(-1)
    if coef.size != 2:
        return None
    return [float(coef[0]), float(coef[1]), float(estimator.intercept_)]


def evaluate(fit, y):
    return float(fit[0] * y * y + fit[1] * y + fit[2])


def numeric_stats(values):
    values = np.asarray(
        [float(value) for value in values if finite(value) is not None],
        dtype=np.float64)
    if values.size == 0:
        return {'count': 0, 'min': None, 'p50': None, 'p95': None, 'max': None}
    return {
        'count': int(values.size),
        'min': float(np.min(values)),
        'mean': float(np.mean(values)),
        'p50': float(np.percentile(values, 50)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def timing_stats(values):
    result = numeric_stats(np.asarray(values, dtype=np.float64) * 1000.0)
    return {key + '_ms' if key != 'count' else key: value
            for key, value in result.items()}


def header_stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(
        msg.header.stamp.nanosec)


def iter_camera_messages(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = {item.name: item.type
              for item in reader.get_all_topics_and_types()}
    if CAMERA_TOPIC not in topics:
        raise RuntimeError('{} has no {}'.format(bag_path, CAMERA_TOPIC))
    message_type = get_message(topics[CAMERA_TOPIC])
    while reader.has_next():
        topic, data, storage_ns = reader.read_next()
        if topic == CAMERA_TOPIC:
            yield int(storage_ns), deserialize_message(data, message_type)


class JsonGzipArrayWriter:
    def __init__(self, path):
        self.path = path
        self.stream = None
        self.first = True

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = gzip.open(self.path, 'wt', encoding='utf-8')
        self.stream.write('[\n')
        return self

    def write(self, value):
        if not self.first:
            self.stream.write(',\n')
        json.dump(value, self.stream, separators=(',', ':'), allow_nan=False)
        self.first = False

    def __exit__(self, exc_type, exc_value, traceback):
        if self.stream is not None:
            self.stream.write('\n]\n')
            self.stream.close()


def primary_reference(components):
    valid = [item for item in components if item.get('x_eval') is not None]
    if not valid:
        return None
    return max(valid, key=lambda item: (
        item['y_max'] - item['y_min'], item['pixels']))


def load_trusted_geometry(path):
    if path is None or not path.exists():
        return {}
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        records = json.load(stream)
    trusted = {}
    for record in records:
        yellow = primary_reference(record['yellow'])
        beige = primary_reference(record['beige'])
        neutral = primary_reference(record['neutral_white'])
        if yellow is None or beige is None or neutral is None:
            continue
        if yellow['x_eval'] < beige['x_eval'] < neutral['x_eval']:
            trusted[int(record['source_header_ns'])] = {
                'yellow': yellow,
                'beige': beige,
                'neutral_white': neutral,
                'production_right_x': float(record['production_right_x']),
            }
    return trusted


def semantic_components(node, image):
    """Reproduce diagnostic thresholds/ROI and retain component pixels."""
    h, w = image.shape[:2]
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(
        hls, np.array([0, 175, 0], dtype=np.uint8),
        np.array([255, 255, 130], dtype=np.uint8))
    yellow = cv2.inRange(
        hsv, np.array([12, 70, 70], dtype=np.uint8),
        np.array([42, 255, 255], dtype=np.uint8))
    roi = node.make_diagnostic_roi(h, w)
    white = cv2.bitwise_and(white, roi)
    yellow = cv2.bitwise_and(yellow, roi)

    close_kernel = int(
        node.get_parameter('diag_component_close_kernel').value)
    if close_kernel > 1:
        if close_kernel % 2 == 0:
            close_kernel += 1
        kernel = np.ones((close_kernel, close_kernel), dtype=np.uint8)
        white = cv2.bitwise_and(
            cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel), roi)
        yellow = cv2.bitwise_and(
            cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kernel), roi)

    min_pixels = int(node.get_parameter('diag_min_component_pixels').value)
    min_span = int(node.get_parameter('diag_min_component_y_span').value)
    neutral_max_s = int(
        node.get_parameter('diag_neutral_white_max_saturation').value)
    beige_h_min = int(node.get_parameter('diag_beige_hue_min').value)
    beige_h_max = int(node.get_parameter('diag_beige_hue_max').value)
    output = []

    def append_mask(mask, source_type):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        for component_id in range(1, count):
            x0, y0, width, height, area = stats[component_id]
            if area < min_pixels or height - 1 < min_span:
                continue
            local = labels[y0:y0 + height, x0:x0 + width] == component_id
            local_y, local_x = np.nonzero(local)
            ys = (local_y + y0).astype(np.float64)
            xs = (local_x + x0).astype(np.float64)
            if xs.size < min_pixels:
                continue
            values = hsv[y0:y0 + height, x0:x0 + width][local]
            median_h = float(np.median(values[:, 0]))
            median_s = float(np.median(values[:, 1]))
            median_v = float(np.median(values[:, 2]))
            semantic = source_type
            if source_type == 'WHITE':
                if median_s <= neutral_max_s:
                    semantic = 'NEUTRAL_WHITE'
                elif beige_h_min <= median_h <= beige_h_max:
                    semantic = 'BEIGE'
                else:
                    continue
            output.append({
                'semantic': semantic,
                'component_id': int(component_id),
                'area': int(area),
                'xs': xs,
                'ys': ys,
                'pixel_count': int(xs.size),
                'y_min': int(np.min(ys)),
                'y_max': int(np.max(ys)),
                'median_h': median_h,
                'median_s': median_s,
                'median_v': median_v,
            })

    append_mask(yellow, 'YELLOW')
    append_mask(white, 'WHITE')
    return output


def make_ransac(random_state):
    estimator = LinearRegression(fit_intercept=True)
    kwargs = {
        'min_samples': RANSAC_MIN_SAMPLES,
        'residual_threshold': RANSAC_RESIDUAL_THRESHOLD_PX,
        'max_trials': RANSAC_MAX_TRIALS,
        'random_state': int(random_state),
        'loss': 'absolute_loss',
    }
    parameter = (
        'estimator' if 'estimator' in inspect.signature(
            RANSACRegressor).parameters else 'base_estimator')
    kwargs[parameter] = estimator
    return RANSACRegressor(**kwargs), parameter


def residual_stats(residual):
    return numeric_stats(np.abs(np.asarray(residual, dtype=np.float64)))


def fit_component(component, previous, frame_index, ordinal):
    started = time.perf_counter()
    xs = component['xs']
    ys = component['ys']
    used = np.ones(xs.size, dtype=bool)
    prior_side = None
    prior_counts = {}

    # SSAFETY-inspired preference: use a well-supported subset near the previous
    # left/right model.  If neither subset independently satisfies the current
    # component gates, RANSAC sees the complete component instead.
    if previous is not None:
        eligible = []
        for side in ('left', 'right'):
            prior_fit = previous[side]['fit']
            near = np.abs(xs - np.polyval(prior_fit, ys)) <= PRIOR_POINT_BAND_PX
            count = int(np.count_nonzero(near))
            span = int(np.ptp(ys[near])) if count else 0
            prior_counts[side] = count
            if count >= 40 and span >= PAIR_MIN_COMMON_SPAN_PX:
                eligible.append((count, span, side, near))
        if eligible:
            _, _, prior_side, used = max(eligible, key=lambda item: (item[0], item[1]))

    used_x = xs[used]
    used_y = ys[used]
    record = {
        'semantic': component['semantic'],
        'component_id': component['component_id'],
        'area': component['area'],
        'pixel_count': component['pixel_count'],
        'used_point_count': int(used_x.size),
        'y_min': component['y_min'],
        'y_max': component['y_max'],
        'median_hsv': [component['median_h'], component['median_s'],
                       component['median_v']],
        'prior_side': prior_side,
        'prior_near_point_counts': prior_counts,
        'success': False,
        'failure_reason': None,
    }
    try:
        model, api_parameter = make_ransac(
            frame_index * 1009 + ordinal * 37 + component['component_id'])
        design = np.column_stack((used_y * used_y, used_y))
        model.fit(design, used_x)
        fit = coefficients(model)
        inliers = np.asarray(model.inlier_mask_, dtype=bool)
        if fit is None or inliers.size != used_x.size or np.count_nonzero(inliers) < 3:
            raise ValueError('invalid fitted estimator or fewer than 3 inliers')
        prediction_used = np.asarray(model.predict(design), dtype=np.float64)
        design_all = np.column_stack((ys * ys, ys))
        prediction_all = np.asarray(model.predict(design_all), dtype=np.float64)
        record.update({
            'success': True,
            'ransac_api_parameter': api_parameter,
            'coefficients': fit,
            'inlier_count': int(np.count_nonzero(inliers)),
            'inlier_ratio': float(np.mean(inliers)),
            'residual_used_all_px': residual_stats(used_x - prediction_used),
            'residual_used_inliers_px': residual_stats(
                used_x[inliers] - prediction_used[inliers]),
            'residual_full_component_px': residual_stats(xs - prediction_all),
        })
        record['_fit'] = np.asarray(fit, dtype=np.float64)
    except Exception as exc:  # A failed component must not abort a bag.
        record['failure_reason'] = '{}: {}'.format(type(exc).__name__, exc)
        record['inlier_count'] = 0
        record['inlier_ratio'] = 0.0
        record['coefficients'] = None
    record['fit_time_ms'] = (time.perf_counter() - started) * 1000.0
    return record


def semantic_score(left_semantic, right_semantic):
    pair = (left_semantic, right_semantic)
    scores = {
        ('YELLOW', 'BEIGE'): 1.00,
        ('YELLOW', 'NEUTRAL_WHITE'): 0.76,
        ('BEIGE', 'NEUTRAL_WHITE'): 0.82,
        ('NEUTRAL_WHITE', 'BEIGE'): 0.68,
        ('BEIGE', 'BEIGE'): 0.66,
        ('NEUTRAL_WHITE', 'NEUTRAL_WHITE'): 0.66,
        ('YELLOW', 'YELLOW'): 0.56,
        ('BEIGE', 'YELLOW'): 0.42,
        ('NEUTRAL_WHITE', 'YELLOW'): 0.38,
    }
    return scores.get(pair, 0.55)


def semantic_pair_name(left_semantic, right_semantic):
    white = {'BEIGE', 'NEUTRAL_WHITE'}
    if left_semantic in white and right_semantic in white:
        return 'WHITE-WHITE'
    return '{}-{}'.format(left_semantic, right_semantic)


def reject_pair(left, right, reason, extra=None):
    result = {
        'left_component': '{}:{}'.format(left['semantic'], left['component_id']),
        'right_component': '{}:{}'.format(right['semantic'], right['component_id']),
        'accepted': False,
        'rejection_reason': reason,
    }
    if extra:
        result.update(extra)
    return result


def evaluate_pair(first, second, image_width, image_height, expected_width,
                  previous):
    common_min = max(first['y_min'], second['y_min'])
    common_max = min(first['y_max'], second['y_max'])
    common_span = common_max - common_min
    basic = {
        'common_y_min': int(common_min),
        'common_y_max': int(common_max),
        'common_y_span': int(max(0, common_span)),
    }
    if common_max < common_min:
        return reject_pair(first, second, 'NO_COMMON_Y_RANGE', basic)
    if common_span < PAIR_MIN_COMMON_SPAN_PX:
        return reject_pair(first, second, 'COMMON_Y_SPAN_TOO_SHORT', basic)

    eval_y = float(round(common_min + PAIR_EVAL_FRACTION * common_span))
    first_x = evaluate(first['_fit'], eval_y)
    second_x = evaluate(second['_fit'], eval_y)
    if not np.isfinite(first_x) or not np.isfinite(second_x):
        return reject_pair(first, second, 'NONFINITE_EVALUATION', basic)
    if first_x <= second_x:
        left, right = first, second
        left_x, right_x = first_x, second_x
    else:
        left, right = second, first
        left_x, right_x = second_x, first_x
    basic.update({
        'left_component': '{}:{}'.format(left['semantic'], left['component_id']),
        'right_component': '{}:{}'.format(right['semantic'], right['component_id']),
        'eval_y': eval_y,
        'left_x': left_x,
        'right_x': right_x,
        'lane_width': right_x - left_x,
    })
    if not (0.0 <= left_x < image_width and 0.0 <= right_x < image_width):
        return reject_pair(left, right, 'OUT_OF_BOUNDS', basic)
    width = right_x - left_x
    if width <= 0.0:
        return reject_pair(left, right, 'LANE_CROSSING', basic)
    if width < WIDTH_MIN_RATIO * expected_width:
        return reject_pair(left, right, 'WIDTH_TOO_NARROW', basic)
    if width > WIDTH_MAX_RATIO * expected_width:
        return reject_pair(left, right, 'WIDTH_TOO_WIDE', basic)

    sample_y = np.linspace(common_min, common_max, 9)
    sampled_left = np.polyval(left['_fit'], sample_y)
    sampled_right = np.polyval(right['_fit'], sample_y)
    sampled_width = sampled_right - sampled_left
    if np.any(~np.isfinite(sampled_width)):
        return reject_pair(left, right, 'NONFINITE_COMMON_RANGE', basic)
    if np.any(sampled_width <= 0.0):
        return reject_pair(left, right, 'LANE_CROSSING_IN_COMMON_RANGE', basic)

    center = 0.5 * (left_x + right_x)
    quality = 0.5 * (left['inlier_ratio'] + right['inlier_ratio'])
    span_score = min(1.0, common_span / max(1.0, image_height * 0.30))
    width_score = math.exp(-abs(width - expected_width) / expected_width)
    edge_clearance = min(left_x, image_width - 1.0 - right_x)
    bounds_score = min(1.0, max(0.0, edge_clearance) / 20.0)
    jumps = {'left': None, 'right': None, 'center': None}
    if previous is None:
        temporal_score = 1.0
    else:
        jumps = {
            'left': abs(left_x - previous['left_x']),
            'right': abs(right_x - previous['right_x']),
            'center': abs(center - previous['center_x']),
        }
        temporal_score = math.exp(
            -np.mean(list(jumps.values())) / TEMPORAL_SCORE_SCALE_PX)
    identity_score = semantic_score(left['semantic'], right['semantic'])
    score_parts = {
        'inlier_quality': 0.30 * quality,
        'common_span': 0.15 * span_score,
        'width_geometry': 0.20 * width_score,
        'bounds': 0.05 * bounds_score,
        'temporal': 0.20 * temporal_score,
        'semantic': 0.10 * identity_score,
    }
    score = float(sum(score_parts.values()))
    return {
        **basic,
        'accepted': True,
        'rejection_reason': 'OK',
        'center_x': center,
        'expected_width': expected_width,
        'semantic_pair': semantic_pair_name(
            left['semantic'], right['semantic']),
        'semantic_pair_detailed': '{}-{}'.format(
            left['semantic'], right['semantic']),
        'score': score,
        'score_parts': score_parts,
        'temporal_jump': jumps,
        '_left': left,
        '_right': right,
    }


def compact_fit(fit):
    return {key: value for key, value in fit.items() if not key.startswith('_')}


def compact_pair(pair):
    return {key: value for key, value in pair.items() if not key.startswith('_')}


def choose_single(fits, image_width):
    if not fits:
        return None, 'RANSAC_NONE'
    candidates = []
    for fit in fits:
        eval_y = round(fit['y_min'] + PAIR_EVAL_FRACTION * (
            fit['y_max'] - fit['y_min']))
        x = evaluate(fit['_fit'], eval_y)
        if np.isfinite(x) and 0.0 <= x < image_width:
            candidates.append((fit['inlier_ratio'], fit['y_max'] - fit['y_min'],
                               fit['pixel_count'], fit, float(eval_y), x))
    if not candidates:
        return None, 'RANSAC_NONE'
    _, _, _, selected, eval_y, x = max(candidates, key=lambda item: item[:3])
    side = 'LEFT' if x <= image_width * 0.5 else 'RIGHT'
    return {
        'side': side,
        'semantic': selected['semantic'],
        'component_id': selected['component_id'],
        'eval_y': eval_y,
        'x': x,
        'inlier_ratio': selected['inlier_ratio'],
        '_fit': selected,
    }, 'RANSAC_SINGLE_' + side


def production_record(diagnostics):
    return {
        'mode': diagnostics.mode,
        'failure_reason': diagnostics.failure_reason,
        'eval_y': finite(diagnostics.eval_y),
        'left_x': finite(diagnostics.left_x_eval),
        'right_x': finite(diagnostics.right_x_eval),
        'lane_width': finite(diagnostics.lane_width_eval),
        'left_y_min': finite(diagnostics.left_y_min),
        'left_y_max': finite(diagnostics.left_y_max),
        'right_y_min': finite(diagnostics.right_y_min),
        'right_y_max': finite(diagnostics.right_y_max),
    }


class BagAccumulator:
    def __init__(self, path):
        self.path = path
        self.frames = 0
        self.production_modes = Counter()
        self.production_reasons = Counter()
        self.statuses = Counter()
        self.rejections = Counter()
        self.semantic_pairs = Counter()
        self.values = defaultdict(list)
        self.timings = defaultdict(list)
        self.comparison = Counter()
        self.trusted = Counter()
        self.dangerous = Counter()
        self.fit_failures = Counter()
        self.component_semantics = Counter()

    def summary(self):
        return {
            'path': str(self.path),
            'total_frames': self.frames,
            'production_modes': dict(self.production_modes),
            'production_failure_reasons': dict(self.production_reasons),
            'ransac_status': dict(self.statuses),
            'ransac_pair_valid_count': self.statuses['RANSAC_PAIR_VALID'],
            'ransac_pair_valid_rate': (
                self.statuses['RANSAC_PAIR_VALID'] / self.frames
                if self.frames else 0.0),
            'pair_rejection_reasons': dict(self.rejections),
            'selected_semantic_pairs': dict(self.semantic_pairs),
            'component_semantics': dict(self.component_semantics),
            'component_fit_failures': dict(self.fit_failures),
            'selected_eval_y': numeric_stats(self.values['eval_y']),
            'selected_left_x': numeric_stats(self.values['left_x']),
            'selected_right_x': numeric_stats(self.values['right_x']),
            'selected_lane_width': numeric_stats(self.values['lane_width']),
            'selected_center_x': numeric_stats(self.values['center_x']),
            'selected_left_inlier_ratio': numeric_stats(
                self.values['left_inlier_ratio']),
            'selected_right_inlier_ratio': numeric_stats(
                self.values['right_inlier_ratio']),
            'selected_common_y_span': numeric_stats(
                self.values['common_y_span']),
            'temporal_left_jump_px': numeric_stats(
                self.values['left_jump']),
            'temporal_right_jump_px': numeric_stats(
                self.values['right_jump']),
            'temporal_center_jump_px': numeric_stats(
                self.values['center_jump']),
            'production_comparison': {
                **dict(self.comparison),
                'left_abs_difference_px': numeric_stats(
                    self.values['production_left_diff']),
                'right_abs_difference_px': numeric_stats(
                    self.values['production_right_diff']),
                'center_abs_difference_px': numeric_stats(
                    self.values['production_center_diff']),
            },
            'dangerous_geometry': dict(self.dangerous),
            'trusted_reference': {
                **dict(self.trusted),
                'ransac_right_abs_error_px': numeric_stats(
                    self.values['trusted_ransac_right_error']),
                'production_right_abs_error_px': numeric_stats(
                    self.values['trusted_production_right_error']),
                'ransac_center_abs_error_px': numeric_stats(
                    self.values['trusted_ransac_center_error']),
                'production_center_abs_error_px': numeric_stats(
                    self.values['trusted_production_center_error']),
                'parking_white_distance_px': numeric_stats(
                    self.values['trusted_parking_distance']),
            },
            'processing_time_per_frame': {
                name: timing_stats(values)
                for name, values in self.timings.items()
            },
        }


def compare_production(selected, diagnostics, snapshot, accumulator):
    comparison = {'available': False}
    if diagnostics.mode != 'BOTH' or snapshot.left_fit is None or snapshot.right_fit is None:
        accumulator.comparison['unavailable'] += 1
        return comparison
    y = selected['eval_y']
    if not (diagnostics.left_y_min <= y <= diagnostics.left_y_max and
            diagnostics.right_y_min <= y <= diagnostics.right_y_max):
        accumulator.comparison['outside_production_observed_range'] += 1
        return comparison
    production_left = float(np.polyval(snapshot.left_fit, y))
    production_right = float(np.polyval(snapshot.right_fit, y))
    production_center = 0.5 * (production_left + production_right)
    differences = {
        'left': abs(selected['left_x'] - production_left),
        'right': abs(selected['right_x'] - production_right),
        'center': abs(selected['center_x'] - production_center),
    }
    accumulator.comparison['available'] += 1
    for name, value in differences.items():
        accumulator.values['production_' + name + '_diff'].append(value)
    if differences['center'] > LARGE_CENTER_DEVIATION_PX:
        accumulator.comparison['large_center_deviation'] += 1
    if max(differences['left'], differences['right']) > LARGE_BOUNDARY_DEVIATION_PX:
        accumulator.comparison['large_boundary_deviation'] += 1
    return {
        'available': True,
        'eval_y': y,
        'production_left_x': production_left,
        'production_right_x': production_right,
        'production_center_x': production_center,
        'absolute_difference_px': differences,
    }


def compare_trusted(header_ns, selected, reference, diagnostics, snapshot,
                    accumulator):
    result = {'matched': reference is not None}
    if reference is None:
        return result
    accumulator.trusted['matched_frames'] += 1
    if selected is None:
        accumulator.trusted['ransac_not_pair_valid'] += 1
        return result
    y = selected['eval_y']
    yellow = reference['yellow']
    beige = reference['beige']
    neutral = reference['neutral_white']
    if not (yellow['y_min'] <= y <= yellow['y_max'] and
            beige['y_min'] <= y <= beige['y_max']):
        accumulator.trusted['reference_outside_selected_eval_y'] += 1
        return result
    trusted_left = float(np.polyval(yellow['fit'], y))
    trusted_right = float(np.polyval(beige['fit'], y))
    trusted_center = 0.5 * (trusted_left + trusted_right)
    r_right_error = abs(selected['right_x'] - trusted_right)
    r_center_error = abs(selected['center_x'] - trusted_center)
    accumulator.values['trusted_ransac_right_error'].append(r_right_error)
    accumulator.values['trusted_ransac_center_error'].append(r_center_error)
    result.update({
        'comparison_available': True,
        'trusted_left_x': trusted_left,
        'trusted_right_x': trusted_right,
        'trusted_center_x': trusted_center,
        'ransac_right_error': r_right_error,
        'ransac_center_error': r_center_error,
    })
    accumulator.trusted['ransac_comparison_available'] += 1

    if neutral['y_min'] <= y <= neutral['y_max']:
        parking_x = float(np.polyval(neutral['fit'], y))
        parking_distance = abs(selected['right_x'] - parking_x)
        accumulator.values['trusted_parking_distance'].append(parking_distance)
        result['parking_white_x'] = parking_x
        result['ransac_parking_white_distance'] = parking_distance
        if parking_distance < r_right_error:
            accumulator.trusted['ransac_closer_to_parking_white'] += 1
        else:
            accumulator.trusted['ransac_closer_to_beige'] += 1

    if (diagnostics.mode == 'BOTH' and snapshot.left_fit is not None and
            snapshot.right_fit is not None and
            diagnostics.left_y_min <= y <= diagnostics.left_y_max and
            diagnostics.right_y_min <= y <= diagnostics.right_y_max):
        p_left = float(np.polyval(snapshot.left_fit, y))
        p_right = float(np.polyval(snapshot.right_fit, y))
        p_center = 0.5 * (p_left + p_right)
        p_right_error = abs(p_right - trusted_right)
        p_center_error = abs(p_center - trusted_center)
        accumulator.values['trusted_production_right_error'].append(p_right_error)
        accumulator.values['trusted_production_center_error'].append(p_center_error)
        if r_right_error < p_right_error:
            accumulator.trusted['ransac_right_improved'] += 1
        elif r_right_error > p_right_error:
            accumulator.trusted['ransac_right_worse'] += 1
        else:
            accumulator.trusted['ransac_right_tied'] += 1
        if r_center_error < p_center_error:
            accumulator.trusted['ransac_center_improved'] += 1
        elif r_center_error > p_center_error:
            accumulator.trusted['ransac_center_worse'] += 1
        else:
            accumulator.trusted['ransac_center_tied'] += 1
        result.update({
            'production_right_error': p_right_error,
            'production_center_error': p_center_error,
        })
    else:
        accumulator.trusted['production_comparison_unavailable'] += 1
    return result


def analyze_bag(node, bag_path, trusted, writer, max_frames=None,
                progress_every=100):
    accumulator = BagAccumulator(bag_path)
    previous = None
    for frame_index, (storage_ns, msg) in enumerate(iter_camera_messages(bag_path)):
        if max_frames is not None and frame_index >= max_frames:
            break
        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        h, w = image.shape[:2]
        frame_started = time.perf_counter()

        production_started = time.perf_counter()
        diagnostics = LaneDiagnostics()
        snapshot = ProductionFitSnapshot()
        production_mask = node.make_lane_mask(image)
        node.sliding_window_lane(
            production_mask, diagnostics, fit_snapshot=snapshot)
        accumulator.timings['production_baseline'].append(
            time.perf_counter() - production_started)

        extraction_started = time.perf_counter()
        components = semantic_components(node, image)
        accumulator.timings['semantic_extraction'].append(
            time.perf_counter() - extraction_started)
        for component in components:
            accumulator.component_semantics[component['semantic']] += 1

        fit_started = time.perf_counter()
        fits = [fit_component(component, previous, frame_index, ordinal)
                for ordinal, component in enumerate(components)]
        accumulator.timings['ransac_fitting'].append(
            time.perf_counter() - fit_started)
        for fit in fits:
            if not fit['success']:
                accumulator.fit_failures[fit['semantic']] += 1
        successful = [fit for fit in fits if fit['success']]

        expected_width = float(
            node.get_parameter('default_lane_width_px_640').value) * w / 640.0
        pairing_started = time.perf_counter()
        pairs = []
        for first_index in range(len(successful)):
            for second_index in range(first_index + 1, len(successful)):
                pair = evaluate_pair(
                    successful[first_index], successful[second_index],
                    w, h, expected_width, previous)
                pairs.append(pair)
                if not pair['accepted']:
                    accumulator.rejections[pair['rejection_reason']] += 1
        valid_pairs = [pair for pair in pairs if pair['accepted']]
        selected = max(valid_pairs, key=lambda item: item['score']) \
            if valid_pairs else None
        single = None
        if selected is not None:
            status = 'RANSAC_PAIR_VALID'
        else:
            single, status = choose_single(successful, w)
        accumulator.timings['pairing_selection'].append(
            time.perf_counter() - pairing_started)

        production = production_record(diagnostics)
        production_comparison = {'available': False}
        trusted_comparison = {'matched': False}
        selected_record = None
        single_record = None
        if selected is not None:
            left = selected['_left']
            right = selected['_right']
            selected_record = compact_pair(selected)
            selected_record.update({
                'left_semantic': left['semantic'],
                'left_component_id': left['component_id'],
                'right_semantic': right['semantic'],
                'right_component_id': right['component_id'],
                'left_inlier_ratio': left['inlier_ratio'],
                'right_inlier_ratio': right['inlier_ratio'],
                'left_coefficients': left['coefficients'],
                'right_coefficients': right['coefficients'],
            })
            accumulator.semantic_pairs[selected['semantic_pair']] += 1
            for key in ('eval_y', 'left_x', 'right_x', 'lane_width',
                        'center_x', 'common_y_span'):
                accumulator.values[key].append(selected[key])
            accumulator.values['left_inlier_ratio'].append(
                left['inlier_ratio'])
            accumulator.values['right_inlier_ratio'].append(
                right['inlier_ratio'])
            jumps = selected['temporal_jump']
            for side in ('left', 'right', 'center'):
                if jumps[side] is not None:
                    accumulator.values[side + '_jump'].append(jumps[side])
            if any(jumps[side] is not None and
                   jumps[side] > EXTREME_TEMPORAL_JUMP_PX
                   for side in ('left', 'right', 'center')):
                accumulator.dangerous['extreme_temporal_jump'] += 1

            # These should remain zero because they are hard gates.  Keeping
            # explicit counters makes accidental gate regressions visible.
            if selected['left_x'] >= selected['right_x']:
                accumulator.dangerous['selected_lane_crossing'] += 1
            if not (0 <= selected['left_x'] < w and
                    0 <= selected['right_x'] < w):
                accumulator.dangerous['selected_out_of_bounds'] += 1
            if not (WIDTH_MIN_RATIO * expected_width <= selected['lane_width']
                    <= WIDTH_MAX_RATIO * expected_width):
                accumulator.dangerous['selected_abnormal_width'] += 1

            production_comparison = compare_production(
                selected, diagnostics, snapshot, accumulator)
            previous = {
                'left': {'fit': left['_fit']},
                'right': {'fit': right['_fit']},
                'left_x': selected['left_x'],
                'right_x': selected['right_x'],
                'center_x': selected['center_x'],
                'width': selected['lane_width'],
            }
        else:
            previous = None
            if single is not None:
                selected_fit = single.pop('_fit')
                inferred_width = expected_width
                inferred_other = (
                    single['x'] + inferred_width if single['side'] == 'LEFT'
                    else single['x'] - inferred_width)
                single_record = {
                    **single,
                    'coefficients': selected_fit['coefficients'],
                    'inferred_other_x_for_statistics_only': inferred_other,
                    'inferred_width_source': 'EXPECTED_WIDTH_NO_VALID_PREVIOUS_PAIR',
                }

        header_ns = header_stamp_ns(msg)
        trusted_comparison = compare_trusted(
            header_ns, selected, trusted.get(header_ns), diagnostics,
            snapshot, accumulator)
        ransac_elapsed = time.perf_counter() - extraction_started
        accumulator.timings['offline_ransac_total'].append(ransac_elapsed)
        accumulator.timings['whole_frame_with_production'].append(
            time.perf_counter() - frame_started)
        accumulator.frames += 1
        accumulator.production_modes[diagnostics.mode] += 1
        accumulator.production_reasons[diagnostics.failure_reason] += 1
        accumulator.statuses[status] += 1

        writer.write({
            'bag': str(bag_path),
            'frame_index': frame_index,
            'storage_ns': storage_ns,
            'source_header_ns': header_ns,
            'image_width': w,
            'image_height': h,
            'production': production,
            'ransac_status': status,
            'components': [compact_fit(item) for item in fits],
            'pairs': [compact_pair(item) for item in pairs],
            'selected_pair': selected_record,
            'single_fallback': single_record,
            'production_comparison': production_comparison,
            'trusted_comparison': trusted_comparison,
            'timing_ms': {
                'offline_ransac_total': ransac_elapsed * 1000.0,
                'whole_frame_with_production': (
                    time.perf_counter() - frame_started) * 1000.0,
            },
        })
        if progress_every and accumulator.frames % progress_every == 0:
            print('[{}] {}/{} pair_valid={} status={}'.format(
                bag_path.name, accumulator.frames,
                max_frames if max_frames is not None else '?',
                accumulator.statuses['RANSAC_PAIR_VALID'], status), flush=True)
    return accumulator.summary()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='*', type=Path, default=DEFAULT_BAGS)
    parser.add_argument('--summary', type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument('--records', type=Path, default=DEFAULT_RECORDS)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--trusted-geometry', type=Path, default=DEFAULT_TRUSTED)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('--progress-every', type=int, default=100)
    args = parser.parse_args()

    missing = [str(path) for path in args.bags if not path.exists()]
    if missing:
        parser.error('missing bags: {}'.format(', '.join(missing)))
    print('scikit-learn version: {}'.format(sklearn.__version__), flush=True)
    _, api_parameter = make_ransac(0)
    print('RANSACRegressor estimator API parameter: {}'.format(
        api_parameter), flush=True)

    trusted = load_trusted_geometry(args.trusted_geometry)
    rclpy_args = ['offline_ransac_lane_analysis']
    if args.config.exists():
        rclpy_args.extend(['--ros-args', '--params-file', str(args.config)])
    rclpy.init(args=rclpy_args)
    node = PerceptionNode()
    try:
        bag_summaries = {}
        started = time.perf_counter()
        with JsonGzipArrayWriter(args.records) as writer:
            for bag_path in args.bags:
                print('Analyzing {}'.format(bag_path), flush=True)
                bag_summaries[str(bag_path)] = analyze_bag(
                    node, bag_path, trusted, writer,
                    max_frames=args.max_frames,
                    progress_every=args.progress_every)
        total_elapsed = time.perf_counter() - started
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = {
        'experiment': 'OFFLINE_ONLY_PIXEL_COORDINATE_RANSAC',
        'scikit_learn_version': sklearn.__version__,
        'ransac_estimator_api_parameter': api_parameter,
        'configuration': {
            'semantic_masks_and_roi': 'same thresholds/ROI as PerceptionNode',
            'coordinate_model': 'x = a*y^2 + b*y + c; no BEV',
            'component_policy': 'all qualifying connected components',
            'residual_threshold_px': RANSAC_RESIDUAL_THRESHOLD_PX,
            'max_trials': RANSAC_MAX_TRIALS,
            'min_samples': RANSAC_MIN_SAMPLES,
            'prior_point_band_px': PRIOR_POINT_BAND_PX,
            'pair_min_common_span_px': PAIR_MIN_COMMON_SPAN_PX,
            'pair_eval_y_policy': (
                'round(common_y_min + 0.70 * (common_y_max-common_y_min))'),
            'width_ratio_range': [WIDTH_MIN_RATIO, WIDTH_MAX_RATIO],
            'pair_score_weights': {
                'inlier_quality': 0.30,
                'common_span': 0.15,
                'width_geometry': 0.20,
                'bounds': 0.05,
                'temporal': 0.20,
                'semantic': 0.10,
            },
            'temporal_is_hard_gate': False,
            'single_fallback_is_pair_valid': False,
            'large_center_deviation_px': LARGE_CENTER_DEVIATION_PX,
            'large_boundary_deviation_px': LARGE_BOUNDARY_DEVIATION_PX,
            'extreme_temporal_jump_px': EXTREME_TEMPORAL_JUMP_PX,
        },
        'trusted_reference_frames_loaded': len(trusted),
        'total_wall_time_seconds': total_elapsed,
        'bags': bag_summaries,
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    with args.summary.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False),
          flush=True)


if __name__ == '__main__':
    main()
