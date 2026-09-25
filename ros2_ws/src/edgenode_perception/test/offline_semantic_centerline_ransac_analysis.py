#!/usr/bin/env python3
"""Offline-only RANSAC on row-median semantic component centerlines."""

import argparse
import inspect
import json
import math
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
import sklearn
from sklearn.linear_model import LinearRegression, RANSACRegressor

import offline_ransac_lane_analysis as pixel
from edgenode_perception.perception_node import (
    LaneDiagnostics,
    PerceptionNode,
    ProductionFitSnapshot,
)


DEFAULT_SUMMARY = Path(
    '/workspace/debug_frames/semantic_centerline_ransac_summary.json')
DEFAULT_RECORDS = Path(
    '/workspace/debug_frames/semantic_centerline_ransac_records.json.gz')

Y_BIN_SIZE_PX = 2
RANSAC_RESIDUAL_THRESHOLD_PX = 3.0
RANSAC_MAX_TRIALS = 50
RANSAC_MIN_POINT_FRACTION = 0.30
RANSAC_MIN_SAMPLES_FLOOR = 6
PAIR_SAMPLE_COUNT = 7
PAIR_MIN_VALID_SAMPLES = 6
PAIR_MIN_COMMON_SPAN_PX = 20
WIDTH_MIN_RATIO = 0.60
WIDTH_MAX_RATIO = 1.55
MAX_WIDTH_VARIATION_RATIO = 1.25
MAX_ABS_WIDTH_SLOPE_PX_PER_Y = 10.0
MAX_DERIVATIVE_DIFFERENCE = 12.0
MAX_CENTER_VARIATION_RATIO = 0.35
TEMPORAL_SCORE_SCALE_PX = 45.0
EXTREME_TEMPORAL_JUMP_PX = 75.0
RAW_RANSAC_MEAN_MS = 61.379469604409785


def centerline_points(component):
    """Collapse a thick component to one robust point for every 2-pixel y bin."""
    ys = component['ys']
    xs = component['xs']
    bins = ((ys.astype(np.int32) - component['y_min']) // Y_BIN_SIZE_PX)
    center_y = []
    center_x = []
    bin_pixel_counts = []
    for bin_id in np.unique(bins):
        selected = bins == bin_id
        center_y.append(float(np.median(ys[selected])))
        center_x.append(float(np.median(xs[selected])))
        bin_pixel_counts.append(int(np.count_nonzero(selected)))
    return (
        np.asarray(center_y, dtype=np.float64),
        np.asarray(center_x, dtype=np.float64),
        bin_pixel_counts,
    )


def make_ransac(point_count, random_state):
    min_samples = max(
        RANSAC_MIN_SAMPLES_FLOOR,
        int(math.ceil(point_count * RANSAC_MIN_POINT_FRACTION)),
    )
    min_samples = min(point_count, min_samples)
    kwargs = {
        'min_samples': min_samples,
        'residual_threshold': RANSAC_RESIDUAL_THRESHOLD_PX,
        'max_trials': RANSAC_MAX_TRIALS,
        'random_state': int(random_state),
        'loss': 'absolute_loss',
    }
    api_parameter = (
        'estimator' if 'estimator' in inspect.signature(
            RANSACRegressor).parameters else 'base_estimator')
    kwargs[api_parameter] = LinearRegression(fit_intercept=True)
    return RANSACRegressor(**kwargs), api_parameter, min_samples


def fit_component(component, frame_index, ordinal):
    started = time.perf_counter()
    ys, xs, bin_counts = centerline_points(component)
    result = {
        'semantic': component['semantic'],
        'component_id': component['component_id'],
        'area': component['area'],
        'raw_pixel_count': component['pixel_count'],
        'centerline_point_count': int(len(xs)),
        'compression_ratio': float(len(xs) / component['pixel_count']),
        'y_bin_size_px': Y_BIN_SIZE_PX,
        'bin_pixel_count': pixel.numeric_stats(bin_counts),
        'y_min': float(np.min(ys)) if len(ys) else None,
        'y_max': float(np.max(ys)) if len(ys) else None,
        'median_hsv': [component['median_h'], component['median_s'],
                       component['median_v']],
        'success': False,
        'failure_reason': None,
    }
    try:
        if len(xs) < RANSAC_MIN_SAMPLES_FLOOR:
            raise ValueError('fewer than {} centerline points'.format(
                RANSAC_MIN_SAMPLES_FLOOR))
        model, api_parameter, min_samples = make_ransac(
            len(xs), ordinal * 37 +
            component['component_id'])
        design = np.column_stack((ys * ys, ys))
        model.fit(design, xs)
        fit = pixel.coefficients(model)
        inliers = np.asarray(model.inlier_mask_, dtype=bool)
        if fit is None or inliers.size != len(xs) or np.count_nonzero(inliers) < 3:
            raise ValueError('invalid estimator or fewer than 3 inliers')
        prediction = np.asarray(model.predict(design), dtype=np.float64)
        residual = xs - prediction
        result.update({
            'success': True,
            'ransac_api_parameter': api_parameter,
            'ransac_min_samples': min_samples,
            'coefficients': fit,
            'inlier_count': int(np.count_nonzero(inliers)),
            'inlier_ratio': float(np.mean(inliers)),
            'residual_all_px': pixel.numeric_stats(np.abs(residual)),
            'residual_inliers_px': pixel.numeric_stats(
                np.abs(residual[inliers])),
            'centerline_y': ys.tolist(),
            'centerline_x': xs.tolist(),
        })
        result['_fit'] = np.asarray(fit, dtype=np.float64)
    except Exception as exc:
        result.update({
            'failure_reason': '{}: {}'.format(type(exc).__name__, exc),
            'coefficients': None,
            'inlier_count': 0,
            'inlier_ratio': 0.0,
        })
    result['fit_time_ms'] = (time.perf_counter() - started) * 1000.0
    return result


def component_name(component):
    return '{}:{}'.format(component['semantic'], component['component_id'])


def compact(value):
    return {key: item for key, item in value.items()
            if not key.startswith('_')}


def reject_pair(left, right, reason, values=None, category='REJECTED'):
    result = {
        'left_component': component_name(left),
        'right_component': component_name(right),
        'left_semantic': left['semantic'],
        'right_semantic': right['semantic'],
        'accepted': False,
        'candidate_category': category,
        'rejection_reason': reason,
    }
    if values:
        result.update(values)
    return result


def semantic_preference(left_semantic, right_semantic):
    scores = {
        ('YELLOW', 'BEIGE'): 1.00,
        ('YELLOW', 'NEUTRAL_WHITE'): 0.78,
        ('BEIGE', 'NEUTRAL_WHITE'): 0.82,
        ('NEUTRAL_WHITE', 'BEIGE'): 0.72,
        ('BEIGE', 'BEIGE'): 0.68,
        ('NEUTRAL_WHITE', 'NEUTRAL_WHITE'): 0.68,
    }
    return scores.get((left_semantic, right_semantic), 0.55)


def semantic_pair_name(left_semantic, right_semantic):
    if left_semantic in {'BEIGE', 'NEUTRAL_WHITE'} and \
            right_semantic in {'BEIGE', 'NEUTRAL_WHITE'}:
        return 'WHITE-WHITE'
    if right_semantic == 'NEUTRAL_WHITE':
        return '{}-NEUTRAL'.format(left_semantic)
    return '{}-{}'.format(left_semantic, right_semantic)


def evaluate_pair(first, second, image_width, image_height, expected_width,
                  previous):
    yellow_yellow_pair = (
        first['semantic'] == second['semantic'] == 'YELLOW')
    default_category = (
        'YELLOW_YELLOW_CANDIDATE' if yellow_yellow_pair else 'REJECTED')

    def reject_current(left, right, reason, values=None):
        return reject_pair(
            left, right, reason, values, category=default_category)

    common_min = max(first['y_min'], second['y_min'])
    common_max = min(first['y_max'], second['y_max'])
    common_span = common_max - common_min
    basic = {
        'common_y_min': pixel.finite(common_min),
        'common_y_max': pixel.finite(common_max),
        'common_y_span': pixel.finite(max(0.0, common_span)),
        'sample_count': PAIR_SAMPLE_COUNT,
    }
    if common_max < common_min:
        return reject_current(first, second, 'NO_COMMON_Y_RANGE', basic)
    if common_span < PAIR_MIN_COMMON_SPAN_PX:
        return reject_current(first, second, 'COMMON_Y_SPAN_TOO_SHORT', basic)

    sample_y = np.linspace(common_min, common_max, PAIR_SAMPLE_COUNT)
    first_x = np.polyval(first['_fit'], sample_y)
    second_x = np.polyval(second['_fit'], sample_y)
    finite = np.isfinite(first_x) & np.isfinite(second_x)
    if np.count_nonzero(finite) < PAIR_MIN_VALID_SAMPLES:
        return reject_current(first, second, 'NONFINITE_SAMPLE_GEOMETRY', basic)

    if float(np.median(first_x[finite])) <= float(np.median(second_x[finite])):
        left, right = first, second
        left_x, right_x = first_x, second_x
    else:
        left, right = second, first
        left_x, right_x = second_x, first_x
    left_semantic = left['semantic']
    right_semantic = right['semantic']
    basic.update({
        'left_component': component_name(left),
        'right_component': component_name(right),
        'left_semantic': left_semantic,
        'right_semantic': right_semantic,
        'semantic_pair': semantic_pair_name(left_semantic, right_semantic),
        'semantic_pair_detailed': '{}-{}'.format(
            left_semantic, right_semantic),
    })

    # K-City semantics permit yellow on the left and white on the right.  The
    # opposite ordering is an explicit contradiction, not merely a low score.
    if left_semantic in {'BEIGE', 'NEUTRAL_WHITE'} and right_semantic == 'YELLOW':
        return reject_pair(
            left, right, 'SEMANTIC_ORDER_REVERSED', basic,
            category='SEMANTIC_REVERSED')

    in_bounds = (
        finite & (left_x >= 0.0) & (left_x < image_width) &
        (right_x >= 0.0) & (right_x < image_width))
    ordered = finite & (right_x > left_x)
    valid = in_bounds & ordered
    valid_count = int(np.count_nonzero(valid))
    sample_records = [{
        'y': float(y),
        'left_x': pixel.finite(lx),
        'right_x': pixel.finite(rx),
        'width': pixel.finite(rx - lx),
        'center': pixel.finite(0.5 * (lx + rx)),
        'finite': bool(is_finite),
        'in_bounds': bool(bounds),
        'ordered': bool(order),
    } for y, lx, rx, is_finite, bounds, order in zip(
        sample_y, left_x, right_x, finite, in_bounds, ordered)]
    basic.update({
        'valid_sample_count': valid_count,
        'valid_sample_ratio': valid_count / PAIR_SAMPLE_COUNT,
        'bounds_fraction': float(np.mean(in_bounds)),
        'order_fraction': float(np.mean(ordered)),
        'samples': sample_records,
    })
    if np.count_nonzero(ordered) < PAIR_MIN_VALID_SAMPLES:
        return reject_current(left, right, 'ORDER_INSUFFICIENT_SAMPLES', basic)
    if valid_count < PAIR_MIN_VALID_SAMPLES:
        return reject_current(left, right, 'BOUNDS_INSUFFICIENT_SAMPLES', basic)

    valid_y = sample_y[valid]
    valid_left = left_x[valid]
    valid_right = right_x[valid]
    widths = valid_right - valid_left
    centers = 0.5 * (valid_left + valid_right)
    width_median = float(np.median(widths))
    width_min = float(np.min(widths))
    width_max = float(np.max(widths))
    width_variation = width_max - width_min
    width_slope = float(np.polyfit(valid_y, widths, 1)[0])
    left_derivative = 2.0 * left['_fit'][0] * valid_y + left['_fit'][1]
    right_derivative = 2.0 * right['_fit'][0] * valid_y + right['_fit'][1]
    derivative_difference = np.abs(right_derivative - left_derivative)
    center_variation = float(np.ptp(centers))
    eval_y = float(np.median(valid_y))
    eval_left_x = pixel.evaluate(left['_fit'], eval_y)
    eval_right_x = pixel.evaluate(right['_fit'], eval_y)
    eval_center = 0.5 * (eval_left_x + eval_right_x)
    geometry = {
        'eval_y': eval_y,
        'left_x': eval_left_x,
        'right_x': eval_right_x,
        'center_x': eval_center,
        'width_at_eval_y': eval_right_x - eval_left_x,
        'width_median': width_median,
        'width_min': width_min,
        'width_max': width_max,
        'width_variation': width_variation,
        'width_slope_px_per_y': width_slope,
        'left_derivative_p50': float(np.median(left_derivative)),
        'right_derivative_p50': float(np.median(right_derivative)),
        'derivative_difference_p50': float(np.median(derivative_difference)),
        'derivative_difference_max': float(np.max(derivative_difference)),
        'center_min': float(np.min(centers)),
        'center_max': float(np.max(centers)),
        'center_variation': center_variation,
        'expected_width': expected_width,
    }
    basic.update(geometry)
    if width_median < WIDTH_MIN_RATIO * expected_width:
        return reject_current(left, right, 'MEDIAN_WIDTH_TOO_NARROW', basic)
    if width_median > WIDTH_MAX_RATIO * expected_width:
        return reject_current(left, right, 'MEDIAN_WIDTH_TOO_WIDE', basic)
    if width_variation > MAX_WIDTH_VARIATION_RATIO * expected_width:
        return reject_current(left, right, 'WIDTH_VARIATION_TOO_LARGE', basic)
    if abs(width_slope) > MAX_ABS_WIDTH_SLOPE_PX_PER_Y:
        return reject_current(left, right, 'WIDTH_SLOPE_TOO_LARGE', basic)
    if np.max(derivative_difference) > MAX_DERIVATIVE_DIFFERENCE:
        return reject_current(left, right, 'DERIVATIVE_DIFFERENCE_TOO_LARGE', basic)
    if center_variation > MAX_CENTER_VARIATION_RATIO * expected_width:
        return reject_current(left, right, 'CENTER_VARIATION_TOO_LARGE', basic)

    # Yellow/yellow geometry is retained for diagnosis but cannot become a
    # trusted pair in this experiment.
    yellow_yellow = yellow_yellow_pair

    quality = 0.5 * (left['inlier_ratio'] + right['inlier_ratio'])
    span_score = min(1.0, common_span / max(1.0, image_height * 0.30))
    width_score = math.exp(-abs(width_median - expected_width) / expected_width)
    consistency_score = math.exp(
        -width_variation / max(1.0, expected_width))
    bounds_score = valid_count / PAIR_SAMPLE_COUNT
    jumps = {'left': None, 'right': None, 'center': None}
    if previous is None:
        temporal_score = 1.0
    else:
        jumps = {
            'left': abs(eval_left_x - previous['left_x']),
            'right': abs(eval_right_x - previous['right_x']),
            'center': abs(eval_center - previous['center_x']),
        }
        temporal_score = math.exp(
            -float(np.mean(list(jumps.values()))) /
            TEMPORAL_SCORE_SCALE_PX)
    score_parts = {
        'inlier_quality': 0.25 * quality,
        'common_span': 0.10 * span_score,
        'width_geometry': 0.20 * width_score,
        'multi_sample_consistency': 0.15 * consistency_score,
        'bounds': 0.05 * bounds_score,
        'temporal': 0.15 * temporal_score,
        'semantic': 0.10 * semantic_preference(
            left_semantic, right_semantic),
    }
    result = {
        **basic,
        'score': float(sum(score_parts.values())),
        'score_parts': score_parts,
        'temporal_jump': jumps,
        '_left': left,
        '_right': right,
    }
    if yellow_yellow:
        result.update({
            'accepted': False,
            'candidate_category': 'YELLOW_YELLOW_CANDIDATE',
            'rejection_reason': 'YELLOW_YELLOW_SEPARATE_CATEGORY',
        })
    else:
        result.update({
            'accepted': True,
            'candidate_category': 'TRUSTED_PAIR',
            'rejection_reason': 'OK',
        })
    return result


def choose_single(fits, image_width):
    candidates = []
    for fit in fits:
        eval_y = 0.5 * (fit['y_min'] + fit['y_max'])
        x = pixel.evaluate(fit['_fit'], eval_y)
        if np.isfinite(x) and 0.0 <= x < image_width:
            candidates.append((
                fit['inlier_ratio'], fit['y_max'] - fit['y_min'],
                fit['centerline_point_count'], fit, eval_y, x))
    if not candidates:
        return None
    _, _, _, selected, eval_y, x = max(
        candidates, key=lambda item: item[:3])
    return {
        'side': 'LEFT' if x <= image_width * 0.5 else 'RIGHT',
        'semantic': selected['semantic'],
        'component_id': selected['component_id'],
        'eval_y': float(eval_y),
        'x': float(x),
        'inlier_ratio': selected['inlier_ratio'],
        'coefficients': selected['coefficients'],
    }


class BagAccumulator:
    def __init__(self, path):
        self.path = path
        self.frames = 0
        self.production_modes = Counter()
        self.production_reasons = Counter()
        self.statuses = Counter()
        self.single_sides = Counter()
        self.rejections = Counter()
        self.categories = Counter()
        self.semantic_pairs = Counter()
        self.component_semantics = Counter()
        self.fit_failures = Counter()
        self.comparison = Counter()
        self.trusted = Counter()
        self.dangerous = Counter()
        self.values = defaultdict(list)
        self.timings = defaultdict(list)

    def summary(self):
        timing = {name: pixel.timing_stats(values)
                  for name, values in self.timings.items()}
        return {
            'path': str(self.path),
            'total_frames': self.frames,
            'production_modes': dict(self.production_modes),
            'production_failure_reasons': dict(self.production_reasons),
            'ransac_status': dict(self.statuses),
            'single_fallback_side': dict(self.single_sides),
            'pair_valid_count': self.statuses['RANSAC_PAIR_VALID'],
            'pair_valid_rate': (
                self.statuses['RANSAC_PAIR_VALID'] / self.frames
                if self.frames else 0.0),
            'pair_candidate_categories': dict(self.categories),
            'pair_rejection_reasons': dict(self.rejections),
            'selected_semantic_pairs': dict(self.semantic_pairs),
            'component_semantics': dict(self.component_semantics),
            'component_fit_failures': dict(self.fit_failures),
            'centerline_point_count': pixel.numeric_stats(
                self.values['centerline_points']),
            'raw_pixel_count': pixel.numeric_stats(
                self.values['raw_pixels']),
            'point_compression_ratio': pixel.numeric_stats(
                self.values['compression_ratio']),
            'selected_eval_y': pixel.numeric_stats(self.values['eval_y']),
            'selected_left_x': pixel.numeric_stats(self.values['left_x']),
            'selected_right_x': pixel.numeric_stats(self.values['right_x']),
            'selected_width_at_eval_y': pixel.numeric_stats(
                self.values['width_at_eval_y']),
            'selected_width_median': pixel.numeric_stats(
                self.values['width_median']),
            'selected_width_min': pixel.numeric_stats(
                self.values['width_min']),
            'selected_width_max': pixel.numeric_stats(
                self.values['width_max']),
            'selected_width_variation': pixel.numeric_stats(
                self.values['width_variation']),
            'selected_width_slope': pixel.numeric_stats(
                self.values['width_slope']),
            'selected_derivative_difference_max': pixel.numeric_stats(
                self.values['derivative_difference_max']),
            'selected_center_x': pixel.numeric_stats(self.values['center_x']),
            'selected_center_variation': pixel.numeric_stats(
                self.values['center_variation']),
            'selected_common_y_span': pixel.numeric_stats(
                self.values['common_y_span']),
            'selected_left_inlier_ratio': pixel.numeric_stats(
                self.values['left_inlier_ratio']),
            'selected_right_inlier_ratio': pixel.numeric_stats(
                self.values['right_inlier_ratio']),
            'temporal_left_jump_px': pixel.numeric_stats(
                self.values['left_jump']),
            'temporal_right_jump_px': pixel.numeric_stats(
                self.values['right_jump']),
            'temporal_center_jump_px': pixel.numeric_stats(
                self.values['center_jump']),
            'reference_y_296': {
                'left_x': pixel.numeric_stats(self.values['y296_left']),
                'right_x': pixel.numeric_stats(self.values['y296_right']),
                'width': pixel.numeric_stats(self.values['y296_width']),
            },
            'production_comparison': {
                **dict(self.comparison),
                'left_abs_difference_px': pixel.numeric_stats(
                    self.values['production_left_diff']),
                'right_abs_difference_px': pixel.numeric_stats(
                    self.values['production_right_diff']),
                'center_abs_difference_px': pixel.numeric_stats(
                    self.values['production_center_diff']),
            },
            'dangerous_geometry': dict(self.dangerous),
            'trusted_reference': {
                **dict(self.trusted),
                'ransac_right_abs_error_px': pixel.numeric_stats(
                    self.values['trusted_ransac_right_error']),
                'production_right_abs_error_px': pixel.numeric_stats(
                    self.values['trusted_production_right_error']),
                'ransac_center_abs_error_px': pixel.numeric_stats(
                    self.values['trusted_ransac_center_error']),
                'production_center_abs_error_px': pixel.numeric_stats(
                    self.values['trusted_production_center_error']),
                'parking_white_distance_px': pixel.numeric_stats(
                    self.values['trusted_parking_distance']),
            },
            'processing_time_per_frame': timing,
        }


def record_selected(selected, accumulator):
    left = selected['_left']
    right = selected['_right']
    accumulator.semantic_pairs[selected['semantic_pair']] += 1
    mappings = {
        'eval_y': 'eval_y',
        'left_x': 'left_x',
        'right_x': 'right_x',
        'width_at_eval_y': 'width_at_eval_y',
        'width_median': 'width_median',
        'width_min': 'width_min',
        'width_max': 'width_max',
        'width_variation': 'width_variation',
        'width_slope_px_per_y': 'width_slope',
        'derivative_difference_max': 'derivative_difference_max',
        'center_x': 'center_x',
        'center_variation': 'center_variation',
        'common_y_span': 'common_y_span',
    }
    for source, destination in mappings.items():
        accumulator.values[destination].append(selected[source])
    accumulator.values['left_inlier_ratio'].append(left['inlier_ratio'])
    accumulator.values['right_inlier_ratio'].append(right['inlier_ratio'])
    jumps = selected['temporal_jump']
    for side in ('left', 'right', 'center'):
        if jumps[side] is not None:
            accumulator.values[side + '_jump'].append(jumps[side])
    if any(value is not None and value > EXTREME_TEMPORAL_JUMP_PX
           for value in jumps.values()):
        accumulator.dangerous['extreme_temporal_jump'] += 1
    if left['y_min'] <= 296.0 <= left['y_max'] and \
            right['y_min'] <= 296.0 <= right['y_max']:
        left_x = pixel.evaluate(left['_fit'], 296.0)
        right_x = pixel.evaluate(right['_fit'], 296.0)
        accumulator.values['y296_left'].append(left_x)
        accumulator.values['y296_right'].append(right_x)
        accumulator.values['y296_width'].append(right_x - left_x)


def analyze_bag(node, bag_path, trusted, writer, max_frames=None,
                progress_every=250):
    accumulator = BagAccumulator(bag_path)
    previous = None
    for frame_index, (storage_ns, msg) in enumerate(
            pixel.iter_camera_messages(bag_path)):
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
        node.sliding_window_lane(
            node.make_lane_mask(image), diagnostics, fit_snapshot=snapshot)
        accumulator.timings['production_baseline'].append(
            time.perf_counter() - production_started)

        offline_started = time.perf_counter()
        extraction_started = time.perf_counter()
        components = pixel.semantic_components(node, image)
        accumulator.timings['semantic_extraction'].append(
            time.perf_counter() - extraction_started)
        for component in components:
            accumulator.component_semantics[component['semantic']] += 1

        fitting_started = time.perf_counter()
        fits = [fit_component(component, frame_index, ordinal)
                for ordinal, component in enumerate(components)]
        accumulator.timings['centerline_ransac_fitting'].append(
            time.perf_counter() - fitting_started)
        for fit in fits:
            accumulator.values['centerline_points'].append(
                fit['centerline_point_count'])
            accumulator.values['raw_pixels'].append(fit['raw_pixel_count'])
            accumulator.values['compression_ratio'].append(
                fit['compression_ratio'])
            if not fit['success']:
                accumulator.fit_failures[fit['semantic']] += 1
        successful = [fit for fit in fits if fit['success']]

        expected_width = float(node.get_parameter(
            'default_lane_width_px_640').value) * w / 640.0
        pairing_started = time.perf_counter()
        pairs = []
        for first_index in range(len(successful)):
            for second_index in range(first_index + 1, len(successful)):
                pair = evaluate_pair(
                    successful[first_index], successful[second_index],
                    w, h, expected_width, previous)
                pairs.append(pair)
                accumulator.categories[pair['candidate_category']] += 1
                if not pair['accepted']:
                    accumulator.rejections[pair['rejection_reason']] += 1
        valid_pairs = [item for item in pairs if item['accepted']]
        yellow_pairs = [item for item in pairs if
                        item['candidate_category'] ==
                        'YELLOW_YELLOW_CANDIDATE']
        selected = max(valid_pairs, key=lambda item: item['score']) \
            if valid_pairs else None
        yellow_candidate = max(yellow_pairs, key=lambda item: item.get('score', float('-inf'))) \
            if yellow_pairs else None
        single = choose_single(successful, w)
        accumulator.timings['pairing_selection'].append(
            time.perf_counter() - pairing_started)

        if selected is not None:
            status = 'RANSAC_PAIR_VALID'
            record_selected(selected, accumulator)
            left = selected['_left']
            right = selected['_right']
            previous = {
                'left_x': selected['left_x'],
                'right_x': selected['right_x'],
                'center_x': selected['center_x'],
                'left_fit': left['_fit'],
                'right_fit': right['_fit'],
            }
        elif yellow_candidate is not None:
            status = 'RANSAC_YELLOW_YELLOW_CANDIDATE'
            previous = None
        elif single is not None:
            status = 'RANSAC_SINGLE_' + single['side']
            previous = None
        else:
            status = 'RANSAC_NONE'
            previous = None
        if (single is not None and selected is None and
                yellow_candidate is None):
            accumulator.single_sides[single['side']] += 1

        production_comparison = {'available': False}
        if selected is not None:
            production_comparison = pixel.compare_production(
                selected, diagnostics, snapshot, accumulator)
        trusted_comparison = pixel.compare_trusted(
            pixel.header_stamp_ns(msg), selected,
            trusted.get(pixel.header_stamp_ns(msg)), diagnostics,
            snapshot, accumulator)

        elapsed = time.perf_counter() - offline_started
        accumulator.timings['offline_centerline_ransac_total'].append(elapsed)
        accumulator.timings['whole_frame_with_production'].append(
            time.perf_counter() - frame_started)
        accumulator.frames += 1
        accumulator.production_modes[diagnostics.mode] += 1
        accumulator.production_reasons[diagnostics.failure_reason] += 1
        accumulator.statuses[status] += 1

        def selected_record(pair):
            if pair is None:
                return None
            record = compact(pair)
            if '_left' not in pair or '_right' not in pair:
                return record
            record.update({
                'left_coefficients': pair['_left']['coefficients'],
                'right_coefficients': pair['_right']['coefficients'],
                'left_inlier_ratio': pair['_left']['inlier_ratio'],
                'right_inlier_ratio': pair['_right']['inlier_ratio'],
            })
            return record

        writer.write({
            'bag': str(bag_path),
            'frame_index': frame_index,
            'storage_ns': storage_ns,
            'source_header_ns': pixel.header_stamp_ns(msg),
            'image_width': w,
            'image_height': h,
            'production': pixel.production_record(diagnostics),
            'ransac_status': status,
            'components': [compact(item) for item in fits],
            'pairs': [compact(item) for item in pairs],
            'selected_pair': selected_record(selected),
            'yellow_yellow_candidate': selected_record(yellow_candidate),
            'single_fallback': (single if selected is None and
                                yellow_candidate is None else None),
            'production_comparison': production_comparison,
            'trusted_comparison': trusted_comparison,
            'timing_ms': {
                'offline_centerline_ransac_total': elapsed * 1000.0,
                'whole_frame_with_production': (
                    time.perf_counter() - frame_started) * 1000.0,
            },
        })
        if progress_every and accumulator.frames % progress_every == 0:
            print('[{}] {} frames pair={} yy={} status={}'.format(
                bag_path.name, accumulator.frames,
                accumulator.statuses['RANSAC_PAIR_VALID'],
                accumulator.statuses['RANSAC_YELLOW_YELLOW_CANDIDATE'],
                status), flush=True)
    return accumulator.summary()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='*', type=Path,
                        default=pixel.DEFAULT_BAGS)
    parser.add_argument('--summary', type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument('--records', type=Path, default=DEFAULT_RECORDS)
    parser.add_argument('--config', type=Path, default=pixel.DEFAULT_CONFIG)
    parser.add_argument('--trusted-geometry', type=Path,
                        default=pixel.DEFAULT_TRUSTED)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('--progress-every', type=int, default=250)
    args = parser.parse_args()

    missing = [str(path) for path in args.bags if not path.exists()]
    if missing:
        parser.error('missing bags: {}'.format(', '.join(missing)))
    _, api_parameter, _ = make_ransac(20, 0)
    print('scikit-learn version: {}'.format(sklearn.__version__), flush=True)
    print('RANSACRegressor estimator API parameter: {}'.format(
        api_parameter), flush=True)

    trusted = pixel.load_trusted_geometry(args.trusted_geometry)
    rclpy_args = ['offline_semantic_centerline_ransac_analysis']
    if args.config.exists():
        rclpy_args.extend(['--ros-args', '--params-file', str(args.config)])
    rclpy.init(args=rclpy_args)
    node = PerceptionNode()
    try:
        started = time.perf_counter()
        bag_summaries = {}
        with pixel.JsonGzipArrayWriter(args.records) as writer:
            for bag_path in args.bags:
                print('Analyzing {}'.format(bag_path), flush=True)
                bag_summaries[str(bag_path)] = analyze_bag(
                    node, bag_path, trusted, writer,
                    max_frames=args.max_frames,
                    progress_every=args.progress_every)
        wall_time = time.perf_counter() - started
    finally:
        node.destroy_node()
        rclpy.shutdown()

    total_frames = sum(item['total_frames']
                       for item in bag_summaries.values())
    weighted_mean_ms = (
        sum(item['total_frames'] * item['processing_time_per_frame'][
            'offline_centerline_ransac_total']['mean_ms']
            for item in bag_summaries.values()) / max(1, total_frames))
    summary = {
        'experiment': 'OFFLINE_ONLY_SEMANTIC_CENTERLINE_RANSAC',
        'scikit_learn_version': sklearn.__version__,
        'ransac_estimator_api_parameter': api_parameter,
        'configuration': {
            'coordinate_model': 'x = a*y^2 + b*y + c; no BEV',
            'centerline_policy': 'median x/y in 2-pixel y bins',
            'y_bin_size_px': Y_BIN_SIZE_PX,
            'ransac_residual_threshold_px': RANSAC_RESIDUAL_THRESHOLD_PX,
            'ransac_max_trials': RANSAC_MAX_TRIALS,
            'ransac_dynamic_min_samples': (
                'max(6, ceil(0.30 * centerline_point_count))'),
            'pair_sample_count': PAIR_SAMPLE_COUNT,
            'pair_min_valid_samples': PAIR_MIN_VALID_SAMPLES,
            'pair_min_common_span_px': PAIR_MIN_COMMON_SPAN_PX,
            'width_ratio_range_for_median': [
                WIDTH_MIN_RATIO, WIDTH_MAX_RATIO],
            'max_width_variation_ratio': MAX_WIDTH_VARIATION_RATIO,
            'max_abs_width_slope_px_per_y':
                MAX_ABS_WIDTH_SLOPE_PX_PER_Y,
            'max_derivative_difference': MAX_DERIVATIVE_DIFFERENCE,
            'max_center_variation_ratio': MAX_CENTER_VARIATION_RATIO,
            'reversed_white_left_yellow_right_is_hard_reject': True,
            'yellow_yellow_is_trusted_pair': False,
            'temporal_is_soft_score': True,
        },
        'trusted_reference_frames_loaded': len(trusted),
        'raw_pixel_ransac_reference_mean_ms': RAW_RANSAC_MEAN_MS,
        'weighted_mean_processing_ms': weighted_mean_ms,
        'speedup_vs_raw_pixel_ransac': (
            RAW_RANSAC_MEAN_MS / weighted_mean_ms
            if weighted_mean_ms > 0.0 else None),
        'total_frames': total_frames,
        'total_wall_time_seconds': wall_time,
        'bags': bag_summaries,
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    with args.summary.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False),
          flush=True)


if __name__ == '__main__':
    main()
