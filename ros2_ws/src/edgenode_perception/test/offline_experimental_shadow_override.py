#!/usr/bin/env python3
"""Offline A/B validation for the default-off shadow-right override."""

import argparse
import gzip
import json
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter

from edgenode_perception.perception_node import (
    LaneDiagnostics, PerceptionNode, ProductionFitSnapshot,
)
from offline_shadow_validation import (
    header_stamp_ns, iter_camera_messages, load_trusted_geometry,
)


def finite(value):
    return float(value) if np.isfinite(value) else None


def result_record(result):
    if result is None:
        return None
    return {
        'lane_error': float(result[0]),
        'confidence': float(result[1]),
        'lane_center': float(result[2]),
        'eval_y': float(result[3]),
    }


def stats(values):
    values = np.asarray([value for value in values if np.isfinite(value)])
    if not values.size:
        return {'count': 0, 'mean': None, 'p50': None, 'p95': None, 'max': None}
    return {
        'count': int(values.size),
        'mean': float(np.mean(values)),
        'p50': float(np.percentile(values, 50)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def timing(values):
    result = stats(np.asarray(values, dtype=np.float64) * 1000.0)
    return {f'{key}_ms' if key != 'count' else key: value
            for key, value in result.items()}


def streaks(flags):
    result = []
    current = 0
    for flag in flags:
        if flag:
            current += 1
        elif current:
            result.append(current)
            current = 0
    if current:
        result.append(current)
    return result


def selected_component(shadow):
    return next((
        evaluation.component for evaluation in shadow.evaluations
        if evaluation.status == 'SELECTED'
    ), None)


def validate_bag(node, bag_path, trusted, records, max_frames=None):
    node.reset_shadow_state()
    reasons = Counter()
    modes = Counter()
    active_flags = []
    selected_right = []
    selected_center = []
    selected_error = []
    transition_right_jumps = []
    transition_center_jumps = []
    transition_error_jumps = []
    all_right_jumps = []
    all_center_jumps = []
    all_error_jumps = []
    dangerous = []
    trusted_rows = []
    trusted_counts = Counter()
    timings = {
        'production': [], 'semantic': [], 'shadow': [],
        'gate_off': [], 'gate_on': [],
    }
    previous = None

    for index, (_, msg) in enumerate(iter_camera_messages(bag_path)):
        if max_frames is not None and index >= max_frames:
            break
        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            dangerous.append({'index': index, 'reason': 'IMAGE_DECODE_FAILED'})
            continue
        height, width = image.shape[:2]
        production = LaneDiagnostics()
        snapshot = ProductionFitSnapshot()

        started = time.perf_counter()
        mask = node.make_lane_mask(image)
        original = node.sliding_window_lane(
            mask, production, fit_snapshot=snapshot)
        timings['production'].append(time.perf_counter() - started)
        modes[production.mode] += 1

        production_right = (
            production.right_x_eval
            if production.mode in ('BOTH', 'RIGHT_ONLY') else float('nan'))
        started = time.perf_counter()
        semantic = node.analyze_semantic_candidates(
            image, production.eval_y, production_right)
        timings['semantic'].append(time.perf_counter() - started)
        started = time.perf_counter()
        shadow = node.select_shadow_right(semantic, width)
        timings['shadow'].append(time.perf_counter() - started)

        # OFF regression must not destroy the stateful ON-path history.
        saved_confirm_count = node.experimental_shadow_confirm_count
        saved_active_state = node.experimental_shadow_override_active_state
        saved_previous_output = (
            node.experimental_shadow_previous_output_right_x)

        node.set_parameters([Parameter(
            'experimental_shadow_right_override_enabled', value=False)])
        started = time.perf_counter()
        off_result, off_diag = node.evaluate_experimental_shadow_override(
            original, production, shadow, width)
        timings['gate_off'].append(time.perf_counter() - started)

        node.experimental_shadow_confirm_count = saved_confirm_count
        node.experimental_shadow_override_active_state = saved_active_state
        node.experimental_shadow_previous_output_right_x = (
            saved_previous_output)

        node.set_parameters([Parameter(
            'experimental_shadow_right_override_enabled', value=True)])
        started = time.perf_counter()
        selected, experimental = node.evaluate_experimental_shadow_override(
            original, production, shadow, width)
        timings['gate_on'].append(time.perf_counter() - started)

        if off_result is not original or off_diag.active:
            dangerous.append({'index': index, 'reason': 'OFF_CHANGED_RESULT'})
        reasons[experimental.reason] += 1
        active_flags.append(experimental.active)
        selected_right.append(experimental.selected_right_x)
        selected_center.append(experimental.selected_lane_center)
        selected_error.append(experimental.selected_lane_error)

        if original is not None and selected is None:
            dangerous.append({
                'index': index, 'reason': 'ORIGINAL_RESULT_BECAME_NONE'})
        if original is not None and selected is not None and selected[1] != original[1]:
            dangerous.append({'index': index, 'reason': 'CONFIDENCE_CHANGED'})
        if experimental.active:
            if not (0.0 <= experimental.selected_right_x < width):
                dangerous.append({'index': index, 'reason': 'RIGHT_OUT_OF_BOUNDS'})
            if experimental.selected_right_x <= production.left_x_eval:
                dangerous.append({'index': index, 'reason': 'RIGHT_NOT_RIGHT_OF_LEFT'})
            if experimental.selected_lane_width <= 0.0:
                dangerous.append({'index': index, 'reason': 'NONPOSITIVE_WIDTH'})
            if not (experimental.selected_right_y_min <= production.eval_y <=
                    experimental.selected_right_y_max):
                dangerous.append({'index': index, 'reason': 'EXTRAPOLATED_RIGHT'})
            if production.right_x_eval >= width - 15:
                dangerous.append({
                    'index': index, 'reason': 'EDGE_OVERRIDE',
                    'original_right_x': production.right_x_eval,
                    'selected_right_x': experimental.selected_right_x,
                })

        current = {
            'active': experimental.active,
            'right': experimental.selected_right_x,
            'center': experimental.selected_lane_center,
            'error': experimental.selected_lane_error,
        }
        if previous is not None:
            if np.isfinite(current['right']) and np.isfinite(previous['right']):
                right_jump = abs(current['right'] - previous['right'])
                all_right_jumps.append(right_jump)
            else:
                right_jump = float('nan')
            if np.isfinite(current['center']) and np.isfinite(previous['center']):
                center_jump = abs(current['center'] - previous['center'])
                all_center_jumps.append(center_jump)
            else:
                center_jump = float('nan')
            if np.isfinite(current['error']) and np.isfinite(previous['error']):
                error_jump = abs(current['error'] - previous['error'])
                all_error_jumps.append(error_jump)
            else:
                error_jump = float('nan')
            if current['active'] != previous['active']:
                transition_right_jumps.append(right_jump)
                transition_center_jumps.append(center_jump)
                transition_error_jumps.append(error_jump)
        previous = current

        reference = trusted.get(header_stamp_ns(msg))
        trusted_record = None
        if reference is not None:
            trusted_counts['frames'] += 1
            beige_x = float(reference['beige']['x_eval'])
            neutral_x = float(reference['neutral_white']['x_eval'])
            original_x = float(reference['production_right_x'])
            experimental_x = float(experimental.selected_right_x)
            original_distance = abs(original_x - beige_x)
            experimental_distance = abs(experimental_x - beige_x)
            improvement = original_distance - experimental_distance
            if improvement > 1e-9:
                outcome = 'improved'
            elif improvement < -1e-9:
                outcome = 'worsened'
            else:
                outcome = 'no_change'
            trusted_counts[outcome] += 1
            if experimental.active:
                trusted_counts['active'] += 1
                if abs(experimental_x - neutral_x) <= experimental_distance:
                    trusted_counts['active_parking_white_closer'] += 1
            trusted_record = {
                'outcome': outcome,
                'beige_x': beige_x,
                'neutral_x': neutral_x,
                'original_beige_distance': original_distance,
                'experimental_beige_distance': experimental_distance,
                'improvement_px': improvement,
            }
            trusted_rows.append({
                'index': index,
                'active': experimental.active,
                **trusted_record,
                'lane_center_shift_px': experimental.lane_center_shift_px,
                'lane_error_delta': experimental.lane_error_delta,
            })

        records.append({
            'bag': str(bag_path),
            'index': index,
            'header_ns': header_stamp_ns(msg),
            'image_width': width,
            'production': {
                'mode': production.mode,
                'failure_reason': production.failure_reason,
                'left_x': finite(production.left_x_eval),
                'right_x': finite(production.right_x_eval),
                'lane_width': finite(production.lane_width_eval),
                'eval_y': finite(production.eval_y),
            },
            'original_result': result_record(original),
            'experimental_result': result_record(selected),
            'shadow': {
                'status': shadow.status,
                'reason': shadow.rejection_reason,
                'score': finite(shadow.score),
                'right_x': finite(shadow.right_x),
                'temporal_delta': finite(shadow.temporal_delta),
                'component_id': shadow.component_id,
            },
            'override': {
                key: finite(value) if isinstance(value, float) else value
                for key, value in asdict(experimental).items()
                if key != 'selected_right_fit'
            },
            'trusted': trusted_record,
        })

    transitions = sum(
        left != right for left, right in zip(active_flags, active_flags[1:]))
    active_streaks = streaks(active_flags)
    hard_danger_reasons = Counter(
        item['reason'] for item in dangerous
        if item['reason'] != 'EDGE_OVERRIDE')
    edge_overrides = [
        item for item in dangerous if item['reason'] == 'EDGE_OVERRIDE']
    return {
        'frames': len(active_flags),
        'production_modes': dict(modes),
        'override_active': int(sum(active_flags)),
        'override_rate': (
            float(np.mean(active_flags)) if active_flags else 0.0),
        'override_reasons': dict(reasons),
        'transitions': transitions,
        'activation_streak': stats(active_streaks),
        'short_active_streaks_le_2': sum(value <= 2 for value in active_streaks),
        'right_jump_px': stats(all_right_jumps),
        'center_jump_px': stats(all_center_jumps),
        'lane_error_jump': stats(all_error_jumps),
        'transition_right_jump_px': stats(transition_right_jumps),
        'transition_center_jump_px': stats(transition_center_jumps),
        'transition_lane_error_jump': stats(transition_error_jumps),
        'dangerous_hard': dict(hard_danger_reasons),
        'dangerous_examples': dangerous[:50],
        'edge_override_count': len(edge_overrides),
        'timing': {name: timing(values) for name, values in timings.items()},
        'trusted': {
            'counts': dict(trusted_counts),
            'original_beige_distance_px': stats(
                row['original_beige_distance'] for row in trusted_rows),
            'experimental_beige_distance_px': stats(
                row['experimental_beige_distance'] for row in trusted_rows),
            'improvement_px': stats(
                row['improvement_px'] for row in trusted_rows),
            'active_center_shift_px': stats(
                abs(row['lane_center_shift_px']) for row in trusted_rows
                if row['active']),
            'active_lane_error_delta': stats(
                abs(row['lane_error_delta']) for row in trusted_rows
                if row['active']),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--records', type=Path, required=True)
    parser.add_argument('--trusted-geometry', type=Path)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('bags', nargs='+', type=Path)
    args = parser.parse_args()
    trusted = (
        load_trusted_geometry(args.trusted_geometry)
        if args.trusted_geometry else {})

    rclpy.init()
    node = PerceptionNode()
    all_records = []
    try:
        bag_reports = {}
        for bag in args.bags:
            print(f'validating {bag}', flush=True)
            bag_reports[str(bag)] = validate_bag(
                node, bag, trusted, all_records, args.max_frames)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    aggregate_dangerous = Counter()
    for report in bag_reports.values():
        aggregate_dangerous.update(report['dangerous_hard'])
    summary = {
        'trusted_reference_frames': len(trusted),
        'bags': bag_reports,
        'hard_dangerous_changes': dict(aggregate_dangerous),
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    with args.summary.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
    with gzip.open(args.records, 'wt', encoding='utf-8') as stream:
        json.dump(all_records, stream, separators=(',', ':'), sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
