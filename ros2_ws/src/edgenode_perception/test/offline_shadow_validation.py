#!/usr/bin/env python3
"""Offline validation for the diagnostic-only shadow right selector."""

import argparse
import gzip
import json
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from edgenode_perception.perception_node import (
    LaneDiagnostics, PerceptionNode, ProductionFitSnapshot,
)


CAMERA_TOPIC = '/camera/image/compressed'


def iter_camera_messages(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_type = get_message(topic_types[CAMERA_TOPIC])
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == CAMERA_TOPIC:
            yield timestamp, deserialize_message(data, message_type)


def header_stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def primary_record_component(components):
    valid = [component for component in components if component['x_eval'] is not None]
    if not valid:
        return None
    return max(
        valid,
        key=lambda component: (
            component['y_max'] - component['y_min'], component['pixels']),
    )


def load_trusted_geometry(path: Path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        records = json.load(stream)
    trusted = {}
    for record in records:
        yellow = primary_record_component(record['yellow'])
        beige = primary_record_component(record['beige'])
        neutral = primary_record_component(record['neutral_white'])
        if yellow is None or beige is None or neutral is None:
            continue
        if yellow['x_eval'] < beige['x_eval'] < neutral['x_eval']:
            trusted[int(record['source_header_ns'])] = {
                'eval_y': float(record['eval_y']),
                'production_right_x': float(record['production_right_x']),
                'yellow': yellow,
                'beige': beige,
                'neutral_white': neutral,
            }
    return trusted


def matched_component(components, reference):
    if not components or reference is None:
        return None
    reference_fit = np.asarray(reference['fit'], dtype=np.float64)
    return min(
        components,
        key=lambda component: float(np.max(np.abs(component.fit - reference_fit))),
    )


def timing_summary(values):
    if not values:
        return {'count': 0, 'mean_ms': None, 'p50_ms': None, 'p95_ms': None}
    values = np.asarray(values, dtype=np.float64) * 1000.0
    return {
        'count': int(values.size),
        'mean_ms': float(np.mean(values)),
        'p50_ms': float(np.percentile(values, 50)),
        'p95_ms': float(np.percentile(values, 95)),
    }


def finite_stats(values):
    values = np.asarray([value for value in values if np.isfinite(value)])
    if not len(values):
        return {'count': 0, 'median': None, 'p95': None, 'max': None}
    return {
        'count': int(len(values)),
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def validate_bag(node, bag_path, trusted, max_frames=None):
    modes = Counter()
    statuses = Counter()
    reasons = Counter()
    candidate_counts = Counter()
    timings = {'production': [], 'semantic': [], 'shadow': [], 'shadow_debug': []}
    jumps = []
    previous_shadow_x = float('nan')
    trusted_metrics = Counter()
    trusted_records = []
    node.reset_shadow_state()

    frames = 0
    for index, (_, msg) in enumerate(iter_camera_messages(bag_path)):
        if max_frames is not None and index >= max_frames:
            break
        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue

        diagnostics = LaneDiagnostics()
        snapshot = ProductionFitSnapshot()
        started = time.perf_counter()
        mask = node.make_lane_mask(image)
        node.sliding_window_lane(mask, diagnostics, fit_snapshot=snapshot)
        timings['production'].append(time.perf_counter() - started)

        production_right_x = (
            diagnostics.right_x_eval
            if diagnostics.mode in ('BOTH', 'RIGHT_ONLY') else float('nan'))
        started = time.perf_counter()
        semantic = node.analyze_semantic_candidates(
            image, diagnostics.eval_y, production_right_x)
        timings['semantic'].append(time.perf_counter() - started)

        started = time.perf_counter()
        shadow = node.select_shadow_right(semantic, image.shape[1])
        timings['shadow'].append(time.perf_counter() - started)

        if len(timings['shadow_debug']) < 30:
            started = time.perf_counter()
            node.publish_shadow_debug_image(
                msg, image, semantic, shadow, snapshot, diagnostics)
            timings['shadow_debug'].append(time.perf_counter() - started)

        frames += 1
        modes[diagnostics.mode] += 1
        statuses[shadow.status] += 1
        reasons[shadow.rejection_reason] += 1
        candidate_counts[shadow.candidate_count] += 1
        if shadow.status == 'VALID':
            if np.isfinite(previous_shadow_x):
                jumps.append(abs(shadow.right_x - previous_shadow_x))
            previous_shadow_x = shadow.right_x
        else:
            previous_shadow_x = float('nan')

        reference = trusted.get(header_stamp_ns(msg))
        if reference is None:
            continue
        trusted_metrics['physical_order_frames'] += 1
        if shadow.status != 'VALID':
            trusted_metrics['unresolved'] += 1
            trusted_metrics[f'rejection:{shadow.rejection_reason}'] += 1
            continue
        trusted_metrics['shadow_valid'] += 1

        selected = next(
            (evaluation.component for evaluation in shadow.evaluations
             if evaluation.status == 'SELECTED'), None)
        actual_beige = matched_component(
            semantic.beige_components, reference['beige'])
        actual_neutral = matched_component(
            semantic.neutral_white_components, reference['neutral_white'])
        eval_y = reference['eval_y']
        if selected is None or actual_beige is None or actual_neutral is None:
            trusted_metrics['comparison_unavailable'] += 1
            continue
        if not (selected.y_min <= eval_y <= selected.y_max):
            trusted_metrics['comparison_unavailable'] += 1
            continue

        shadow_x = float(np.polyval(selected.fit, eval_y))
        beige_x = float(reference['beige']['x_eval'])
        neutral_x = float(reference['neutral_white']['x_eval'])
        production_x = float(reference['production_right_x'])
        shadow_beige_distance = abs(shadow_x - beige_x)
        shadow_neutral_distance = abs(shadow_x - neutral_x)
        production_beige_distance = abs(production_x - beige_x)
        production_neutral_distance = abs(production_x - neutral_x)
        if shadow_beige_distance < shadow_neutral_distance:
            trusted_metrics['shadow_beige_closer'] += 1
        else:
            trusted_metrics['parking_white_closer'] += 1
        if shadow_beige_distance < production_beige_distance:
            trusted_metrics['shadow_better_than_production'] += 1
        if production_beige_distance < production_neutral_distance:
            trusted_metrics['production_beige_closer'] += 1
        else:
            trusted_metrics['production_parking_closer'] += 1
        trusted_records.append({
            'index': index,
            'shadow_component_id': shadow.component_id,
            'shadow_score': shadow.score,
            'shadow_beige_distance': shadow_beige_distance,
            'shadow_neutral_distance': shadow_neutral_distance,
            'production_beige_distance': production_beige_distance,
            'production_neutral_distance': production_neutral_distance,
        })

    return {
        'frames': frames,
        'production_modes': dict(modes),
        'shadow_status': dict(statuses),
        'shadow_rejection_reasons': dict(reasons),
        'shadow_candidate_counts': {str(key): value for key, value in candidate_counts.items()},
        'temporal_jump_px': finite_stats(jumps),
        'timing': {name: timing_summary(values) for name, values in timings.items()},
        'trusted_metrics': dict(trusted_metrics),
        'trusted_records': trusted_records,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trusted-geometry', type=Path)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('bags', nargs='+', type=Path)
    args = parser.parse_args()

    trusted = (
        load_trusted_geometry(args.trusted_geometry)
        if args.trusted_geometry else {})
    rclpy.init()
    node = PerceptionNode()
    try:
        report = {
            'trusted_reference_frames': len(trusted),
            'bags': {
                str(path): validate_bag(
                    node, path, trusted, max_frames=args.max_frames)
                for path in args.bags
            },
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
