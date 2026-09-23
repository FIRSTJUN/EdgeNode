#!/usr/bin/env python3
"""Capture or compare deterministic camera-lane outputs from ROS 2 bags."""

import argparse
import gzip
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from edgenode_perception.perception_node import LaneDiagnostics, PerceptionNode


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


def json_float(value):
    value = float(value)
    if np.isnan(value):
        return 'NaN'
    if np.isposinf(value):
        return 'Infinity'
    if np.isneginf(value):
        return '-Infinity'
    return value


def normalize_record(value):
    if isinstance(value, dict):
        return {key: normalize_record(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalize_record(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return json_float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def capture_bag(node: PerceptionNode, bag_path: Path):
    records = []
    original_polyfit = np.polyfit
    for index, (timestamp, msg) in enumerate(iter_camera_messages(bag_path)):
        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'cannot decode frame {index} in {bag_path}')

        mask = node.make_lane_mask(image)
        fit_hasher = hashlib.sha256()
        fit_count = 0

        def traced_polyfit(*args, **kwargs):
            nonlocal fit_count
            fit = original_polyfit(*args, **kwargs)
            fit_hasher.update(np.asarray(fit, dtype=np.float64).tobytes())
            fit_count += 1
            return fit

        diagnostics = LaneDiagnostics()
        np.polyfit = traced_polyfit
        try:
            result = node.sliding_window_lane(mask, diagnostics)
        finally:
            np.polyfit = original_polyfit

        records.append(normalize_record({
            'index': index,
            'timestamp': timestamp,
            'mask_sha256': hashlib.sha256(mask.tobytes()).hexdigest(),
            'polyfit_count': fit_count,
            'polyfit_sha256': fit_hasher.hexdigest(),
            'result': result,
            'diagnostics': asdict(diagnostics),
        }))
    return records


def load_capture(path: Path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return json.load(stream)


def compare_values(reference, candidate, path='', differences=None):
    if differences is None:
        differences = []
    if type(reference) is not type(candidate):
        differences.append((path, reference, candidate, None))
    elif isinstance(reference, dict):
        if reference.keys() != candidate.keys():
            differences.append((path, list(reference), list(candidate), None))
        else:
            for key in reference:
                compare_values(
                    reference[key], candidate[key], f'{path}.{key}', differences)
    elif isinstance(reference, list):
        if len(reference) != len(candidate):
            differences.append((path, len(reference), len(candidate), None))
        else:
            for index, (left, right) in enumerate(zip(reference, candidate)):
                compare_values(left, right, f'{path}[{index}]', differences)
    elif isinstance(reference, float):
        if reference != candidate:
            differences.append(
                (path, reference, candidate, abs(reference - candidate)))
    elif reference != candidate:
        differences.append((path, reference, candidate, None))
    return differences


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('bags', type=Path, nargs='+')
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionNode()
    try:
        capture = {
            str(path): capture_bag(node, path)
            for path in args.bags
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with gzip.open(args.output, 'wt', encoding='utf-8') as stream:
        json.dump(capture, stream, separators=(',', ':'), sort_keys=True)

    frame_count = sum(len(records) for records in capture.values())
    print(f'wrote {frame_count} frames to {args.output}')
    if args.reference:
        differences = compare_values(load_capture(args.reference), capture)
        print(f'differences: {len(differences)}')
        for difference in differences[:20]:
            print(difference)
        if differences:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
