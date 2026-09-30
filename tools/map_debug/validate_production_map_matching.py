#!/usr/bin/env python3
"""Validate the production matcher offline; never publishes ROS messages.

Deduplicate EKF samples by exact header nanoseconds, retaining the last record.
Run the two matchers independently in bag-time order. Match every Ego sample
once before looking up nearest references, so repeated references do not alter
matcher history. Synchronization exclusions affect comparison statistics only.
"""

import argparse
from bisect import bisect_left
from collections import Counter
import csv
from dataclasses import dataclass
import hashlib
import inspect
import math
from pathlib import Path
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Import the production source requested for this validation, even if the
# installed workspace contains an older copy. No matching code is duplicated.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / 'ros2_ws/src/edgenode_planning'))
from edgenode_planning.map_matcher import MapMatcher

NS_PER_SEC = 1_000_000_000
ODOMETRY_TOPIC = '/localization/odometry'
EGO_TOPIC = '/ego_vehicle_status'
DEFAULT_MAX_SYNC_DT_SEC = 0.05  # 50 ms: comparison cutoff, not an EKF filter.


@dataclass(frozen=True)
class PoseSample:
    bag_ns: int
    x: float
    y: float
    heading_deg: float
    header_ns: int = 0


def quaternion_yaw_deg(q):
    """Standard ROS ENU yaw, with no sign change or heading offset."""
    return math.degrees(math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    ))


def read_samples(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    for topic in (ODOMETRY_TOPIC, EGO_TOPIC):
        if topic not in topic_types:
            raise RuntimeError(f'Required topic missing: {topic}')
    if topic_types[ODOMETRY_TOPIC] != 'nav_msgs/msg/Odometry':
        raise RuntimeError(f'Unexpected odometry type: {topic_types[ODOMETRY_TOPIC]}')
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (ODOMETRY_TOPIC, EGO_TOPIC)
    }
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[ODOMETRY_TOPIC, EGO_TOPIC],
    ))

    raw_odometry = 0
    ekf_by_header = {}
    ego_samples = []
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        if topic == ODOMETRY_TOPIC:
            raw_odometry += 1
            stamp = msg.header.stamp
            header_ns = int(stamp.sec) * NS_PER_SEC + int(stamp.nanosec)
            position = msg.pose.pose.position
            # Retain the final published state for this exact source timestamp.
            ekf_by_header[header_ns] = PoseSample(
                int(bag_ns), float(position.x), float(position.y),
                quaternion_yaw_deg(msg.pose.pose.orientation), header_ns,
            )
        else:
            ego_samples.append(PoseSample(
                int(bag_ns), float(msg.position.x), float(msg.position.y),
                float(msg.heading),
            ))

    ekf_samples = sorted(ekf_by_header.values(), key=lambda sample: sample.bag_ns)
    ego_samples.sort(key=lambda sample: sample.bag_ns)
    if not ekf_samples or not ego_samples:
        raise RuntimeError('Both topics must contain samples; validation is incomplete.')
    return raw_odometry, ekf_samples, ego_samples


def nearest_index(sorted_times_ns, timestamp_ns):
    """Nearest bag timestamp; ties select the earlier reference."""
    index = bisect_left(sorted_times_ns, timestamp_ns)
    candidates = [
        i for i in (index - 1, index) if 0 <= i < len(sorted_times_ns)
    ]
    if not candidates:
        raise ValueError('Cannot synchronize against an empty reference.')
    return min(candidates, key=lambda i: (
        abs(sorted_times_ns[i] - timestamp_ns), sorted_times_ns[i], i,
    ))


def describe(values):
    """Mean, median, linearly interpolated p95, and maximum."""
    ordered = sorted(values)
    if not ordered:
        return dict.fromkeys(('mean', 'median', 'p95', 'max'), math.nan)
    position = 0.95 * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    p95 = ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    return {
        'mean': statistics.mean(ordered),
        'median': statistics.median(ordered),
        'p95': p95,
        'max': ordered[-1],
    }


def success_rate(numerator, denominator):
    return 100.0 * numerator / denominator if denominator else math.nan


def temporal_metrics(samples, results):
    """Count transitions between successful links, retaining history over loss.

    A NO_MATCH run lasts from its first failure to the next successful sample.
    A run still open at recording end stops at the final observed EKF sample.
    """
    previous_link = None
    changes = 0
    run_count = max_count = 0
    run_start_ns = None
    max_duration_ns = 0
    for sample, result in zip(samples, results):
        if result is None:
            if run_start_ns is None:
                run_start_ns = sample.bag_ns
            run_count += 1
            max_count = max(max_count, run_count)
        else:
            if run_start_ns is not None:
                max_duration_ns = max(
                    max_duration_ns, sample.bag_ns - run_start_ns,
                )
            run_start_ns = None
            run_count = 0
            if previous_link is not None and result['link_id'] != previous_link:
                changes += 1
            previous_link = result['link_id']
    if run_start_ns is not None:
        max_duration_ns = max(
            max_duration_ns, samples[-1].bag_ns - run_start_ns,
        )
    return changes, max_count, max_duration_ns / NS_PER_SEC


def timestamp_text(timestamp_ns):
    return f'{timestamp_ns // NS_PER_SEC}.{timestamp_ns % NS_PER_SEC:09d}'


def make_rows(ekf_samples, ekf_results, ego_samples, ego_results, max_sync_dt_sec):
    ego_times = [sample.bag_ns for sample in ego_samples]
    max_sync_ns = round(max_sync_dt_sec * NS_PER_SEC)
    origin_ns = min(ekf_samples[0].bag_ns, ego_samples[0].bag_ns)
    rows = []
    for ekf, ekf_result in zip(ekf_samples, ekf_results):
        index = nearest_index(ego_times, ekf.bag_ns)
        ego, ego_result = ego_samples[index], ego_results[index]
        dt_ns = abs(ekf.bag_ns - ego.bag_ns)
        sync_valid = dt_ns <= max_sync_ns
        both_matched = ekf_result is not None and ego_result is not None
        rows.append({
            'time': timestamp_text(ekf.bag_ns),
            'elapsed_sec': (ekf.bag_ns - origin_ns) / NS_PER_SEC,
            'ekf_bag_timestamp_ns': ekf.bag_ns,
            'ekf_header_timestamp_ns': ekf.header_ns,
            'ekf_x': ekf.x,
            'ekf_y': ekf.y,
            'ekf_yaw_deg': ekf.heading_deg,
            'ekf_link': ekf_result['link_id'] if ekf_result else '',
            'ekf_distance': ekf_result['distance'] if ekf_result else math.nan,
            'ekf_heading_diff': ekf_result['heading_diff'] if ekf_result else math.nan,
            'ekf_score': ekf_result['score'] if ekf_result else math.nan,
            'ego_bag_timestamp_ns': ego.bag_ns,
            'ego_x': ego.x,
            'ego_y': ego.y,
            'ego_heading_deg': ego.heading_deg,
            'ego_link': ego_result['link_id'] if ego_result else '',
            'ego_distance': ego_result['distance'] if ego_result else math.nan,
            'ego_heading_diff': ego_result['heading_diff'] if ego_result else math.nan,
            'ego_score': ego_result['score'] if ego_result else math.nan,
            'same_link': int(ekf_result['link_id'] == ego_result['link_id'])
            if sync_valid and both_matched else '',
            'sync_dt_sec': dt_ns / NS_PER_SEC,
            'sync_valid': int(sync_valid),
            'ekf_matched': int(ekf_result is not None),
            'ego_matched': int(ego_result is not None),
        })
    return rows


def comparison_counts(rows):
    counts = Counter()
    for row in rows:
        if not row['sync_valid']:
            counts['excluded'] += 1
            continue
        counts['comparable'] += 1
        if row['ekf_matched'] and row['ego_matched']:
            counts['both_matched'] += 1
            counts['same_link' if row['same_link'] else 'different_link'] += 1
        elif row['ekf_matched']:
            counts['ekf_only'] += 1
        elif row['ego_matched']:
            counts['ego_only'] += 1
        else:
            counts['both_unmatched'] += 1
    return counts


def print_distribution(label, values, unit, keys=('mean', 'median', 'p95', 'max')):
    stats = describe(values)
    print(label)
    for key in keys:
        print(f'  {key:8s}: {stats[key]:.6f} {unit}')


def print_report(raw_odometry, ekf_samples, ekf_results, ego_samples,
                 ego_results, rows, max_sync_dt_sec):
    ekf_matched = [result for result in ekf_results if result is not None]
    ego_matched = [result for result in ego_results if result is not None]
    changes, max_loss_count, max_loss_sec = temporal_metrics(ekf_samples, ekf_results)
    counts = comparison_counts(rows)

    print('\n[EKF PRODUCTION MAP MATCH]')
    print(f'raw odometry samples: {raw_odometry}')
    print(f'unique odometry samples (header timestamps): {len(ekf_samples)}')
    print(f'duplicate odometry samples removed: {raw_odometry - len(ekf_samples)}')
    print(f'unique EKF bag-time span: '
          f'{(ekf_samples[-1].bag_ns - ekf_samples[0].bag_ns) / NS_PER_SEC:.6f} s')
    print(f'matched: {len(ekf_matched)}')
    print(f'unmatched: {len(ekf_results) - len(ekf_matched)}')
    print(f'success rate: {success_rate(len(ekf_matched), len(ekf_results)):.6f} %')
    print_distribution('Distance (matched samples):',
                       [result['distance'] for result in ekf_matched], 'm')
    print_distribution('Heading difference (matched samples):',
                       [result['heading_diff'] for result in ekf_matched], 'deg')
    print(f'link change count (between successful matches): {changes}')
    print(f'max consecutive NO_MATCH samples: {max_loss_count}')
    print(f'max consecutive NO_MATCH duration: {max_loss_sec:.6f} s')
    print('NO_MATCH duration: first failure to recovery; '
          'an open final run ends at the last observed EKF sample.')

    print('\n[SYNCHRONIZATION]')
    print('Basis: nearest bag timestamp; ties choose earlier; reference reuse allowed.')
    print(f'maximum accepted sync time difference: {max_sync_dt_sec:.6f} s')
    print(f'nearest pairs before cutoff: {len(rows)}')
    print(f'excluded by sync threshold: {counts["excluded"]}')
    print_distribution('Sync time difference before cutoff:',
                       [row['sync_dt_sec'] for row in rows], 's',
                       ('median', 'p95', 'max'))
    print_distribution('Sync time difference after cutoff:',
                       [row['sync_dt_sec'] for row in rows if row['sync_valid']],
                       's', ('median', 'p95', 'max'))

    print('\n[EKF vs EGO REFERENCE]')
    print(f'synchronized comparable samples: {counts["comparable"]}')
    print(f'both matched: {counts["both_matched"]}')
    print(f'same link count: {counts["same_link"]}')
    print(f'different link count: {counts["different_link"]}')
    print(f'exact link agreement rate (same / both matched): '
          f'{success_rate(counts["same_link"], counts["both_matched"]):.6f} %')
    print(f'EKF matched / Ego unmatched count: {counts["ekf_only"]}')
    print(f'EKF unmatched / Ego matched count: {counts["ego_only"]}')
    print(f'both unmatched count: {counts["both_unmatched"]}')

    print('\n[EGO REFERENCE MAP MATCH]')
    print('Scope: every Ego sample once in bag-time order, before synchronization.')
    print(f'raw Ego samples: {len(ego_samples)}')
    print(f'matched: {len(ego_matched)}')
    print(f'unmatched: {len(ego_results) - len(ego_matched)}')
    print(f'success rate: {success_rate(len(ego_matched), len(ego_results)):.6f} %')
    print_distribution('Distance (matched samples):',
                       [result['distance'] for result in ego_matched], 'm',
                       ('median', 'p95'))
    print_distribution('Heading difference (matched samples):',
                       [result['heading_diff'] for result in ego_matched], 'deg',
                       ('median', 'p95'))

    mismatches = sorted(
        (row for row in rows if row['same_link'] == 0),
        key=lambda row: (-row['ekf_distance'], row['ekf_bag_timestamp_ns']),
    )[:10]
    print('\n[LINK MISMATCH EXAMPLES: at most 10, largest EKF distance first]')
    print('timestamp (bag epoch s), EKF link, Ego reference link, '
          'EKF distance (m), Ego distance (m), sync dt (s)')
    for row in mismatches:
        print(f'{row["time"]}, {row["ekf_link"]}, {row["ego_link"]}, '
              f'{row["ekf_distance"]:.6f}, {row["ego_distance"]:.6f}, '
              f'{row["sync_dt_sec"]:.6f}')
    if not mismatches:
        print('No synchronized link mismatches.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag-dir', type=Path,
                        default=PROJECT_ROOT / 'local_data/ekf_validation_v2')
    parser.add_argument('--mgeo-dir', type=Path,
                        default=PROJECT_ROOT / 'local_data/c_track_mgeo')
    parser.add_argument('--output-csv', type=Path, default=(
        PROJECT_ROOT / 'local_data/c_track_mgeo/map_match_production_validation.csv'
    ))
    parser.add_argument('--max-sync-dt-sec', type=float, default=DEFAULT_MAX_SYNC_DT_SEC)
    args = parser.parse_args()
    if not math.isfinite(args.max_sync_dt_sec) or args.max_sync_dt_sec < 0.0:
        parser.error('--max-sync-dt-sec must be finite and nonnegative')

    matcher_source = Path(inspect.getfile(MapMatcher)).resolve()
    print(f'Bag: {args.bag_dir}')
    print(f'MGeo: {args.mgeo_dir}')
    print(f'Production matcher source: {matcher_source}')
    print(f'Matcher SHA256: {hashlib.sha256(matcher_source.read_bytes()).hexdigest()}')
    print('Odometry deduplication: exact header nanoseconds; last record retained.')
    print('Sequence: retained EKF samples and all Ego samples in bag-time order.')
    print('Yaw: standard ROS ENU quaternion formula, converted to degrees.')
    raw_odometry, ekf_samples, ego_samples = read_samples(args.bag_dir)

    # Defaults belong to production MapMatcher; this validation does not tune them.
    ekf_matcher = MapMatcher(args.mgeo_dir)
    ego_matcher = MapMatcher(args.mgeo_dir)
    print(f'Loaded links per matcher: {len(ekf_matcher.links)}')
    print('Production parameters (unchanged):')
    for name in (
        'search_radius_m', 'max_match_distance_m', 'max_heading_diff_deg',
        'heading_weight', 'same_link_bonus', 'connected_link_bonus',
        'unrelated_link_penalty',
    ):
        print(f'  {name}: {getattr(ekf_matcher, name)}')
    ekf_results = [
        ekf_matcher.match(sample.x, sample.y, sample.heading_deg)
        for sample in ekf_samples
    ]
    ego_results = [
        ego_matcher.match(sample.x, sample.y, sample.heading_deg)
        for sample in ego_samples
    ]
    rows = make_rows(
        ekf_samples, ekf_results, ego_samples, ego_results, args.max_sync_dt_sec,
    )
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print_report(raw_odometry, ekf_samples, ekf_results, ego_samples,
                 ego_results, rows, args.max_sync_dt_sec)
    print(f'\nCSV saved: {args.output_csv} ({len(rows)} data rows)')
    print('CSV time = bag epoch seconds; elapsed_sec = time since the first input sample.')
    print('Excluded and unmatched rows are retained; same_link is blank unless '
          'synchronized and both matched; missing match diagnostics are NaN.')
    print('Agreement measures consistency with an Ego-pose-derived matcher reference, '
          'not independently labeled link accuracy.')


if __name__ == '__main__':
    main()
