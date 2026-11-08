#!/usr/bin/env python3
"""Offline synthetic distance report. No ROS nodes, messages or map generation.

Run after sourcing ROS/workspace, with --reproduce to include the existing
optimistic-sensing model. Timing is a host sample, never a WCET guarantee.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from edgenode_planning.s4_distance_safety import DistancePolicy, DistanceSafety
from edgenode_planning.static_obstacle_planner import AvoidanceConfig, StaticObstaclePlanner
from edgenode_planning.vehicle_geometry import sonata_geometry


def report(reproduce=False):
    model = DistanceSafety(DistancePolicy(), AvoidanceConfig(), sonata_geometry())
    data = dict(data_kind='offline_synthetic_analysis_NOT_DRIVING_APPROVAL',
                roi_ceiling_m=21.19, verified_observation_m=None, speeds={})
    for speed in (3., 10., 20., 50.):
        fresh = model.metrics(speed)
        extra = fresh['entry_reserve_m']
        data['speeds'][str(int(speed))] = dict(
            fresh=fresh, age_600ms=model.metrics(speed, .6),
            geometric_entry_m={k: v-extra for k, v in fresh['entry_required_m'].items()})
    if reproduce:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/edgenode_planning/test'))
        from offline_static_avoidance import run
        from avoidance_world import world
        data['optimistic_observation_reproduction'] = {}
        for speed in (3., 10., 20., 50.):
            result = run(speed=speed, reference_horizon_m=20., observation_range_m=21.19)
            data['optimistic_observation_reproduction'][str(int(speed))] = {
                k: result[k] for k in ('success', 'phases', 'failure', 'controller_config')}
        data['cold_candidate_samples'] = []
        for speed in (3., 10., 20., 50.):
            lanes, area, point, obstacle = world()
            p = StaticObstaclePlanner(lanes, drivable_area=area)
            reference = [point(float(x), 0.) for x in range(51)]
            started = time.perf_counter()
            result = p.plan(reference, [obstacle()], (0., 0., 0.), speed_kmh=speed,
                            current_link_id='O0', route_link_ids=('O0', 'O1'))
            elapsed = time.perf_counter() - started
            data['cold_candidate_samples'].append(dict(speed_kmh=speed, elapsed_s=elapsed,
                state=result['state'], reason=result['reason'],
                candidates=[dict(length_m=c.transition_length_m, entry_end_s=c.entry_end_s,
                    return_start_s=c.return_start_s, return_end_s=c.return_end_s,
                    rejection=c.rejection) for c in p.candidates]))
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reproduce', action='store_true')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    text = json.dumps(report(args.reproduce), indent=2, allow_nan=False) + '\n'
    if args.out:
        args.out.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
