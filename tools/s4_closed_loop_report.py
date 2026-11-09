#!/usr/bin/env python3
"""Isolated Stage 5A matrix; source the ROS workspace, never start a ROS graph.

A assumes a declared virtual sensor and surveyed synthetic edges. B has the
unchanged 21.19 m ROI ceiling and NO observation/boundary/crossing approvals.
Ten/twenty km/h are initial-speed model analyses; the avoidance cap stays 3.
"""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/edgenode_planning/test'))
from synthetic_s4_loop import Scene, simulate  # noqa: E402


def matrix():
    cases = []
    def add(name, expected='stop', **values):
        cases.append((Scene(name=name, **values), expected))
    for radius, shape in ((0., 'straight'), (300., 'curve')):
        for side in ('left', 'right'):
            add(f'A_{shape}_{side}_3', 'return', radius_m=radius, side=side)
        for front, lateral, yaw in ((45., .01, .002), (65., -.01, -.002)):
            add(f'A_{shape}_position_{front:g}_3', 'return', radius_m=radius,
                obstacle_front_m=front, initial_lateral_m=lateral, initial_yaw_rad=yaw,
                duration_s=180.)
        for speed in (10., 20.):
            add(f'A_{shape}_initial_{speed:g}_analysis', 'analysis', radius_m=radius,
                initial_speed_kmh=speed)
        for fault in ('sensor_dropout', 'tracking_failure'):
            add(f'A_{shape}_{fault}_3', radius_m=radius, fault=fault)
        for speed in (3., 10., 20.):
            add(f'B_{shape}_{speed:g}', experiment='B', radius_m=radius,
                initial_speed_kmh=speed)
    for speed in (3., 10., 20.):
        add(f'A_boundary_missing_{speed:g}', initial_speed_kmh=speed,
            obstacle_front_m=35., fault='boundary_missing')
        add(f'A_permission_missing_{speed:g}', initial_speed_kmh=speed,
            obstacle_front_m=35., fault='permission_missing')
        add(f'A_narrow_road_{speed:g}', initial_speed_kmh=speed,
            obstacle_front_m=35., fault='no_space', lane_width_m=3.)
        add(f'A_wall_{speed:g}', initial_speed_kmh=speed,
            obstacle_front_m=35., fault='wall')
        add(f'A_late_detection_{speed:g}', initial_speed_kmh=speed,
            obstacle_front_m={3.:6., 10.:10., 20.:12.}[speed])
    add('A_short_road_3', road_end_m=83., adjacent_end_m=83.)
    add('A_short_adjacent_3', adjacent_end_m=70.)
    add('A_short_control_window_3', control_horizon_m=5.)
    add('A_compute_timeout_3', fault='compute_timeout')
    add('A_return_obstruction_3', fault='return_obstruction')
    add('A_boundary_lost_on_return_3', fault='boundary_lost_on_return')
    add('A_tight_curve_100m_3', radius_m=100.)
    return cases


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean(value), indent=2, allow_nan=False)+'\n')


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_result(result, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    world, rows = result['world'], result['rows']
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    xy = axes[0, 0]
    for key in ('road_low', 'road_high'):
        points = world[key]; xy.plot(*zip(*points), color='black', linewidth=.8)
    xy.plot([p[0] for p in world['original']], [p[1] for p in world['original']],
            '--', color='gray', linewidth=.7, label='original global path')
    for obstacle in world['obstacles']:
        xy.add_patch(Polygon(obstacle, color='firebrick', alpha=.8))
    for selected in result.get('selected_geometry', []):
        xy.plot([p[0] for p in selected['points']], [p[1] for p in selected['points']],
                color='orange', linewidth=1., label='fixed validated lattice')
    colors = {'NORMAL':'tab:blue', 'AVOID_ENTRY':'tab:orange', 'AVOID_HOLD':'tab:green',
              'RETURN':'tab:purple', 'BLOCKED':'tab:red'}
    for phase, color in colors.items():
        points = [r for r in rows if r['state'] == phase]
        if points:
            xy.scatter([r['x'] for r in points], [r['y'] for r in points],
                       color=color, s=2, label=phase)
    # Draw the measured body, including front/rear overhang, at several times.
    from edgenode_planning.vehicle_geometry import sonata_geometry
    body = sonata_geometry()
    for row in rows[::max(1, len(rows)//12)]:
        xy.add_patch(Polygon(body.polygon(row['x'], row['y'], row['yaw']),
                            fill=False, edgecolor='steelblue', linewidth=.5))
    xs, ys = [r['x'] for r in rows], [r['y'] for r in rows]
    xy.set_xlim(min(xs)-7, max(xs)+7); xy.set_ylim(min(ys)-7, max(ys)+7)
    xy.set_aspect('equal', adjustable='datalim'); xy.legend(fontsize=6)
    xy.set_xlabel('map x (m)'); xy.set_ylabel('map y (m)')
    t = [r['t'] for r in rows]
    axes[0,1].plot(t, [r['speed_kmh'] for r in rows], label='model measured speed (km/h)')
    axes[0,1].plot(t, [r['target_speed_kmh'] for r in rows], '--', label='target (km/h)')
    axes[0,1].plot(t, [r['brake'] for r in rows], label='production brake command')
    axes[0,1].legend(fontsize=7)
    axes[1,0].plot(t, [r['obstacle_clearance_m'] for r in rows], label='swept body clearance (m)')
    axes[1,0].plot(t, [r['road_margin_m'] for r in rows], label='outer road margin (m)')
    axes[1,0].plot(t, [r['lateral_error_m'] for r in rows], label='path error (m)')
    axes[1,0].axhline(.25, color='gray', linestyle=':', linewidth=.8)
    axes[1,0].set_ylim(-.1, min(5., max(r['road_margin_m'] for r in rows)+1.))
    axes[1,0].legend(fontsize=7)
    axes[1,1].plot(t, [r['wheel_rad'] for r in rows], label='physical wheel (rad)')
    axes[1,1].plot(t, [r['wheel_rate_radps'] for r in rows], label='physical wheel rate (rad/s)')
    axes[1,1].plot(t, [r['requested_wheel_rad'] for r in rows], ':', label='requested wheel (rad)')
    axes[1,1].legend(fontsize=7)
    for axis in axes.flat:
        axis.grid(alpha=.2)
    for axis in (axes[0,1], axes[1,0], axes[1,1]):
        axis.set_xlabel('virtual time (s)')
        for transition in result['transitions'][1:]:
            axis.axvline(transition['t'], color=colors[transition['state']], alpha=.3)
    fig.suptitle(f"{result['scene']['name']} | return={result['success']} | "
                 f"physical stop={result['safe_model_stop']} | {result['reason']}\n"
                 'SYNTHETIC ONLY - no MORAI / real-map / real-sensor approval', fontsize=11)
    fig.tight_layout(); fig.savefig(path, dpi=145); plt.close(fig)


def summary(result, expected):
    keys = ('success', 'blocked', 'safe_model_stop', 'reason', 'failure_phase',
            'min_obstacle_clearance_m', 'min_road_margin_m', 'max_lateral_error_m',
            'max_wheel_rad', 'max_wheel_rate_radps', 'max_requested_wheel_rate_radps',
            'max_cpu_compute_s', 'cpu_budget_overruns', 'modeled_budget_overruns',
            'cpu_freshness_budget_exceedances', 'stop_time_s', 'stop_distance_m',
            'final_speed_kmh', 'return_error_m', 'post_return_duration_s')
    passed = (result['success'] if expected == 'return' else
              result['blocked'] and result['safe_model_stop'] if expected == 'stop' else
              result['success'] or result['blocked'] and result['safe_model_stop'])
    return dict(name=result['scene']['name'], experiment=result['scene']['experiment'],
                initial_speed_kmh=result['scene']['initial_speed_kmh'], expected=expected,
                expectation_met=bool(passed), **{k:result[k] for k in keys},
                fsm=' -> '.join(r['state'] for r in result['transitions']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--match', default='', help='comma-separated exact scene names')
    parser.add_argument('--resume', action='store_true', help='reuse results with identical scene/source hashes')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    selected = [(s,e) for s,e in matrix() if not args.match or s.name in args.match.split(',')]
    if args.list:
        print(json.dumps([dict(scene=asdict(s),expected=e) for s,e in selected],indent=2)); return
    args.out.mkdir(parents=True, exist_ok=True)
    sources = [ROOT/'ros2_ws/src/edgenode_planning/test/synthetic_s4_loop.py', Path(__file__)]
    sources += sorted((ROOT/'ros2_ws/src/edgenode_planning/edgenode_planning').glob('*.py'))
    sources += sorted((ROOT/'ros2_ws/src/edgenode_control/edgenode_control').glob('*.py'))
    sources += sorted((ROOT/'ros2_ws/src').glob('edgenode_*/config/*.yaml'))
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    source_digest = hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
    write_json(args.out/'sources.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        sha256=hashes,source_digest=source_digest))
    results = []
    for scene, expected in selected:
        folder = args.out/scene.name; folder.mkdir(exist_ok=True)
        artifact = folder/'result.json'
        cached = json.loads(artifact.read_text()) if args.resume and artifact.exists() else None
        if cached and cached.get('source_digest') == source_digest and cached['scene'] == asdict(scene):
            result = cached
        else:
            start = time.perf_counter(); result = simulate(scene)
            result['host_elapsed_s'] = time.perf_counter()-start
            result['source_digest'] = source_digest
            write_json(artifact, result)
            write_csv(folder/'trajectory.csv', result['rows'])
            write_csv(folder/'planning.csv', result['planning_rows'])
            write_json(folder/'fsm.json',dict(avoidance=result['transitions'],planning=result['planning_fsm']))
            plot_result(result, folder/'trajectory.png')
        row = summary(result, expected); results.append(row)
        write_json(args.out/'summary.json',dict(data_kind='SYNTHETIC_NOT_DRIVING_APPROVAL',
            cases=len(results),expectations_met=sum(r['expectation_met'] for r in results),
            returns=sum(r['success'] for r in results),safe_stops=sum(r['safe_model_stop'] for r in results),
            blocked_reasons=dict(Counter(r['reason'] for r in results if r['blocked'])),results=results))
        write_csv(args.out/'summary.csv', results)
        print(json.dumps(clean(row),ensure_ascii=False),flush=True)
    if not all(r['expectation_met'] for r in results):
        sys.exit(1)


if __name__ == '__main__':
    main()
