#!/usr/bin/env python3
"""Compare preserved MORAI runs by placement, speed and independently audited outcomes."""
import argparse
import csv
import json
import math
from pathlib import Path

from s4_entry_timing_report import report


def summarize(directory):
    summary=json.loads((directory/'summary.json').read_text())
    if not (directory/'samples.jsonl').exists() or not (directory/'live_audit.json').exists():
        return dict(trial=directory.name,classification='STARTUP_OR_PREFLIGHT_FAILURE',error=summary.get('error'),
            restored=summary.get('restored'),strict_continuous_success=False)
    metrics=report(directory,run_safety_audit=False)
    safety=json.loads((directory/'live_audit.json').read_text())
    spec=summary['trial_spec'];events=metrics['events']
    row=dict(trial=directory.name,method=summary.get('s4_parameters',{}).get('static_avoidance_method') or 'lattice',
        link=spec['link'],start_m=spec['start_m'],obstacle_link=spec['obstacle_link'],obstacle_m=spec['obstacle_m'],
        obstacle_yaw_deg=spec['obstacle_yaw_deg'],obstacle_lateral_m=spec['obstacle_lateral_m'],
        additional_obstacles=';'.join(entry['specification'] for entry in summary.get('additional_obstacles',[])),
        approach_target_kmh=spec['approach_speed_kmh'],peak_actual_kmh=summary.get('peak_speed_kmh'),
        path_and_normal_resume=bool(summary.get('success')),strict_continuous_success=metrics['strict_continuous_success'],
        classification=('CONTINUOUS_DRIVING_PASS' if metrics['strict_continuous_success'] else
            'PATH_COMPLETED_WITH_FAILED_OR_UNPROVEN_CRITERIA' if summary.get('success') else 'FAILED'),
        error=summary.get('error'),restored=summary.get('restored'))
    for key in ('lidar_detection_to_path_commit_s','planner_detection_to_path_commit_s',
            'lateral_movement_to_lane_change_complete_s','path_commit_to_lane_change_complete_s',
            'local_path_loss_episodes','stale_local_path_samples','sensor_hold_episodes','fresh_validation_wait_episodes',
            'zero_target_episodes_after_entry','physical_stop_episodes_after_entry','selected_lane_maintained_distance_m',
            'selected_lane_switches_after_commit','independent_road_containment_proven','natural_rejoins','return_commits',
            'minimum_measured_body_obstacle_gap_m','maximum_lateral_tracking_error_m','prepared_before_obstacle_spawn',
            'pre_spawn_candidate_count'):
        row[key]=metrics.get(key)
    row.update(avoidance_planning_p95_ms=metrics.get('avoidance_planning_p95_ms'),
               avoidance_planning_max_ms=metrics.get('avoidance_planning_max_ms'))
    row.update(collision_samples=safety['collision_samples'],
        measured_body_forbidden_boundary_contacts=safety.get('measured_ego_box_boundary_contacts'),
        sensor_unready_positive_targets=safety.get('sensor_unready_positive_target_samples'),
        zero_target_duration_s=safety.get('s4_zero_target_duration_s'),s4_duration_s=safety.get('s4_duration_s'))
    for key in ('first_lidar_detection','planner_detection','deceleration_start','measured_deceleration_start','lattice_calculation_start',
                'lattice_committed','lateral_motion_start','lane_change_complete'):
        event=events.get(key) or {}
        row[key+'_front_gap_m']=event.get('path_front_gap_m')
        row[key+'_actual_kmh']=event.get('speed_kmh')
    return row


def pair_rows(rows,root):
    pairs=[]
    keys=('link','start_m','obstacle_link','obstacle_m','obstacle_yaw_deg','obstacle_lateral_m','additional_obstacles','approach_target_kmh')
    for old in rows:
        if old.get('method')!='lattice':continue
        for new in rows:
            if new.get('method')!='lane_candidates' or any(old[k]!=new[k] for k in keys):continue
            a=json.loads((root/old['trial']/'summary.json').read_text())
            b=json.loads((root/new['trial']/'summary.json').read_text())
            left=a['obstacle_state']['transform']['location'];right=b['obstacle_state']['transform']['location']
            start_a=a['preflight']['values']['/ego_vehicle_status'];start_b=b['preflight']['values']['/ego_vehicle_status']
            additional=[]
            for old_actor,new_actor in zip(a.get('additional_obstacles',[]),b.get('additional_obstacles',[])):
                x=old_actor['state']['transform']['location'];y=new_actor['state']['transform']['location']
                additional.append(math.hypot(x['x']-y['x'],x['y']-y['y']))
            pair=dict(baseline=old['trial'],native=new['trial'],same_requested_placement_and_speed=True,
                measured_obstacle_position_difference_m=math.hypot(left['x']-right['x'],left['y']-right['y']),
                measured_start_position_difference_m=math.hypot(start_a['x']-start_b['x'],start_a['y']-start_b['y']),
                measured_additional_obstacle_position_differences_m=additional,
                scope='Independent MORAI drives with live LiDAR; sensor frames are not identical replay inputs',metrics={})
            for metric in ('planner_detection_to_path_commit_s','lidar_detection_to_path_commit_s',
                    'lateral_movement_to_lane_change_complete_s','local_path_loss_episodes','sensor_hold_episodes',
                    'fresh_validation_wait_episodes','zero_target_episodes_after_entry','physical_stop_episodes_after_entry',
                    'selected_lane_maintained_distance_m','maximum_lateral_tracking_error_m','strict_continuous_success'):
                pair['metrics'][metric]=dict(baseline=old[metric],native=new[metric])
            pairs.append(pair)
    return pairs


def plot_pair(pair,root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,(ax,bx)=plt.subplots(1,2,figsize=(11,4))
    for key,label in (('baseline','Lattice'),('native','Native lane candidates')):
        rows=[json.loads(s) for s in (root/pair[key]/'samples.jsonl').read_text().splitlines()]
        ax.plot([r['t'] for r in rows],[r['values']['/ego_vehicle_status']['speed_kmh'] for r in rows],label=label)
        bx.plot([r['values']['/ego_vehicle_status']['x'] for r in rows],
                [r['values']['/ego_vehicle_status']['y'] for r in rows],label=label)
    ax.set(xlabel='Time from drive request (s)',ylabel='Measured speed (km/h)');ax.grid(alpha=.3);ax.legend()
    bx.set(xlabel='Map X (m)',ylabel='Map Y (m)');bx.set_aspect('equal',adjustable='datalim');bx.grid(alpha=.3);bx.legend()
    fig.tight_layout();fig.savefig(root/(pair['native']+'-comparison.png'),dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);args=parser.parse_args()
    rows=[summarize(path.parent) for path in sorted(args.root.glob('*/summary.json'))]
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with (args.root/'comparison.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    pairs=pair_rows(rows,args.root)
    for pair in pairs:plot_pair(pair,args.root)
    (args.root/'comparison.json').write_text(json.dumps(dict(trials=rows,pairs=pairs,
        default_method='lattice',default_changed=False,dynamic_driving_development=False),indent=2)+'\n')
    print(json.dumps(dict(trials=len(rows),matched_pairs=len(pairs),strict_passes=[r['trial'] for r in rows if r['strict_continuous_success']])),flush=True)


if __name__=='__main__':main()
