#!/usr/bin/env python3
"""Closed-bag static campaign audits, paired metrics and measured trajectories."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from s4_live_trial_report import audit
from s4_road_coverage_report import measured_road_mask,report as road_report
from s4_entry_timing_report import report as timing_report,actor_polygon


def csv_write(path,rows):
    fields=list(dict.fromkeys(k for row in rows for k in row)) or ['trial']
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)


def normal_follow_metrics(directory,safety):
    moving=observed=zero=False;stops=zeros=stale=0;maximum_error=0.;planning=[]
    for line in (directory/'samples.jsonl').open():
        row=json.loads(line);values=row['values'];speed=values.get('/ego_vehicle_status',{}).get('speed_kmh',0.)
        if speed>1.:moving=observed=True
        if moving and speed<.2:stops+=1;moving=False
        current=observed and values.get('/planning/target_speed',0.)==0.
        if current and not zero:zeros+=1
        zero=current;stale+=row.get('ages_s',{}).get('/planning/local_path',0.)>.6
        maximum_error=max(maximum_error,abs(values.get('/control/path_error',0.)))
        ms=values.get('/planning/diagnostics',{}).get('planning_elapsed_ms')
        if ms is not None:planning.append(ms)
    integrity=safety.get('all_recorded_message_integrity',{})
    result=dict(physical_stop_episodes_whole_drive=stops,zero_target_episodes_whole_drive_after_motion=zeros,
        sensor_hold_episodes=integrity.get('sensor_hold_episodes'),
        fresh_validation_wait_episodes=integrity.get('fresh_validation_wait_episodes'),
        local_path_loss_episodes=integrity.get('empty_local_path_episodes'),stale_local_path_samples=stale,
        maximum_tracking_error_m=maximum_error,normal_planning_max_ms=max(planning,default=None),
        normal_planning_p95_ms=sorted(planning)[int(.95*(len(planning)-1))] if planning else None)
    cause=directory/'first-emergency-analysis.json'
    if cause.exists():
        data=json.loads(cause.read_text())
        result.update(first_emergency_t_s=data.get('first_emergency_t_s'),first_emergency_reason=data.get('first_emergency_reason'))
    return result


def plot(directory,road_mask=None,boundaries=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    summary=json.loads((directory/'summary.json').read_text())
    rows=[json.loads(line) for line in (directory/'samples.jsonl').open()]
    if not rows:return
    values=[r['values'] for r in rows];egos=[v['/ego_vehicle_status'] for v in values]
    ts=[r['t'] for r in rows];xs=[e['x'] for e in egos];ys=[e['y'] for e in egos]
    figure,axes=plt.subplots(2,2,figsize=(13,9));map_ax,speed_ax,error_ax,path_ax=axes.flat
    route=json.loads((directory/'global-path.json').read_text())
    x0,x1=min(xs)-10,max(xs)+10;y0,y1=min(ys)-10,max(ys)+10
    if road_mask is not None:
        from shapely.geometry import box
        clipped=road_mask.geometry.intersection(box(x0,y0,x1,y1))
        def draw_area(part):
            if part.is_empty:return
            if part.geom_type=='Polygon':
                map_ax.add_patch(Polygon(list(part.exterior.coords),facecolor='lightgray',edgecolor='gray',alpha=.4))
                for ring in part.interiors:
                    map_ax.add_patch(Polygon(list(ring.coords),facecolor='white',edgecolor='gray'))
            elif hasattr(part,'geoms'):
                for child in part.geoms:draw_area(child)
        draw_area(clipped)
    if boundaries is not None:
        labelled=False
        for a,b,(lo_x,hi_x,lo_y,hi_y),identifier in boundaries.forbidden:
            if hi_x<x0 or x1<lo_x or hi_y<y0 or y1<lo_y:continue
            map_ax.plot((a[0],b[0]),(a[1],b[1]),color='firebrick',linewidth=.7,
                label='Forbidden surveyed B2' if not labelled else None)
            labelled=True
    local=[p for p in route if x0<=p[0]<=x1 and y0<=p[1]<=y1]
    if local:map_ax.plot([p[0] for p in local],[p[1] for p in local],'k--',alpha=.5,label='Saved global path')
    colors={'NORMAL':'gray','DETECT':'orange','AVOID_ENTRY':'tab:blue','AVOID_HOLD':'tab:green','RETURN':'tab:purple','BLOCKED':'red'}
    for phase,color in colors.items():
        points=[(e['x'],e['y']) for e,v in zip(egos,values) if v.get('/planning/avoidance_debug',{}).get('stage')==phase]
        if points:map_ax.scatter(*zip(*points),s=4,c=color,label=phase)
    convergence=values[0].get('/localization/grid_convergence_deg',0.)
    actors=([summary['obstacle_state']] if 'obstacle_state' in summary else [])
    actors += [entry['state'] for entry in summary.get('additional_obstacles',[])]
    for actor in actors:
        map_ax.add_patch(Polygon(actor_polygon(actor['bounding_box'],convergence),color='red',alpha=.5))
    map_ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Measured Ego trajectory and post-run Actor boxes')
    map_ax.set_aspect('equal',adjustable='datalim');map_ax.legend(fontsize=7);map_ax.grid(alpha=.3)
    speed_ax.plot(ts,[e['speed_kmh'] for e in egos],label='Measured Ego')
    speed_ax.plot(ts,[v.get('/planning/target_speed',0.) for v in values],label='Target',alpha=.7)
    speed_ax.set(xlabel='Time (s)',ylabel='Speed (km/h)');speed_ax.legend();speed_ax.grid(alpha=.3)
    error_ax.plot(ts,[abs(v.get('/control/path_error',0.)) for v in values])
    error_ax.set(xlabel='Time (s)',ylabel='Absolute control path error (m)');error_ax.grid(alpha=.3)
    path_ax.plot(ts,[v.get('/planning/local_path',{}).get('points',0) for v in values],label='Local path points')
    path_ax.set(xlabel='Time (s)',ylabel='Published control path points');path_ax.grid(alpha=.3)
    figure.suptitle(directory.name);figure.tight_layout();figure.savefig(directory/'measured-trajectory.png',dpi=150);plt.close(figure)


def collect(root,refresh=False):
    boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    mask=measured_road_mask();rows=[]
    for path in sorted(root.rglob('summary.json')):
        directory=path.parent
        summary=json.loads(path.read_text());spec=summary.get('trial_spec',{})
        base=dict(trial=str(directory.relative_to(root)),method=summary.get('s4_parameters',{}).get('static_avoidance_method'),
            **spec,avoidance_speed_kmh=summary.get('s4_parameters',{}).get('avoidance_avoid_speed_kmh'),
            tracking_limit_m=summary.get('s4_parameters',{}).get('avoidance_tracking_limit_m'),
            deceleration_mps2=summary.get('s4_parameters',{}).get('avoidance_deceleration_mps2'),
            success=bool(summary.get('success')),error=summary.get('error'),restored=summary.get('restored'),
            peak_speed_kmh=summary.get('peak_speed_kmh'),code_sha=summary.get('code_sha'),
            drive_started_ros_ns=summary.get('drive_started_ros_ns'),
            followed_distance_m=summary.get('followed_distance_m'),
            working_diff_sha256=summary.get('working_diff_sha256'))
        expected=summary.get('expected_implementation_sha256')
        base['runtime_implementation_sha256']=summary.get('runtime_implementation_sha256')
        base['runtime_code_verified']=(expected==base['runtime_implementation_sha256']) if expected else None
        base.setdefault('obstacle_scale',1.)
        base.setdefault('obstacle_height_scale',base['obstacle_scale'])
        base.setdefault('additional_obstacles',[entry['specification'] for entry in summary.get('additional_obstacles',[])])
        base.setdefault('minimum_progress_m',0.)
        base['comparison_eligible']=not any((directory/name).exists() for name in
            ('placement-geometry-mismatch.json','interrupted-trial.json'))
        base['placement_scale_verified']=summary.get('placement_scale_verification',{}).get('verified')
        extent=summary.get('obstacle_state',{}).get('bounding_box',{}).get('extent',{})
        for dimension,axis in (('width','x'),('length','y'),('height','z')):
            base['measured_obstacle_'+dimension+'_m']=round(2*extent[axis],3) if axis in extent else None
        if any((parent/'runtime-version-unverified.json').exists() for parent in (directory,*directory.parents)):
            rows.append(dict(base,classification='EXCLUDED_RUNTIME_VERSION_UNVERIFIED',strict_continuous_success=False))
            continue
        if not (directory/'bag/metadata.yaml').exists():
            rows.append(dict(base,classification='NO_CLOSED_BAG',strict_continuous_success=False))
            continue
        samples=directory/'samples.jsonl'
        if not samples.exists() or not samples.stat().st_size:
            rows.append(dict(base,classification='PREFLIGHT_FAILURE',strict_continuous_success=False));continue
        if refresh or not (directory/'live_audit.json').exists():audit(directory,boundaries=boundaries)
        if refresh or not (directory/'road_coverage.json').exists():road_report(directory,mask)
        safety=json.loads((directory/'live_audit.json').read_text())
        coverage=json.loads((directory/'road_coverage.json').read_text())
        base.update(collision_samples=safety.get('collision_samples'),
            nonzero_collision_messages=safety.get('all_recorded_message_integrity',{}).get('nonzero_collision_messages'),
            measured_body_boundary_contacts=safety.get('measured_ego_box_boundary_contacts'),
            road_unproven_samples=coverage.get('avoidance_unproven_samples'),
            normal_road_unproven_samples=coverage.get('normal_unproven_samples'),
            minimum_measured_gap_m=safety.get('minimum_measured_all_static_actor_gap_m',safety.get('minimum_measured_actor_box_swept_gap_m')),
            maximum_tracking_error_m=summary.get('max_control_path_error_m'))
        if summary.get('obstacle_state'):
            if refresh or not (directory/'entry_timing.json').exists():timing_report(directory,run_safety_audit=False)
            metrics=json.loads((directory/'entry_timing.json').read_text())
            base['maximum_tracking_error_m']=metrics.get('maximum_lateral_tracking_error_m')
            for key in ('strict_continuous_success','lidar_detection_to_path_commit_s','planner_detection_to_path_commit_s',
                    'local_path_loss_episodes','stale_local_path_samples','sensor_hold_episodes','fresh_validation_wait_episodes',
                    'physical_stop_episodes_after_entry','zero_target_episodes_after_entry','selected_lane_maintained_distance_m',
                    'physical_stop_episodes_whole_drive','zero_target_episodes_whole_drive_after_motion',
                    'selected_lane_switches_after_commit','actual_offset_passage_observed','actual_lane_acquisition_observed',
                    'actual_global_rejoin_observed','independent_road_containment_proven','avoidance_planning_p95_ms',
                    'committed_maneuver_count','all_committed_maneuvers_rejoined',
                    'avoidance_planning_max_ms','natural_rejoins','return_commits','pre_spawn_candidate_count',
                    'normal_resumed_max_target_kmh','normal_resumed_peak_actual_kmh','s4_speed_cap_cleared_after_return'):
                base[key]=metrics.get(key)
            base['strategy_sequence']=metrics.get('strategy_sequence',[])
            base['whole_drive_continuous_success']=bool(base.get('strict_continuous_success') and
                metrics.get('physical_stop_episodes_whole_drive')==0 and
                metrics.get('zero_target_episodes_whole_drive_after_motion')==0)
            strategies=set(base['strategy_sequence'])
            base['both_static_strategies_observed']=bool('OFFSET_STATIC' in strategies and strategies&{'LANE_STATIC','LANE_CANDIDATES'})
            for event,measurement in metrics.get('events',{}).items():
                if measurement:
                    for field in ('t_s','path_front_gap_m','speed_kmh'):
                        if field in measurement:base[event+'_'+field]=measurement[field]
            base['classification']=('STRICT_CONTINUOUS_PASS' if base['strict_continuous_success'] else
                'COMPLETED_WITH_FAILED_OR_UNPROVEN_CRITERIA' if base['success'] else 'FAILED')
        else:
            base.update(normal_follow_metrics(directory,safety))
            base.update(classification='NORMAL_FOLLOW_DISTANCE_COMPLETED' if base['success'] else 'NORMAL_FOLLOW_FAILED',
                strict_continuous_success=False,autonomous_goal_stop_verified=False)
        if (directory/'placement-geometry-mismatch.json').exists():base['classification']='PLACEMENT_GEOMETRY_MISMATCH'
        if (directory/'interrupted-trial.json').exists():base['classification']='INTERRUPTED_INCOMPLETE'
        if refresh or not (directory/'measured-trajectory.png').exists():plot(directory,mask[0],boundaries)
        rows.append(base);print(json.dumps({k:base.get(k) for k in ('trial','classification','error','sensor_hold_episodes','measured_body_boundary_contacts')}),flush=True)
        csv_write(root/'experiment_summary.csv',rows)
    pairs=[]
    keys=('link','start_m','obstacle_link','obstacle_m','obstacle_yaw_deg','obstacle_lateral_m','obstacle_scale',
        'obstacle_height_scale','approach_speed_kmh','avoidance_speed_kmh','tracking_limit_m','deceleration_mps2','additional_obstacles','alternate_goal_node',
        'minimum_progress_m','follow_distance_m','measured_obstacle_width_m','measured_obstacle_length_m','measured_obstacle_height_m')
    for i,first in enumerate(rows):
        for second in rows[i+1:]:
            if any(row.get('classification','').startswith('EXCLUDED_') or not row.get('comparison_eligible',True) for row in (first,second)):continue
            if any(row.get('classification') in ('PREFLIGHT_FAILURE','NO_CLOSED_BAG','INTERRUPTED_INCOMPLETE') for row in (first,second)):continue
            if any(first.get(k)!=second.get(k) for k in keys):continue
            before,after=first,second
            if (before.get('method')!='lattice' and after.get('method')=='lattice' or
                    before.get('method')==after.get('method') and
                    (before.get('drive_started_ros_ns') or 0)>(after.get('drive_started_ros_ns') or 0)):
                before,after=after,before
            pair=dict(before=before['trial'],after=after['trial'],before_method=before.get('method'),after_method=after.get('method'))
            for key in ('success','strict_continuous_success','sensor_hold_episodes','local_path_loss_episodes',
                    'planner_detection_to_path_commit_s','minimum_measured_gap_m','maximum_tracking_error_m','measured_body_boundary_contacts'):
                pair['before_'+key]=before.get(key);pair['after_'+key]=after.get(key)
            pairs.append(pair)
    csv_write(root/'experiment_summary.csv',rows);csv_write(root/'before_after_comparison.csv',pairs)
    digest={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest() for name in
        ('s4_static_campaign_report.py','s4_entry_timing_report.py','s4_live_trial_report.py','s4_road_coverage_report.py')}
    (root/'experiment_summary.json').write_text(json.dumps(dict(trials=rows,pairs=pairs,audit_code_sha256=digest),indent=2)+'\n')
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);parser.add_argument('--refresh',action='store_true')
    args=parser.parse_args();collect(args.root,args.refresh)
