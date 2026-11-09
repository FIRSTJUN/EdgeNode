#!/usr/bin/env python3
"""Compare captured Stage 5C runs without invoking Planning or ROS."""
import argparse
import csv
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text())


def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path)
    args=parser.parse_args();root=args.runs;out=root/'comparison';out.mkdir(exist_ok=True)
    performance=[];phases=[];signatures=[];lifetimes=[]
    for kind in ('timing','node'):
        for file in sorted((root/('before_'+kind)).glob('*.json')):
            if file.name=='summary.json':continue
            before=read(file);after=read(root/('after_'+kind)/file.name)
            signatures.append(dict(kind=kind,case=file.stem,
                algorithm_signatures_equal=(before['signatures']==after['signatures']) if kind=='timing' else None,
                candidate_count_equal=before['candidate_count']==after['candidate_count'],
                candidate_samples_equal=before['candidate_points']==after['candidate_points'],
                candidate_rejections_equal=before['candidate_rejections']==after['candidate_rejections']))
            for version,value in (('before',before),('after',after)):
                row=dict(version=version,kind=kind,case=file.stem,
                    candidates=value['candidate_count'],candidate_points=json.dumps(value['candidate_points']),
                    obstacles=value['obstacle_count'],preview_points=value['preview_points'])
                for metric in ('wall','cpu'):
                    row['cold_'+metric+'_s']=value['cold'][metric+'_s']
                    for workload in ('warm_search','steady'):
                        for key,val in value[workload+'_'+metric].items():row[workload+'_'+metric+'_'+key]=val
                performance.append(row)
                for phase,values in value['profile_spans'].items():
                    phases.append(dict(version=version,kind=kind,case=file.stem,phase=phase,
                        **{(key[:-3]+'_s' if key.endswith('_ns') else key):
                           (val/1e9 if key.endswith('_ns') else val) for key,val in values.items()}))
                for i,timeline in enumerate(value['timelines']):
                    pubs=timeline['publications'];stamp=timeline['source_stamp_ns']
                    paths=[p for p in pubs if p['topic']=='local_path']
                    lifetimes.append(dict(version=version,case=file.stem,iteration=i,
                        callback_wall_s=(timeline['end_monotonic_ns']-timeline['start_monotonic_ns'])/1e9,
                        source_to_start_s=(timeline['start_ros_ns']-stamp)/1e9,
                        source_to_end_s=(timeline['end_ros_ns']-stamp)/1e9,
                        source_to_last_path_s=(paths[-1]['ros_ns']-stamp)/1e9,
                        source_to_last_output_s=(pubs[-1]['ros_ns']-stamp)/1e9,
                        sensor_reason=timeline['sensor_reason_at_end'],local_points=timeline['local_points'],
                        target_speed=timeline['final_speed']))
    write_csv(out/'performance.csv',performance);write_csv(out/'phases_seconds.csv',phases)
    write_csv(out/'source_lifetimes.csv',lifetimes)
    # Timing and source metadata are intentionally excluded from semantic tests.
    fields=('success','safe_model_stop','full_fsm_sequence','global_path_return','mission_stop_at_s',
        'stopped_at_s','stop_hold_s','blocked','failure_phase','reason','transitions','planning_fsm',
        'candidates','min_obstacle_clearance_m','min_road_margin_m','max_lateral_error_m','max_yaw_error_rad',
        'max_wheel_rad','max_wheel_rate_radps','max_requested_wheel_rate_radps','steering_rate_limited_steps',
        'max_speed_kmh','max_avoidance_speed_kmh','return_error_m','return_heading_rad','post_return_duration_s',
        'stop_time_s','stop_distance_m','blocked_pose','final_pose','final_speed_kmh','modeled_budget_overruns',
        'max_observation_age_s','planning_cycles','validating_cycles','sensor_dropout_at_s',
        'global_reference_sha256','rows','selected_geometry','rollout_failures','world')
    cycle_fields=('t','state','reason','modeled_s','source_age_s','new_validation','speed_kmh','stopping_required_m')
    regressions=[];measured=[];transitions=[]
    for file in sorted((root/'before_matrix').glob('*/result.json')):
        name=file.parent.name;b=read(file);a=read(root/'after_matrix_final'/name/'result.json')
        differences=[key for key in fields if b[key]!=a[key]]
        def projected(value):return [{key:r[key] for key in cycle_fields} for r in value['planning_rows']]
        if projected(b)!=projected(a):differences.append('planning_rows_semantics')
        regressions.append(dict(name=name,equal=not differences,differing_fields=','.join(differences),
            previously_blocked_now_accepted=b['blocked'] and not a['blocked'],before_reason=b['reason'],after_reason=a['reason']))
        for entry in a['transitions']:transitions.append(dict(name=name,fsm='avoidance',**entry))
        for entry in a['planning_fsm']:transitions.append(dict(name=name,fsm='planning',**entry))
    for version in ('before','after'):
        for file in sorted((root/(version+'_measured_loop')).glob('*/result.json')):
            d=read(file)
            measured.append(dict(version=version,name=file.parent.name,success=d['success'],blocked=d['blocked'],
                reason=d['reason'],safe_stop=d['safe_model_stop'],min_clearance_m=d['min_obstacle_clearance_m'],
                min_boundary_m=d['min_road_margin_m'],max_tracking_m=d['max_lateral_error_m'],
                stop_distance_m=d['stop_distance_m'],stop_time_s=d['stop_time_s'],stop_hold_s=d['stop_hold_s'],
                max_source_age_s=d['max_observation_age_s'],
                completed_stale_cycles=sum(bool(r.get('freshness_reason_at_completion')) for r in d['planning_rows']),
                fsm=' -> '.join(t['state'] for t in d['transitions'])))
    write_csv(out/'matrix_semantic_regression.csv',regressions);write_csv(out/'fsm_transitions.csv',transitions)
    write_csv(out/'measured_closed_loop.csv',measured)
    probes=[]
    for version in ('before','after'):
        probe=root/(version+'_ros_probe_10.json')
        if not probe.exists():continue
        for i,trial in enumerate(read(probe)['periodic_trials']):
            calls=trial['callbacks']
            probes.append(dict(version=version,trial=i,calls=len(calls),
                cold_wall_s=calls[0]['wall_s'],max_source_age_s=max(c['source_age_at_end_s'] for c in calls),
                final_state=calls[-1]['state'],final_reason=calls[-1]['reason'],
                over_period=trial['calls_exceeding_period'],missed_start_slots=trial['missed_50ms_start_slots']))
    write_csv(out/'executor_trials.csv',probes)
    report=dict(observed_max_is_not_wcet=True,percentiles_are_sample_estimates=True,
        semantic_fields=list(fields),signature_checks=signatures,matrix=regressions,measured=measured,
        executor_trials=probes,performance=performance)
    (out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    # Export plots as analysis artifacts, never as operational evidence.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names=['straight_left','curve_left','curve_right']
    fig,axes=plt.subplots(1,3,figsize=(17,4.5))
    for version,shift in (('before',-.18),('after',.18)):
        rows=[next(r for r in performance if r['kind']=='node' and r['version']==version and r['case']==name) for name in names]
        axes[0].bar([i+shift for i in range(3)],[r['warm_search_wall_p95'] for r in rows],.36,label=version)
        ages=[max(r['source_to_last_output_s'] for r in lifetimes if r['version']==version and r['case']==name) for name in names]
        axes[1].bar([i+shift for i in range(3)],ages,.36,label=version)
    axes[0].set_title('Fresh-input single callbacks: warm p95')
    axes[1].set_title('Single callbacks: maximum input lifetime')
    axes[1].axhline(.6,color='red',linestyle='--',label='0.6 s unchanged gate')
    for axis in axes[:2]:
        axis.set_xticks(range(3),names);axis.set_ylabel('seconds');axis.legend();axis.grid(axis='y',alpha=.2)
    versions=('before','after')
    axes[2].bar(versions,[sum(r['final_state']!='BLOCKED' for r in probes if r['version']==v) for v in versions],
                color=['C0','C1'])
    axes[2].set_ylim(0,10);axes[2].set_ylabel('trials without stale latch / 10')
    axes[2].set_title('20 Hz executor / fixed pose / 3 s trials');axes[2].grid(axis='y',alpha=.2)
    fig.suptitle('Synthetic inputs / memory outputs / observed values, not WCET or driving approval')
    fig.tight_layout();fig.savefig(out/'timing_and_freshness.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(matrix=len(regressions),exact_matches=sum(r['equal'] for r in regressions),
        unexpected_unblocks=sum(r['previously_blocked_now_accepted'] for r in regressions),
        algorithm_signatures=sum(r['algorithm_signatures_equal'] is True for r in signatures),
        output=str(out))))


if __name__=='__main__':main()
