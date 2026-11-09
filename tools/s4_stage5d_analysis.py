#!/usr/bin/env python3
"""Export Stage 5D raw timing/steering evidence; maxima are observations only."""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
from unittest.mock import patch


def read(path):return json.loads(path.read_text())


def csv_file(path,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)


def replay(root,runs,out):
    from s4_timing_audit import load
    loop,m=load(root);module=m['static_obstacle_planner'];out.mkdir(parents=True,exist_ok=True)
    for file in runs.glob('*/result.json'):
        d=read(file)
        if 'STEERING_RATE' not in d['reason'] or not d['rollout_failures']:continue
        failure=d['rollout_failures'][-1];scene=loop.Scene(**d['scene']);road=loop.Road(scene)
        geometry=d['selected_geometry'][0];path=geometry['entry_path']
        config=loop.AvoidanceConfig();planner=loop.StaticObstaclePlanner(road.network,drivable_area=road.area,config=config)
        if failure['state']=='RETURN' or d['reason'].startswith('RETURN_'):path=geometry['return_path']
        elif failure['state']=='AVOID_HOLD' and root.name=='baseline_src':
            pair=road.network.pair('SYN_O0',scene.side,road.route_ids)
            ego=pair.original_index.project(geometry['entry_path'][0][:2])
            start=pair.at(ego)['adjacent_s'];end=pair.at(min(pair.length-planner.body.wheelbase_m-
                planner.body.front_overhang_m-1.,ego+config.prediction_distance_m))['adjacent_s']
            path=module.center_suffix(pair.adjacent,start,end)
        started=d['planning_rows'][-1]['t']
        prior=max((r for r in d['rows'] if r['t']<started-1e-9),key=lambda r:r['t'])
        curvature=math.tan(prior['wheel_rad'])/config.wheelbase_m
        initial=math.atan(curvature*config.wheelbase_m);applied=initial;previous=initial;requested=initial
        queue=deque();rows=[];original=planner.pp.compute;dt=1/config.control_rate_hz
        def traced(*args,**kwargs):
            nonlocal applied,previous,requested
            response=original(*args,**kwargs)
            if response is None:return response
            tick=len(rows);command=module.to_morai_front_steer(response['steering_normalized'],
                config.steering_sign,config.max_front_steer_normalized)
            angle=command/config.steering_sign*config.max_wheel_angle_rad
            queue.append((tick*dt+config.command_delay_s,angle))
            while queue and queue[0][0]<=tick*dt+1e-12:applied=queue.popleft()[1]
            rate=abs(applied-previous)/dt
            rows.append(dict(tick=tick,rollout_s=tick*dt,scene_t=started+tick*dt,
                x=args[1],y=args[2],yaw=args[3],target_x=response['target_point'][0],target_y=response['target_point'][1],
                requested_wheel_rad=angle,previous_requested_rad=requested,applied_wheel_rad=applied,
                wheel_rate_radps=rate,rate_limit_radps=config.max_wheel_rate_radps,
                first_violation=rate>config.max_wheel_rate_radps,initial_actual_wheel_rad=initial))
            previous=applied;requested=angle
            return response
        with patch.object(planner.pp,'compute',traced):
            reason=planner._rollout(path,tuple(failure['pose']),failure['speed_kmh'],curvature,
                module.obstacle_polygons(d['world']['obstacles']))
        assert reason=='STEERING_RATE',(file,reason,failure)
        value=dict(file=str(file),module=module.__file__,failure=failure,planning_t=started,
            replay_reason=reason,first_violation=next(r for r in rows if r['first_violation']),rows=rows,
            candidate_curvature_max=max(map(abs,loop.curvatures(geometry['points']))),
            curvature_limit=config.curvature_limit)
        (out/(file.parent.name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
        csv_file(out/(file.parent.name+'.csv'),rows)


def compare(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=root/'comparison';out.mkdir(exist_ok=True)
    loops=[];regressions=[];raw=[];latencies=[];summaries=[]
    for version in ('before','after'):
        for file in sorted((root/(version+'_measured')).glob('*/result.json')):
            d=read(file)
            loops.append(dict(version=version,name=file.parent.name,success=d['success'],reason=d['reason'],
                safe_stop=d['safe_model_stop'],min_obstacle_m=d['min_obstacle_clearance_m'],min_boundary_m=d['min_road_margin_m'],
                max_tracking_m=d['max_lateral_error_m'],max_wheel_rad=d['max_wheel_rad'],
                max_physical_wheel_rate=d['max_wheel_rate_radps'],max_requested_wheel_rate=d['max_requested_wheel_rate_radps'],
                stop_distance_m=d['stop_distance_m'],stop_time_s=d['stop_time_s'],hold_s=d['stop_hold_s'],
                fsm=' -> '.join(r['state'] for r in d['transitions'])))
        for file in sorted((root/(version+'_dds')).glob('*.json')):
            if file.name=='summary.json':continue
            d=read(file);rows=d['rows'];ticks=[r for r in rows if r['kind']=='planning_tick']
            emitted={e['source_stamp_ns']:e for e in d['emissions']}
            for row in rows:
                raw.append(dict(version=version,trial=file.stem,**row))
                source=row.get('source_stamp_ns');emission=emitted.get(source)
                if row['kind']=='planning_receive' and emission:
                    latency=(row['monotonic_ns']-emission['emitted_monotonic_ns'])/1e9
                    latencies.append(dict(version=version,trial=file.stem,topic=row['topic'],
                        source_stamp_ns=source,queue_plus_transport_s=latency,
                        source_to_receipt_s=(row['ros_ns']-source)/1e9))
            durations=[t['wall_s'] for t in ticks if not t['skipped']]
            waits=[r['queue_plus_transport_s'] for r in latencies if r['version']==version and r['trial']==file.stem]
            summaries.append(dict(version=version,trial=file.stem,event=d['event'],workload=d['workload'],
                state=d['final_state'],reason=d['final_reason'],ticks=len(ticks),
                skipped=sum(t['skipped'] for t in ticks),max_callback_s=max(durations,default=0),
                max_queue_s=max(waits,default=0),control_commands=sum(r['kind']=='control_execution' for r in rows),
                nonzero_accel=sum(r.get('accel',0)>0 for r in rows)))
    for before in sorted((root/'before_matrix').glob('*/result.json')):
        after=root/'after_matrix'/before.parent.name/'result.json'
        if not after.exists():continue
        b=read(before);a=read(after)
        fields=('success','blocked','safe_model_stop','reason','global_reference_sha256')
        regressions.append(dict(name=before.parent.name,expected_outcomes_equal=all(b[k]==a[k] for k in fields),
            unsafe_unblock=b['blocked'] and not a['blocked'],before_reason=b['reason'],after_reason=a['reason'],
            transitions_equal=b['transitions']==a['transitions'],
            min_clearance_before=b['min_obstacle_clearance_m'],min_clearance_after=a['min_obstacle_clearance_m'],
            min_boundary_before=b['min_road_margin_m'],min_boundary_after=a['min_road_margin_m'],
            max_tracking_before=b['max_lateral_error_m'],max_tracking_after=a['max_lateral_error_m']))
    csv_file(out/'measured_loops.csv',loops);csv_file(out/'matrix_regression.csv',regressions)
    csv_file(out/'ros_events.csv',raw);csv_file(out/'queue_and_transport.csv',latencies);csv_file(out/'ros_summary.csv',summaries)
    (out/'comparison.json').write_text(json.dumps(dict(measured=loops,regressions=regressions,ros=summaries,
        observed_max_is_not_wcet=True),indent=2)+'\n')
    fig,axes=plt.subplots(3,2,figsize=(14,11))
    colors={'before':'C0','after':'C1'}
    for version in ('before','after'):
        file=root/(version+'_measured')/'A_curve_left_3/result.json'
        d=read(file);rows=d['rows'];t=[r['t'] for r in rows]
        axes[0,0].plot([r['x'] for r in rows],[r['y'] for r in rows],label=version,color=colors[version])
        axes[0,1].plot(t,[r['lateral_error_m'] for r in rows],label=version)
        axes[1,0].plot(t,[r['requested_wheel_rad'] for r in rows],label=version+' requested')
        axes[1,0].plot(t,[r['wheel_rad'] for r in rows],label=version+' physical',alpha=.6,linestyle='--')
        cycles=d['planning_rows'];axes[1,1].plot([r['t'] for r in cycles],[r['wall_s'] for r in cycles],label=version)
        sample=[r for r in raw if r['version']==version and r['trial']=='normal_nominal_0' and r['kind']=='planning_tick']
        if sample:
            start=sample[0]['monotonic_ns']
            axes[2,0].plot([(r['monotonic_ns']-start)/1e9 for r in sample],[r['wall_s'] for r in sample],label=version)
        waits=[r for r in latencies if r['version']==version and r['trial']=='normal_nominal_0' and r['topic'].endswith('lidar_status')]
        axes[2,1].plot(range(len(waits)),[r['queue_plus_transport_s'] for r in waits],label=version)
    for axis,title in zip(axes.flat,['Curve left trajectory','Lateral error (m)',
        'Production PP command / physical wheel (rad)','Measured core wall time (s)',
        'Live DDS Planning callback wall (s)','Status transport + executor wait (s)']):
        axis.set_title(title);axis.grid(alpha=.2);axis.legend()
    axes[1,1].axhline(.05,color='red',linestyle='--');axes[2,0].axhline(.05,color='red',linestyle='--')
    fig.suptitle('Synthetic / disarmed: full logs include failed cases; no driving approval')
    fig.tight_layout();fig.savefig(out/'delay_and_steering.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(modeled=len(regressions),same_expected=sum(r['expected_outcomes_equal'] for r in regressions),
        measured_after_returns=sum(r['success'] for r in loops if r['version']=='after'),
        measured_after_cases=sum(r['version']=='after' for r in loops),ros_trials=len(summaries))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path)
    parser.add_argument('--replay-code-root',type=Path)
    parser.add_argument('--replay-runs',type=Path)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    if args.replay_code_root:replay(args.replay_code_root,args.replay_runs,args.out)
    else:compare(args.runs)


if __name__=='__main__':main()
