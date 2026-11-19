#!/usr/bin/env python3
"""Additional closed-trial figures for A; never alters qualification results."""
import argparse,bisect,json,math,pathlib
import numpy as np
from s4_failure_forensics import messages

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path);a=p.parse_args();d=a.trial
    analysis=json.loads((d/'scurve-analysis.json').read_text());s=json.loads((d/'summary.json').read_text());out=d/'scurve-control-detail.json';assert not out.exists()
    rows=analysis['rows'];start=s['drive_started_ros_ns'];end=s.get('distance_target_reached_ros_ns',s['supervisor_stop_requested_ros_ns'])
    auxiliary=d.parent/(d.name+'-control-detail');series={}
    for topic,ns,m in messages(auxiliary,('/control/target_left','/control/target_forward','/planning/speed_curvature','/planning/recommended_speed'),start,end):
        series.setdefault(topic,[]).append(((ns-start)/1e9,m.data))
    for name,values in series.items():
        times=[x[0] for x in values]
        for row in rows:
            i=bisect.bisect_right(times,row['t'])-1
            row[name]=values[i][1] if i>=0 and row['t']-times[i]<=.2 else None
    commands=analysis['commands'];rates=[];accels=[]
    for previous,current in zip(commands,commands[1:]):
        dt=(current['t']-current['source_delay_s'])-(previous['t']-previous['source_delay_s'])
        if dt>0:rates.append(abs(current['steer']-previous['steer'])*.610865/dt)
    for i,row in enumerate(rows):
        previous=next((v for v in reversed(rows[:i]) if row['t']-v['t']>=.25),None)
        if previous is not None:accels.append((row['speed_kmh']-previous['speed_kmh'])/3.6/(row['t']-previous['t']))
    times=[v['t'] for v in rows]
    def located(command):
        if command is None:return None
        station=float(np.interp(command['t'],times,[v['station_m'] for v in rows]))
        speed=float(np.interp(command['t'],times,[v['speed_kmh'] for v in rows]))
        return dict(command,station_m=station,actual_speed_kmh=speed)
    relevant=[v for v in commands if 650<=float(np.interp(v['t'],times,[v['station_m'] for v in rows]))<722]
    brake_start=next((v for v in relevant if v['brake']>=.03),None)
    requested=s['trial_spec']['approach_speed_kmh']
    target_reduction=next((v for v in rows if 640<=v['station_m']<722 and v['target_kmh'] is not None and v['target_kmh']<requested-.5),None)
    result=dict(observed_command_braking_start=located(brake_start),target_reduction_start=target_reduction,scope='CLOSED_BAG_ADDITIONAL_DIAGNOSTICS; NO_PLANNER_INPUT_OR_THRESHOLD_CHANGE',
        pp_frame='Recorded target_forward/target_left relative to the Control predicted vehicle pose; not reconstructed as unmeasured world truth',
        lateral_acceleration_definition='Bicycle estimate from actual measured speed and front wheel angle; not an IMU force measurement',
        command_wheel_rate_p95_max_radps=[float(np.percentile(rates,95)),max(rates)] if rates else None,
        longitudinal_acceleration_p05_p95_min_max_mps2=[float(np.percentile(accels,5)),float(np.percentile(accels,95)),min(accels),max(accels)] if accels else None,
        pp_targets_available_samples=sum(row.get('/control/target_forward') is not None and row.get('/control/target_left') is not None for row in rows),rows=rows)
    out.write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    reference=json.loads(pathlib.Path('/workspace/local_data/s4_final_completion_20261118/full_v61_qualified_normal_01/global-path.json').read_text())
    from edgenode_planning.lane_geometry import arc
    original=[point for point,station in zip(reference,arc(reference)) if 680<=station<=930]
    actual=[v for v in rows if 680<=v['station_m']<=930];fig,axes=plt.subplots(1,2,figsize=(11,6))
    axes[0].plot([v[0] for v in original],[v[1] for v in original],label='Fixed original reference')
    axes[0].plot([v['x'] for v in actual],[v['y'] for v in actual],'--',label='Measured Ego rear axle');axes[0].axis('equal');axes[0].set_xlabel('Map x [m]');axes[0].set_ylabel('Map y [m]');axes[0].legend()
    stations=[v['station_m'] for v in rows]
    axes[1].plot(stations,[v.get('/control/target_forward') for v in rows],label='PP target forward [m]');axes[1].plot(stations,[v.get('/control/target_left') for v in rows],label='PP target left [m]');axes[1].plot(stations,[v.get('lookahead_m') for v in rows],label='Arc lookahead [m]');axes[1].set_xlabel('Full route station [m]');axes[1].legend()
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle(d.name+' | actual trajectory and recorded PP target');fig.tight_layout();fig.savefig(d/'scurve-trajectory-pp.png',dpi=150);fig.savefig(d/'scurve-trajectory-pp.svg');plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
    axes[0].plot(stations,[v['signed_lateral_m'] for v in rows],label='Signed lateral error');axes[0].plot(stations,[min(5.,v['b2_gap_m']) for v in rows],label='Measured body B2 gap (display capped 5m)');axes[0].plot(stations,[min(5.,v['c3_gap_m']) for v in rows],label='Measured body C3 gap (display capped 5m)');axes[0].set_ylabel('Distance [m]');axes[0].legend()
    axes[1].plot([v['station_m'] for v in rows],[math.degrees(v['heading_error_rad']) for v in rows],label='Heading error [deg]');axes[1].set_xlabel('Full route station [m]');axes[1].set_ylabel('Heading error [deg]');axes[1].legend()
    for ax in axes:ax.axvspan(722,896,color='#aaa',alpha=.12);ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(d/'scurve-clearance-heading.png',dpi=150);fig.savefig(d/'scurve-clearance-heading.svg');plt.close(fig)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))

if __name__=='__main__':main()
