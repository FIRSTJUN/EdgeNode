#!/usr/bin/env python3
"""Closed-bag measured avoidance speed, steering, recovery and standstill metrics."""
import argparse,json,pathlib,math
import numpy as np
from s4_failure_forensics import messages

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path);p.add_argument('--requested-kmh',type=float,required=True);p.add_argument('--output',type=pathlib.Path);a=p.parse_args();d=a.trial;out=a.output or d/'avoidance-speed-analysis.json';assert not out.exists()
    s=json.loads((d/'summary.json').read_text());start=s['drive_started_ros_ns'];end=s['supervisor_stop_requested_ros_ns'];latest={};episodes=[];rows=[];previous=None;command_rates=[];commands=[]
    topics=('/ego_vehicle_status','/planning/avoidance_debug','/planning/target_speed','/planning/fsm_state','/planning/goal_state','/ctrl_cmd')
    for name,ns,m in messages(d,topics,start,end):
        if name=='/ctrl_cmd':
            if commands and ns>commands[-1][0]:command_rates.append(abs(m.front_steer-commands[-1][1])*.610865/((ns-commands[-1][0])/1e9))
            commands.append((ns,m.front_steer,m.accel,m.brake));continue
        if name!='/ego_vehicle_status':
            try:latest[name]=json.loads(m.data) if isinstance(m.data,str) else m.data
            except ValueError:latest[name]=m.data
            continue
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if previous is not None and stamp<=previous:continue
        previous=stamp;v=math.hypot(m.velocity.x,m.velocity.y,m.velocity.z)*3.6;debug=latest.get('/planning/avoidance_debug',{});stage=debug.get('stage',debug.get('state','UNKNOWN'))
        goal=latest.get('/planning/goal_state',{})
        row=dict(goal_position_error_m=goal.get('position_error_m'),goal_phase=goal.get('phase'),t=(stamp-start)/1e9,speed_kmh=v,stage=stage,state=debug.get('state'),reason=debug.get('reason'),target_kmh=latest.get('/planning/target_speed'),wheel_rad=math.radians(m.front_steer_angle),x=m.position.x,y=m.position.y,fsm=latest.get('/planning/fsm_state'),selected_strategy=debug.get('selected_strategy'),entry_commits=debug.get('entry_commits',0),return_commits=debug.get('return_commits',0),natural_rejoins=debug.get('natural_rejoins',0))
        rows.append(row)
        if not episodes or episodes[-1]['stage']!=stage:episodes.append(dict(stage=stage,rows=[]))
        episodes[-1]['rows'].append(row)
    def stats(records):
        values=[v['speed_kmh'] for v in records];floor=a.requested_kmh-1.;since=None;held=0.;previous_time=None
        for row in records:
            if previous_time is not None and row['t']-previous_time>.2:since=None
            previous_time=row['t']
            since=(row['t'] if since is None else since) if row['speed_kmh']>=floor else None
            if since is not None:held=max(held,row['t']-since)
        return dict(first_t_s=records[0]['t'],last_t_s=records[-1]['t'],duration_s=records[-1]['t']-records[0]['t'],samples=len(records),minimum_kmh=min(values),p05_p50_p95_kmh=[float(v) for v in np.percentile(values,[5,50,95])],maximum_kmh=max(values),mean_kmh=float(np.mean(values)),floor_kmh=floor,longest_at_or_above_floor_s=held)
    timeline=[dict(stage=e['stage'],**stats(e['rows'])) for e in episodes];avoid=[r for r in rows if r['stage'] in ('AVOID_ENTRY','AVOID_HOLD','RETURN')]
    active_started=next((r['t'] for r in rows if r['entry_commits']>0),math.inf);resumed=[r for r in rows if r['t']>=active_started and r['stage']=='NORMAL' and (r['return_commits']>0 or r['natural_rejoins']>0)]
    nonterminal=[];since=None
    for row in rows:
        terminal=row['goal_position_error_m'] is not None and row['goal_position_error_m']<=.8
        stopped=not terminal and row['speed_kmh']<=.15 and row['t']>=next((r['t'] for r in rows if r['speed_kmh']>=1),math.inf)
        if stopped:since=row['t'] if since is None else since
        elif since is not None:
            if row['t']-since>=1:nonterminal.append(dict(start_t_s=since,duration_s=row['t']-since))
            since=None
    if since is not None and rows[-1]['t']-since>=1:nonterminal.append(dict(start_t_s=since,duration_s=rows[-1]['t']-since))
    result=dict(scope='ALL_RAW_EGO_DURING_AUTHORIZED_TRIAL; FSM_PHASES_ARE_RECORDED_SEMANTICS',requested_kmh=a.requested_kmh,timeline=timeline,avoidance_combined=stats(avoid) if avoid else None,normal_after_avoidance=stats(resumed) if resumed else None,nonterminal_standstill_episodes=nonterminal,max_bicycle_lateral_acceleration_mps2=max((r['speed_kmh']/3.6)**2*abs(math.tan(r['wheel_rad']))/2.84 for r in avoid) if avoid else None,command_wheel_rate_p95_max_radps=[float(np.percentile(command_rates,95)),max(command_rates)] if command_rates else None,rows=rows)
    out.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
