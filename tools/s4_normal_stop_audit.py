#!/usr/bin/env python3
"""Independently audit pre-intervention NORMAL hazard standstill in a closed bag.

0.05 km/h is unchanged Control's explicit-stop threshold. An obstacle case
never counts as a successful obstacle-free Phase A driving mission.
"""
import argparse,json,math,pathlib
from s4_failure_forensics import messages

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('trial',type=pathlib.Path);args=parser.parse_args()
    p=args.trial;out=p/'normal-real-obstacle-stop-audit.json';assert not out.exists()
    summary=json.loads((p/'summary.json').read_text());start=summary['drive_started_ros_ns'];end=summary['supervisor_stop_requested_ros_ns']
    latest={};episode=None;best=None;checked=0
    topics=('/ego_vehicle_status','/ctrl_cmd','/planning/target_speed','/planning/avoidance_debug','/planning/s4_sensor_state','/planning/drive_status','/planning/fsm_state')
    for name,ns,m in messages(p,topics,start,end):
        if name!='/ego_vehicle_status':
            if name=='/ctrl_cmd':latest[name]=dict(accel=m.accel,brake=m.brake,steer=m.front_steer)
            else:
                try:latest[name]=json.loads(m.data) if isinstance(m.data,str) else m.data
                except ValueError:latest[name]=m.data
            continue
        checked+=1;speed=math.hypot(m.velocity.x,m.velocity.y,m.velocity.z)*3.6
        debug=latest.get('/planning/avoidance_debug',{});normal=debug.get('phase_a_normal_safety',{});sensor=latest.get('/planning/s4_sensor_state',{});control=latest.get('/ctrl_cmd',{});target=latest.get('/planning/target_speed',999)
        conflict=normal.get('first_conflict_distance_m')
        valid=(speed<=.15 and 0<=target<=.05 and control.get('accel',1)==0 and control.get('brake',0)>=.6 and
            sensor.get('ready') and not sensor.get('latched_reason') and sensor.get('matched_source_age_sec',999)<.6 and
            debug.get('selected_strategy')=='NORMAL_SAFETY_ONLY' and debug.get('entry_commits',0)==debug.get('return_commits',0)==0 and
            debug.get('path_obstacle_count',0)>0 and isinstance(conflict,(float,int)) and conflict<=2.1)
        if valid:
            if episode is None:episode=dict(first_ns=ns,last_ns=ns,max_speed_kmh=speed,max_target_kmh=target,min_brake=control['brake'],first_conflict_distance_m=conflict,frames=0)
            episode.update(last_ns=ns,max_speed_kmh=max(episode['max_speed_kmh'],speed),max_target_kmh=max(episode['max_target_kmh'],target),min_brake=min(episode['min_brake'],control['brake']),frames=episode['frames']+1)
            episode['hold_s']=(ns-episode['first_ns'])/1e9
            if best is None or episode['hold_s']>best['hold_s']:best=dict(episode)
        else:episode=None
    passed=best is not None and best['hold_s']>=2.
    result=dict(passed=passed,classification='EXPECTED_REAL_OBSTACLE_NORMAL_STOP_ENVIRONMENT_CASE',driving_trial_success=False,
        observed_before_supervisor_stop=True,control_stop_threshold_kmh=.05,actual_speed_threshold_kmh=.15,checked_raw_ego_frames=checked,proof=best,summary_error_preserved=summary.get('error'))
    out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));assert passed,'no required actual NORMAL hazard standstill'
if __name__=='__main__':main()
