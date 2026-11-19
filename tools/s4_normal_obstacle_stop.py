#!/usr/bin/env python3
"""Observe a real NPC stop in NORMAL; never send Actor truth to Planning.

The production trial owns NPC placement and safe restoration. This observer
requires a fresh source, a path conflict, zero target, physical standstill and
brake for two seconds BEFORE asking that trial to stop and restore. Its
interrupted driving-trial result remains unsuccessful and is kept separately.
"""
import argparse,json,pathlib,signal,subprocess,sys,time
from morai_drive import Monitor,rclpy
from s4_stage_trial import capture


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    args=parser.parse_args();out=args.output;assert not out.exists()
    before=capture();rclpy.init();node=Monitor(reliable_supervision=True)
    node.spin(2.);assert node.count_publishers('/ctrl_cmd')==1 and node.stopped()
    command=[sys.executable,str(pathlib.Path(__file__).with_name('s4_morai_generalized_trial.py')),
        '--link','A219BS010632','--start','5','--obstacle','42','--approach-speed-kmh','10',
        '--seconds','100','--record-bag','--output',str(out)]
    child=subprocess.Popen(command,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    since=None;proof=None;deadline=time.monotonic()+120
    try:
        while child.poll() is None and time.monotonic()<deadline:
            node.spin(.05);values=node.values;debug=values.get('/planning/avoidance_debug',{})
            normal=debug.get('phase_a_normal_safety',{});conflict=normal.get('first_conflict_distance_m')
            sensor=values.get('/planning/s4_sensor_state',{});control=values.get('/ctrl_cmd',{})
            stopped=(values.get('/ego_vehicle_status',{}).get('speed_kmh',999)<=.15 and
                control.get('accel',1)==0 and control.get('brake',0)>=.1)
            hazard=(isinstance(conflict,(int,float)) and conflict<=2.1 or debug.get('blocked',False))
            valid=(node.fresh() and sensor.get('ready') and not sensor.get('latched_reason') and
                sensor.get('matched_source_age_sec',999)<.6 and debug.get('selected_strategy')=='NORMAL_SAFETY_ONLY' and
                debug.get('entry_commits',0)==debug.get('return_commits',0)==0 and
                (values.get('/planning/drive_status')=='ACTIVE' or debug.get('blocked') and values.get('/planning/fsm_state')=='EMERGENCY_STOP') and
                debug.get('path_obstacle_count',0)>0 and 0<=values.get('/planning/target_speed',999)<=.05 and stopped and hazard)
            since=(since or time.monotonic()) if valid else None
            if since is not None and time.monotonic()-since>=2.:
                proof=dict(hold_s=time.monotonic()-since,observed_before_supervisor_stop=True,snapshot=node.snapshot());break
    finally:
        if child.poll() is None:child.send_signal(signal.SIGINT)
        child.wait(timeout=60)
        node.spin(1.);assert node.stopped()
        after=capture();assert before['modules']==after['modules']
        result=dict(expected_safety_case=True,proof=proof,command=command,child_returncode=child.returncode,
            runtime_before=before,runtime_after=after,final_snapshot=node.snapshot())
        (out/'expected-real-obstacle-stop.json').write_text(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown()
    if proof is None:raise RuntimeError('required pre-intervention NORMAL real-obstacle standstill was not observed')
    print('Verified real-obstacle NORMAL stop before intervention; driving trial remains a failed/blocked environment case.')

if __name__=='__main__':main()
