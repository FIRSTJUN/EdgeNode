#!/usr/bin/env python3
"""Measured dynamics and path handoff evidence from a closed trial bag."""
import argparse,json,math,pathlib
from collections import deque
import numpy as np
from s4_failure_forensics import messages

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial',type=pathlib.Path);a=p.parse_args();d=a.trial
    out=d/'avoidance-dynamics.json';assert not out.exists()
    s=json.loads((d/'summary.json').read_text())
    start=s['drive_started_ros_ns'];end=s['supervisor_stop_requested_ros_ns']
    previous=None;history=deque();rates=[];accels=[];phases=[];stage=None
    for topic,ns,m in messages(d,('/ego_vehicle_status','/planning/avoidance_debug'),start,end):
        if topic=='/planning/avoidance_debug':
            v=json.loads(m.data);new=v.get('stage',v.get('state'))
            if new!=stage:phases.append(dict(t_s=(ns-start)/1e9,stage=new,reason=v.get('reason')))
            stage=new;continue
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        v=math.hypot(m.velocity.x,m.velocity.y,m.velocity.z)
        wheel=math.radians(m.front_steer_angle)
        if previous is not None:
            if stamp<=previous[0]:continue
            dt=(stamp-previous[0])/1e9
            if dt<=.2:rates.append(abs(wheel-previous[1])/dt)
        previous=(stamp,wheel)
        history.append((stamp,v))
        while len(history)>1 and stamp-history[1][0]>=250_000_000:history.popleft()
        dt=(stamp-history[0][0])/1e9
        if .25<=dt<=.45:accels.append((v-history[0][1])/dt)
    def quantiles(values):return [float(v) for v in np.percentile(values,[0,5,50,95,100])] if values else None
    result=dict(scope='Closed raw Ego source timestamps; duplicate/backwards frames excluded',
        measured_front_wheel_rate_p0_p5_p50_p95_p100_radps=quantiles(rates),
        measured_speed_derivative_250ms_p0_p5_p50_p95_p100_mps2=quantiles(accels),
        recorded_phase_transitions=phases,
        notes='Finite differences of simulator-reported speed/wheel angle, not IMU force or commanded slew. No universal physical guarantee.')
    out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
