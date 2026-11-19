#!/usr/bin/env python3
"""Closed-bag S073 analysis; original sensor and measured body evidence only."""
import argparse,json,math,pathlib,sys
import numpy as np
from shapely.geometry import Polygon,LineString,box
from shapely.strtree import STRtree
from s4_failure_forensics import messages
from s4_live_trial_report import measured_body_for_audit
from edgenode_planning.lane_geometry import PolylineIndex,sample
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.hdmap_audit import read_layer,translated

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path);p.add_argument('--target-kmh',type=float,required=True);a=p.parse_args()
    d=a.trial;out=d/'scurve-analysis.json';assert not out.exists()
    s=json.loads((d/'summary.json').read_text());baseline=pathlib.Path('/workspace/local_data/s4_final_completion_20261118/full_v61_qualified_normal_01/global-path.json')
    reference=json.loads(baseline.read_text());index=PolylineIndex(reference)
    start=s['drive_started_ros_ns'];end=s.get('distance_target_reached_ros_ns',s['supervisor_stop_requested_ros_ns'])
    convergence=s['preflight']['values']['/localization/grid_convergence_deg'];body=measured_body_for_audit(s,convergence)
    boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    b2=[LineString((x,y)) for x,y,_,identifier in boundaries.forbidden]
    c3=[LineString([v[:2] for v in part]) for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')
        if str(f['attributes']['Type']) in ('2','3','4','5','7','8','99') and str(f['attributes']['LowHigh'])=='2'
        for part in translated(f,boundaries.origin) if len(part)>1]
    trees={k:(STRtree(lines),lines) for k,lines in [('B2',b2),('C3',c3)]}
    def gap(polygon,key):
        tree,lines=trees[key];near=tree.query(box(*polygon.buffer(5.).bounds))
        return min((polygon.distance(v if hasattr(v,'geom_type') else lines[int(v)]) for v in near),default=5.)
    state={};rows=[];commands=[];diagnostics=[]
    topics=('/ego_vehicle_status','/planning/target_speed','/control/path_error','/control/lookahead_distance',
        '/control/pure_pursuit_steering','/planning/diagnostics','/ctrl_cmd')
    for topic,ns,m in messages(d,topics,start,end):
        t=(ns-start)/1e9
        if topic=='/planning/diagnostics':diagnostics.append(json.loads(m.data));continue
        if topic=='/ctrl_cmd':
            commands.append(dict(t=t,steer=m.front_steer,accel=m.accel,brake=m.brake,source_delay_s=(ns-m.header.stamp.sec*10**9-m.header.stamp.nanosec)/1e9));continue
        if topic!='/ego_vehicle_status':state[topic]=m.data;continue
        speed=math.sqrt(m.velocity.x*m.velocity.x+m.velocity.y*m.velocity.y+m.velocity.z*m.velocity.z)*3.6
        pose=(m.position.x,m.position.y,math.radians(m.heading+convergence));station=index.project(pose[:2])
        if not 590<=station<=940:continue
        point,tangent=sample(reference,station,index.lengths);yaw=math.atan2(tangent[1],tangent[0]);lateral=-(pose[0]-point[0])*tangent[1]+(pose[1]-point[1])*tangent[0]
        shape=Polygon(body.polygon(*pose));angle=math.radians(m.front_steer_angle);curvature=math.tan(angle/-1.)/2.84
        rows.append(dict(t=t,station_m=station,x=pose[0],y=pose[1],speed_kmh=speed,target_kmh=state.get('/planning/target_speed'),
            signed_lateral_m=lateral,heading_error_rad=math.atan2(math.sin(pose[2]-yaw),math.cos(pose[2]-yaw)),
            measured_wheel_rad=angle,lateral_acceleration_mps2=(speed/3.6)**2*abs(curvature),
            lookahead_m=state.get('/control/lookahead_distance'),b2_gap_m=gap(shape,'B2'),c3_gap_m=gap(shape,'C3')))
    approach=[v for v in rows if 610<=v['station_m']<=665];curve=[v for v in rows if 722<=v['station_m']<=896];exit=[v for v in rows if 900<=v['station_m']<=922]
    duration=best=0.;previous=None
    for v in approach:
        if v['speed_kmh']>=a.target_kmh-2.:
            duration=duration+v['t']-previous['t'] if previous is not None and previous['speed_kmh']>=a.target_kmh-2. else 0.
            best=max(best,duration)
        else:duration=0.
        previous=v
    braking=next((v for v in rows if 640<=v['station_m']<722 and v['target_kmh'] is not None and v['target_kmh']<min(55.,a.target_kmh-1.)),None)
    errors=[abs(v['signed_lateral_m']) for v in curve];max_error=max(errors,default=math.inf)
    recovery=max((v['speed_kmh'] for v in exit),default=0.);speed_gain=recovery-min((v['speed_kmh'] for v in curve if v['station_m']>870),default=0.)
    cutoff_a=next((v['t'] for v in rows if v['station_m']>=610),0);cutoff_b=next((v['t'] for v in rows if v['station_m']>=922),float('inf'))
    commands=[v for v in commands if cutoff_a<=v['t']<=cutoff_b]
    wheel_rates=[abs(b['steer']-v['steer'])*.610865/(b['t']-v['t']) for v,b in zip(commands,commands[1:]) if b['t']>v['t']]
    thresholds=dict(approach_floor_kmh=a.target_kmh-2.,approach_hold_s=2.,maximum_lateral_error_m=.6,p95_lateral_error_m=.25,recovery_floor_kmh=min(.8*a.target_kmh,45.))
    pass_approach=best>=2.;p95=float(np.percentile(errors,95)) if errors else None
    result=dict(scope='POST_RUN_ALL_EGO_S073_AND_ORIGINAL_REFERENCE',success=s['success'] and pass_approach and max_error<=.6 and p95 is not None and p95<=.25 and recovery>=thresholds['recovery_floor_kmh'],thresholds=thresholds,
        actual_approach_max_kmh=max((v['speed_kmh'] for v in approach),default=0.),actual_approach_floor_hold_s=best,
        braking_start=braking,curve_speed_range_kmh=[min((v['speed_kmh'] for v in curve),default=0.),max((v['speed_kmh'] for v in curve),default=0.)],
        curve_lateral_error_p50_p95_p99_max_m=[float(np.percentile(errors,q)) for q in [50,95,99,100]] if errors else None,
        curve_heading_error_max_deg=max((math.degrees(abs(v['heading_error_rad'])) for v in curve),default=None),
        minimum_actual_body_b2_gap_m=min((v['b2_gap_m'] for v in curve),default=None),minimum_actual_body_c3_gap_m=min((v['c3_gap_m'] for v in curve),default=None),
        max_measured_lateral_acceleration_mps2=max((v['lateral_acceleration_mps2'] for v in curve),default=None),
        max_commanded_wheel_rate_radps=max(wheel_rates,default=None),max_control_receipt_delay_s=max((v['source_delay_s'] for v in commands),default=None),
        planning_p95_ms=float(np.percentile([v['planning_elapsed_ms'] for v in diagnostics],95)) if diagnostics else None,
        recovery_peak_kmh=recovery,recovery_gain_kmh=speed_gain,rows=rows,commands=commands)
    out.write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    stations=[v['station_m'] for v in rows];axes[0].plot(stations,[v['speed_kmh'] for v in rows],label='Measured speed');axes[0].plot(stations,[v['target_kmh'] for v in rows],label='Target speed');axes[0].set_ylabel('Speed [km/h]');axes[0].legend()
    axes[1].plot(stations,[v['signed_lateral_m'] for v in rows],label='Rear-axle lateral error');axes[1].plot(stations,[v['b2_gap_m'] for v in rows],label='Measured body B2 gap');axes[1].plot(stations,[v['c3_gap_m'] for v in rows],label='Measured body C3 gap');axes[1].set_ylabel('Distance [m]');axes[1].legend()
    axes[2].plot(stations,[math.degrees(v['measured_wheel_rad']) for v in rows],label='Measured front wheel [deg]');axes[2].plot(stations,[v['lateral_acceleration_mps2'] for v in rows],label='Measured lateral accel [m/s2]');axes[2].set_ylabel('Steering / acceleration');axes[2].legend();axes[2].set_xlabel('Station on preserved full route [m]')
    for ax in axes:ax.axvspan(722,896,color='#aaa',alpha=.12);ax.grid(alpha=.2)
    fig.suptitle('A073 S curve: '+d.name);fig.tight_layout();fig.savefig(d/'scurve-analysis.png',dpi=150);fig.savefig(d/'scurve-analysis.svg');plt.close(fig)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','commands')},indent=2))
    return int(not result['success'])

if __name__=='__main__':raise SystemExit(main())
