#!/usr/bin/env python3
"""Post-run analysis only; Actor truth never returns to the live planner."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from edgenode_planning.avoidance_geometry import polygon_distance, swept_body
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.lane_geometry import LaneNetwork, PolylineIndex, sample
from edgenode_planning.vehicle_geometry import sonata_geometry


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('trial',type=Path)
    args=parser.parse_args();summary=json.loads((args.trial/'summary.json').read_text())
    rows=[json.loads(line) for line in (args.trial/'samples.jsonl').read_text().splitlines()]
    if not rows:raise RuntimeError('no actual motion records')
    global_path=json.loads((args.trial/'global-path.json').read_text())
    index=PolylineIndex(global_path);body=sonata_geometry()
    mapping=DijkstraPlanner('/workspace/local_data/kcity_mgeo');network=LaneNetwork(mapping)
    network.boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    settings=summary['settings'];side=summary.get('selected_side');road=None
    if settings.get('goal_node'):
        route=mapping.plan_route(settings['link'],[settings['goal_node']])
    else:
        from edgenode_planning.route_resume import mission_suffix
        import yaml
        saved=yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
        route=mission_suffix(mapping,saved.get('start_link_id') or summary['original_mission']['route_start_link_id'],
            saved['waypoint_node_ids'],settings['link'])
    if side:
        pair=network.pair(settings['link'],side,route['link_ids'],route_link_index=0)
        if pair is not None:road=network.boundaries.road_corridor_for_pair(network,pair)
    trajectory=[];boundary_contacts=[];unproven=[];sensor_motion=[];events={}
    actual_gap=math.inf;previous=None;last_state=None;obstacle_state=summary.get('obstacle_state')
    for number,row in enumerate(rows):
        values=row['values'];ego=values['/ego_vehicle_status'];debug=values.get('/planning/avoidance_debug',{})
        basis=values.get('/localization/grid_convergence_deg')
        if basis is None:raise RuntimeError('missing measured heading basis')
        pose=(ego['x'],ego['y'],math.radians(ego['heading_deg']+basis))
        station=index.project(pose[:2]);nearest=sample(global_path,max(0.,min(index.lengths[-1],station)))
        delta=(pose[0]-nearest[0][0],pose[1]-nearest[0][1]);tangent=nearest[1]
        cross=delta[0]*(-tangent[1])+delta[1]*tangent[0]
        solid=swept_body(body,previous or pose,pose);previous=pose
        reason=network.boundaries.swept_reason(solid)
        if reason:boundary_contacts.append(dict(t=row['t'],reason=reason,pose=pose))
        if road is not None and not road.covers(solid):unproven.append(dict(t=row['t'],pose=pose))
        sensor=values.get('/planning/s4_sensor_state',{})
        if not sensor.get('ready') and values.get('/planning/target_speed',0.)>0.:
            sensor_motion.append(row['t'])
        obstacle=None;front_gap=None;rear_passed=False;detected=False
        if obstacle_state:
            box=obstacle_state['bounding_box'];yaw=math.radians(box['rotation']['z']+basis)
            c,s=math.cos(yaw),math.sin(yaw);width=box['extent']['x'];length=box['extent']['y']
            x,y=box['location']['x'],box['location']['y']
            obstacle=tuple((x+c*a-s*b,y+s*a+c*b) for a,b in
                ((-length,-width),(length,-width),(length,width),(-length,width)))
            actual_gap=min(actual_gap,polygon_distance(solid,obstacle))
            ego_body=body.polygon(*pose)
            front_gap=min(index.project(p) for p in obstacle)-max(index.project(p) for p in ego_body)
            rear_passed=min(index.project(p) for p in ego_body)>max(index.project(p) for p in obstacle)+.5
            # Ground truth association is performed HERE, after the run. It
            # neither publishes observations nor influences control decisions.
            detected=any(polygon_distance(tuple(map(tuple,group)),obstacle)<.5
                         for group in row.get('solids',[]) if len(group)>=3)
        stage=debug.get('stage',debug.get('state'));state=debug.get('state')
        event=dict(t=row['t'],speed_kmh=ego['speed_kmh'],front_route_gap_m=front_gap,
            obstacle_stamp_ns=row.get('obstacle_stamp_ns'),pose_stamp_ns=values.get('pose_stamp_ns'))
        checks={
            'first_lidar_observation':detected,
            'first_path_confirmation':debug.get('confirmed_path_obstacles',0)>0,
            'first_s4_detection':stage=='DETECT',
            'first_s4_deceleration_command':stage=='DETECT' and
                values.get('/planning/target_speed',settings['normal_kmh'])<settings['normal_kmh']-.05,
            'first_entry_search':debug.get('reason')=='ENTRY_SEARCH_PENDING',
            'first_candidate_result':debug.get('candidate_count',0)>0,
            'first_avoid_entry_state':stage=='AVOID_ENTRY',
            'first_actual_lateral_entry':stage=='AVOID_ENTRY' and abs(cross)>.2,
            'first_rear_body_pass':rear_passed,
            'first_return':stage=='RETURN',
            'return_completed':stage=='NORMAL' and debug.get('return_commits',0)>0,
        }
        for name,condition in checks.items():
            if condition and name not in events:events[name]=event
        if stage!=last_state:
            events.setdefault('transitions',[]).append(dict(event,stage=stage,state=state))
            last_state=stage
        wheel=-math.radians(ego.get('front_steer_deg',0.))
        command=-values.get('/ctrl_cmd',{}).get('steer',0.)*.610865
        trajectory.append(dict(t=row['t'],pose=pose,station=station,cross_track_m=cross,
            control_error_m=values.get('/control/path_error',0.),speed_kmh=ego['speed_kmh'],
            target_kmh=values.get('/planning/target_speed',0.),wheel_rad=wheel,command_rad=command,
            pp_rad=values.get('/control/pure_pursuit_steering',0.)*.610865,
            lookahead_m=values.get('/control/lookahead_distance'),stage=stage,
            source_age_s=sensor.get('matched_source_age_sec'),
            raw_yaw_rate=ego.get('angular_velocity_xyz',[0.,0.,0.])[2]))
    times=np.array([r['t'] for r in trajectory]);speeds=np.array([r['speed_kmh'] for r in trajectory])
    wheel=np.array([r['wheel_rad'] for r in trajectory]);command=np.array([r['command_rad'] for r in trajectory])
    # Source time is essential: asynchronous callbacks may deliver a heading
    # change in a short wall-time interval without a physical yaw-rate spike.
    unique=[];last_stamp=None
    for i,row in enumerate(rows):
        source=row['values']['/ego_vehicle_status'].get('stamp_ns')
        if source is not None and source!=last_stamp:unique.append((source,i));last_stamp=source
    source_times=np.array([(stamp-unique[0][0])/1e9 for stamp,_ in unique])
    indices=np.array([i for _,i in unique]);yaw=np.unwrap([trajectory[i]['pose'][2] for i in indices])
    yaw_rate=np.gradient(yaw,source_times)
    predicted_rate=speeds[indices]/3.6*np.tan(wheel[indices])/2.84
    mask=(speeds[indices]>10.)&(np.abs(wheel[indices])>.04)
    ratio=float(np.dot(predicted_rate[mask],yaw_rate[mask])/np.dot(predicted_rate[mask],predicted_rate[mask])) if mask.any() else None
    delay=[]
    for lag in np.arange(0.,.401,.02):
        delayed=np.interp(times-lag,times,command)
        delay.append((float(np.mean((wheel-delayed)**2)),float(lag)))
    active=[r for r in trajectory if r['speed_kmh']>1.]
    result=dict(success=summary.get('success',False),settings=settings,events=events,
        phases=summary.get('phases'),peak_speed_kmh=float(speeds.max()),
        mean_moving_speed_kmh=float(np.mean([r['speed_kmh'] for r in active])) if active else 0.,
        max_control_path_error_m=max(r['control_error_m'] for r in trajectory),
        mean_control_path_error_m=float(np.mean([r['control_error_m'] for r in trajectory])),
        max_measured_global_cross_track_m=max(abs(r['cross_track_m']) for r in trajectory),
        mean_measured_global_cross_track_m=float(np.mean([abs(r['cross_track_m']) for r in trajectory])),
        collision_samples=sum(r['values'].get('/collision_data',0)!=0 for r in rows),
        forbidden_boundary_contacts=boundary_contacts,unproven_road_samples=unproven,
        road_proof_available=road is not None,sensor_unready_positive_target_samples=sensor_motion,
        max_matched_source_age_s=max((r['source_age_s'] for r in trajectory if r['source_age_s'] is not None),default=None),
        return_global_cross_track_m=trajectory[-1]['cross_track_m'],
        actor_box_min_body_gap_m=actual_gap if math.isfinite(actual_gap) else None,
        estimated_command_to_wheel_delay_s=min(delay)[1],
        moving_yaw_response_vs_bicycle=ratio,
        identification_note='Approximate asynchronous telemetry fit; not a calibrated tire model.',
        yaw_rate_time_basis='unique Ego source headers',
        restored=summary.get('restored'),final_stop_confirmed=summary.get('final_stop_confirmed'),
        saved_route_unchanged=summary.get('saved_route_unchanged'),distance_m=summary.get('distance_m'),
        driving_time_s=summary.get('driving_time_s'))
    (args.trial/'analysis.json').write_text(json.dumps(result,indent=2))
    (args.trial/'trajectory.json').write_text(json.dumps(trajectory,separators=(',',':')))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,2,figsize=(14,10))
        x=[r['pose'][0] for r in trajectory];y=[r['pose'][1] for r in trajectory]
        axes[0,0].plot([p[0] for p in global_path],[p[1] for p in global_path],'k--',lw=1,label='Global Path')
        axes[0,0].scatter(x,y,c=speeds,s=6,cmap='viridis',label='actual Ego')
        if obstacle_state:
            axes[0,0].plot([p[0] for p in obstacle+(obstacle[0],)],
                [p[1] for p in obstacle+(obstacle[0],)],'r-',label='trial object: evaluation only')
        axes[0,0].set(xlim=(min(x)-8,max(x)+8),ylim=(min(y)-8,max(y)+8),xlabel='map x (m)',ylabel='map y (m)')
        axes[0,0].axis('equal');axes[0,0].set_xlim(min(x)-8,max(x)+8);axes[0,0].set_ylim(min(y)-8,max(y)+8)
        axes[0,1].plot(times,speeds,label='actual km/h');axes[0,1].plot(times,[r['target_kmh'] for r in trajectory],label='target km/h')
        axes[1,0].plot(times,[r['control_error_m'] for r in trajectory],label='Control path error (m)')
        axes[1,0].plot(times,[r['cross_track_m'] for r in trajectory],label='signed Global cross track (m)')
        axes[1,1].plot(times,np.degrees(command),label='command: left positive deg')
        axes[1,1].plot(times,np.degrees(wheel),label='measured wheel deg')
        for ax in axes.flat:ax.grid();ax.legend(fontsize=8)
        fig.suptitle(args.trial.name);fig.tight_layout();fig.savefig(args.trial/'trajectory-and-control.png',dpi=150);plt.close(fig)
    except ImportError:pass
    print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))


if __name__=='__main__':main()
