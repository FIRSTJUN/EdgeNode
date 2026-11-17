#!/usr/bin/env python3
"""Audit completed MORAI trials: observe, slow, commit, move laterally, acquire.

Actor geometry is used only for post-run measurement association and audit.
This tool neither publishes ROS data nor changes planner inputs.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sqlite3
from functools import lru_cache

from edgenode_planning.avoidance_geometry import polygon_distance,swept_body
from edgenode_planning.lane_geometry import PolylineIndex,sample
from edgenode_planning.vehicle_geometry import sonata_geometry
from s4_live_trial_report import audit,measured_body_for_audit


@lru_cache(maxsize=1)
def native_centerlines():
    from edgenode_planning.dijkstra_planner import DijkstraPlanner
    return DijkstraPlanner('/workspace/local_data/kcity_mgeo').links_by_id


def actor_polygon(box,convergence):
    yaw=math.radians(box['rotation']['z']+convergence);c,s=math.cos(yaw),math.sin(yaw)
    center=box['location'];extent=box['extent']
    return tuple((center['x']+c*f-s*l,center['y']+s*f+c*l)
        for f,l in ((-extent['y'],-extent['x']),(extent['y'],-extent['x']),
                    (extent['y'],extent['x']),(-extent['y'],extent['x'])))


def event_metrics(ego,convergence,index,target,body):
    pose=(ego['x'],ego['y'],math.radians(ego['heading_deg']+convergence))
    polygon=body.polygon(*pose)
    return dict(path_front_gap_m=min(index.project(p) for p in target)-max(index.project(p) for p in polygon),
        body_euclidean_gap_m=polygon_distance(polygon,target),speed_kmh=ego['speed_kmh'])


def first_lidar_detection(directory,target,index,body,fallback_convergence):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from morai_ros2_msgs.msg import EgoVehicleStatus
    history=deque(maxlen=300);convergence=fallback_convergence
    for database in sorted((directory/'bag').glob('*.db3')):
        with sqlite3.connect('file:'+str(database)+'?mode=ro',uri=True) as connection:
            kinds={i:(name,get_message(kind)) for i,name,kind in connection.execute('select id,name,type from topics')
                   if name in ('/ego_vehicle_status','/perception/lidar_frame','/localization/grid_convergence_deg')}
            if not kinds:continue
            marks=','.join('?' for _ in kinds)
            for tid,stamp,data in connection.execute(f'select topic_id,timestamp,data from messages where topic_id in ({marks}) order by timestamp',tuple(kinds)):
                name,kind=kinds[tid];msg=deserialize_message(data,kind)
                if isinstance(msg,EgoVehicleStatus):
                    source=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                    ego=dict(x=msg.position.x,y=msg.position.y,heading_deg=msg.heading,
                        speed_kmh=3.6*math.hypot(msg.velocity.x,msg.velocity.y,msg.velocity.z))
                    history.append((source or stamp,ego));continue
                if name=='/localization/grid_convergence_deg':convergence=msg.data;continue
                envelope=json.loads(msg.data);status=envelope['status']
                if not status['state'].startswith('READY_') or not history:continue
                points=envelope.get('points',[])
                pieces=[tuple(map(tuple,points[i:i+8])) for i in range(0,len(points),8)]
                if not any(piece and polygon_distance(piece,target)<=.5 for piece in pieces):continue
                source=status['source_stamp_ns'];pose_stamp,ego=min(history,key=lambda row:abs(row[0]-source))
                if abs(pose_stamp-source)>250_000_000:continue
                return dict(event_metrics(ego,convergence,index,target,body),
                    source_stamp_ns=source,ego_stamp_gap_s=abs(pose_stamp-source)/1e9,
                    source='CLOSED_ROSBAG_LIDAR_FOOTPRINT_ASSOCIATION_AUDIT_ONLY')
    return None


def report(directory,run_safety_audit=True):
    if not (directory/'bag/metadata.yaml').exists():raise ValueError('a closed rosbag is required')
    summary=json.loads((directory/'summary.json').read_text())
    if summary.get('verification','MORAI_LIVE')!='MORAI_LIVE':
        raise ValueError('static driving audit cannot classify a tracking-only experiment as driving success')
    rows=[json.loads(s) for s in (directory/'samples.jsonl').read_text().splitlines()]
    route=json.loads((directory/'global-path.json').read_text());index=PolylineIndex(route)
    first=rows[0]['values'];convergence=first.get('/localization/grid_convergence_deg',0.)
    target=actor_polygon(summary['obstacle_state']['bounding_box'],convergence)
    body=measured_body_for_audit(summary,convergence) or sonata_geometry()
    events={};initial_lateral=None;lateral_since=None;movement_started=False
    sensor_holds=validation_waits=path_losses=stale_paths=stops=0;previous_key=None;lost=False;stopped=False
    max_error=0.;min_gap=math.inf;before=None
    lane_distance=0.;previous_xy=None;previous_stage=None;lane_tokens=[];planning_times=[]
    physical_stops=0;moving=False;previous_target=None;previous_ego=None
    lane_index=None;lane_points=None;lane_id=None;acquired_since=None;acquired_measure=None
    rejoin_since=None;rejoin_measure=None
    for row in rows:
        values=row['values'];ego=values.get('/ego_vehicle_status',{});debug=values.get('/planning/avoidance_debug',{})
        if not ego:continue
        convergence=values.get('/localization/grid_convergence_deg',convergence)
        measure=dict(event_metrics(ego,convergence,index,target,body),t_s=row['t'])
        station=index.project((ego['x'],ego['y']));point,tangent=sample(route,max(0.,min(index.lengths[-1],station)))
        lateral=-tangent[1]*(ego['x']-point[0])+tangent[0]*(ego['y']-point[1])
        if initial_lateral is None:initial_lateral=lateral
        key=debug.get('state'),debug.get('reason')
        if key!=previous_key:
            sensor_holds+=key[0]=='SENSOR_HOLD' and (previous_key is None or previous_key[0]!='SENSOR_HOLD')
            validation_waits+=key[1]=='WAIT_FRESH_VALIDATION' and (previous_key is None or previous_key[1]!='WAIT_FRESH_VALIDATION')
        previous_key=key
        stage=debug.get('stage');xy=(ego['x'],ego['y'])
        planning_ms=values.get('/planning/diagnostics',{}).get('planning_elapsed_ms')
        if stage in ('AVOID_ENTRY','AVOID_HOLD','RETURN') and planning_ms is not None:planning_times.append(planning_ms)
        if stage=='AVOID_HOLD' and previous_stage=='AVOID_HOLD' and previous_xy is not None:
            lane_distance+=math.dist(previous_xy,xy)
        previous_xy,previous_stage=xy,stage
        token=debug.get('committed_lane_token') or debug.get('selected_lane')
        if token and debug.get('entry_commits',0) and (not lane_tokens or token!=lane_tokens[-1]):lane_tokens.append(token)
        if debug.get('entry_commits',0) and stage in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
            if ego['speed_kmh']>1.:moving=True
            if moving and ego['speed_kmh']<.2:physical_stops+=1;moving=False
        if 'planner_detection' not in events and debug.get('stage')=='DETECT':events['planner_detection']=measure
        if 'lattice_calculation_start' not in events and debug.get('reason') in ('ENTRY_SEARCH_PENDING','DECELERATING_ENTRY_SEARCH'):
            events['lattice_calculation_start']=measure
        avoid=summary.get('s4_parameters',{}).get('avoidance_avoid_speed_kmh',10.)
        target_speed=values.get('/planning/target_speed',0.)
        if ('deceleration_start' not in events and previous_target is not None and
                stage=='DETECT' and ego['speed_kmh']>1. and target_speed<previous_target-.2):
            events['deceleration_start']=dict(measure,previous_target_kmh=previous_target,target_kmh=target_speed,
                measurement='First reduction >0.2 km/h of commanded speed cap while moving in DETECT')
        previous_target=target_speed
        if previous_ego is not None and 'deceleration_start' in events and stage=='DETECT':
            elapsed_source=(ego.get('source_stamp_ns',0)-previous_ego.get('source_stamp_ns',0))/1e9
            if .02<=elapsed_source<=.3:
                acceleration=(ego['speed_kmh']-previous_ego['speed_kmh'])/(3.6*elapsed_source)
                if 'measured_deceleration_start' not in events and acceleration<=-.4 and ego['speed_kmh']>1.:
                    events['measured_deceleration_start']=dict(measure,acceleration_mps2=acceleration,
                        measurement='First EgoStatus source-time speed derivative <= -0.4 m/s^2 after speed-cap reduction')
        previous_ego=ego
        if ('above_s4_deceleration_start' not in events and debug.get('stage')=='DETECT' and
                summary['trial_spec'].get('approach_speed_kmh',10.)>avoid+.2 and debug.get('state')=='DETECT' and
                ego['speed_kmh']>avoid+.2 and values.get('/planning/target_speed',math.inf)<ego['speed_kmh']-.2):
            events['above_s4_deceleration_start']=measure
        if 'lattice_committed' not in events and debug.get('entry_commits',0)>0:events['lattice_committed']=measure
        intentional=debug.get('entry_commits',0)>0 and debug.get('stage') in ('AVOID_ENTRY','AVOID_HOLD','RETURN')
        lateral_since=(row['t'] if lateral_since is None else lateral_since) if intentional and abs(lateral-initial_lateral)>.15 else None
        if ('lateral_motion_start' not in events and lateral_since is not None and row['t']-lateral_since>=.15):
            # Report the beginning of the sustained excursion, not its end.
            events['lateral_motion_start']=next(dict(event_metrics(r['values']['/ego_vehicle_status'],
                r['values'].get('/localization/grid_convergence_deg',convergence),index,target,body),t_s=r['t'])
                for r in rows if r['t']>=lateral_since)
        if 'planner_hold_started' not in events and debug.get('stage') in ('AVOID_HOLD','RETURN'):
            events['planner_hold_started']=measure
        if token and debug.get('selected_mode')=='lane' and token!=lane_id:
            link=native_centerlines().get(token)
            if link is not None:
                lane_id=token;lane_points=link['points'];lane_index=PolylineIndex(lane_points)
        if lane_index is not None and debug.get('entry_commits',0) and 'lane_change_complete' not in events:
            at=lane_index.project(xy);center,lane_heading=sample(lane_points,max(0.,min(lane_index.lengths[-1],at)))
            separation=math.hypot(xy[0]-center[0],xy[1]-center[1])
            angle=math.radians(ego['heading_deg']+convergence)-math.atan2(lane_heading[1],lane_heading[0])
            heading_error=abs(math.degrees(math.atan2(math.sin(angle),math.cos(angle))))
            acquired=(0<=at<=lane_index.lengths[-1] and separation<=.2 and heading_error<=3. and ego['speed_kmh']>1.)
            if acquired and acquired_since is None:
                acquired_since=row['t'];acquired_measure=dict(measure,target_lane_id=lane_id,
                    target_lane_center_distance_m=separation,target_lane_heading_error_deg=heading_error)
            if not acquired:acquired_since=acquired_measure=None
            if acquired_since is not None and row['t']-acquired_since>=.3:events['lane_change_complete']=acquired_measure
        global_angle=math.radians(ego['heading_deg']+convergence)-math.atan2(tangent[1],tangent[0])
        global_heading_error=abs(math.degrees(math.atan2(math.sin(global_angle),math.cos(global_angle))))
        joined=(debug.get('entry_commits',0)>0 and stage=='NORMAL' and abs(lateral)<=.2 and
            global_heading_error<=3. and ego['speed_kmh']>1. and values.get('/planning/s4_sensor_state',{}).get('ready'))
        if joined and rejoin_since is None:
            rejoin_since=row['t'];rejoin_measure=dict(measure,global_cross_track_m=lateral,global_heading_error_deg=global_heading_error)
        if not joined:rejoin_since=rejoin_measure=None
        if rejoin_since is not None and row['t']-rejoin_since>=.3 and 'global_rejoin_observed' not in events:
            events['global_rejoin_observed']=rejoin_measure
        if values.get('/planning/target_speed',0.)>0:movement_started=True
        if movement_started:
            empty=values.get('/planning/local_path',{}).get('points',0)<2
            if empty and not lost:path_losses+=1
            lost=empty
            stale_paths+=row.get('ages_s',{}).get('/planning/local_path',0.)>.6
        if debug.get('stage') in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
            zero=values.get('/planning/target_speed',0.)==0.
            if zero and not stopped:stops+=1
            stopped=zero
        max_error=max(max_error,abs(values.get('/control/path_error',0.)))
        pose=(ego['x'],ego['y'],math.radians(ego['heading_deg']+convergence))
        min_gap=min(min_gap,polygon_distance(swept_body(body,before or pose,pose),target));before=pose
    events['first_lidar_detection']=first_lidar_detection(directory,target,index,body,convergence)
    if events['first_lidar_detection'] and summary.get('drive_started_ros_ns'):
        events['first_lidar_detection']['t_s']=(events['first_lidar_detection']['source_stamp_ns']-summary['drive_started_ros_ns'])/1e9
    safety=audit(directory) if run_safety_audit else json.loads((directory/'live_audit.json').read_text())
    if safety.get('minimum_measured_actor_box_swept_gap_m') is not None:
        min_gap=min(min_gap,safety['minimum_measured_actor_box_swept_gap_m'])
    if safety.get('minimum_measured_all_static_actor_gap_m') is not None:
        min_gap=min(min_gap,safety['minimum_measured_all_static_actor_gap_m'])
    coverage_path=directory/'road_coverage.json'
    coverage=json.loads(coverage_path.read_text()) if coverage_path.exists() else {}
    containment=coverage.get('avoidance_road_containment_proven',False) and coverage.get('normal_unproven_samples',1)==0
    integrity=safety.get('all_recorded_message_integrity',{})
    complete_streams=bool(integrity.get('available') and integrity.get('collision_messages',0)>0 and
        integrity.get('local_path_messages_after_motion_request',0)>0 and
        not any(integrity.get(k,1) for k in ('nonzero_collision_messages','empty_local_path_episodes',
            'sensor_hold_episodes','fresh_validation_wait_episodes','zero_target_episodes_during_avoidance')) and
        integrity.get('maximum_local_path_publication_gap_s',math.inf)<=.6)
    strict=bool(summary.get('success') and summary.get('restored') and min_gap>0 and
        safety.get('road_area_checked_samples',0)>0 and safety.get('measured_ego_box_used') and
        not any((safety['collision_samples'],safety['forbidden_boundary_contact_samples'],
                 safety.get('measured_ego_box_boundary_contacts'),sensor_holds,validation_waits,
                 path_losses,stale_paths,stops,physical_stops)) and containment and complete_streams and
        'lane_change_complete' in events and 'global_rejoin_observed' in events)
    def elapsed(first,last):
        begin,end=events.get(first),events.get(last)
        return end['t_s']-begin['t_s'] if begin and end and 't_s' in begin and 't_s' in end else None
    preparation=summary.get('candidate_preparation_before_spawn',{}).get('values',{}).get('/planning/avoidance_debug',{})
    result=dict(trial=directory.name,verification=summary.get('verification','MORAI_LIVE'),
        success=bool(summary.get('success')),strict_continuous_success=strict,error=summary.get('error'),
        events=events,deceleration_not_observed=('No moving DETECT speed-cap reduction observed; low-speed approach may already satisfy its target'
            if 'deceleration_start' not in events else None),
        lateral_start_measurement='GPS cross-track displacement >0.15 m for >=0.15 s relative to initial global-path offset',
        lane_completion_measurement='Moving EgoStatus within 0.20 m and 3 degrees of the selected MGeo native centerline for >=0.30 s',
        global_rejoin_measurement='Moving NORMAL with ready sensors, GPS global cross-track <=0.20 m and heading error <=3 degrees for >=0.30 s',
        actual_lane_acquisition_observed='lane_change_complete' in events,
        actual_global_rejoin_observed='global_rejoin_observed' in events,
        minimum_measured_body_obstacle_gap_m=min_gap,maximum_lateral_tracking_error_m=max_error,
        sensor_hold_episodes=sensor_holds,fresh_validation_wait_episodes=validation_waits,
        local_path_loss_episodes=path_losses,stale_local_path_samples=stale_paths,
        zero_target_episodes_after_entry=stops,
        physical_stop_episodes_after_entry=physical_stops,
        lidar_detection_to_path_commit_s=elapsed('first_lidar_detection','lattice_committed'),
        planner_detection_to_path_commit_s=elapsed('planner_detection','lattice_committed'),
        lateral_movement_to_lane_change_complete_s=elapsed('lateral_motion_start','lane_change_complete'),
        path_commit_to_lane_change_complete_s=elapsed('lattice_committed','lane_change_complete'),
        selected_lane_maintained_distance_m=lane_distance,selection_tokens=lane_tokens,
        selected_lane_switches_after_commit=max(0,len(lane_tokens)-1),
        prepared_before_obstacle_spawn=bool(preparation.get('lane_candidate_bank_ready') and
            summary.get('candidate_prepared_ros_ns',math.inf)<summary.get('spawn_requested_ros_ns',0)),
        pre_spawn_candidate_count=preparation.get('lane_candidate_count'),
        independent_road_containment_proven=containment,
        all_recorded_messages_passed=complete_streams,
        avoidance_planning_p95_ms=sorted(planning_times)[int(.95*(len(planning_times)-1))] if planning_times else None,
        avoidance_planning_max_ms=max(planning_times,default=None),
        natural_rejoins=max((r['values'].get('/planning/avoidance_debug',{}).get('natural_rejoins',0) for r in rows),default=0),
        return_commits=max((r['values'].get('/planning/avoidance_debug',{}).get('return_commits',0) for r in rows),default=0),
        safety_audit='live_audit.json',actor_truth_use='Post-run association and body/obstacle measurement only; no planner injection')
    (directory/'entry_timing.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args();print(json.dumps(report(args.directory),ensure_ascii=False))
