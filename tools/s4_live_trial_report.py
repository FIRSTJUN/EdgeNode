#!/usr/bin/env python3
"""Audit a completed MORAI trial without opening a live rosbag database."""
import argparse
import csv
import json
import math
import sqlite3
from pathlib import Path

from edgenode_planning.avoidance_geometry import obstacle_polygons, polygon_distance, swept_body, polygon_bounds, bounds_distance
from edgenode_planning.vehicle_geometry import sonata_geometry,VerifiedVehicleGeometry
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.route_resume import mission_suffix
import yaml


def measured_body_for_audit(summary,convergence_deg):
    """Simulator box dimensions are post-run evidence, never planner input."""
    actor=summary.get('ego_before',{});box=actor.get('bounding_box');transform=actor.get('transform')
    if not box or not transform:return None
    yaw=math.radians(transform['rotation']['z']+convergence_deg)
    dx=box['location']['x']-transform['location']['x'];dy=box['location']['y']-transform['location']['y']
    center=math.cos(yaw)*dx+math.sin(yaw)*dy
    lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
    front=center+box['extent']['y'];rear=center-box['extent']['y'];base=sonata_geometry().wheelbase_m
    if front<base or rear>=0:raise ValueError('Actor box/reference axes cannot be verified')
    return VerifiedVehicleGeometry(width_m=2*box['extent']['x'],length_m=front-rear,
        wheelbase_m=base,front_overhang_m=front-base,rear_overhang_m=-rear,
        rear_axle_from_pose_forward_m=0.,rear_axle_from_pose_left_m=lateral,
        pose_reference='Recorded MORAI Ego transform / EgoStatus position',
        source='MORAI Actor box; independent audit only; extent.y longitudinal, extent.x lateral')


def actor_box_polygon(box,convergence):
    yaw=math.radians(box['rotation']['z']+convergence);c,s=math.cos(yaw),math.sin(yaw)
    center=box['location'];extent=box['extent']
    return [(center['x']+c*f-s*l,center['y']+s*f+c*l)
        for f,l in ((-extent['y'],-extent['x']),(extent['y'],-extent['x']),
                    (extent['y'],extent['x']),(-extent['y'],extent['x']))]


def bag_rows(directory):
    """Read only a closed bag; reconstruct independent observation snapshots."""
    if not (directory/'bag'/'metadata.yaml').exists():
        raise ValueError('close rosbag before analysis')
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from morai_ros2_msgs.msg import EgoVehicleStatus,CtrlCmd,CollisionData
    from geometry_msgs.msg import PoseArray
    from nav_msgs.msg import Odometry
    topics=('/ego_vehicle_status','/ctrl_cmd','/collision_data','/control/path_error',
        '/planning/avoidance_debug','/planning/s4_sensor_state','/planning/target_speed',
        '/localization/grid_convergence_deg','/localization/odometry','/planning/obstacles',
        '/planning/current_link','/perception/lidar_diagnostics')
    values={};rows=[];solids=[];start=None
    for database in sorted((directory/'bag').glob('*.db3')):
        with sqlite3.connect('file:'+str(database)+'?mode=ro',uri=True) as connection:
            kinds={i:(name,get_message(kind)) for i,name,kind in connection.execute('select id,name,type from topics') if name in topics}
            placeholders=','.join('?' for _ in kinds)
            for tid,stamp,data in connection.execute(f'select topic_id,timestamp,data from messages where topic_id in ({placeholders}) order by timestamp',tuple(kinds)):
                name,kind=kinds[tid];msg=deserialize_message(data,kind)
                if start is None:start=stamp
                if isinstance(msg,EgoVehicleStatus):
                    value=dict(x=msg.position.x,y=msg.position.y,heading_deg=msg.heading,
                        speed_kmh=math.hypot(msg.velocity.x,msg.velocity.y,msg.velocity.z)*3.6)
                elif isinstance(msg,CtrlCmd):value=dict(accel=msg.accel,brake=msg.brake,steer=msg.front_steer)
                elif isinstance(msg,CollisionData):value=len(msg.collision_object)
                elif isinstance(msg,PoseArray):
                    solids=[[[p.position.x,p.position.y] for p in msg.poses[i:i+8]] for i in range(0,len(msg.poses),8)]
                    continue
                elif isinstance(msg,Odometry):
                    q=msg.pose.pose.orientation;p=msg.pose.pose.position
                    value=[p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)];name='pose'
                else:
                    value=msg.data
                    if isinstance(value,str):
                        try:value=json.loads(value)
                        except ValueError:pass
                values[name]=value
                if name=='/planning/avoidance_debug':rows.append(dict(t=(stamp-start)/1e9,values=dict(values),solids=solids))
    return rows


def bag_integrity(directory,summary):
    """Count every recorded safety/path message in the driven interval."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    start=summary.get('drive_started_ros_ns')
    if start is None:return dict(available=False,reason='drive source time absent')
    samples=[json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines()]
    end=start+int(samples[-1]['t']*1e9)
    stop_ego=summary.get('before_restore',{}).get('values',{}).get('/ego_vehicle_status',{})
    stop_end=max(end,stop_ego.get('source_stamp_ns',end))
    topics=('/collision_data','/planning/local_path','/planning/avoidance_debug','/planning/target_speed')
    collisions=collision_messages=path_messages=empty_episodes=holds=waits=stops=0
    phase=None;previous_state=previous_reason=None;empty=False;zero=False;moving=False
    last_path=None;max_gap=0.
    for database in sorted((directory/'bag').glob('*.db3')):
        with sqlite3.connect('file:'+str(database)+'?mode=ro',uri=True) as connection:
            kinds={i:(name,get_message(kind)) for i,name,kind in connection.execute('select id,name,type from topics') if name in topics}
            marks=','.join('?' for _ in kinds)
            for tid,stamp,data in connection.execute(f'select topic_id,timestamp,data from messages where topic_id in ({marks}) and timestamp between ? and ? order by timestamp',(*kinds,start,stop_end)):
                name,kind=kinds[tid]
                if stamp>end and name!='/collision_data':continue
                message=deserialize_message(data,kind)
                if name=='/collision_data':
                    collision_messages+=1;collisions+=bool(message.collision_object)
                elif name=='/planning/avoidance_debug':
                    debug=json.loads(message.data);state=debug.get('state');reason=debug.get('reason')
                    holds+=state=='SENSOR_HOLD' and previous_state!='SENSOR_HOLD'
                    waits+=reason=='WAIT_FRESH_VALIDATION' and previous_reason!='WAIT_FRESH_VALIDATION'
                    previous_state,previous_reason=state,reason;phase=debug.get('stage')
                elif name=='/planning/target_speed':
                    if message.data>0:moving=True
                    if phase in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
                        current=message.data==0.;stops+=current and not zero;zero=current
                elif name=='/planning/local_path' and moving:
                    path_messages+=1;current=len(message.poses)<2
                    empty_episodes+=current and not empty;empty=current
                    if last_path is not None:max_gap=max(max_gap,(stamp-last_path)/1e9)
                    last_path=stamp
    return dict(available=True,window_start_ns=start,window_end_ns=end,collision_window_through_stop_ns=stop_end,
        collision_messages=collision_messages,nonzero_collision_messages=collisions,
        local_path_messages_after_motion_request=path_messages,empty_local_path_episodes=empty_episodes,
        maximum_local_path_publication_gap_s=max_gap,sensor_hold_episodes=holds,
        fresh_validation_wait_episodes=waits,zero_target_episodes_during_avoidance=stops)


def audit(directory,*,boundaries=None):
    summary=json.loads((directory/'summary.json').read_text())
    rows=[json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines()]
    if rows and 'solids' not in rows[0] and (directory/'bag').exists():rows=bag_rows(directory)
    body=sonata_geometry()
    boundaries=boundaries or HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    area=None
    for row in rows:
        debug=row['values'].get('/planning/avoidance_debug',{})
        start=debug.get('entry_context',{}).get('current_link_id');side=debug.get('selected_side')
        if not (start and side):continue
        routing=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
        saved=yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
        goal=summary.get('trial_spec',{}).get('alternate_goal_node')
        route=(routing.plan_route(start,[goal]) if goal else
               mission_suffix(routing,'A219BS010380',saved['waypoint_node_ids'],start))
        if not route:raise ValueError('recorded alternate route has no directed map connection')
        network=LaneNetwork(routing,morai_unmarked_junction_experiment=debug.get(
            'entry_context',{}).get('morai_unmarked_junction_experiment',False))
        network.boundaries=boundaries
        if (debug.get('selected_mode')=='road_offset' and
                'MORAI_HDMap_SURFACES:' in debug.get('entry_context',{}).get('road_offset_evidence','')):
            candidate=boundaries.road_surface_corridor(network)
            if candidate.source in debug['entry_context']['road_offset_evidence']:area=candidate
        else:
            pair=network.pair(start,side,route['link_ids'])
            if pair is not None:area=boundaries.road_corridor_for_pair(network,pair)
        break
    previous=None;timeline=[];last_phase=None;clearance=math.inf;contacts=[]
    collision_samples=unsafe_sensor_commands=0
    motion=[];stage_speeds={};max_steer=0.;freshness=[];actor_gap=math.inf
    actor_box=summary.get('obstacle_state',{}).get('bounding_box')
    outside_area=[];area_checks=0
    previous_time=None;s4_seconds=stopped_target_seconds=s4_speed_integral=0.
    processing=[];measured_body=None;measured_contacts=[];measured_outside=[];measured_gap=math.inf
    additional_boxes=[entry['state']['bounding_box'] for entry in summary.get('additional_obstacles',[])]
    additional_gap=math.inf
    for row in rows:
        values=row['values'];debug=values.get('/planning/avoidance_debug',{})
        sensor=values.get('/planning/s4_sensor_state',{})
        ego=values.get('/ego_vehicle_status',{});stage=debug.get('stage',debug.get('state'))
        stamp=row.get('t',row.get('elapsed_s'));dt=max(0.,stamp-previous_time) if previous_time is not None else 0.
        previous_time=stamp
        process_ms=values.get('/perception/lidar_diagnostics',{}).get('processing_ms')
        if process_ms is not None:processing.append(process_ms)
        phase=(stage,debug.get('state'),debug.get('reason'))
        if phase!=last_phase:
            timeline.append(dict(t_s=row.get('t',row.get('elapsed_s')),stage=stage,
                state=debug.get('state'),reason=debug.get('reason'),speed_kmh=ego.get('speed_kmh'),
                target_kmh=values.get('/planning/target_speed'),
                link_id=values.get('/planning/current_link'),
                ego_x=ego.get('x'),ego_y=ego.get('y'),selected_side=debug.get('selected_side'),
                detection_available_m=debug.get('distance_context',{}).get('available_distance_m'),
                detection_required_m=debug.get('distance_context',{}).get('required_stopping_m'),
                generated=debug.get('generated_transition_count'),entry=debug.get('entry_commits'),
                returns=debug.get('return_commits')))
            last_phase=phase
        collision_samples+=bool(values.get('/collision_data'))
        unsafe_sensor_commands+=not sensor.get('ready',False) and values.get('/planning/target_speed',0.)>0
        if sensor.get('matched_source_age_sec') is not None:freshness.append(sensor['matched_source_age_sec'])
        max_steer=max(max_steer,abs(values.get('/ctrl_cmd',{}).get('steer',0.)))
        if stage in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
            s4_seconds+=dt;s4_speed_integral+=dt*ego.get('speed_kmh',0.)
            if values.get('/planning/target_speed',0.)==0.:stopped_target_seconds+=dt
            stage_speeds.setdefault(stage,[]).append(ego.get('speed_kmh',0.))
            if ego.get('speed_kmh',0.)>1.:motion.append(ego['speed_kmh'])
        # Ego GPS and its ENU heading are independent of Localization's stopped
        # filter transients. Apply the recorded map convergence, as production.
        if ego and '/localization/grid_convergence_deg' in values:
            pose=(ego['x'],ego['y'],math.radians(ego['heading_deg']+values['/localization/grid_convergence_deg']))
            before=previous or pose
            polygon=swept_body(body,before,pose);previous=pose
            if measured_body is None:measured_body=measured_body_for_audit(summary,values['/localization/grid_convergence_deg'])
            measured_polygon=swept_body(measured_body,before,pose) if measured_body is not None else None
            if measured_polygon is not None:
                for box in additional_boxes:
                    additional_gap=min(additional_gap,polygon_distance(measured_polygon,
                        actor_box_polygon(box,values['/localization/grid_convergence_deg'])))
            if actor_box is not None:
                # Actor API is post-hoc ground truth only. MORAI's extent.y is
                # longitudinal, extent.x lateral; headings are ENU, XY UTM.
                yaw=math.radians(actor_box['rotation']['z']+values['/localization/grid_convergence_deg'])
                c,s=math.cos(yaw),math.sin(yaw);center=actor_box['location'];extent=actor_box['extent']
                target=[(center['x']+c*f-s*l,center['y']+s*f+c*l)
                    for f,l in ((-extent['y'],-extent['x']),(extent['y'],-extent['x']),
                                (extent['y'],extent['x']),(-extent['y'],extent['x']))]
                actor_gap=min(actor_gap,polygon_distance(polygon,target))
                if measured_polygon is not None:measured_gap=min(measured_gap,polygon_distance(measured_polygon,target))
            reason=boundaries.swept_reason(polygon)
            if reason:contacts.append(dict(t_s=row['t'],reason=reason,stage=stage))
            if measured_polygon is not None:
                actual_reason=boundaries.swept_reason(measured_polygon)
                if actual_reason:measured_contacts.append(dict(t_s=row['t'],reason=actual_reason,stage=stage))
            if area is not None and stage in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
                area_checks+=1
                if not area.covers(polygon):outside_area.append(dict(t_s=row['t'],stage=stage))
                if measured_polygon is not None and not area.covers(measured_polygon):
                    measured_outside.append(dict(t_s=row['t'],stage=stage))
            solids=obstacle_polygons(row.get('solids',[]))
            bounds=polygon_bounds(polygon)
            for solid in solids:
                if bounds_distance(bounds,polygon_bounds(solid))<clearance:
                    clearance=min(clearance,polygon_distance(polygon,solid))
    stop_values=summary.get('before_restore',{}).get('values',{})
    stopped_ego=stop_values.get('/ego_vehicle_status',{})
    stop_convergence=stop_values.get('/localization/grid_convergence_deg')
    if stopped_ego and stop_convergence is not None and previous is not None:
        stop_pose=(stopped_ego['x'],stopped_ego['y'],math.radians(stopped_ego['heading_deg']+stop_convergence))
        polygon=swept_body(body,previous,stop_pose)
        measured_polygon=swept_body(measured_body,previous,stop_pose) if measured_body is not None else None
        if measured_polygon is not None:
            for box in additional_boxes:
                additional_gap=min(additional_gap,polygon_distance(measured_polygon,actor_box_polygon(box,stop_convergence)))
        if actor_box is not None:
            actor_gap=min(actor_gap,polygon_distance(polygon,target))
            if measured_polygon is not None:measured_gap=min(measured_gap,polygon_distance(measured_polygon,target))
        if boundaries.swept_reason(polygon):contacts.append(dict(stage='STOPPING_AUDIT',reason=boundaries.swept_reason(polygon)))
        if measured_polygon is not None and boundaries.swept_reason(measured_polygon):
            measured_contacts.append(dict(stage='STOPPING_AUDIT',reason=boundaries.swept_reason(measured_polygon)))
    result=dict(success=bool(summary.get('success')),error=summary.get('error'),
        all_recorded_message_integrity=bag_integrity(directory,summary),
        restored=summary.get('restored'),samples=len(rows),collision_samples=collision_samples,
        forbidden_boundary_contact_samples=len(contacts),contacts=contacts,
        road_area_checked_samples=area_checks,
        road_area_outside_samples=len(outside_area) if area is not None else None,
        road_area_source=area.source if area is not None else None,
        road_area_scope_note='Uncovered cells indicate missing map proof, not a physical departure measurement.',
        road_area_violations=outside_area,
        sensor_unready_positive_target_samples=unsafe_sensor_commands,
        minimum_observed_lidar_swept_gap_m=clearance if math.isfinite(clearance) else None,
        minimum_static_actor_box_swept_gap_m=actor_gap if math.isfinite(actor_gap) else None,
        measured_ego_box_used=measured_body is not None,
        minimum_measured_actor_box_swept_gap_m=measured_gap if math.isfinite(measured_gap) else None,
        minimum_measured_all_static_actor_gap_m=min(measured_gap,additional_gap) if math.isfinite(measured_gap) else None,
        measured_ego_box_boundary_contacts=len(measured_contacts) if measured_body is not None else None,
        measured_ego_box_road_area_outside_samples=len(measured_outside) if area is not None and measured_body is not None else None,
        measured_ego_box_contacts=measured_contacts,measured_ego_box_area_violations=measured_outside,
        maximum_control_error_m=summary.get('max_control_path_error_m',summary.get('max_path_error_m')),
        peak_speed_kmh=summary.get('peak_speed_kmh'),maximum_command_steer=max_steer,
        maximum_matched_source_age_s=max(freshness,default=None),
        s4_duration_s=s4_seconds,s4_zero_target_duration_s=stopped_target_seconds,
        s4_time_weighted_mean_speed_kmh=s4_speed_integral/s4_seconds if s4_seconds else None,
        maximum_perception_processing_ms=max(processing,default=None),
        s4_moving_peak_kmh=max(motion,default=None),
        stage_speed_ranges_kmh={k:[min(v),max(v)] for k,v in stage_speeds.items()},
        timeline=timeline,rosbag_closed=summary.get('rosbag_metadata_present'))
    (directory/'live_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    with (directory/'timeline.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(timeline[0]) if timeline else ['t_s'])
        writer.writeheader();writer.writerows(timeline)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    print(json.dumps(audit(args.directory),ensure_ascii=False))


if __name__=='__main__':main()
