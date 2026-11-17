#!/usr/bin/env python3
"""Audit a completed MORAI trial without opening a live rosbag database."""
import argparse
import csv
import json
import math
import sqlite3
from pathlib import Path

from edgenode_planning.avoidance_geometry import obstacle_polygons, polygon_distance, swept_body
from edgenode_planning.vehicle_geometry import sonata_geometry
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.route_resume import mission_suffix
import yaml


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


def audit(directory):
    summary=json.loads((directory/'summary.json').read_text())
    rows=[json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines()]
    if rows and 'solids' not in rows[0] and (directory/'bag').exists():rows=bag_rows(directory)
    body=sonata_geometry()
    boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    area=None
    for row in rows:
        debug=row['values'].get('/planning/avoidance_debug',{})
        start=debug.get('entry_context',{}).get('current_link_id');side=debug.get('selected_side')
        if not (start and side):continue
        routing=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
        saved=yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
        route=mission_suffix(routing,'A219BS010380',saved['waypoint_node_ids'],start)
        network=LaneNetwork(routing,morai_unmarked_junction_experiment=debug.get(
            'entry_context',{}).get('morai_unmarked_junction_experiment',False))
        network.boundaries=boundaries
        pair=network.pair(start,side,route['link_ids'])
        if pair is not None:area=boundaries.road_corridor_for_pair(network,pair)
        break
    previous=None;timeline=[];last_phase=None;clearance=math.inf;contacts=[]
    collision_samples=unsafe_sensor_commands=0
    motion=[];stage_speeds={};max_steer=0.;freshness=[];actor_gap=math.inf
    actor_box=summary.get('obstacle_state',{}).get('bounding_box')
    outside_area=[];area_checks=0
    previous_time=None;s4_seconds=stopped_target_seconds=s4_speed_integral=0.
    processing=[]
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
            polygon=swept_body(body,previous or pose,pose);previous=pose
            if actor_box is not None:
                # Actor API is post-hoc ground truth only. MORAI's extent.y is
                # longitudinal, extent.x lateral; headings are ENU, XY UTM.
                yaw=math.radians(actor_box['rotation']['z']+values['/localization/grid_convergence_deg'])
                c,s=math.cos(yaw),math.sin(yaw);center=actor_box['location'];extent=actor_box['extent']
                target=[(center['x']+c*f-s*l,center['y']+s*f+c*l)
                    for f,l in ((-extent['y'],-extent['x']),(extent['y'],-extent['x']),
                                (extent['y'],extent['x']),(-extent['y'],extent['x']))]
                actor_gap=min(actor_gap,polygon_distance(polygon,target))
            reason=boundaries.swept_reason(polygon)
            if reason:contacts.append(dict(t_s=row['t'],reason=reason,stage=stage))
            if area is not None and stage in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
                area_checks+=1
                if not area.covers(polygon):outside_area.append(dict(t_s=row['t'],stage=stage))
            solids=obstacle_polygons(row.get('solids',[]))
            if solids:clearance=min(clearance,*(polygon_distance(polygon,solid) for solid in solids))
    result=dict(success=bool(summary.get('success')),error=summary.get('error'),
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
