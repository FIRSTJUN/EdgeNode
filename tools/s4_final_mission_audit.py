#!/usr/bin/env python3
"""Independent all-Ego-frame audit, retaining raw and directional B2 results.

Read only closed bags. Actor collider dimensions are post-run evidence only.
No recorded contact is discarded: a mixed-line crossing is classified only
after the recorded movement proves the surveyed permitted direction.
"""
import argparse
from bisect import bisect_left
from collections import Counter,defaultdict
import json
import math
import hashlib
from pathlib import Path

from s4_failure_forensics import messages
from s4_live_trial_report import measured_body_for_audit,actor_box_polygon
from s4_road_coverage_report import measured_road_mask
from edgenode_planning.avoidance_geometry import swept_body
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.hdmap_audit import nearest
from edgenode_planning.route_clearance import boundary_contacts
from shapely.geometry import Polygon,LineString,Point
from shapely.ops import unary_union
from edgenode_planning.hdmap_audit import read_layer,translated
from shapely.prepared import prep


def unexpected_empty_stamps(events,zero_targets,window_ns=20_000_000):
    """Associate an empty safety path with the explicit same-cycle STOP.

    DDS ordering across Path, target and FSM topics is not guaranteed. Only
    a recorded zero target within 20ms can resolve an otherwise unexplained
    empty path; a moving target or an older stop cannot excuse disappearance.
    """
    stamps=sorted(zero_targets);unexpected=[]
    for ns,unexplained in events:
        if not unexplained:continue
        i=bisect_left(stamps,ns-window_ns)
        if i==len(stamps) or stamps[i]>ns+window_ns:unexpected.append(ns)
    return unexpected


def audit(directory,resources=None,*,road_override=None,audit_tag='v5'):
    output=directory/('final-mission-audit-'+audit_tag+'.json')
    if output.exists():raise ValueError('audit output already exists: '+str(output))
    summary=json.loads((directory/'summary.json').read_text())
    rows=[json.loads(s) for s in (directory/'samples.jsonl').read_text().splitlines()]
    if not rows or 'drive_started_ros_ns' not in summary:return dict(available=False,reason='no armed trial')
    if resources is None:
        boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
        mask,_,_=measured_road_mask();resources=boundaries,mask
    boundaries,mask=resources
    geometry=mask.geometry;road_source=mask.source
    profile=summary.get('s4_parameters',{})
    recorded_road=dict(file=profile.get('surveyed_road_file'),sha256=profile.get('surveyed_road_sha256'))
    artifact,digest=road_override if road_override is not None else (recorded_road['file'],recorded_road['sha256'])
    if artifact:
        content=Path(artifact).read_bytes()
        if hashlib.sha256(content).hexdigest()!=digest:
            raise ValueError('recorded road artifact has changed')
        proof=json.loads(content)
        for name,digest in proof['map_files_sha256'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:
                raise ValueError('recorded source map changed: '+name)
        # Independent polygon union, without the planner triangulation or its
        # body-containment cache. Keep the exact archived survey rings.
        geometry=unary_union([geometry]+[Polygon(p['ring'],p.get('holes',[])) for p in proof['patches']])
        islands=[Polygon(p['ring'],p.get('holes',[])) for p in proof.get('negative_islands',[])]
        if islands:geometry=geometry.difference(unary_union(islands))
        road_source+=';INDEPENDENT_EXACT_SURVEY_RINGS:'+digest
    road=prep(geometry)
    physical=[];physical_cells=defaultdict(set)
    for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY'):
        a=f['attributes']
        # NGII 2019 p104: type 6 is a median opening, not a physical edge.
        if str(a['Type']) not in ('2','3','4','5','7','8','99') or str(a['LowHigh'])!='2':continue
        for part in translated(f,boundaries.origin):
            if len(part)<2:continue
            edge=LineString([p[:2] for p in part]);i=len(physical);physical.append((a['ID'],edge))
            x0,y0,x1,y1=edge.bounds
            for x in range(math.floor(x0/10),math.floor(x1/10)+1):
                for y in range(math.floor(y0/10),math.floor(y1/10)+1):physical_cells[x,y].add(i)
    start=summary['drive_started_ros_ns'];end=start+round(rows[-1]['t']*1e9)
    end=max(end,summary.get('before_restore',{}).get('values',{}).get('/ego_vehicle_status',{}).get('source_stamp_ns',end))
    convergence=summary.get('preflight',{}).get('values',{}).get('/localization/grid_convergence_deg')
    previous=None;body=None;checked=0;max_speed=0.;max_error=0.;max_steer=0.;speeds=[]
    physical_contacts=Counter()
    final_path=json.loads((directory/'global-path.json').read_text())
    goal_xy=final_path[-1][:2];q=next(p for p in reversed(final_path[:-1]) if math.dist(p[:2],goal_xy)>1e-6)
    goal_yaw=math.atan2(goal_xy[1]-q[1],goal_xy[0]-q[0]);goal_actual=[]
    road_unknown=Counter();raw_contacts=Counter();contact_rows=defaultdict(list)
    collisions=holds=empty_paths=0;fsm='';debug={};sensor_age=[];planning_ms=[];goals=[];hold_events={}
    empty_events=[];zero_targets=[]
    armed_end=summary.get('supervisor_stop_requested_ros_ns',end)
    unproven_rows=[]
    certificates=summary.get('normal_merge_corrections',[])
    for row in rows:
        certificates=certificates or row['values'].get('/planning/diagnostics',{}).get('normal_merge_corrections',[])
    by_id={c['shared_boundary_id']:c for c in certificates}
    source_lines={};attributes={}
    for options in boundaries.observed_sides.values():
        for attr,parts in options:
            if attr['ID'] in by_id and len(parts)==1:
                attributes[attr['ID']]=attr;source_lines[attr['ID']]=parts[0]
    link='';actual=[];minimum_actor_gap=math.inf
    boxes=[s['state']['bounding_box'] for s in summary.get('additional_obstacles',[])]
    if summary.get('obstacle_state',{}).get('bounding_box'):boxes.append(summary['obstacle_state']['bounding_box'])
    topics=('/ego_vehicle_status','/localization/grid_convergence_deg','/collision_data',
        '/planning/s4_sensor_state','/planning/goal_state','/planning/current_link',
        '/planning/fsm_state','/planning/local_path','/planning/avoidance_debug',
        '/planning/diagnostics','/planning/target_speed','/control/path_error','/ctrl_cmd')
    for name,ns,msg in messages(directory,topics,start,end):
        if name=='/localization/grid_convergence_deg':convergence=msg.data;continue
        if name=='/collision_data':collisions+=bool(msg.collision_object);continue
        if name=='/planning/fsm_state':fsm=msg.data;continue
        if name=='/planning/current_link':link=msg.data;continue
        if name=='/planning/target_speed':
            if msg.data==0.:zero_targets.append(ns)
            continue
        if name=='/planning/avoidance_debug':
            debug=json.loads(msg.data);holds+=debug.get('state')=='SENSOR_HOLD';continue
        if name=='/planning/local_path':
            if len(msg.poses)<2:
                empty_paths+=1
                empty_events.append((ns,fsm in ('NORMAL_DRIVE','OBSTACLE_AVOID') and
                    debug.get('state') not in ('VALIDATING','SENSOR_HOLD','BLOCKED')))
            continue
        if name=='/planning/s4_sensor_state':
            value=json.loads(msg.data);age=value.get('matched_source_age_sec')
            if age is not None:sensor_age.append(age)
            event=value.get('last_hold_event')
            if event and start<=event['ros_ns']<=armed_end:hold_events[event['ros_ns']]=event
            continue
        if name=='/planning/diagnostics':
            value=json.loads(msg.data)
            if value.get('planning_elapsed_ms') is not None:planning_ms.append(value['planning_elapsed_ms'])
            continue
        if name=='/planning/goal_state':goals.append(json.loads(msg.data));continue
        if name=='/control/path_error':max_error=max(max_error,abs(msg.data));continue
        if name=='/ctrl_cmd':max_steer=max(max_steer,abs(msg.front_steer));continue
        if convergence is None:continue
        if body is None:body=measured_body_for_audit(summary,convergence)
        if body is None:raise ValueError('recorded collider geometry unavailable')
        speed=math.hypot(msg.velocity.x,msg.velocity.y,msg.velocity.z)*3.6;max_speed=max(max_speed,speed)
        pose=(msg.position.x,msg.position.y,math.radians(msg.heading+convergence))
        if fsm=='GOAL_REACHED':
            goal_actual.append(dict(ns=ns,speed_kmh=speed,position_error_m=math.dist(pose[:2],goal_xy),
                heading_error_rad=abs(math.atan2(math.sin(pose[2]-goal_yaw),math.cos(pose[2]-goal_yaw)))))
        actual.append(pose);speeds.append(speed);polygon=swept_body(body,previous or pose,pose);previous=pose;checked+=1
        shape=Polygon(polygon)
        x0,y0,x1,y1=shape.bounds;indices=set()
        for x in range(math.floor(x0/10),math.floor(x1/10)+1):
            for y in range(math.floor(y0/10),math.floor(y1/10)+1):indices.update(physical_cells[x,y])
        for i in indices:
            identifier,edge=physical[i]
            if shape.intersects(edge):physical_contacts[identifier]+=1
        if not road.covers(shape):
            road_unknown[link]+=1;unproven_rows.append(dict(t_s=(ns-start)/1e9,link=link,pose=pose,
                fsm=fsm,stage=debug.get('stage'),outside_area_m2=shape.difference(geometry).area))
        for box in boxes:minimum_actor_gap=min(minimum_actor_gap,shape.distance(Polygon(actor_box_polygon(box,convergence))))
        for identifier in boundary_contacts(boundaries,polygon):
            raw_contacts[identifier]+=1
            lateral=nearest(source_lines[identifier],pose[:2])[2] if identifier in source_lines else None
            contact_rows[identifier].append(dict(t_s=(ns-start)/1e9,pose=pose,signed_left_m=lateral))
    permitted={};forbidden=dict(raw_contacts)
    for identifier,c in by_id.items():
        attr=attributes.get(identifier,{});samples=contact_rows.get(identifier,[])
        if not samples:continue
        values=[s['signed_left_m'] for s in samples]
        permitted_left=(c['direction']=='left' and attr.get('Type')=='224' and attr.get('Kind')=='506'
            and attr.get('R_linkID')==c['original_link_id'] and attr.get('L_linkID')==c['target_link_id'])
        permitted_right=(c['direction']=='right' and attr.get('Type')=='223' and attr.get('Kind')=='506'
            and attr.get('L_linkID')==c['original_link_id'] and attr.get('R_linkID')==c['target_link_id'])
        signed=[v if permitted_left else -v for v in values]
        if ((permitted_left or permitted_right) and signed[0]<0<signed[-1]
                and min((b-a for a,b in zip(signed,signed[1:])),default=0.)>=-.05):
            permitted[identifier]=dict(samples=len(samples),from_left_m=values[0],to_left_m=values[-1],
                rule='NGII 2019 p88, white mixed divider, movement from dashed side only',
                evidence=attr)
            forbidden.pop(identifier,None)
    percent=lambda values,q:sorted(values)[min(len(values)-1,round((len(values)-1)*q))] if values else None
    from edgenode_planning.dijkstra_planner import DijkstraPlanner
    nodes=DijkstraPlanner('/workspace/local_data/kcity_mgeo').nodes_by_id
    visits=[];cursor=0
    for identifier in summary.get('original_mission',{}).get('route_node_ids',[]):
        xy=nodes[identifier]['point'][:2]
        found=next((i for i in range(cursor,len(actual)) if math.dist(actual[i][:2],xy)<=1.),None)
        visits.append(dict(node=identifier,visited=found is not None,actual_frame=found,tolerance_m=1.))
        if found is not None:cursor=found+1
    unexplained_empty=unexpected_empty_stamps(empty_events,zero_targets)
    result=dict(available=True,trial=directory.name,checked_actual_ego_frames=checked,
        actual_body=body.__dict__ if body else None,peak_speed_kmh=max_speed,max_control_path_error_m=max_error,
        maximum_normalized_steering=max_steer,collision_messages=collisions,sensor_hold_messages=holds,
        armed_sensor_hold_events=list(hold_events.values()),
        empty_path_messages=empty_paths,unexpected_empty_path_messages=len(unexplained_empty),
        unexpected_empty_path_ros_ns=unexplained_empty,
        expected_stop_empty_path_messages=empty_paths-len(unexplained_empty),
        explicit_zero_target_association_window_s=.02,
        raw_boundary_contact_samples=dict(raw_contacts),authorized_directional_crossings=permitted,
        forbidden_boundary_contact_samples=forbidden,physical_c3_contact_samples=dict(physical_contacts),
        ordered_saved_waypoint_visits=visits,mean_observed_speed_kmh=sum(speeds)/len(speeds) if speeds else None,
        road_containment_unproven_frames=sum(road_unknown.values()),road_unproven_by_link=dict(road_unknown),
        road_unknown_is_physical_departure=False,road_source=road_source,
        recorded_planner_road=recorded_road,
        posthoc_road_override=(dict(file=artifact,sha256=digest,note='Revised map processing of unchanged source maps; not the recorded planner input') if road_override else None),
        minimum_actual_actor_clearance_m=None if minimum_actor_gap==math.inf else minimum_actor_gap,
        source_age_max_s=max(sensor_age,default=None),planning_p95_ms=percent(planning_ms,.95),
        planning_max_ms=max(planning_ms,default=None),last_goal_state=goals[-1] if goals else None,
        autonomous_goal=summary.get('autonomous_goal'),restored=summary.get('restored'),
        independent_actual_goal=dict(frames=len(goal_actual),
            hold_s=(goal_actual[-1]['ns']-goal_actual[0]['ns'])/1e9 if goal_actual else 0.,
            max_speed_kmh=max((s['speed_kmh'] for s in goal_actual),default=None),
            max_position_error_m=max((s['position_error_m'] for s in goal_actual),default=None),
            max_heading_error_rad=max((s['heading_error_rad'] for s in goal_actual),default=None)),
        runtime_fingerprint=summary.get('runtime_implementation_sha256'))
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (directory/('all-boundary-contact-frames-'+audit_tag+'.json')).write_text(json.dumps(contact_rows,indent=2)+'\n')
    (directory/('all-road-unproven-frames-'+audit_tag+'.json')).write_text(json.dumps(unproven_rows,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    parser.add_argument('--road-artifact',type=Path);parser.add_argument('--road-sha256');parser.add_argument('--tag',default='v5');args=parser.parse_args()
    if bool(args.road_artifact)!=bool(args.road_sha256):parser.error('an explicit artifact pin is required')
    override=(str(args.road_artifact),args.road_sha256) if args.road_artifact else None
    print(json.dumps(audit(args.directory,road_override=override,audit_tag=args.tag),indent=2))
