"""Causal production-PP closed loop on explicit synthetic road ground truth.

No ROS nodes or recorded future poses. Constant-speed/no-slip bicycle is a
kinematic model. Optional braking uses the declared conservative deceleration,
not a claim about production PID, tires, MORAI or physical brake capability.
"""
from dataclasses import replace
import math
from pathlib import Path
import time

import yaml

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_control.steering_command import to_morai_front_steer
from edgenode_planning.avoidance_geometry import polygon_distance,swept_body
from edgenode_planning.lane_geometry import PolylineIndex,sample
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.lattice_path import control_local_path,curvatures
from edgenode_planning.static_obstacle_planner import AvoidanceConfig,StaticObstaclePlanner,bicycle_step

from avoidance_world import world


def production_config():
    """Authoritative controller configuration; no independent PP tuning."""
    filename=Path(__file__).parents[2]/'edgenode_control/config/control.yaml'
    values=yaml.safe_load(filename.read_text())['control_node']['ros__parameters']
    fields=('wheelbase_m','max_wheel_angle_rad','max_front_steer_normalized',
            'steering_sign','control_rate_hz','lookahead_min_m','lookahead_max_m',
            'lookahead_preview_time_sec')
    return replace(AvoidanceConfig(),**{key:values[key] for key in fields})


def run(speed=3.,side='left',radius=math.inf,origin=(0.,0.),yaw=0.,lateral_error=0.,
        heading_error=0.,periods=(.05,),delay_steps=0,brake_to_default=False,
        disappear=False,interrupt_confirmation=False,duration=110.,
        reference_horizon_m=50.,observation_range_m=None):
    lanes,area,point,obstacle=world(side,radius=radius,origin=origin,yaw=yaw)
    config=production_config()
    if not brake_to_default:config=replace(config,avoid_speed_kmh=speed)
    planner=StaticObstaclePlanner(lanes,drivable_area=area,config=config)
    local=LocalPathPlanner();pp=PurePursuit(config.wheelbase_m,config.max_wheel_angle_rad)
    original=lanes.links['O0']['points']+lanes.links['O1']['points'][1:]
    obstacles=[obstacle()];static_obstacles=tuple(tuple(p) for p in obstacle())
    pose=(*point(0.,lateral_error)[:2],yaw+heading_error)
    dt=1/config.control_rate_hz;velocity=speed/3.6;actual_curvature=0.;distance=0.
    phases=[];rows=[];timings=[];commands=[];measurements=[pose]
    active_path=control_local_path(original,pose)
    pending_paths=[]
    command_delay_ticks=math.ceil(config.command_delay_s/dt-1e-12)+delay_steps
    next_plan=0.;plans=0;max_tracking=max_curvature=max_steer=max_heading=0.
    max_planned_curvature=0.
    turn_history=[actual_curvature]
    min_clearance=math.inf;max_clamp=0;entry_objects=set();return_objects=set()
    previous_wheel_angle=0.;max_wheel_rate=0.
    result=None;failure=None;interrupted=False
    for tick in range(math.ceil(duration/dt)):
        now=tick*dt
        measurement=measurements[max(0,len(measurements)-1-delay_steps)]
        if now+1e-10>=next_plan:
            if interrupt_confirmation and not interrupted and planner.confirmations:
                planner.invalidate_observation();interrupted=True
            reference=local.extract(original,*measurement[:2],horizon_m=reference_horizon_m)
            started=time.perf_counter()
            measured_turn=turn_history[max(0,len(turn_history)-1-delay_steps)]
            # Optional optimistic sensing: the full footprint becomes known
            # when its FIRST point enters the forward range. No future poses
            # or early out-of-range obstacle knowledge in this configuration.
            visible=observation_range_m is None or min(
                (x-measurement[0])*math.cos(measurement[2])+(y-measurement[1])*math.sin(measurement[2])
                for x,y in static_obstacles)<=observation_range_m
            observations=obstacles if visible and not (disappear and now>1.) else []
            result=planner.plan(reference['points'],observations,
                measurement,speed_kmh=velocity*3.6,actual_curvature=measured_turn,
                current_link_id='O0' if (planner.pair is None or planner.pair.original_index.project(measurement)<50.) else 'A1',
                route_link_ids=['O0','O1'])
            timings.append((time.perf_counter()-started)*1000);plans+=1
            if not phases or phases[-1]!=result['state']:phases.append(result['state'])
            if planner.selected:
                if id(planner.selected.entry_path) not in entry_objects:
                    max_planned_curvature=max(max_planned_curvature,max(map(abs,curvatures(planner.selected.points))))
                entry_objects.add(id(planner.selected.entry_path))
                if planner.state=='RETURN':return_objects.add(id(planner.selected.return_path))
            next_plan+=periods[(plans-1)%len(periods)]
            if result['blocked']:
                failure=dict(reason=result['reason'],pose=pose,t=now,
                    candidates=[dict(length_m=c.transition_length_m,reason=c.rejection) for c in planner.candidates]);break
            if phases[-1]=='NORMAL' and 'RETURN' in phases:
                break
            # This is the same progress-only window sent by PlanningNode.
            # Future body poses still come solely from the applied PP command.
            output=control_local_path(result['points'],measurement) if result['active'] else result['points']
            pending_paths.append((now+delay_steps*dt,output))
        while pending_paths and pending_paths[0][0]<=now+1e-12:
            active_path=pending_paths.pop(0)[1]
        if not active_path:continue
        lookahead=calculate_lookahead_distance(velocity*3.6,config.lookahead_min_m,
            config.lookahead_max_m,config.lookahead_preview_time_sec)
        control=pp.compute(active_path,*measurement,lookahead_distance_m=lookahead)
        if control is None:
            failure=dict(reason='CONTROL_NONE',pose=pose,t=now);break
        requested=control['steering_normalized'];max_clamp+=int(abs(requested)>config.max_front_steer_normalized)
        command=to_morai_front_steer(requested,config.steering_sign,config.max_front_steer_normalized)
        commands.append(command)
        applied=commands[-1-command_delay_ticks] if len(commands)>command_delay_ticks else 0.
        wheel_angle=applied/config.steering_sign*config.max_wheel_angle_rad
        wheel_rate=abs(wheel_angle-previous_wheel_angle)/dt
        previous_wheel_angle=wheel_angle;max_wheel_rate=max(max_wheel_rate,wheel_rate)
        actual_curvature=math.tan(wheel_angle)/config.wheelbase_m
        next_pose=bicycle_step(pose,actual_curvature,velocity*dt)
        swept=swept_body(planner.body,pose,next_pose)
        inside=area.covers(swept)
        clearance=polygon_distance(swept,static_obstacles)
        min_clearance=min(min_clearance,clearance)
        index=PolylineIndex(active_path);s=index.project(pose[:2]);reference=sample(active_path,s)
        if reference:
            max_tracking=max(max_tracking,math.dist(pose[:2],reference[0][:2]))
            target_yaw=math.atan2(reference[1][1],reference[1][0])
            max_heading=max(max_heading,abs(math.atan2(math.sin(pose[2]-target_yaw),math.cos(pose[2]-target_yaw))))
        max_curvature=max(max_curvature,abs(actual_curvature));max_steer=max(max_steer,abs(applied))
        rows.append(dict(t=now,pose=pose,next_pose=next_pose,state=result['state'],
            speed_kmh=velocity*3.6,curvature=actual_curvature,steering=applied,
            inside=inside,body_clearance_m=clearance,wheel_rate_radps=wheel_rate))
        if not inside or clearance<=config.obstacle_margin_m:
            failure=dict(reason='ACTUAL_DRIVABLE_AREA' if not inside else 'ACTUAL_BODY_COLLISION',pose=next_pose,t=now+dt);break
        pose=next_pose;measurements.append(pose);turn_history.append(actual_curvature);distance+=velocity*dt
        if brake_to_default and result.get('speed_cap_kmh') is not None:
            target=result['speed_cap_kmh']/3.6
            velocity=max(target,velocity-config.deceleration_mps2*dt) if velocity>target else velocity
    else:failure=dict(reason='TIMEOUT',pose=pose,t=duration)
    ordered=sorted(timings);rank=.95*(len(ordered)-1)
    fraction=rank-math.floor(rank)
    p95=(1-fraction)*ordered[math.floor(rank)]+fraction*ordered[math.ceil(rank)]
    return dict(success=failure is None and phases[-1]=='NORMAL' and 'RETURN' in phases,
        phases=phases,failure=failure,time_s=rows[-1]['t']+dt if rows else 0.,distance_m=distance,
        max_tracking_m=max_tracking,max_heading_rad=max_heading,max_curvature_m_inv=max_curvature,
        max_planned_curvature_m_inv=max_planned_curvature,
        max_steering=max_steer,max_wheel_rate_radps=max_wheel_rate,
        clamp_count=max_clamp,min_body_clearance_m=min_clearance,
        entry_commits=planner.entry_commits,return_commits=planner.return_commits,
        generated_transition_count=planner.generated_transition_count,
        candidate_lane_batches=planner.candidate_lane_batches,
        entry_path_objects=len(entry_objects),return_path_objects=len(return_objects),
        confirmation_interrupted=interrupted,rows=rows,
        controller_config=dict(wheelbase_m=config.wheelbase_m,max_wheel_angle_rad=config.max_wheel_angle_rad,
            max_front_steer_normalized=config.max_front_steer_normalized,steering_sign=config.steering_sign,
            control_rate_hz=config.control_rate_hz,command_delay_s=config.command_delay_s,
            additional_delay_steps=delay_steps,planning_periods_s=periods,
            reference_horizon_m=reference_horizon_m,observation_range_m=observation_range_m),
        timing=dict(mean_ms=sum(timings)/len(timings),p95_ms=p95,worst_ms=max(timings),cycles=len(timings)))
