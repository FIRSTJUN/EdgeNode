#!/usr/bin/env python3
"""Production PP comparison with a declared, measured-response bicycle plant.

No ROS driving. Tire gain/delay are explicit approximate fits, not MORAI truth.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path

import yaml
from edgenode_control.dynamic_lookahead import path_lookahead_distance
from edgenode_control.pure_pursuit import PurePursuit
from edgenode_control.steering_command import limit_steering_change
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.lane_geometry import sample
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.static_obstacle_planner import bicycle_step
from edgenode_planning.vehicle_geometry import sonata_geometry
from edgenode_planning.speed_planner import SpeedPlanner


def run(points,start,end,speed,minimum,budget,prediction,delay,gain,boundaries,max_steer=.7,physical_profile=False):
    local=LocalPathPlanner();prepared=local.prepare(points);pp=PurePursuit()
    p,tangent=sample(points,start);pose=(*p[:2],math.atan2(tangent[1],tangent[0]))
    command=0.;queue=deque();dt=1/30;progress=None;index=None;rows=[];body=sonata_geometry()
    velocity=speed/3.6;target=speed;previous_mode=None
    speed_planner=SpeedPlanner(cruise_speed_kmh=speed,sharp_curve_speed_kmh=min(speed,30.),
        preview_distance_m=80.,curve_curvature_threshold=.02,sharp_curvature_threshold=.03,
        anticipatory=True,physical_lateral_acceleration_mps2=9.)
    bound_speed=min(speed,15.) if physical_profile else speed
    for tick in range(math.ceil((end-start)/(bound_speed/3.6)*30)+300):
        extracted=local.extract(prepared,*pose[:2],horizon_m=80.,previous_index=index,previous_arc_m=progress)
        index=extracted['nearest_index'];progress=extracted['progress_arc_m'];path=extracted['points']
        if physical_profile and tick%3==0:
            profile=speed_planner.plan(path,previous_mode,velocity*3.6,math.tan(command)/2.84)
            target=profile['target_speed_kmh'];previous_mode=profile['speed_mode']
        current_speed=velocity*3.6
        look=path_lookahead_distance(current_speed,path,pose,minimum,12.,.8,budget)
        measured=pp.compute(path,*pose,lookahead_distance_m=look)
        predicted=bicycle_step(pose,math.tan(command)/2.84,velocity*prediction)
        response=pp.compute(path,*predicted,lookahead_distance_m=look)
        if response is None or measured is None:return dict(success=False,reason='PP_NONE',rows=rows)
        requested=max(-max_steer,min(max_steer,response['steering_normalized']))*.610865
        command=limit_steering_change(requested,command,.6,dt)
        queue.append((tick*dt+delay,command))
        applied=rows[-1]['applied_rad'] if rows else 0.
        while queue and queue[0][0]<=tick*dt+1e-9:applied=queue.popleft()[1]
        curvature=gain*math.tan(applied)/2.84
        contact=boundaries.swept_reason(body.polygon(*pose))
        rows.append(dict(t=tick*dt,pose=pose,station=progress,error_m=measured['nearest_distance_m'],
            command_rad=command,applied_rad=applied,curvature=curvature,lateral_mps2=velocity**2*abs(curvature),
            speed_kmh=current_speed,target_kmh=target,
            lookahead_m=look,boundary=contact))
        if progress>=end:break
        pose=bicycle_step(pose,curvature,velocity*dt)
        if physical_profile:
            desired=target/3.6
            velocity=max(desired,velocity-2.*dt) if velocity>desired else min(desired,velocity+2.*dt)
    contacts=sum(bool(r['boundary']) for r in rows)
    friction=sum(r['lateral_mps2']>9.81 for r in rows)
    maximum=max(r['error_m'] for r in rows)
    return dict(completed=progress>=end,
        success=progress>=end and maximum<=.5 and contacts==0 and friction==0,max_error_m=maximum,
        mean_error_m=sum(r['error_m'] for r in rows)/len(rows),
        max_lateral_mps2=max(r['lateral_mps2'] for r in rows),
        friction_exceeded_samples=friction,boundary_contacts=contacts,rows=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--start',type=float,default=95.)
    parser.add_argument('--end',type=float,default=360.)
    parser.add_argument('--max-steer',type=float,default=.7)
    parser.add_argument('--minimums',type=float,nargs='+',default=[3.5,3.,4.])
    parser.add_argument('--predictions',type=float,nargs='+',default=[0.,.1,.15])
    parser.add_argument('--speeds',type=float,nargs='+',default=[20.,30.])
    parser.add_argument('--physical-profile',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    mapping=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
    saved=yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
    route=mapping.plan_route(saved.get('start_link_id') or 'A219BS010380',saved['waypoint_node_ids'])
    boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    results=[]
    for speed in args.speeds:
        fraction=max(0.,min(1.,(speed-20.)/10.));delay=.08+.04*fraction;gain=.96-.026*fraction
        from itertools import product
        for minimum,prediction in product(args.minimums,args.predictions):
            budget=.25
            r=run(route['points'],args.start,args.end,speed,minimum,budget,prediction,delay,gain,boundaries,args.max_steer,args.physical_profile)
            settings=dict(speed=speed,minimum=minimum,budget=budget,prediction=prediction,
                          plant_delay=delay,plant_gain=gain,max_steer=args.max_steer,physical_profile=args.physical_profile)
            filename=f'{speed:g}-{minimum:g}-{budget:g}-{prediction:g}.json'
            (args.output/filename).write_text(json.dumps(dict(settings=settings,**r),separators=(',',':')))
            compact=dict(settings=settings,**{k:v for k,v in r.items() if k!='rows'})
            results.append(compact);print(json.dumps(compact),flush=True)
    (args.output/'summary.json').write_text(json.dumps(results,indent=2))


if __name__=='__main__':main()
