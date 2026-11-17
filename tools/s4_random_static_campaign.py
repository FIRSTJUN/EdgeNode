#!/usr/bin/env python3
"""Seeded MORAI static placements and a sequential, restoring trial runner.

Actor geometry is used by the existing trial for placement and audit only.
This tool never publishes obstacles, paths, control, or planner approvals.
Generate a manifest first; run the saved manifest without resampling failures.
"""
import argparse
import json
import math
import random
import signal
import subprocess
import sys
from pathlib import Path


def generate(seed,count):
    import yaml
    from edgenode_planning.dijkstra_planner import DijkstraPlanner
    from edgenode_planning.hdmap_boundaries import HDMapBoundaries
    from edgenode_planning.lane_geometry import LaneNetwork,arc,curvature_at
    from edgenode_planning.lane_candidate_bank import build_lane_bank
    root=Path(__file__).resolve().parents[1]
    planner=DijkstraPlanner(root/'local_data/kcity_mgeo')
    network=LaneNetwork(planner)
    network.boundaries=HDMapBoundaries(root/'local_data/kcity_hdmap',root/'local_data/kcity_mgeo')
    waypoints=yaml.safe_load((root/'local_data/kcity_routes/kcity_demo_route.yaml').read_text())['waypoint_node_ids']
    route=planner.plan_route('A219BS010380',waypoints)['link_ids']
    eligible=[]
    for i,key in enumerate(route):
        link=network.links[key];length=arc(link['points'])[-1]
        if route.count(key)!=1 or link.get('link_type') not in ('6',6) or length<65.:continue
        snapshot=build_lane_bank(network,key,tuple(route[i:]))
        options=[o for o in snapshot.options if o.pair is not None and not o.exit_reason]
        if not options:continue
        eligible.append(dict(link=key,length_m=length,candidate_count=len(snapshot.options),
            available_sides=sorted({o.pair.side for o in options}),
            midpoint_curvature=curvature_at(link['points'],length/2)))
    if not eligible:raise ValueError('no eligible saved-route native lane segments')
    rng=random.Random(seed);pool=eligible.copy();rng.shuffle(pool);scenes=[]
    for i in range(count):
        road=pool[i%len(pool)];length=road['length_m']
        station=round(rng.uniform(38.,min(length-12.,110.)),3)
        start=round(max(5.,station-rng.uniform(35.,65.)),3)
        scenes.append(dict(name=f'random_{seed}_{i+1:02d}',kind='RANDOM_NATIVE_ROAD',
            link=road['link'],start=start,obstacle=station,
            obstacle_yaw_deg=round(rng.uniform(-24.,24.),3),
            obstacle_lateral_m=round(rng.uniform(-.45,.45),3),
            obstacle_scale=round(rng.uniform(.7,1.15),3),
            seconds=160.,approach_speed_kmh=10.,road_context=road))
    return dict(seed=seed,sampling='Uniform stations/yaw/lateral/scale; shuffled eligible saved-route links, no success-based resampling',
        eligible_roads=eligible,scenarios=scenes,actor_truth_used_by_planning=False)


def run(manifest,output):
    root=Path(__file__).resolve().parents[1];output.mkdir(exist_ok=False)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    outcomes=[]
    for scene in manifest['scenarios']:
        destination=output/scene['name']
        argv=[sys.executable,'-u',str(root/'tools/s4_morai_generalized_trial.py'),
            '--output',str(destination),'--prepare-before-spawn','--record-bag']
        for name in ('link','start','obstacle','obstacle_link','obstacle_yaw_deg',
                'obstacle_lateral_m','obstacle_scale','obstacle_height_scale','approach_speed_kmh','seconds','minimum_progress_m','follow_distance_m','alternate_goal_node'):
            if name in scene:argv.extend(('--'+name.replace('_','-'),str(scene[name])))
        for value in scene.get('additional_obstacles',[]):argv.extend(('--additional-obstacle',value))
        interrupted=False
        with (output/(scene['name']+'-console.jsonl')).open('w') as stream:
            process=subprocess.Popen(argv,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=process.wait()
            except KeyboardInterrupt:
                interrupted=True
                previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
                try:
                    if process.poll() is None:process.send_signal(signal.SIGINT)
                    # Give the trial's physical stop/restore/bag-close finally
                    # block ownership of cancellation before another scenario.
                    code=process.wait(timeout=60.)
                finally:signal.signal(signal.SIGINT,previous)
        path=destination/'summary.json'
        summary=json.loads(path.read_text()) if path.exists() else {}
        outcome=dict(name=scene['name'],exit_code=code,success=bool(summary.get('success')),
            error=summary.get('error'),restored=summary.get('restored'),
            peak_speed_kmh=summary.get('peak_speed_kmh'),phases=summary.get('phases'))
        outcomes.append(outcome)
        (output/'outcomes.json').write_text(json.dumps(outcomes,indent=2)+'\n')
        print(json.dumps(outcome),flush=True)
        if 'runtime Planning source fingerprint' in (summary.get('error') or ''):
            raise RuntimeError('campaign stopped: Planning code fingerprint must be verified before another trial')
        if not summary.get('restored') or not summary.get('final_stop_confirmed'):
            raise RuntimeError('campaign stopped: scene or stationary Ego restoration unconfirmed')
        if interrupted:return 130
    return 0 if all(x['success'] for x in outcomes) else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    gen=commands.add_parser('generate');gen.add_argument('--seed',type=int,required=True)
    gen.add_argument('--count',type=int,default=8);gen.add_argument('--output',type=Path,required=True)
    execute=commands.add_parser('run');execute.add_argument('manifest',type=Path);execute.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='generate':
        if not 1<=args.count<=100:parser.error('count must be in [1,100]')
        with args.output.open('x') as stream:json.dump(generate(args.seed,args.count),stream,indent=2)
        return 0
    return run(json.loads(args.manifest.read_text()),args.output.resolve())


if __name__=='__main__':raise SystemExit(main())
