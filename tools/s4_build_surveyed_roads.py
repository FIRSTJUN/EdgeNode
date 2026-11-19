#!/usr/bin/env python3
"""Prepare missing local road evidence offline from unchanged NGII/MGeo data."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import hashlib
import json
import multiprocessing
from pathlib import Path
import time

import yaml
from shapely.geometry import LineString
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork,arc,sample
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.surveyed_road_patches import local_patches
from edgenode_planning.surveyed_road_artifact import file_sha
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope
from edgenode_planning.avoidance_geometry import inflate


BOUNDARIES=None


def work(task):
    key,ids,reference=task;started=time.monotonic()
    try:
        patches=local_patches(BOUNDARIES,reference)
        return dict(seed_link=key,reference_link_ids=ids,patches=[
            dict(ring=p['ring'],holes=p.get('holes',[]),source_ids=p['source_ids'],station_range_m=p['station_range_m']) for p in patches],
            elapsed_s=time.monotonic()-started)
    except Exception as error:return dict(seed_link=key,error=type(error).__name__+': '+str(error),elapsed_s=time.monotonic()-started)


def main():
    global BOUNDARIES
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=2);parser.add_argument('--near-route-m',type=float,default=15.)
    args=parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.progress.jsonl').exists():raise ValueError('new artifact output required')
    root=Path('/workspace');mgeo=root/'local_data/kcity_mgeo';hdmap=root/'local_data/kcity_hdmap'
    saved=root/'local_data/kcity_routes/kcity_demo_route.yaml';mission=yaml.safe_load(saved.read_text())
    planner=DijkstraPlanner(mgeo);network=LaneNetwork(planner);BOUNDARIES=HDMapBoundaries(hdmap,mgeo)
    network.boundaries=BOUNDARIES;base=BOUNDARIES.road_surface_corridor(network)
    route=planner.plan_route('A219BS010380',mission['waypoint_node_ids']);route_line=LineString(route['points'])
    body=morai_sonata_collision_envelope();tasks=[];skipped=[]
    for key in sorted(network.valid&BOUNDARIES.aligned_links):
        points=network.links[key]['points'];lengths=arc(points)
        if lengths[-1]<8. or LineString(points).distance(route_line)>args.near_route_m:continue
        if all(base.covers(inflate(body.polygon(*p[:2],__import__('math').atan2(t[1],t[0])),.3))
               for f in (.02,.25,.5,.75,.98) for p,t in [sample(points,lengths[-1]*f,lengths)]):
            skipped.append(dict(link=key,reason='EXISTING_BODY_ROAD_PROOF_AT_SCREENING_STATIONS'));continue
        ids=[key];reference=[list(p) for p in points]
        if route['link_ids'].count(key)==1:
            following=route['link_ids'][route['link_ids'].index(key)+1:]
        else:
            following=[];at=key;seen={key}
            while arc(reference+[])[-1]<120. and len(following)<8:
                outgoing=planner.graph.get(planner.links_by_id[at]['to_node_idx'],[])
                if len(outgoing)!=1 or outgoing[0]['link_id'] in seen:break
                at=outgoing[0]['link_id'];seen.add(at);following.append(at)
                # Length is checked below after constructing the full window.
                if sum(planner._link_costs[k] for k in [key]+following)>=120.:break
        for next_id in following:
            if arc(reference)[-1]>=120.:break
            previous=planner.links_by_id[ids[-1]];nxt=planner.links_by_id[next_id]
            if previous['to_node_idx']!=nxt['from_node_idx'] or reference[-1][:2]!=nxt['points'][0][:2]:break
            reference.extend([list(p) for p in nxt['points'][1:]]);ids.append(next_id)
        tasks.append((key,ids,reference))
    names=[mgeo/'global_info.json',mgeo/'link_set.json',mgeo/'node_set.json',saved]
    names.extend(hdmap/(stem+suffix) for stem in ('A2_LINK','A3_DRIVEWAYSECTION','B2_SURFACELINEMARK','C3_VEHICLEPROTECTIONSAFETY')
                 for suffix in ('.shp','.dbf','.prj'))
    data=dict(schema='edgenode.surveyed-local-road.v1',actor_data_used=False,
        source='Original NGII lower C3 Type2/3/4 exact local strips; all exclusions retained; no inferred road width',
        manual_url='https://www.ngii.go.kr/other/file_down.do?sq=76349',
        manual_sha256='93958d4079dcdde311368753049f0df95238122ab6bde0ea241fed89499a6541',
        map_files_sha256={str(p):file_sha(p) for p in names},patches=[],seeds=[],skipped=skipped,
        generation=dict(workers=args.workers,near_route_m=args.near_route_m,tasks=len(tasks)))
    print(json.dumps(dict(tasks=len(tasks),already_proved=len(skipped))),flush=True)
    seen=set();started=time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('fork')) as pool:
        futures=[pool.submit(work,t) for t in tasks]
        for future in as_completed(futures):
            result=future.result();patches=result.pop('patches',[]);result['patch_count']=len(patches);data['seeds'].append(result)
            for p in patches:
                token=hashlib.sha256(json.dumps(p['ring'],separators=(',',':')).encode()).hexdigest()
                if token not in seen:seen.add(token);data['patches'].append(dict(p,seed_link=result['seed_link']))
            with args.output.with_suffix('.progress.jsonl').open('a') as log:log.write(json.dumps(result)+'\n')
            print(json.dumps(dict(done=len(data['seeds']),tasks=len(tasks),patches=len(data['patches']),**result)),flush=True)
    data['generation']['elapsed_s']=time.monotonic()-started
    data['seeds'].sort(key=lambda r:r['seed_link']);data['patches'].sort(key=lambda p:(p['seed_link'],p['station_range_m']))
    args.output.write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(dict(path=str(args.output),sha256=file_sha(args.output),patches=len(data['patches']))),flush=True)


if __name__=='__main__':main()
