#!/usr/bin/env python3
"""Use original A2 survey selectors with original C3 physical road evidence.

MGeo interpolation agreement is required for lane permissions elsewhere. It
does not determine whether two layers of the same pinned NGII survey agree.
No lane permission, MGeo geometry, source curb, control path or width changes.
"""
import argparse,hashlib,json,multiprocessing,time
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
import yaml
from shapely.geometry import LineString,Polygon
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.hdmap_audit import read_layer,translated
from edgenode_planning.surveyed_road_artifact import file_sha
from s4_partitioned_surveyed_roads import local_patches
from s4_survey_islands import prepare

BOUNDARIES=None


def work(task):
    key,part,registration=task;started=time.monotonic()
    patches=local_patches(BOUNDARIES,part)
    return dict(seed_link=key,reference_source='ORIGINAL_HDMap_A2_NOT_MGEO_INTERPOLATION',
        original_a2_to_native_hausdorff_m=registration,elapsed_s=time.monotonic()-started,
        patches=[dict(ring=p['ring'],holes=p.get('holes',[]),source_ids=p['source_ids'],
            station_range_m=p['station_range_m'],seed_link=key,kind='original_hdmap_a2_selector') for p in patches])


def main():
    global BOUNDARIES
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--workers',type=int,default=2)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('new output required')
    data=json.loads(args.input.read_text());assert data['schema'].endswith('v2') and data['actor_data_used'] is False
    for name,digest in data['map_files_sha256'].items():
        if file_sha(name)!=digest:raise ValueError('source map changed')
    planner=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
    BOUNDARIES=prepare(HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo'))
    mission=yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
    route=LineString(planner.plan_route('A219BS010380',mission['waypoint_node_ids'])['points'])
    tasks=[]
    for feature in read_layer(BOUNDARIES.directory/'A2_LINK'):
        key=feature['attributes']['ID']
        if key in BOUNDARIES.aligned_links:continue
        for part in translated(feature,BOUNDARIES.origin):
            line=LineString(part)
            if line.length<1. or line.distance(route)>15.:continue
            native=planner.links_by_id.get(key)
            registration=line.hausdorff_distance(LineString(native['points'])) if native else None
            tasks.append((key,part,registration))
    print(json.dumps(dict(tasks=len(tasks))),flush=True)
    seen={Polygon(p['ring'],p.get('holes',[])).wkb for p in data['patches']};evidence=[]
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('fork')) as pool:
        for future in as_completed([pool.submit(work,t) for t in tasks]):
            row=future.result();patches=row.pop('patches');added=0
            for patch in patches:
                token=Polygon(patch['ring'],patch.get('holes',[])).wkb
                if token in seen:continue
                seen.add(token);data['patches'].append(patch);added+=1
            row['added']=added;evidence.append(row);print(json.dumps(row),flush=True)
    data['original_a2_supplement']=dict(parent=str(args.input.resolve()),parent_sha256=file_sha(args.input),
        generator_sha256=file_sha(__file__),selector_sha256=file_sha('/workspace/tools/s4_partitioned_surveyed_roads.py'),
        original_mgeo_and_hdmap_changed=False,lane_permission_and_alignment_gates_changed=False,
        source='Original A2/C3 in the same pinned survey frame; every interior physical barrier and exclusion remains checked',
        seeds=evidence)
    args.output.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),sha256=file_sha(args.output),patches=len(data['patches']))),flush=True)


if __name__=='__main__':main()
