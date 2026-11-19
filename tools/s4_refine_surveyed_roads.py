#!/usr/bin/env python3
"""Complete real map continuations and classify locally bounded road cells.

Road cells come from a planar arrangement of actual lower C3 boundaries and
already proved road boundaries. An aligned A2 traversal must enter and leave
through proved road cross sections. The cell must fit within those observed
local entry edges; their convex hull is only a rejection filter, never the
output road geometry. Holes, exclusions, barriers and unbounded land stay out.
No control/global path or simulator actor state is read or changed.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import yaml
from shapely.geometry import Polygon,LineString,Point,MultiPoint
from shapely.ops import unary_union,polygonize,snap
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork,arc
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.hdmap_audit import read_layer,translated
from edgenode_planning.surveyed_road_artifact import file_sha
from edgenode_planning.surveyed_road_patches import local_patches
from edgenode_planning.route_clearance import _native_slice
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope


def pieces(geometry):
    if geometry.geom_type=='LineString':return [geometry]
    return [p for g in getattr(geometry,'geoms',()) for p in pieces(g)]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--numeric-noding',action='store_true',help='Node derived survey caps onto their source lines within 0.1 micrometre; original physical checks stay unchanged')
    parser.add_argument('--coalesce-selector-m',type=float,default=0.,help='Coalesce nearly coincident A2 selector samples only; never change C3 or control path geometry')
    args=parser.parse_args()
    if not 0<=args.coalesce_selector_m<=.01:parser.error('selector coalescing is limited to 1cm')
    stage=args.output.with_name(args.output.stem+'-strips.json')
    if args.output.exists() or stage.exists():raise ValueError('new output required')
    data=json.loads(args.input.read_text());assert data['actor_data_used'] is False
    for name,digest in data['map_files_sha256'].items():
        if file_sha(name)!=digest:raise ValueError('map changed: '+name)
    started=time.monotonic();p=DijkstraPlanner('/workspace/local_data/kcity_mgeo');network=LaneNetwork(p)
    b=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo');network.boundaries=b
    base=b.road_surface_corridor(network)
    mission=yaml.safe_load(Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml').read_text())
    route=p.plan_route('A219BS010380',mission['waypoint_node_ids']);ids=route['link_ids']
    closed=p.links_by_id[ids[-1]]['to_node_idx']==p.links_by_id[ids[0]]['from_node_idx']
    seen={hashlib.sha256(json.dumps(x['ring'],separators=(',',':')).encode()).hexdigest() for x in data['patches']}
    extensions=[]
    for i,key in enumerate(ids):
        points=p.links_by_id[key]['points'];length=arc(points)[-1]
        if length<120. and i!=len(ids)-1:continue
        following=ids[i+1:]+(ids[:i+1] if closed else [])
        reference=[list(x) for x in points];at=key;used=[key];seams=[]
        for nxt in following:
            if arc(reference)[-1]>=length+10.:break
            if p.links_by_id[at]['to_node_idx']!=p.links_by_id[nxt]['from_node_idx']:break
            following_points=p.links_by_id[nxt]['points']
            same=reference[-1][:2]==following_points[0][:2]
            # These are selector samples from two real links sharing a node,
            # not road-area edges. Retain both surveyed samples at a numeric
            # seam. Never join or snap any of the physical C3 boundaries.
            if not same:seams.append(dict(from_link=at,to_link=nxt,
                samples=[reference[-1],following_points[0]],distance_m=math.dist(reference[-1][:2],following_points[0][:2])))
            reference.extend([list(x) for x in following_points[1 if same else 0:]]);used.append(nxt);at=nxt
        if len(used)==1:continue
        reference=list(_native_slice(reference,0.,min(arc(reference)[-1],length+10.)))
        coalesced=[]
        if args.coalesce_selector_m:
            selector=[]
            for point in reference:
                if selector and math.dist(selector[-1][:2],point[:2])<=args.coalesce_selector_m:
                    coalesced.append(point)
                else:selector.append(point)
            reference=selector
        patches=local_patches(b,reference);added=0
        for x in patches:
            token=hashlib.sha256(json.dumps(x['ring'],separators=(',',':')).encode()).hexdigest()
            if token in seen:continue
            seen.add(token);added+=1
            data['patches'].append(dict(ring=x['ring'],holes=x.get('holes',[]),source_ids=x['source_ids'],station_range_m=x['station_range_m'],
                seed_link=key,kind='real_map_continuation',reference_link_ids=used))
        extensions.append(dict(seed=key,reference_link_ids=used,added=added,selector_seams=seams,
            selector_coalescing_m=args.coalesce_selector_m,coalesced_selector_samples=coalesced,
            original_control_path_and_physical_boundaries_changed=False))
        print(json.dumps(extensions[-1]),flush=True)
    data['refinement']=dict(parent=str(args.input.resolve()),parent_sha256=file_sha(args.input),
        execution_argv=sys.argv,real_map_extensions=extensions,global_control_path_changed=False)
    stage.write_text(json.dumps(data,indent=2)+'\n')
    physical_parts=[]
    from edgenode_planning.surveyed_road_artifact import PHYSICAL_LOWER_TYPES
    physical_types=PHYSICAL_LOWER_TYPES if data['schema'].endswith('v2') else frozenset(('2','3','4'))
    for f in read_layer(b.directory/'C3_VEHICLEPROTECTIONSAFETY'):
        a=f['attributes']
        if a['Type'] not in physical_types or a['LowHigh']!='2':continue
        physical_parts.extend((a['ID'],LineString([x[:2] for x in part])) for part in translated(f,b.origin) if len(part)>1)
    physical=unary_union([x for _,x in physical_parts])
    known=unary_union([base._exact_geometry]+[Polygon(x['ring'],x.get('holes',[])) for x in data['patches']])
    if data.get('negative_islands'):
        known=known.difference(unary_union([Polygon(x['ring'],x.get('holes',[])) for x in data['negative_islands']]))
    arrangement_physical=physical;arrangement_known=known.boundary
    if args.numeric_noding:
        # An analytically computed cap point on its source C3 segment can
        # differ from GEOS's segment by one floating-point ulp. Use a separate
        # noded arrangement to identify bounded cells; NEVER replace physical
        # source geometry in barrier, containment or independent body checks.
        # This is 10,000 times smaller than a real millimetre map gap.
        tolerance=1e-7
        arrangement_physical=snap(physical,arrangement_known,tolerance)
        arrangement_known=snap(arrangement_known,arrangement_physical,tolerance)
        error=physical.hausdorff_distance(arrangement_physical)
        if error>tolerance:raise ValueError('auxiliary noding exceeded numeric precision bound')
        data['refinement']['numeric_cap_noding']=dict(tolerance_m=tolerance,
            maximum_auxiliary_physical_hausdorff_m=error,original_physical_geometry_used_for_all_checks=True,
            original_source_maps_changed=False,auxiliary_only=True)
    faces=list(polygonize(unary_union([arrangement_physical,arrangement_known])))
    exclusions=[Polygon([x[:2] for x in part]) for f in read_layer(b.directory/'A3_DRIVEWAYSECTION')
        if f['attributes']['Kind']=='7' for part in translated(f,b.origin)]
    aligned=[(key,LineString(network.links[key]['points']),arc(network.links[key]['points'])[-1])
             for key in sorted(network.valid&b.aligned_links)]
    body=morai_sonata_collision_envelope();accepted=[];rejected=[]
    for face in faces:
        if known.covers(face.representative_point()) or face.area<1e-6:continue
        numerical_band=face.boundary.buffer(1e-7)
        physical_inside=(face.relate_pattern(physical,'T********') and
            face.intersection(physical).difference(numerical_band).length>1e-6)
        if face.interiors or physical_inside or any(face.intersects(e) for e in exclusions):
            rejected.append(dict(area=face.area,bounds=face.bounds,reason='HOLE_BARRIER_OR_EXCLUSION'));continue
        matched=None
        for key,line,length in aligned:
            if not face.intersects(line):continue
            traversals=[]
            for segment in pieces(line.intersection(face)):
                if segment.length<=.1 or not face.contains(segment.interpolate(.5,normalized=True)):continue
                ends=[Point(segment.coords[0]),Point(segment.coords[-1])]
                if all(e.distance(known)<=1e-7 and e.distance(physical)>=body.width_m/2 for e in ends):
                    traversals.append([list(e.coords)[0] for e in ends])
            if not traversals:continue
            contact=face.boundary.intersection(known).difference(physical)
            entry_edges=[c for c in pieces(contact) if c.length>=body.width_m and c.distance(line)<=max(length/2,2*c.length)]
            if len(entry_edges)<2:continue
            hull=MultiPoint([xy for edge in entry_edges for xy in edge.coords]).convex_hull
            outside=face.difference(hull).area
            if outside>1e-6:continue
            sources={name for name,edge in physical_parts if edge.intersection(face.boundary).length>1e-4}
            inherited=[]
            if not sources:
                # An intersection can be enclosed entirely by already proved
                # entrance cross sections. Require its WHOLE perimeter to be
                # such a boundary and retain the physical survey ancestry of
                # each adjoining strip. A2 must still traverse the interior;
                # all barrier/exclusion/locality checks above remain required.
                if face.boundary.difference(known.boundary).length>1e-7:continue
                for patch in data['patches']:
                    if Polygon(patch['ring'],patch.get('holes',[])).boundary.intersection(face.boundary).length>1e-4:
                        sources.update(patch['source_ids']);inherited.append(patch['seed_link'])
            sources=sorted(sources)
            if not sources:continue
            matched=dict(ring=list(face.exterior.coords)[:-1],source_ids=sources,kind='bounded_road_cell',
                seed_link=key,area_m2=face.area,aligned_a2_traversals=traversals,
                observed_entry_edges=[list(edge.coords) for edge in entry_edges],
                outside_observed_entry_hull_area_m2=outside,interior_physical_barrier=False,
                adjoining_survey_strip_seeds=sorted(set(inherited)),
                geometry_source='Exact physical C3 and existing observed road boundaries; no hull filling')
            break
        if matched:accepted.append(matched)
        else:rejected.append(dict(area=face.area,bounds=face.bounds,reason='NO_LOCAL_ALIGNED_A2_ENTRY_EXIT_PROOF'))
    data['patches'].extend(accepted)
    data['refinement'].update(observed_strips_stage=str(stage.resolve()),observed_strips_sha256=file_sha(stage),
        accepted_bounded_cells=len(accepted),rejected_faces=rejected,elapsed_s=time.monotonic()-started)
    args.output.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),sha256=file_sha(args.output),bounded_cells=len(accepted),
        rejected_faces=len(rejected),elapsed_s=time.monotonic()-started)),flush=True)


if __name__=='__main__':main()
