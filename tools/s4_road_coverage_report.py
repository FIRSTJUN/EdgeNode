#!/usr/bin/env python3
"""Independent closed-run road containment audit using measured map boundaries.

This mask is never supplied to a planner. It includes the original A3 surfaces
and ordinary B2 strips with measured end caps. No buffers, convex hull filling,
gap closing, or inferred lane widths are used. An uncovered sweep is unproven,
not automatically a physical road departure.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from edgenode_planning.avoidance_geometry import swept_body
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_audit import marking_definition,read_layer,translated
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.lane_geometry import LaneNetwork,PolylineIndex
from s4_live_trial_report import measured_body_for_audit


class AuditMask:
    def __init__(self,geometry,source):self.geometry,self.source=geometry,source

    def covers(self,polygon):
        from shapely.geometry import Polygon
        query=Polygon(polygon)
        return query.is_valid and self.geometry.covers(query)


def measured_road_mask():
    boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo')
    network=LaneNetwork(DijkstraPlanner('/workspace/local_data/kcity_mgeo'))
    network.boundaries=boundaries
    base=boundaries.road_surface_corridor(network)
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    polygons=[base._exact_geometry];sources=[];skipped=[]
    exclusions=[tuple(p[:2] for p in part) for feature in read_layer(boundaries.directory/'A3_DRIVEWAYSECTION')
        if feature['attributes']['Kind']=='7' for part in translated(feature,boundaries.origin)]
    for key in sorted(network.valid&boundaries.aligned_links):
        if network.links[key].get('link_type') not in ('6',6):continue
        index=PolylineIndex(network.links[key]['points']);sides=[]
        for side in ('right','left'):
            options=[]
            for attributes,parts in boundaries.observed_sides.get((key,side),()):
                if (len(parts)!=1 or attributes['Kind'] not in ('501','503','505','506') or
                    not marking_definition(attributes['Type'],attributes['Kind'])['definition_confirmed']):continue
                line=parts[0]
                if index.project(line[-1])<index.project(line[0]):line=tuple(reversed(line))
                # Select the source whose actual caps match this directed link.
                # This filter never extends, snaps or repairs source geometry.
                mismatch=abs(index.project(line[0]))+abs(index.project(line[-1])-index.lengths[-1])
                options.append((mismatch,attributes['ID'],line))
            if not options:break
            sides.append(min(options,key=lambda row:row[0]))
        if len(sides)!=2 or max(row[0] for row in sides)>2.:continue
        right,left=sides
        # Keep the original ring verbatim. Survey polylines can repeat a
        # consecutive point; GEOS accepts its zero-length edge without changing
        # the polygon. Reject self-intersections; never repair with buffer(0).
        polygon=Polygon([p[:2] for p in list(right[2])+list(reversed(left[2]))])
        if polygon.is_empty or not polygon.is_valid:
            skipped.append(key);continue
        polygons.append(polygon)
        sources.append(dict(link=key,right=right[1],left=left[1],cap_projection_mismatch_m=[right[0],left[0]]))
    digest=hashlib.sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest()
    geometry=unary_union(polygons)
    for excluded in exclusions:
        exclusion=Polygon(excluded)
        if not exclusion.is_valid:raise ValueError('invalid source exclusion ring')
        geometry=geometry.difference(exclusion)
    if not geometry.is_valid:raise ValueError('invalid measured road union')
    mask=AuditMask(geometry,'OFFLINE_MEASURED_B2_CAPS_PLUS_'+base.source+':'+digest)
    return mask,sources,skipped


def report(directory,mask=None):
    if not (directory/'bag/metadata.yaml').exists():raise ValueError('close rosbag before analysis')
    summary=json.loads((directory/'summary.json').read_text())
    rows=[json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines()]
    mask,sources,skipped=mask or measured_road_mask()
    body=None;previous=None;unproven=[];checked=0;normal_checked=normal_unproven=0
    for row in rows:
        values=row['values'];ego=values.get('/ego_vehicle_status',{})
        convergence=values.get('/localization/grid_convergence_deg')
        if not ego or convergence is None:continue
        if body is None:body=measured_body_for_audit(summary,convergence)
        if body is None:raise ValueError('recorded actual Ego body dimensions required')
        pose=(ego['x'],ego['y'],math.radians(ego['heading_deg']+convergence))
        polygon=swept_body(body,previous or pose,pose);previous=pose
        stage=values.get('/planning/avoidance_debug',{}).get('stage')
        if stage not in ('AVOID_ENTRY','AVOID_HOLD','RETURN'):
            normal_checked+=1;normal_unproven+=not mask.covers(polygon);continue
        checked+=1
        if not mask.covers(polygon):unproven.append(dict(t_s=row['t'],stage=stage,
            link=values.get('/planning/current_link'),pose=pose))
    result=dict(trial=directory.name,scope='INDEPENDENT_POST_RUN_MEASURED_EGO_SWEEPS_NO_PLANNER_INPUT',
        road_source=mask.source,avoidance_checked_samples=checked,
        avoidance_unproven_samples=len(unproven),avoidance_road_containment_proven=checked>0 and not unproven,
        normal_checked_samples=normal_checked,normal_unproven_samples=normal_unproven,
        note='Uncovered geometry is missing evidence, not a measured physical departure. No map holes or gaps are filled.',
        added_boundary_sources=sources,skipped_non_simple_source_rings=skipped,unproven=unproven)
    (directory/'road_coverage.json').write_text(json.dumps(result,indent=2)+'\n')
    return {k:v for k,v in result.items() if k not in ('added_boundary_sources','skipped_non_simple_source_rings','unproven')}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directories',nargs='+',type=Path)
    args=parser.parse_args();mask=measured_road_mask()
    for directory in args.directories:print(json.dumps(report(directory,mask)),flush=True)
