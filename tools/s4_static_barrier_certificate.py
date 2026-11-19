#!/usr/bin/env python3
"""Post-run actual vehicle/curb cross-section certificate, never planner input."""
import argparse,hashlib,json,math
from pathlib import Path
from shapely.geometry import LineString,Point,Polygon
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.hdmap_audit import read_layer,translated
from edgenode_planning.lane_geometry import PolylineIndex,sample
from edgenode_planning.surveyed_road_artifact import PHYSICAL_LOWER_TYPES
from s4_live_trial_report import actor_box_polygon,measured_body_for_audit


def points(geometry):
    if geometry.geom_type=='Point':yield geometry
    else:
        for part in getattr(geometry,'geoms',()):yield from points(part)


def certify(directory):
    path=directory/'summary.json';summary=json.loads(path.read_text())
    convergence=summary['preflight']['values']['/localization/grid_convergence_deg']
    actor=Polygon(actor_box_polygon(summary['obstacle_state']['bounding_box'],convergence))
    link=summary['trial_spec']['obstacle_link'];network=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
    index=PolylineIndex(network.links_by_id[link]['points']);station=index.project(actor.centroid.coords[0]);center,tangent=sample(index.points,station,index.lengths)
    normal=(-tangent[1],tangent[0]);radius=100.
    line=LineString([(center[0]+normal[0]*d,center[1]+normal[1]*d) for d in (-radius,radius)])
    scalar=lambda q:(q.x-center[0])*normal[0]+(q.y-center[1])*normal[1]
    boundaries=HDMapBoundaries('/workspace/local_data/kcity_hdmap','/workspace/local_data/kcity_mgeo');hits=[]
    for feature in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY'):
        a=feature['attributes']
        if a['Type'] not in PHYSICAL_LOWER_TYPES or a['LowHigh']!='2':continue
        for part in translated(feature,boundaries.origin):
            edge=LineString([p[:2] for p in part]);intersection=edge.intersection(line)
            hits.extend(dict(id=a['ID'],type=a['Type'],signed_left_m=scalar(q)) for q in points(intersection))
    right=max((h for h in hits if h['signed_left_m']<0),key=lambda x:x['signed_left_m'],default=None)
    left=min((h for h in hits if h['signed_left_m']>0),key=lambda x:x['signed_left_m'],default=None)
    cut=actor.intersection(line);body=measured_body_for_audit(summary,convergence)
    result=dict(source='INDEPENDENT_CLOSED_TRIAL_ACTOR_AND_ORIGINAL_PHYSICAL_C3',
        summary_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),link=link,station_m=station,
        section_center_xy=center[:2],section_normal_xy=normal,physical_right=right,physical_left=left,
        actual_ego_minimum_body_width_m=body.width_m,actor_box=summary['obstacle_state']['bounding_box'],
        certified_blocked=False,scope='This surveyed road cross section; no claim about a different global route')
    if right and left and cut.geom_type=='LineString' and not cut.is_empty:
        values=[scalar(Point(q)) for q in cut.coords];low,high=min(values),max(values)
        free=[max(0.,low-right['signed_left_m']),max(0.,left['signed_left_m']-high)]
        result.update(actor_section_m=[low,high],remaining_right_left_width_m=free,
            certified_blocked=max(free)<body.width_m,
            proof='Every crossing vehicle has a centre chord at least its minimum body width; neither physical-curb-side opening admits that chord')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();out=a.directory/'independent-physical-barrier-certificate.json'
    if out.exists():raise ValueError('new certificate required')
    result=certify(a.directory);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
