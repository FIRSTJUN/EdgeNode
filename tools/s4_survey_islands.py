"""Observed closed protection enclosures without any native driving center.

Only negative constraints are inferred from closed, actually surveyed C3
perimeters. No global polygonized face is declared drivable. Positive road
surfaces still require the separate two-sided local survey proof.
"""
from shapely.geometry import Polygon,LineString
from shapely.ops import unary_union,polygonize
from shapely.strtree import STRtree
from edgenode_planning.hdmap_audit import read_layer,translated
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.surveyed_road_artifact import PHYSICAL_LOWER_TYPES


def prepare(boundaries):
    boundaries._survey_boundary_types=PHYSICAL_LOWER_TYPES
    # Discovery ceiling only: every approved edge must still be an actual
    # finite physical survey polyline, with every interior barrier excluded.
    # Wide intersections must not lose their measured far-side curb merely
    # because it is farther away than the old 20m search window.
    boundaries._survey_search_distance_m=100.
    parts=[((f['attributes']['ID'],),p) for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')
        if f['attributes']['Type'] in PHYSICAL_LOWER_TYPES and f['attributes']['LowHigh']=='2'
        for p in translated(f,boundaries.origin) if len(p)>1]
    boundaries._road_protection_parts=parts
    edges=[LineString([p[:2] for p in points]) for _,points in parts]
    centerlines=[LineString([p[:2] for p in link['points']]) for link in
        DijkstraPlanner('/workspace/local_data/kcity_mgeo').links_by_id.values()
        if not link.get('lazy_init') and len(link['points'])>1]
    centerlines.extend(LineString([p[:2] for p in part]) for f in read_layer(boundaries.directory/'A2_LINK')
                       for part in translated(f,boundaries.origin) if len(part)>1)
    tree=STRtree(centerlines);islands=[]
    for face in polygonize(unary_union(edges)):
        if face.area<=1e-8:continue
        possible=tree.query(face)
        if any(face.relate_pattern(line if hasattr(line,'geom_type') else centerlines[int(line)],'T********') for line in possible):continue
        sources=sorted({name for (ids,_),line in zip(parts,edges)
            if line.intersection(face.boundary).length>1e-5 for name in ids})
        if sources:islands.append(dict(geometry=face,source_ids=sources,driving_center_intersections=0))
    boundaries._survey_islands=islands
    boundaries._survey_island_union=unary_union([p['geometry'] for p in islands])
    return boundaries


def records(boundaries):
    return [dict(ring=list(x['geometry'].exterior.coords)[:-1],
        holes=[list(h.coords)[:-1] for h in x['geometry'].interiors],source_ids=x['source_ids'],
        driving_center_intersections=0,classification='CLOSED_SURVEYED_PROTECTION_ENCLOSURE_WITHOUT_A2_OR_NATIVE_MGEO_DRIVING_CENTER')
        for x in boundaries._survey_islands]
