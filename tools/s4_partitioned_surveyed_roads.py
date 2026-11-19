"""Exact surveyed strips partitioned at observed source endpoints.

Short real curb transitions are retained. A change of the nearest observed
face ends a strip instead of discarding its entire valid prefix. There is no
cap buffer, endpoint snapping, guessed width, or extrapolation of a source.
"""
import math
from shapely.geometry import Polygon,Point,LineString

from edgenode_planning.hdmap_audit import read_layer,translated,nearest
from edgenode_planning.lane_geometry import PolylineIndex,arc,sample
from edgenode_planning.surveyed_road_patches import _chains
from edgenode_planning.route_clearance import _native_slice
from edgenode_planning.road_surface import triangulate,cells_from_geometry
from edgenode_planning.avoidance_geometry import area


def local_patches(boundaries,reference):
    if len(reference)<2:return []
    if not hasattr(boundaries,'_road_protection_parts'):
        boundaries._road_protection_parts=[((f['attributes']['ID'],),p)
            for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')
            if f['attributes']['Type'] in getattr(boundaries,'_survey_boundary_types',('2','3','4')) and f['attributes']['LowHigh']=='2'
            for p in translated(f,boundaries.origin) if len(p)>1]
    xs=[p[0] for p in reference];ys=[p[1] for p in reference]
    discovery=float(getattr(boundaries,'_survey_search_distance_m',20.))
    bounds=(min(xs)-discovery,max(xs)+discovery,min(ys)-discovery,max(ys)+discovery)
    parts=[]
    for ids,p in boundaries._road_protection_parts:
        px=[q[0] for q in p];py=[q[1] for q in p]
        if max(px)<bounds[0] or min(px)>bounds[1] or max(py)<bounds[2] or min(py)>bounds[3]:continue
        parts.append((ids,p))
    candidates={'left':[],'right':[]};total=arc(reference)[-1]
    for ids,chain in _chains(parts):
        for direction in (chain,tuple(reversed(chain))):
            projection=[nearest(reference,p) for p in direction]
            for side,sign in (('left',1),('right',-1)):
                run=[];stations=[]
                def finish():
                    if len(run)>1 and stations[-1]-stations[0]>1e-7:
                        candidates[side].append(dict(ids=ids,points=tuple(run),index=PolylineIndex(run),
                            low=stations[0],high=stations[-1]))
                for p,q in zip(direction,projection):
                    valid=.5<sign*q[2]<discovery
                    if not valid or stations and q[1]<stations[-1]-.01:
                        finish();run=[];stations=[]
                    if valid:run.append(p);stations.append(q[1])
                finish()
    cuts=sorted({0.,total}|{max(0.,min(total,r[k])) for rows in candidates.values() for r in rows for k in ('low','high')})
    exclusions=[Polygon([p[:2] for p in part]) for f in read_layer(boundaries.directory/'A3_DRIVEWAYSECTION')
        if f['attributes']['Kind']=='7' for part in translated(f,boundaries.origin)]
    result=[];seen=set()
    for low,high in zip(cuts,cuts[1:]):
        if high-low<=1e-7:continue
        midpoint,tangent=sample(reference,(low+high)/2);chosen={};valid=True
        for side in ('left','right'):
            hits=[]
            for candidate in candidates[side]:
                if candidate['low']>low+1e-9 or candidate['high']<high-1e-9:continue
                hit=candidate['index'].normal_intersection(midpoint,tangent,side)
                if hit is not None:hits.append((math.dist(midpoint[:2],hit[1][0][:2]),candidate))
            if not hits:valid=False;break
            chosen[side]=min(hits,key=lambda item:item[0])[1]
        if not valid:continue
        def supported_section(station):
            p,t=sample(reference,station)
            return all(chosen[side]['index'].normal_intersection(p,t,side) is not None for side in ('left','right'))
        middle=(low+high)/2
        # A source endpoint can project onto a different native segment at a
        # corner. Locate the actual supported cap rather than discarding the
        # entire valid interior. Every retained endpoint still intersects BOTH
        # finite surveyed polylines; unsupported intervals remain unapproved.
        if not supported_section(low):
            invalid,valid_cap=low,middle
            for _ in range(40):
                probe=(invalid+valid_cap)/2
                if supported_section(probe):valid_cap=probe
                else:invalid=probe
            low=valid_cap
        if not supported_section(high):
            valid_cap,invalid=middle,high
            for _ in range(40):
                probe=(valid_cap+invalid)/2
                if supported_section(probe):valid_cap=probe
                else:invalid=probe
            high=valid_cap
        if high-low<=1e-7:continue
        caps=[]
        for fraction in (0.,.1,.3,.5,.7,.9,1.):
            station=low+(high-low)*fraction;point,tangent=sample(reference,station);row=[]
            for side in ('left','right'):
                selected=chosen[side];hit=selected['index'].normal_intersection(point,tangent,side)
                if hit is None:valid=False;break
                distance=math.dist(point[:2],hit[1][0][:2])
                for other in candidates[side] if 0.<fraction<1. else ():
                    if other is selected or not other['low']<=station<=other['high']:continue
                    alternative=other['index'].normal_intersection(point,tangent,side)
                    if alternative is not None and math.dist(point[:2],alternative[1][0][:2])<distance-1e-6:
                        valid=False;break
                if not valid:break
                row.append(hit)
            if not valid:break
            caps.append(row)
        if not valid:continue
        a=_native_slice(chosen['left']['points'],caps[0][0][0],caps[-1][0][0])
        b=_native_slice(chosen['right']['points'],caps[0][1][0],caps[-1][1][0])
        ring=tuple(p[:2] for p in a+tuple(reversed(b)));shape=Polygon(ring)
        if not shape.is_valid or shape.is_empty or shape.area<=1e-8:continue
        islands=getattr(boundaries,'_survey_island_union',None)
        if islands is not None:shape=shape.difference(islands)
        if shape.is_empty or not shape.is_valid:continue
        if not all(shape.covers(Point(sample(reference,low+(high-low)*f)[0][:2])) for f in (.01,.25,.5,.75,.99)):continue
        selected_ids=set(chosen['left']['ids']+chosen['right']['ids'])
        if islands is not None:
            band=shape.boundary.buffer(1e-7)
            if any(shape.relate_pattern(LineString(p),'T********') and
                   shape.intersection(LineString(p)).difference(band).length>1e-6 for ids,p in parts):continue
            for island in boundaries._survey_islands:
                if island['geometry'].intersects(Polygon(ring)):selected_ids.update(island['source_ids'])
        elif any(not selected_ids.intersection(ids) and shape.relate_pattern(LineString(p),'T********') for ids,p in parts):continue
        if any(not e.is_valid or shape.intersects(e) for e in exclusions):continue
        shapes=list(shape.geoms) if shape.geom_type=='MultiPolygon' else [shape]
        for component in shapes:
            token=component.wkb
            if token in seen:continue
            seen.add(token)
            try:cells=tuple(c for c in cells_from_geometry(component) if area(c)>1e-8)
            except ValueError:continue
            if cells:result.append(dict(cells=cells,ring=tuple(component.exterior.coords)[:-1],
                holes=[tuple(h.coords)[:-1] for h in component.interiors],source_ids=tuple(sorted(selected_ids)),
                station_range_m=(low,high),source='NGII_EXACT_C3_ENDPOINT_PARTITION_WITH_SURVEYED_ISLAND_EXCLUSIONS'))
    return result
