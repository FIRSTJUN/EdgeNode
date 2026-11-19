"""Map-proved mandatory merges for tapering native links; original route is kept.

Only a white directional mixed divider can authorize the one-way merge. The
target must be a real same-direction MGeo lane with the identical terminal node
and endpoint. Every new body sweep needs measured road coverage and all other
boundaries. No width, connector, or road surface is inferred.
"""
import hashlib
import json
import math

from .avoidance_geometry import inflate,intersects,polygon_bounds,swept_body
from .lane_geometry import PolylineIndex,arc,sample,curvature_at
from .lattice_path import curvatures,poses,transition


def _native_slice(points,start,end):
    lengths=arc(points)
    result=[tuple(sample(points,start,lengths)[0])]
    result.extend(tuple(p) for p,s in zip(points,lengths) if start<s<end)
    endpoint=tuple(sample(points,end,lengths)[0])
    if result[-1]!=endpoint:result.append(endpoint)
    return tuple(result)


def boundary_contacts(boundaries, polygon):
    lo,hi,bottom,top=polygon_bounds(polygon);indices=set()
    for x in range(math.floor(lo/10),math.floor(hi/10)+1):
        for y in range(math.floor(bottom/10),math.floor(top/10)+1):
            indices.update(boundaries.cells.get((x,y),()))
    return {boundaries.forbidden[i][3] for i in indices
        if intersects(polygon,boundaries.forbidden[i][:2])}


def _permission(boundaries, original, target, side):
    rows=boundaries.observed_sides.get((original,side),())
    permitted=[]
    for attributes,parts in rows:
        if attributes['Kind']!='506' or len(parts)!=1:continue
        if ((attributes['Type']=='224' and side=='left' and
             attributes['R_linkID']==original and attributes['L_linkID']==target) or
            (attributes['Type']=='223' and side=='right' and
             attributes['L_linkID']==original and attributes['R_linkID']==target)):
            permitted.append(attributes)
    return permitted[0] if len(permitted)==1 else None


def correct_route(route,network,body,*,tracking_margin_m=.15,surveyed_centering=False):
    boundaries=network.boundaries
    if boundaries is None:raise ValueError('original boundary evidence required')
    replacements={};certificates=[];road=getattr(network,'road_corridor',None)
    for identifier in route['link_ids']:
        original=network.links[identifier];points=original['points']
        if original.get('link_type') not in ('6',6):continue
        # Repair only real native paths whose full bodies contact restrictions.
        failures=set().union(*(boundary_contacts(boundaries,body.polygon(*pose)) for pose in poses(points)))
        if not failures:continue
        for side in ('left','right'):
            target_id=original.get(side+'_lane_change_dst_link_idx');target=network.links.get(target_id)
            if (target is None or target_id not in boundaries.aligned_links or
                identifier not in boundaries.aligned_links or
                target.get('to_node_idx')!=original.get('to_node_idx') or
                math.dist(points[-1][:2],target['points'][-1][:2])>1e-6):continue
            permission=_permission(boundaries,identifier,target_id,side)
            if permission is None:continue
            if road is None:road=boundaries.road_surface_corridor(network)
            if road is None:raise ValueError('measured road union unavailable')
            total=arc(points)[-1];target_index=PolylineIndex(target['points']);approved=None;rejected={}
            for start in (20.,30.,40.,50.,60.):
                for length in (48.,40.,32.,24.):
                    if start+length>=total-10.:continue
                    source,tangent=sample(points,start);end,_=sample(points,start+length)
                    target_station=target_index.project(end[:2])
                    recovery=transition(source[:2],math.atan2(tangent[1],tangent[0]),
                        curvature_at(points,start),target['points'],target_station,length)
                    candidate=(_native_slice(points,0.,start)+recovery[1:]+
                        _native_slice(target['points'],target_station,target_index.lengths[-1])[1:])
                    if max(map(abs,curvatures(recovery)),default=math.inf)>.04:
                        rejected['CURVATURE']=rejected.get('CURVATURE',0)+1;continue
                    previous=None;reason='';touches=set()
                    for pose in poses(candidate):
                        polygon=inflate(swept_body(body,previous or pose,pose),tracking_margin_m)
                        previous=pose
                        contacts=boundary_contacts(boundaries,polygon);touches.update(contacts)
                        if contacts-{permission['ID']}:reason='OTHER_FORBIDDEN_BOUNDARY';break
                        if not road.covers(polygon):reason='MEASURED_ROAD_CONTAINMENT_UNPROVEN';break
                    if reason:rejected[reason]=rejected.get(reason,0)+1;continue
                    approved=candidate
                    certificates.append(dict(original_link_id=identifier,target_link_id=target_id,
                        direction=side,shared_boundary_id=permission['ID'],type=permission['Type'],kind=permission['Kind'],
                        merge_start_m=start,transition_m=length,tracking_margin_m=tracking_margin_m,
                        raw_boundary_touches=sorted(touches),forbidden_boundary_touches=[],
                        road_source=road.source,endpoint_preserved=True,
                        authority='NGII 2019 map construction manual p88 tables 55-56',
                        manual_sha256='93958d4079dcdde311368753049f0df95238122ab6bde0ea241fed89499a6541'))
                    break
                if approved is not None:break
            if approved is None:
                raise ValueError('DIRECTED_MERGE_UNPROVEN:'+identifier+':'+json.dumps(rejected,sort_keys=True))
            replacements[identifier]=approved
            break
    center_certificates=[];center_rejections=[]
    if surveyed_centering:
        from .surveyed_road import evidence,centered_path
        cache=getattr(network,'_normal_center_cache',None)
        if cache is None:cache={};network._normal_center_cache=cache
        for identifier in dict.fromkeys(route['link_ids']):
            if identifier in replacements:continue
            cache_key=(identifier,body,tracking_margin_m)
            if cache_key not in cache:
                item=evidence(boundaries,network,identifier)
                if item is None:cache[cache_key]=None
                else:
                    try:cache[cache_key]=centered_path(boundaries,network,body,item,tracking_margin_m)
                    except ValueError as error:cache[cache_key]=(None,str(error))
            cached=cache[cache_key]
            if cached is None:continue
            replacement,certificate=cached
            if replacement is None:
                # An optional failed correction leaves the original link and
                # every live safety gate intact; it is not a safety approval.
                center_rejections.append(dict(link=identifier,reason=certificate,original_retained=True))
                continue
            if certificate['max_center_shift_m']<.05:continue
            replacements[identifier]=replacement;center_certificates.append(certificate)
    if not replacements:return dict(route,normal_merge_corrections=[],normal_center_corrections=[],
        normal_center_rejections=center_rejections)
    result=[]
    for identifier in route['link_ids']:
        for point in replacements.get(identifier,network.links[identifier]['points']):
            point=list(point)
            if not result or result[-1]!=point:result.append(point)
    return dict(route,points=result,normal_merge_corrections=certificates,normal_center_corrections=center_certificates,
        normal_center_rejections=center_rejections,
        original_points_sha256=hashlib.sha256(json.dumps(route['points'],separators=(',',':')).encode()).hexdigest())
