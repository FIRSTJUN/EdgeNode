#!/usr/bin/env python3
"""Read-only K-City delivery audit. Diagnostic evidence only; no road cells.

Only existing stdlib, numpy, pyproj, matplotlib and workspace modules are used.
ZIP members remain in memory. No extraction, runtime map edits or approvals.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import posixpath
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from pyproj import CRS, Transformer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/edgenode_planning'))
sys.path.insert(0, str(ROOT/'ros2_ws/src/edgenode_control'))
from edgenode_planning.hdmap_audit import read_layer, marking_definition, nearest  # noqa: E402
from edgenode_planning.lane_geometry import PolylineIndex, arc, sample  # noqa: E402
from edgenode_planning.lattice_path import curvatures  # noqa: E402
from edgenode_planning.static_obstacle_planner import AvoidanceConfig  # noqa: E402
from edgenode_planning.vehicle_geometry import sonata_geometry  # noqa: E402


def digest(data):
    return hashlib.sha256(data).hexdigest()


def member_name(info):
    name = info.filename
    if not info.flag_bits & 0x800:
        try:
            name = name.encode('cp437').decode('cp949')
        except UnicodeError:
            pass
    path = PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts or '\\' in name:
        raise ValueError('unsafe archive member name')
    return str(path)


def read_archive(path):
    members, manifest = {}, []
    with zipfile.ZipFile(path, 'r') as archive:
        for info in archive.infolist():
            name = member_name(info)
            if name in members:
                raise ValueError('duplicate archive member: '+name)
            if info.is_dir():
                continue
            data = archive.read(info)  # zipfile checks each member's CRC.
            members[name] = data
            manifest.append(dict(path=name, bytes=len(data), crc32=f'{info.CRC:08x}',
                                 sha256=digest(data)))
    return members, manifest


def read_dbf(raw):
    if len(raw) < 33:
        raise ValueError('truncated DBF header')
    count = struct.unpack_from('<I', raw, 4)[0]
    header, size = struct.unpack_from('<HH', raw, 8)
    if header < 33 or (header-33) % 32 or header+count*size > len(raw):
        raise ValueError('invalid DBF layout')
    fields = []
    for position in range(32, header-1, 32):
        data = raw[position:position+32]
        fields.append(dict(name=data[:11].split(b'\0')[0].decode('ascii'),
                           type=chr(data[11]), width=data[16], decimals=data[17]))
    if raw[header-1] != 13 or sum(f['width'] for f in fields)+1 != size:
        raise ValueError('invalid DBF fields')
    rows = []
    for i in range(count):
        row = raw[header+i*size:header+(i+1)*size]
        if row[0] != 32:
            raise ValueError('deleted/invalid DBF record needs explicit review')
        values = {}; offset = 1
        for field in fields:
            width = field['width']
            values[field['name']] = row[offset:offset+width].strip().decode('cp949')
            offset += width
        rows.append(values)
    return fields, rows


def read_shapes(raw, index, *, forensic_header=False):
    for number,data in enumerate((raw, index)):
        if (len(data) < 100 or struct.unpack_from('>i', data, 0)[0] != 9994 or
                (struct.unpack_from('>i', data, 24)[0]*2 != len(data) and not (forensic_header and number==0)) or
                struct.unpack_from('<i', data, 28)[0] != 1000):
            raise ValueError('invalid SHP/SHX header')
    kind = struct.unpack_from('<i', raw, 32)[0]
    if kind not in (11, 13, 15) or struct.unpack_from('<i', index, 32)[0] != kind:
        raise ValueError('unsupported/mismatched SHP/SHX type')
    if (len(index)-100) % 8:
        raise ValueError('truncated SHX index')
    rows = []; position = 100
    while position < len(raw) and len(rows)<(len(index)-100)//8:
        if position+12 > len(raw):
            raise ValueError('truncated SHP record')
        number, words = struct.unpack_from('>ii', raw, position)
        end = position+8+words*2
        body = raw[position+8:end]
        if words < 2 or end > len(raw) or number != len(rows)+1:
            raise ValueError('invalid SHP record order/length')
        ix = 100+8*len(rows)
        if ix+8 > len(index) or struct.unpack_from('>ii', index, ix) != (position//2, words):
            raise ValueError('SHP/SHX offset or length mismatch')
        if struct.unpack_from('<i', body, 0)[0] != kind:
            raise ValueError('null/mixed geometry requires explicit review')
        if kind == 11:
            if len(body) < 28:
                raise ValueError('truncated PointZ')
            parts = [(struct.unpack_from('<3d', body, 4),)]
        else:
            if len(body) < 44:
                raise ValueError('truncated multipart header')
            nparts, npoints = struct.unpack_from('<ii', body, 36)
            xy_start = 44+4*nparts; z_start = xy_start+16*npoints+16
            if nparts < 1 or npoints < 2 or z_start+8*npoints > len(body):
                raise ValueError('invalid multipart coordinate range')
            starts = list(struct.unpack_from('<'+'i'*nparts, body, 44))+[npoints]
            if starts[0] != 0 or any(b <= a for a, b in zip(starts, starts[1:])):
                raise ValueError('invalid multipart indices')
            points = [(*struct.unpack_from('<2d', body, xy_start+16*i),
                       struct.unpack_from('<d', body, z_start+8*i)[0]) for i in range(npoints)]
            parts = [tuple(points[a:b]) for a, b in zip(starts, starts[1:])]
        if not all(math.isfinite(v) for part in parts for point in part for v in point):
            raise ValueError('nonfinite shape coordinate')
        rows.append(dict(shape_type=kind, parts=parts)); position = end
    if len(index) != 100+8*len(rows):
        raise ValueError('unused SHX records')
    if position!=len(raw) and not forensic_header:
        raise ValueError('unindexed trailing SHP bytes')
    return rows


def load_layers(members):
    layers = {}
    for path in sorted(members):
        if not path.lower().endswith('.shp'):
            continue
        stem = path[:-4]
        fields, attrs = read_dbf(members[stem+'.dbf'])
        issues=[]
        try:
            shapes = read_shapes(members[path], members[stem+'.shx'])
        except ValueError:
            # Forensic comparison only: some delivered orthometric SHPs have a
            # stale *file header* size. Preserve strict failure and still check
            # every real record against its SHX/DBF, without changing bytes.
            raw=members[path]
            if len(raw)>=100 and struct.unpack_from('>i',raw,24)[0]*2!=len(raw):
                shapes=read_shapes(raw,members[stem+'.shx'],forensic_header=True)
                issues.append(dict(code='SHP_HEADER_LENGTH_MISMATCH',
                    declared_bytes=struct.unpack_from('>i',raw,24)[0]*2,actual_bytes=len(raw),
                    forensic_records_still_shx_dbf_checked=True))
                last_offset,last_words=struct.unpack_from('>ii',members[stem+'.shx'],len(members[stem+'.shx'])-8)
                used=last_offset*2+8+last_words*2
                issues[-1]['unindexed_trailing_bytes']=len(raw)-used
                issues[-1]['unindexed_trailing_sha256']=digest(raw[used:])
            else:
                raise
        if len(attrs) != len(shapes):
            raise ValueError('SHP/DBF record count mismatch: '+path)
        for shape, attr in zip(shapes, attrs):
            shape['attributes'] = attr
        wkt = members.get(stem+'.prj', b'').decode('utf-8-sig')
        layers[stem] = dict(path=stem, fields=fields, features=shapes, wkt=wkt,source_format_issues=issues)
    return layers


def layer_summary(layer):
    features = layer['features']
    points = np.asarray([p for f in features for part in f['parts'] for p in part])
    fields = []
    for field in layer['fields']:
        counts = Counter(f['attributes'][field['name']] for f in features)
        fields.append(dict(field, nonempty=sum(n for v, n in counts.items() if v),
            unique=len(counts), values=dict(counts) if len(counts) <= 40 else None))
    parts = [p for f in features for p in f['parts']]
    crs = CRS.from_wkt(layer['wkt']) if layer['wkt'] else None
    return dict(path=layer['path'], count=len(features), fields=fields,
        strict_shape_header_valid=not layer['source_format_issues'],source_format_issues=layer['source_format_issues'],
        shape_type=features[0]['shape_type'] if features else None,
        part_count=len(parts), multipart_features=sum(len(f['parts'])>1 for f in features),
        closed_parts=sum(len(p)>2 and p[0]==p[-1] for p in parts),
        degenerate_parts=sum(len(p)<(1 if features[0]['shape_type']==11 else 2) for p in parts),
        vertices=len(points), minimum_xyz=points.min(axis=0).tolist(), maximum_xyz=points.max(axis=0).tolist(),
        line_length_m=sum(arc(p)[-1] for p in parts),
        signed_ring_area_m2=sum(.5*sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(p,p[1:]))
                                for p in parts) if features[0]['shape_type']==15 else None,
        crs_present=bool(crs), horizontal_authority=crs.to_authority() if crs else None,
        crs_wkt=layer['wkt'], vertical_crs_declared=crs.is_vertical or crs.is_compound if crs else False)


def read_workbook(raw):
    ns = {'m':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
    rid = '{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id'
    with zipfile.ZipFile(io.BytesIO(raw)) as book:
        shared = []
        if 'xl/sharedStrings.xml' in book.namelist():
            shared = [''.join(t.text or '' for t in si.findall('.//m:t',ns))
                      for si in ET.fromstring(book.read('xl/sharedStrings.xml'))]
        relationships = {r.attrib['Id']:r.attrib['Target'] for r in
                         ET.fromstring(book.read('xl/_rels/workbook.xml.rels'))}
        result = []
        for sheet in ET.fromstring(book.read('xl/workbook.xml')).findall('m:sheets/m:sheet', ns):
            target = relationships[sheet.attrib[rid]]
            path = target.lstrip('/') if target.startswith('/') else posixpath.normpath('xl/'+target)
            cells = []
            for cell in ET.fromstring(book.read(path)).findall('.//m:sheetData/m:row/m:c', ns):
                value = cell.find('m:v', ns)
                text = value.text or '' if value is not None else ''.join(
                    t.text or '' for t in cell.findall('.//m:t', ns))
                if cell.attrib.get('t') == 's':
                    text = shared[int(text)]
                formula = cell.find('m:f',ns)
                if text or formula is not None:
                    cells.append(dict(address=cell.attrib['r'], value=text,
                        formula=formula.text if formula is not None else None))
            result.append(dict(name=sheet.attrib['name'],state=sheet.attrib.get('state','visible'),
                               source=path,cells=cells))
        return result


def coordinate_comparison(reference, variants):
    rows = []
    for group, layers in variants.items():
        for name, layer in layers.items():
            base = reference[name]
            index = {f['attributes']['ID']:f for f in base['features']}
            other = {f['attributes']['ID']:f for f in layer['features']}
            common = sorted(set(index)&set(other))
            transform = Transformer.from_crs(CRS.from_wkt(layer['wkt']),
                                             CRS.from_wkt(base['wkt']), always_xy=True)
            xymax = zmax = 0.; dz = []; total = 0; mismatched = []; layout_diagnostics=[]
            for key in common:
                a, b = index[key],other[key]
                if [len(p) for p in a['parts']] != [len(p) for p in b['parts']]:
                    mismatched.append(key)
                    details=dict(id=key,reference_part_vertices=[len(p) for p in a['parts']],
                                 variant_part_vertices=[len(p) for p in b['parts']],
                                 vertex_to_segment_xy_max_m=None,vertex_to_segment_z_max_m=None)
                    if len(a['parts'])==len(b['parts']):
                        errors=[]
                        for pa,pb in zip(a['parts'],b['parts']):
                            bb=np.asarray(pb);x,y=transform.transform(bb[:,0],bb[:,1])
                            transformed=np.column_stack((x,y,bb[:,2])).tolist()
                            errors.extend((alignment(list(pa),transformed),alignment(transformed,list(pa))))
                        details.update(vertex_to_segment_xy_max_m=max(e[0] for e in errors),
                                       vertex_to_segment_z_max_m=max(e[1] for e in errors))
                    layout_diagnostics.append(details);continue
                for pa,pb in zip(a['parts'],b['parts']):
                    aa,bb=np.asarray(pa),np.asarray(pb)
                    x,y=transform.transform(bb[:,0],bb[:,1])
                    xymax=max(xymax,float(np.hypot(aa[:,0]-x,aa[:,1]-y).max()))
                    delta=aa[:,2]-bb[:,2];zmax=max(zmax,float(np.abs(delta).max()))
                    dz.extend(delta.tolist());total+=len(pa)
            rows.append(dict(group=group,layer=name,reference_count=len(index),count=len(other),
                identical_ids=set(index)==set(other),identical_attributes=all(index[k]['attributes']==other[k]['attributes'] for k in common),
                missing_ids=sorted(set(index)-set(other)),extra_ids=sorted(set(other)-set(index)),
                part_layout_mismatch_ids=mismatched,compared_vertices=total,
                part_layout_mismatch_diagnostics=layout_diagnostics,
                transformed_xy_max_m=xymax,z_abs_difference_max_m=zmax,
                reference_minus_variant_z_min_m=min(dz,default=None),
                reference_minus_variant_z_max_m=max(dz,default=None),
                reference_minus_variant_z_mean_m=float(np.mean(dz)) if dz else None))
    return rows


def write_json(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader()
        for row in rows:
            writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict,tuple)) else v for k,v in row.items()})


def reference_audit(layers):
    indices={name:{f['attributes']['ID']:f for f in layer['features']} for name,layer in layers.items()}
    specs=[('A2_LINK','FromNodeID','A1_NODE'),('A2_LINK','ToNodeID','A1_NODE'),
        ('A2_LINK','L_LinkID','A2_LINK'),('A2_LINK','R_LinkID','A2_LINK'),
        ('A2_LINK','SectionID','A3_DRIVEWAYSECTION'),
        ('B2_SURFACELINEMARK','L_linkID','A2_LINK'),('B2_SURFACELINEMARK','R_linkID','A2_LINK'),
        ('B1_SAFETYSIGN','LinkID','A2_LINK'),('B1_SAFETYSIGN','PostID','C6_POSTPOINT'),
        ('B3_SURFACEMARK','LinkID','A2_LINK'),('C1_TRAFFICLIGHT','LinkID','A2_LINK'),
        ('C1_TRAFFICLIGHT','postID','C6_POSTPOINT'),('C4_SPEEDBUMP','LinkID','A2_LINK'),
        ('C3_VEHICLEPROTECTIONSAFETY','Ref_ID','C3_VEHICLEPROTECTIONSAFETY')]
    rows=[]
    for source,field,target in specs:
        present=[(key,f['attributes'][field]) for key,f in indices[source].items() if f['attributes'][field]]
        missing=[dict(source_id=key,target_id=value) for key,value in present if value not in indices[target]]
        rows.append(dict(layer=source,field=field,target_layer=target,nonempty=len(present),
                         empty=len(indices[source])-len(present),resolved=len(present)-len(missing),unresolved=missing))
    nodes=indices['A1_NODE'];node_gaps=[]
    for key,f in indices['A2_LINK'].items():
        for field,point in [('FromNodeID',f['parts'][0][0]),('ToNodeID',f['parts'][-1][-1])]:
            target=f['attributes'][field]
            if target in nodes:
                delta=math.dist(point[:2],nodes[target]['parts'][0][0][:2])
                if delta>0:node_gaps.append(dict(link=key,node=target,end=field,xy_gap_m=delta))
    return dict(references=rows,node_geometry_nonzero_gaps=node_gaps,
                raw_permission_fields=[name for layer in layers.values() for name in
                    [f['name'] for f in layer['fields']] if any(word in name.lower() for word in ('permit','can_move','change'))])


def adjacent_chains(link_features):
    """Maximal unbranched chains of explicitly reciprocal ordinary-lane pairs.

    Exact actual endpoints are required. No node-nearness joining, gap filling,
    intersection traversal or lane-width inference is used.
    """
    links={f['attributes']['ID']:f for f in link_features}
    outgoing=defaultdict(list)
    for key,f in links.items():
        outgoing[f['attributes']['FromNodeID']].append(key)
    states=set();rejected=Counter()
    for key,f in links.items():
        if f['attributes']['LinkType']!='6' or len(f['parts'])!=1:
            continue
        for side,field,back in [('left','L_LinkID','R_LinkID'),('right','R_LinkID','L_LinkID')]:
            other=f['attributes'][field]
            if not other or other not in links:
                continue
            b=links[other]
            if b['attributes']['LinkType']!='6' or len(b['parts'])!=1 or b['attributes'][back]!=key:
                rejected['nonordinary_nonreciprocal_or_multipart']+=1;continue
            a_points,b_points=f['parts'][0],b['parts'][0]
            mid=sample(a_points,arc(a_points)[-1]/2)
            other_mid=sample(b_points,nearest(b_points,mid[0])[1])
            if sum(x*y for x,y in zip(mid[1],other_mid[1]))<=0:
                rejected['opposed_direction']+=1;continue
            lateral=mid[1][0]*(other_mid[0][1]-mid[0][1])-mid[1][1]*(other_mid[0][0]-mid[0][0])
            if lateral*(1 if side=='left' else -1)<=0:
                rejected['wrong_geometric_side']+=1;continue
            states.add((key,other,side))
    graph={s:[] for s in states};parents={s:[] for s in states};gaps=[]
    for state in sorted(states):
        a,b,side=state
        for nxt_a in outgoing[links[a]['attributes']['ToNodeID']]:
            for nxt_b in outgoing[links[b]['attributes']['ToNodeID']]:
                target=(nxt_a,nxt_b,side)
                if target not in states:
                    continue
                delta_a=math.dist(links[a]['parts'][0][-1],links[nxt_a]['parts'][0][0])
                delta_b=math.dist(links[b]['parts'][0][-1],links[nxt_b]['parts'][0][0])
                if delta_a or delta_b:
                    gaps.append(dict(from_pair=state,to_pair=target,gap_a_m=delta_a,gap_b_m=delta_b));continue
                graph[state].append(target);parents[target].append(state)
    chains=[];used=set()
    def walk(start):
        chain=[start];used.add(start)
        while len(graph[chain[-1]])==1:
            target=graph[chain[-1]][0]
            if target in chain or len(parents[target])!=1:
                break
            chain.append(target);used.add(target)
        return chain
    starts=[s for s in sorted(states) if len(parents[s])!=1 or len(graph[parents[s][0]])!=1]
    for state in starts:
        if state not in used:
            chains.append(walk(state))
    for state in sorted(states-used):
        if state not in used:
            chains.append(walk(state))
    unique={}
    for chain in chains:
        key=tuple(tuple(sorted((a,b))) for a,b,_ in chain)
        unique.setdefault(key,chain)
    return list(unique.values()),dict(directed_reciprocal_ordinary_pairs=len(states),
        undirected_pair_sections=len(states)//2,chains=len(unique),rejected=dict(rejected),
        endpoint_gap_transitions=gaps,join_tolerance_m=0.,branch_traversal=False)


def concatenate(ids, links, origin):
    result=[]
    for key in ids:
        points=[tuple(v-o for v,o in zip(p,origin)) for p in links[key]['parts'][0]]
        if result and result[-1]!=points[0]:
            raise ValueError('cannot concatenate a real geometry gap')
        result.extend(points if not result else points[1:])
    return result


def sampled_spans(stations, flags):
    spans=[];start=end=None
    for station,valid in zip(stations,flags):
        if valid:
            if start is None:start=station
            end=station
        elif start is not None:
            spans.append([start,end]);start=end=None
    if start is not None:spans.append([start,end])
    return spans


def scan_candidates(layers, mgeo, origin):
    links={f['attributes']['ID']:f for f in layers['A2_LINK']['features']}
    marks=layers['B2_SURFACELINEMARK']['features']
    chains,topology=adjacent_chains(list(links.values()))
    lookup={s:defaultdict(list) for s in ('left','right')};shared=defaultdict(list)
    for f in marks:
        attr=f['attributes'];a,b=attr['L_linkID'],attr['R_linkID']
        if b:lookup['left'][b].append(f)
        if a:lookup['right'][a].append(f)
        if a and b:shared[frozenset((a,b))].append(f)
    config=AvoidanceConfig();body=sonata_geometry();velocity=config.avoid_speed_kmh/3.6
    reserve=2*velocity*config.rollout_seconds+config.lookahead_min_m+2*config.tracking_limit_m
    entry_end=config.lookahead_min_m+velocity*config.command_delay_s+20.
    return_start=45.+4.+body.rear_overhang_m+config.return_margin_m+config.lookahead_min_m
    return_end=return_start+20.
    forward_required=return_end+reserve+body.wheelbase_m+body.front_overhang_m+1.
    required=body.rear_overhang_m+forward_required
    assumptions=dict(speed_kmh=3.,obstacle_front_m=45.,obstacle_extent_m=4.,transition_m=20.,
        entry_end_m=entry_end,return_start_m=return_start,return_end_m=return_end,
        rollout_reserve_m=reserve,terminal_body_and_margin_m=body.wheelbase_m+body.front_overhang_m+1.,
        initial_rear_support_m=body.rear_overhang_m,required_forward_support_m=forward_required,
        required_actual_map_support_m=required,gentle_curvature_threshold_per_m=1/250.,station_step_m=1.,approval=False)
    candidates=[];profiles={}
    from bisect import bisect_right
    for chain in chains:
        ids_a=[a for a,b,s in chain];ids_b=[b for a,b,s in chain];side=chain[0][2]
        center=concatenate(ids_a,links,origin);neighbor=concatenate(ids_b,links,origin)
        lengths=arc(center);length=lengths[-1];index=PolylineIndex(center)
        ends=[];total=0.
        for key in ids_a:
            total+=arc(links[key]['parts'][0])[-1];ends.append(total)
        stations=sorted(set(lengths+[float(s) for s in range(math.ceil(length))]+[length]))
        mark_indices={}
        neighbor_indices={b:PolylineIndex([tuple(v-o for v,o in zip(p,origin)) for p in links[b]['parts'][0]])
                          for _,b,_ in chain}
        relevant={}
        for a,b,_ in chain:
            left_owner=b if side=='left' else a;right_owner=a if side=='left' else b
            relevant[(a,b)]={'left':lookup['left'][left_owner],
                'right':lookup['right'][right_owner],'shared':shared[frozenset((a,b))]}
            for features in relevant[(a,b)].values():
                for f in features:
                    key=f['attributes']['ID']
                    if key in mark_indices:continue
                    parts=[]
                    for part in f['parts']:
                        local=[tuple(v-o for v,o in zip(p,origin)) for p in part]
                        if index.project(local[-1])<index.project(local[0]):local.reverse()
                        parts.append(PolylineIndex(local))
                    mark_indices[key]=parts
        observations=[]
        for station in stations:
            section=min(len(chain)-1,bisect_right(ends,station));a,b,_=chain[section]
            point,tangent=sample(center,station,lengths)
            evidence={}
            for label,features in relevant[(a,b)].items():
                direction=side if label=='shared' else label
                hits=[]
                for f in features:
                    for idx in mark_indices[f['attributes']['ID']]:
                        hit=idx.normal_intersection(point,tangent,direction)
                        if hit is not None:
                            xy=tuple(hit[1][0][:2]);distance=math.dist(point[:2],xy)
                            hits.append(dict(id=f['attributes']['ID'],xy=xy,distance_m=distance,
                                type=f['attributes']['Type'],kind=f['attributes']['Kind']))
                positions={tuple(round(v,7) for v in hit['xy']) for hit in hits}
                evidence[label]=dict(unambiguous=len(positions)==1,hits=hits)
            marking_hits=all(evidence[k]['unambiguous'] for k in ('left','right','shared'))
            adjacent_hit=neighbor_indices[b].normal_intersection(point,tangent,side)
            ordered=False
            if marking_hits and adjacent_hit is not None:
                left=evidence['left']['hits'][0]['distance_m'];right=-evidence['right']['hits'][0]['distance_m']
                sign=1 if side=='left' else -1
                divider=sign*evidence['shared']['hits'][0]['distance_m']
                adjacent=sign*math.dist(point[:2],adjacent_hit[1][0][:2])
                ordered=(right<min(0.,adjacent) and max(0.,adjacent)<left and
                         min(0.,adjacent)<divider<max(0.,adjacent))
            supported=bool(marking_hits and ordered)
            dashed=supported and all(hit['type']=='212' and hit['kind']=='503' for hit in evidence['shared']['hits'])
            width=(math.dist(evidence['left']['hits'][0]['xy'],evidence['right']['hits'][0]['xy']) if supported else None)
            pa=sample(center,max(0.,station-.5),lengths)[0]
            pc=sample(center,min(length,station+.5),lengths)[0]
            denominator=math.dist(pa[:2],point[:2])*math.dist(point[:2],pc[:2])*math.dist(pa[:2],pc[:2])
            kappa=(2*((point[0]-pa[0])*(pc[1]-point[1])-(point[1]-pa[1])*(pc[0]-point[0]))/denominator
                   if denominator>1e-12 else None)
            observations.append(dict(s_m=station,x=point[0],y=point[1],original=a,adjacent=b,
                supported=supported,dashed_marking_candidate=dashed,width_m=width,
                geometric_marking_order_valid=ordered,adjacent_center_normal_present=adjacent_hit is not None,
                diagnostic_curvature_1m_per_m=kappa,
                gentle_dashed_support=bool(dashed and kappa is not None and abs(kappa)<=1/250.),
                left_ids=sorted({h['id'] for h in evidence['left']['hits']}),
                shared_ids=sorted({h['id'] for h in evidence['shared']['hits']}),
                right_ids=sorted({h['id'] for h in evidence['right']['hits']})))
        spans=sampled_spans(stations,[r['supported'] for r in observations])
        max_span=max((b-a for a,b in spans),default=0.)
        gentle_spans=sampled_spans(stations,[r['gentle_dashed_support'] for r in observations])
        gentle_max=max((b-a for a,b in gentle_spans),default=0.)
        ratio=sum(r['supported'] for r in observations)/len(observations)
        dashed_ratio=sum(r['dashed_marking_candidate'] for r in observations)/len(observations)
        all_ids=set(ids_a+ids_b);present=all_ids&set(mgeo);mgeo_ratio=len(present)/len(all_ids)
        curvature_issue=''
        try:
            ks=list(map(abs,curvatures(center)))
        except ValueError as error:
            ks=[];curvature_issue=str(error)
        p95=float(np.percentile(ks,95)) if ks else None
        score=30*min(length/required,1)+20*ratio+25*min(gentle_max/required,1)+20*mgeo_ratio+5*dashed_ratio
        if curvature_issue:score-=35
        any_left=any(rows['left'] for rows in relevant.values());any_right=any(rows['right'] for rows in relevant.values())
        classification='B' if any_left and any_right and not curvature_issue else 'C'
        name='PAIR_'+ids_a[0]+'_'+ids_b[0]+'_'+digest(json.dumps(chain).encode())[:8]
        reasons=['NO_VERIFIED_DRIVABLE_SURFACE_OR_HOLE_MASK','NO_EXPLICIT_OUTWARD_AND_RETURN_PERMISSION',
                 'NO_DELIVERY_COMPLETENESS_OR_MORAI_SCENE_ATTESTATION']
        if max_span<required:reasons.append('INSUFFICIENT_CONTIGUOUS_SAMPLED_BOUNDARY_SUPPORT')
        if length<required:reasons.append('INSUFFICIENT_PAIRED_LONGITUDINAL_SUPPORT')
        if mgeo_ratio<1:reasons.append('INCOMPLETE_CURRENT_MGEO_COVERAGE')
        if curvature_issue:reasons.append('DEGENERATE_NATIVE_CENTER_GEOMETRY')
        def linked_records(layer,fields):
            return [{key:f['attributes'][key] for key in fields}
                    for f in layers.get(layer,{}).get('features',[]) if f['attributes']['LinkID'] in all_ids]
        signs=linked_records('B1_SAFETYSIGN',('ID','Type','SubType','LinkID','Ref_Lane'))
        signals=linked_records('C1_TRAFFICLIGHT',('ID','Type','LinkID','Ref_Lane'))
        bumps=linked_records('C4_SPEEDBUMP',('ID','Type','LinkID','Ref_Lane'))
        if signs or signals:reasons.append('RELATED_SIGN_OR_SIGNAL_REQUIRES_SECTION_SPECIFIC_REVIEW')
        row=dict(candidate_id=name,original_ids=ids_a,adjacent_ids=ids_b,side=side,
            paired_section_count=len(chain),original_length_m=length,adjacent_length_m=arc(neighbor)[-1],
            required_support_m=required,length_sufficient=min(length,arc(neighbor)[-1])>=required,
            sampled_support_ratio=ratio,sampled_support_spans_m=spans,max_sampled_support_span_m=max_span,
            gentle_dashed_sampled_spans_m=gentle_spans,max_gentle_dashed_sampled_span_m=gentle_max,
            sampled_dashed_ratio=dashed_ratio,curvature_p95_per_m=p95,max_native_curvature_per_m=max(ks,default=None),
            native_curvature_issue=curvature_issue,
            mgeo_coverage_ratio=mgeo_ratio,mgeo_present_ids=sorted(present),mgeo_missing_ids=sorted(all_ids-set(mgeo)),
            shared_marking_ids=sorted({f['attributes']['ID'] for pair in relevant.values() for f in pair['shared']}),
            observed_outer_left_ids=sorted({f['attributes']['ID'] for pair in relevant.values() for f in pair['left']}),
            observed_outer_right_ids=sorted({f['attributes']['ID'] for pair in relevant.values() for f in pair['right']}),
            section_ids=sorted({links[k]['attributes'].get('SectionID','') for k in all_ids}- {''}),
            linked_safety_signs=signs,linked_traffic_lights=signals,linked_speed_bumps=bumps,
            classification=classification,diagnostic_score=score,approved_for_runtime=False,reasons=reasons)
        candidates.append(row);profiles[name]=dict(samples=observations,center=center,adjacent=neighbor)
    candidates.sort(key=lambda c:(-c['diagnostic_score'],-c['original_length_m'],c['candidate_id']))
    chosen=[];taken=set()
    for candidate in candidates:
        ids=set(candidate['original_ids']+candidate['adjacent_ids'])
        if ids&taken:continue
        chosen.append(candidate);taken.update(ids)
        if len(chosen)==3:break
    return candidates,chosen,profiles,topology,assumptions


def alignment(points, reference):
    """Vertex-to-existing-segment diagnostics, never a continuous error bound."""
    a=np.asarray(reference[:-1],dtype=float);b=np.asarray(reference[1:],dtype=float)
    delta=b-a;length2=(delta[:,:2]**2).sum(axis=1)
    valid=length2>1e-20;a=a[valid];delta=delta[valid];length2=length2[valid]
    xy=z=0.
    for start in range(0,len(points),128):
        p=np.asarray(points[start:start+128],dtype=float)
        u=np.clip(((p[:,None,:2]-a[None,:,:2])*delta[None,:,:2]).sum(axis=2)/length2[None,:],0.,1.)
        projected=a[None,:,:]+u[:,:,None]*delta[None,:,:]
        distances=((p[:,None,:2]-projected[:,:,:2])**2).sum(axis=2)
        chosen=distances.argmin(axis=1);order=np.arange(len(p))
        xy=max(xy,float(np.sqrt(distances[order,chosen]).max()))
        z=max(z,float(np.abs(p[:,2]-projected[order,chosen,2]).max()))
    return xy,z


def mgeo_comparison(layers, mgeo, origin):
    links={f['attributes']['ID']:f for f in layers['A2_LINK']['features']}
    ordinary={k:v for k,v in mgeo.items() if not v.get('lazy_init') and not v.get('lane_ch_link_path')}
    shared=sorted(set(ordinary)&set(links));rows=[]
    marks=layers['B2_SURFACELINEMARK']['features']
    for key in shared:
        raw=links[key];attr=raw['attributes']
        points=[tuple(v-o for v,o in zip(p,origin)) for p in raw['parts'][0]]
        forward=alignment(ordinary[key]['points'],points);back=alignment(points,ordinary[key]['points'])
        rows.append(dict(id=key,xy_vertex_segment_max_m=max(forward[0],back[0]),z_vertex_segment_max_m=max(forward[1],back[1]),
            link_type_matches=str(ordinary[key].get('link_type'))==attr['LinkType'],
            node_ids_match=(ordinary[key]['from_node_idx']==attr['FromNodeID'] and ordinary[key]['to_node_idx']==attr['ToNodeID']),
            left_adjacency_matches=(ordinary[key].get('left_lane_change_dst_link_idx') or '')==attr['L_LinkID'],
            right_adjacency_matches=(ordinary[key].get('right_lane_change_dst_link_idx') or '')==attr['R_LinkID']))
    both=sum(any(f['attributes']['R_linkID']==k for f in marks) and
             any(f['attributes']['L_linkID']==k for f in marks) for k in shared)
    return dict(total_mgeo=len(mgeo),ordinary_mgeo=len(ordinary),lazy_or_lane_change=len(mgeo)-len(ordinary),
        shp_links=len(links),common_ordinary_ids=len(shared),both_side_marking_associations=both,
        missing_ordinary_ids=sorted(set(ordinary)-set(links)),shp_ids_not_in_ordinary_mgeo=sorted(set(links)-set(ordinary)),
        mgeo_explicit_permission_field_count=sum(any(k in l for k in ('can_move_left_lane','can_move_right_lane')) for l in mgeo.values()),
        center_comparisons=rows,xy_vertex_segment_max_m=max(r['xy_vertex_segment_max_m'] for r in rows),
        z_vertex_segment_max_m=max(r['z_vertex_segment_max_m'] for r in rows))


def control_point_audit(layers):
    rows=[];sets={};checks=[]
    for stem,layer in layers.items():
        if '/ControlPoint/' not in stem:continue
        attrs=layer['features'];numeric_errors=[];deltas=[]
        key='GCP_ID' if 'GCP_ID' in attrs[0]['attributes'] else 'MMP_ID'
        for f in attrs:
            a=f['attributes']
            try:
                numeric=[float(a[k]) for k in ('X','Y','Z')];shape=list(f['parts'][0][0])
                difference=[abs(x-y) for x,y in zip(numeric,shape)];deltas.append(difference)
                checks.append(dict(layer=stem,id=a[key],attribute_xyz=numeric,shape_xyz=shape,
                                   abs_difference_xyz=difference,authoritative_representation_verified=False))
            except ValueError:numeric_errors.append(a[key])
        sets[stem]={f['attributes'][key]:f['parts'][0][0] for f in attrs}
        rows.append(dict(layer=stem,count=len(attrs),id_field=key,unique_ids=len(sets[stem]),
            prj_present=bool(layer['wkt']),horizontal_authority=CRS.from_wkt(layer['wkt']).to_authority() if layer['wkt'] else None,
            dbf_geometry_abs_difference_xyz_max=np.max(deltas,axis=0).tolist() if deltas else None,
            nonnumeric_attribute_ids=numeric_errors,does_not_define_road_surface=True))
    comparisons=[]
    for a,aa in sets.items():
        for b,bb in sets.items():
            if a>=b or Path(a).parent!=Path(b).parent:continue
            common=sorted(set(aa)&set(bb))
            if not common:continue
            differences=np.asarray([np.asarray(aa[k])-np.asarray(bb[k]) for k in common])
            comparisons.append(dict(a=a,b=b,common_ids=len(common),
                numeric_xy_difference_max_m=float(np.hypot(differences[:,0],differences[:,1]).max()),
                numeric_z_difference_max_m=float(np.abs(differences[:,2]).max()),
                crs_present_for_both=bool(layers[a]['wkt'] and layers[b]['wkt']),
                registration_certified=False))
    return dict(layers=rows,total_records=sum(r['count'] for r in rows),comparisons=comparisons,
                per_point_coordinate_checks=checks,
                accuracy_or_vertical_datum_report_present=False)


def validate_output(destination, sources):
    destination=Path(destination).resolve()
    for source in sources:
        source=Path(source).resolve()
        if destination==source or source in destination.parents:
            raise ValueError('diagnostic output must be outside source map directories')
    return destination


def plot_evidence(report, layers, profiles, mgeo, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    origin=report['mgeo_info']['local_origin_in_global']
    def local_parts(feature):
        return [[tuple(v-o for v,o in zip(p,origin)) for p in part] for part in feature['parts']]
    def overlaps(points, box):
        return not (max(p[0] for p in points)<box[0] or min(p[0] for p in points)>box[1] or
                    max(p[1] for p in points)<box[2] or min(p[1] for p in points)>box[3])
    def draw_context(axis, box, project=None):
        def plot(points,**kwargs):
            values=[project(p) if project else p for p in points]
            axis.plot([p[0] for p in values],[p[1] for p in values],**kwargs)
        for feature in layers['A2_LINK']['features']:
            for part in local_parts(feature):
                if overlaps(part,box):plot(part,color='0.75',linestyle='--',linewidth=.6)
        for link in mgeo.values():
            if link.get('lazy_init') or link.get('lane_ch_link_path'):continue
            if overlaps(link['points'],box):plot(link['points'],color='royalblue',linewidth=.8,alpha=.8)
        for feature in layers['C3_VEHICLEPROTECTIONSAFETY']['features']:
            for part in local_parts(feature):
                if overlaps(part,box):plot(part,color='sienna',linewidth=.7,linestyle=':')
        for name in ('A3_DRIVEWAYSECTION','B3_SURFACEMARK','C4_SPEEDBUMP'):
            for feature in layers[name]['features']:
                for part in local_parts(feature):
                    if overlaps(part,box):plot(part,color='gray',linewidth=.6,linestyle='-.',alpha=.65)
        for feature in layers['B2_SURFACELINEMARK']['features']:
            attr=feature['attributes'];known=marking_definition(attr['Type'],attr['Kind'])['definition_confirmed']
            color='purple' if attr['Kind']=='505' else 'darkcyan' if known else 'magenta'
            for part in local_parts(feature):
                if overlaps(part,box):plot(part,color=color,linewidth=.65,linestyle='-' if known else ':',alpha=.8)
        for name,marker,color in [('B1_SAFETYSIGN','^','firebrick'),('C1_TRAFFICLIGHT','o','black')]:
            for feature in layers[name]['features']:
                for part in local_parts(feature):
                    if overlaps(part,box):plot(part,color=color,marker=marker,markersize=3,linestyle='')
    legend=[Line2D([],[],color='royalblue',label='MGeo ordinary centerline'),
        Line2D([],[],color='0.65',linestyle='--',label='SHP A2 centerline'),
        Line2D([],[],color='darkcyan',label='B2 observed marking; no crossing approval'),
        Line2D([],[],color='purple',label='B2 Kind 505 roadside marking; no road-edge approval'),
        Line2D([],[],color='magenta',linestyle=':',label='B2 undefined/unknown code'),
        Line2D([],[],color='sienna',linestyle=':',label='C3 observed facility; inside/outside unverified'),
        Line2D([],[],color='gray',linestyle='-.',label='A3/B3/C4 observed polygons; not road surface'),
        Line2D([],[],color='red',marker='x',linestyle='',label='Missing/ambiguous boundary sample'),
        Line2D([],[],color='firebrick',marker='^',linestyle='',label='B1 safety sign / C1 signal require local rule review')]
    styles=[('entry','darkorange'),('hold','seagreen'),('return','purple'),('control reserve','dodgerblue')]
    for rank,candidate in enumerate(report['top_candidates'],1):
        folder=output/f'candidate_{rank}';data=profiles[candidate['candidate_id']]
        center=data['center'];neighbor=data['adjacent'];samples=data['samples'];lengths=arc(center)
        all_points=center+neighbor
        box=[min(p[0] for p in all_points)-15,max(p[0] for p in all_points)+15,
             min(p[1] for p in all_points)-15,max(p[1] for p in all_points)+15]
        fig,(axis,profile)=plt.subplots(1,2,figsize=(16,8),gridspec_kw={'width_ratios':[1.1,1]})
        draw_context(axis,box)
        axis.plot([p[0] for p in center],[p[1] for p in center],color='black',linewidth=1.6)
        axis.plot([p[0] for p in neighbor],[p[1] for p in neighbor],color='black',linewidth=1.2,linestyle='--')
        missing=[r for r in samples if not r['supported']]
        axis.scatter([r['x'] for r in missing],[r['y'] for r in missing],marker='x',c='red',s=15,zorder=6)
        spans=candidate['gentle_dashed_sampled_spans_m']
        usable=[span for span in spans if span[1]-span[0]>=candidate['required_support_m']]
        if not usable:usable=candidate['sampled_support_spans_m']
        chosen=max(usable,key=lambda span:span[1]-span[0]) if usable else [0.,0.]
        assumptions=report['candidate_assumptions'];start=chosen[0]+assumptions['initial_rear_support_m']
        offsets=[start,start+assumptions['entry_end_m'],start+assumptions['return_start_m'],
                 start+assumptions['return_end_m'],start+assumptions['return_end_m']+assumptions['rollout_reserve_m']]
        for (label,color),a,b in zip(styles,offsets,offsets[1:]):
            if a<=lengths[-1]:
                end=min(b,lengths[-1]);points=[sample(center,a,lengths)[0]]
                points += [p for p,s in zip(center,lengths) if a<s<end]
                points += [sample(center,end,lengths)[0]]
                axis.plot([p[0] for p in points],[p[1] for p in points],color=color,linewidth=3.,alpha=.8)
        axis.set_xlim(box[:2]);axis.set_ylim(box[2:]);axis.set_aspect('equal',adjustable='box')
        axis.set_xlabel('MGeo local x (m)');axis.set_ylabel('MGeo local y (m)');axis.grid(alpha=.2)
        axis.legend(handles=legend,fontsize=6,loc='best')
        station=[r['s_m'] for r in samples]
        profile.step(station,[int(r['supported']) for r in samples],where='post',label='3 observed markings supported')
        profile.step(station,[.6*int(r['gentle_dashed_support']) for r in samples],where='post',label='dashed + gentle diagnostic')
        for (label,color),a,b in zip(styles,offsets,offsets[1:]):
            profile.axvspan(a,b,color=color,alpha=.15,label=label+' length budget')
        end=chosen[0]+assumptions['required_actual_map_support_m']
        profile.axvline(end,color='red',linestyle='--',label=f"required observed support {assumptions['required_actual_map_support_m']:.2f} m")
        profile.set_ylim(-.1,1.2);profile.set_xlim(0,max(lengths[-1],end)+3)
        profile.set_xlabel('original-lane station (m)');profile.set_ylabel('diagnostic support flags')
        profile.grid(alpha=.2);profile.legend(fontsize=7,loc='upper right')
        fig.suptitle(f"Candidate {rank}: {candidate['original_ids'][0]} / {candidate['adjacent_ids'][0]} | class {candidate['classification']}\n"
            f"MGeo coverage {candidate['mgeo_coverage_ratio']:.0%}; max gentle/dashed sample span {candidate['max_gentle_dashed_sampled_span_m']:.1f} m\n"
            'ANALYSIS ONLY: longitudinal budget on observed centerline; no lattice trajectory / road polygon / approval',fontsize=11)
        fig.tight_layout();fig.savefig(folder/'map_evidence.png',dpi=170);fig.savefig(folder/'map_evidence.svg');plt.close(fig)
        selected=[r for r in samples if chosen[0]<=r['s_m']<=min(end,lengths[-1])]
        anchor,tangent=sample(center,chosen[0],lengths)
        def project(point):
            dx,dy=point[0]-anchor[0],point[1]-anchor[1]
            return (dx*tangent[0]+dy*tangent[1],-dx*tangent[1]+dy*tangent[0])
        detail_box=[min(r['x'] for r in selected)-20,max(r['x'] for r in selected)+20,
                    min(r['y'] for r in selected)-20,max(r['y'] for r in selected)+20]
        fig,detail=plt.subplots(figsize=(14,6))
        draw_context(detail,detail_box,project)
        xy=[project((r['x'],r['y'])) for r in selected]
        detail.plot([p[0] for p in xy],[p[1] for p in xy],color='black',linewidth=1.1)
        for (label,color),a,b in zip(styles,offsets,offsets[1:]):
            values=[project((r['x'],r['y'])) for r in selected if a<=r['s_m']<=b]
            if values:detail.plot([p[0] for p in values],[p[1] for p in values],color=color,linewidth=3,label=label+' budget')
        bad=[project((r['x'],r['y'])) for r in selected if not r['supported']]
        detail.scatter([p[0] for p in bad],[p[1] for p in bad],color='red',marker='x',s=35,zorder=7)
        detail.set_xlim(min(p[0] for p in xy)-6,max(p[0] for p in xy)+6)
        detail.set_ylim(min(p[1] for p in xy)-12,max(p[1] for p in xy)+12)
        detail.set_aspect('equal',adjustable='box');detail.grid(alpha=.2)
        detail.set_xlabel('along tangent at observation start (m)');detail.set_ylabel('left of tangent (m)')
        detail.legend(handles=legend,fontsize=7,loc='upper center',bbox_to_anchor=(.5,-.2),ncol=2)
        detail.set_title(f"Candidate {rank} detail: observed station {chosen[0]:.2f}..{min(end,lengths[-1]):.2f} m\n"
            'Observed lines only; colors show longitudinal budgets, not an avoidance path. No surface/crossing approval.')
        fig.tight_layout();fig.savefig(folder/'map_detail.png',dpi=180);fig.savefig(folder/'map_detail.svg');plt.close(fig)
        transform=Transformer.from_crs(32652,4326,always_xy=True)
        lonlat=[transform.transform(r['x']+origin[0],r['y']+origin[1]) for r in selected]
        scope=dict(artifact_type='diagnostic_data_request_scope',approved_for_runtime=False,
            original_ids=candidate['original_ids'],adjacent_ids=candidate['adjacent_ids'],
            observation_span_m=chosen,longitudinal_example_rear_axle_start_m=start,
            required_observation_end_m=end,required_observation_length_m=assumptions['required_actual_map_support_m'],
            longitude_latitude_bbox=[min(p[0] for p in lonlat),min(p[1] for p in lonlat),
                                     max(p[0] for p in lonlat),max(p[1] for p in lonlat)] if lonlat else None,
            warning='Request extent only. No surface polygon, width, crossing permission or runtime geometry is defined.')
        write_json(folder/'request_scope.json',scope)
    fig,axis=plt.subplots(figsize=(8,15))
    full=[p for f in layers['A2_LINK']['features'] for part in local_parts(f) for p in part]
    box=[min(p[0] for p in full)-10,max(p[0] for p in full)+10,min(p[1] for p in full)-10,max(p[1] for p in full)+10]
    draw_context(axis,box)
    for rank,candidate in enumerate(report['top_candidates'],1):
        points=profiles[candidate['candidate_id']]['center']
        axis.plot([p[0] for p in points],[p[1] for p in points],linewidth=2,label=f"Candidate {rank}")
        p=points[len(points)//2];axis.text(p[0],p[1],str(rank),fontsize=15,color='red')
    axis.set_aspect('equal');axis.set_xlabel('MGeo local x (m)');axis.set_ylabel('MGeo local y (m)')
    axis.legend(fontsize=8);axis.set_title('Whole delivery vs current MGeo - diagnostic only');axis.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output/'whole_map_evidence.png',dpi=170);plt.close(fig)


def run(zip_path, hdmap, mgeo_dir, output):
    output=validate_output(output,[hdmap,mgeo_dir,Path(zip_path).parent])
    output.mkdir(parents=True,exist_ok=True)
    source_files=[Path(zip_path)]+[p for folder in (Path(hdmap),Path(mgeo_dir)) for p in folder.rglob('*') if p.is_file()]
    before={str(p.resolve()):digest(p.read_bytes()) for p in source_files}
    members,manifest=read_archive(zip_path);all_layers=load_layers(members)
    summaries=[layer_summary(l) for l in all_layers.values()]
    groups=defaultdict(dict)
    for stem,layer in all_layers.items():
        if '/HDMap_' in stem:groups[PurePosixPath(stem).parent.name][PurePosixPath(stem).name]=layer
    ref_name=next(g for g in groups if g.startswith('HDMap_UTM52N_'))
    reference=groups[ref_name];existing=[];parser_equivalence=[]
    for member in manifest:
        path=PurePosixPath(member['path'])
        if path.parent.name!=ref_name:continue
        destination=Path(hdmap)/path.name
        existing.append(dict(member=member['path'],path=str(destination),exists=destination.is_file(),
            byte_identical=destination.is_file() and digest(destination.read_bytes())==member['sha256']))
    for name,layer in reference.items():
        parsed=read_layer(Path(hdmap)/name)
        parser_equivalence.append(dict(layer=name,equal=parsed==layer['features']))
    info=json.loads((Path(mgeo_dir)/'global_info.json').read_text());origin=info['local_origin_in_global']
    if info['global_coordinate_system']!='UTM52N' or CRS.from_wkt(reference['A2_LINK']['wkt']).to_epsg()!=32652:
        raise ValueError('no verified horizontal registration to current MGeo')
    mgeo={l['idx']:l for l in json.loads((Path(mgeo_dir)/'link_set.json').read_text())}
    workbooks={p:read_workbook(raw) for p,raw in members.items() if p.endswith('.xlsx')}
    current=next(sheet for book in workbooks.values() for sheet in book if sheet['name']=='도화데이터')
    cells={c['address']:c['value'] for c in current['cells']}
    count_cells={'A1_NODE':'BL4','A2_LINK':'BN4','A3_DRIVEWAYSECTION':'BP4','B1_SAFETYSIGN':'BV4',
        'B2_SURFACELINEMARK':'BX4','B3_SURFACEMARK':'BZ4','C1_TRAFFICLIGHT':'CB4',
        'C3_VEHICLEPROTECTIONSAFETY':'CF4','C4_SPEEDBUMP':'CH4','C6_POSTPOINT':'CL4'}
    declared=[dict(layer=name,workbook_sheet='도화데이터',cell=cell,declared=int(cells[cell]),
                   delivered=len(reference[name]['features']),difference=len(reference[name]['features'])-int(cells[cell]))
              for name,cell in count_cells.items()]
    candidates,top,profiles,topology,assumptions=scan_candidates(reference,mgeo,origin)
    report=dict(artifact_type='diagnostic_full_map_delivery_audit',approved_for_runtime=False,
        runtime_boundary_generated=False,zip_path=str(Path(zip_path).resolve()),zip_sha256=before[str(Path(zip_path).resolve())],
        zip_file_count=len(manifest),zip_uncompressed_bytes=sum(m['bytes'] for m in manifest),zip_crc_all_passed=True,
        zip_manifest=manifest,zip_extension_counts=dict(Counter(PurePosixPath(p).suffix.lower() for p in members)),
        layers=summaries,existing_extraction=existing,independent_parser_equivalence=parser_equivalence,
        coordinate_variant_comparison=coordinate_comparison(reference,{g:l for g,l in groups.items() if g!=ref_name}),
        control_points=control_point_audit(all_layers),workbook_declared_count_comparison=declared,
        metadata_xml={p:raw.decode('utf-8-sig') for p,raw in members.items() if p.endswith('.xml')},
        additional_payload_missing_types=['road-surface mask','lane-boundary topology','explicit lane-change/return permission',
            'datum/geoid/accuracy attestation','MORAI scene revision binding'],
        mgeo_info=info,mgeo_comparison=mgeo_comparison(reference,mgeo,origin),
        referential_integrity=reference_audit(reference),pair_topology=topology,
        candidate_assumptions=assumptions,candidate_count=len(candidates),classification_counts=dict(Counter(c['classification'] for c in candidates)),
        ranking_formula='30*length_fraction + 20*sample_support_ratio + 25*gentle_dashed_span_fraction + 20*MGeo_ID_coverage + 5*dashed_ratio; fractions capped at 1; native geometry error penalty 35',
        gentle_screen_is_operational_limit=False,
        top_candidates=top,all_candidates=candidates,
        methodology=dict(no_zip_extraction=True,no_source_writes=True,no_assumed_lane_width=True,
            sampled_boundary_support_not_continuous_certificate=True,dashed_marking_not_crossing_permission=True,
            c3_facility_not_surface_mask=True,scores_not_safety_approval=True),source_sha256_before=before)
    after={str(p.resolve()):digest(p.read_bytes()) for p in source_files}
    report['source_sha256_after']=after;report['all_source_hashes_unchanged']=before==after
    if before!=after:raise RuntimeError('source changed during audit')
    write_json(output/'audit.json',report);write_json(output/'workbook_cells.json',workbooks)
    write_csv(output/'zip_manifest.csv',manifest);write_csv(output/'layer_inventory.csv',summaries)
    write_csv(output/'coordinate_variants.csv',report['coordinate_variant_comparison'])
    write_csv(output/'control_point_coordinate_checks.csv',report['control_points']['per_point_coordinate_checks'])
    write_csv(output/'candidates.csv',candidates)
    for rank,candidate in enumerate(top,1):
        folder=output/f'candidate_{rank}';folder.mkdir(exist_ok=True)
        write_json(folder/'evidence.json',candidate)
        write_csv(folder/'sampled_boundary_support.csv',profiles[candidate['candidate_id']]['samples'])
    write_json(output/'candidate_profiles.json',{c['candidate_id']:profiles[c['candidate_id']] for c in top})
    return report,reference,profiles,mgeo


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip',required=True,type=Path)
    parser.add_argument('--hdmap',required=True,type=Path)
    parser.add_argument('--mgeo',required=True,type=Path)
    parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    report,layers,profiles,mgeo=run(args.zip,args.hdmap,args.mgeo,args.out)
    plot_evidence(report,layers,profiles,mgeo,args.out)
    print(json.dumps({k:report[k] for k in ('zip_file_count','candidate_count','classification_counts',
        'all_source_hashes_unchanged','top_candidates')},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
