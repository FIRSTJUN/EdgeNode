"""Offline SHP/MGeo screening. Never writes road cells or runtime approvals."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct

from .lane_geometry import PolylineIndex, arc, sample
from .lane_geometry import LaneNetwork
from .dijkstra_planner import DijkstraPlanner


MANUAL_URL = 'https://www.ngii.go.kr/other/file_down.do?sq=76349'
MANUAL_SHA256 = '93958d4079dcdde311368753049f0df95238122ab6bde0ea241fed89499a6541'
TYPE_NAMES = {
    '111':'yellow single solid', '112':'yellow single dashed',
    '121':'yellow double solid', '122':'yellow double dashed',
    '123':'yellow double left-dashed mixed', '124':'yellow double right-dashed mixed',
    '211':'white single solid', '212':'white single dashed',
    '221':'white double solid', '222':'white double dashed',
    '223':'white double left-dashed mixed', '224':'white double right-dashed mixed',
    '311':'blue single solid', '312':'blue single dashed',
    '321':'blue double solid', '322':'blue double dashed',
    '323':'blue double left-dashed mixed', '324':'blue double right-dashed mixed',
    '999':'other marking',
}
KIND_NAMES = {
    '501':'center divider', '5011':'reversible divider', '502':'U-turn marking',
    '503':'lane divider', '504':'bus-only lane marking', '505':'roadside edge zone',
    '506':'lane-change restriction marking', '515':'stopping/parking prohibition',
    '525':'guidance line', '530':'stop line', '531':'safety zone',
    '535':'bicycle lane', '599':'other regulatory marking',
}


def marking_definition(type_code, kind_code):
    # Tables 55-57, pp.87-89. Single mixed types with '-' definitions are
    # intentionally absent; do not infer permission from their digits.
    known = type_code in TYPE_NAMES and kind_code in KIND_NAMES
    return dict(type_name=TYPE_NAMES.get(type_code), kind_name=KIND_NAMES.get(kind_code),
                definition_confirmed=known,
                simple_dashed_lane_candidate=known and type_code == '212' and kind_code == '503',
                crossing_approved=False)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_layer(stem):
    """Strict subset reader for the supplied PointZ/PolyLineZ/PolygonZ layers.

    Keep multipart boundaries separate. Never join parts or deleted records.
    DBF names/codes remain strings; CP949 text is decoded without replacement.
    """
    stem = Path(stem)
    dbf = stem.with_suffix('.dbf').read_bytes()
    if len(dbf) < 33:
        raise ValueError('truncated DBF header')
    count = struct.unpack_from('<I', dbf, 4)[0]
    header, size = struct.unpack_from('<HH', dbf, 8)
    if header < 33 or (header - 33) % 32 or len(dbf) < header + count * size:
        raise ValueError('invalid DBF record layout')
    fields = []
    for pos in range(32, header - 1, 32):
        field = dbf[pos:pos+32]
        fields.append((field[:11].split(b'\0')[0].decode('ascii'), field[16]))
    if dbf[header-1] != 13 or sum(n for _, n in fields) + 1 != size:
        raise ValueError('invalid DBF field lengths')
    records = []
    for i in range(count):
        row = dbf[header+i*size:header+(i+1)*size]
        if row[0] != 32:
            raise ValueError('deleted/invalid DBF record requires review')
        values = {}; offset = 1
        for name, length in fields:
            values[name] = row[offset:offset+length].strip().decode('cp949')
            offset += length
        records.append(values)
    raw = stem.with_suffix('.shp').read_bytes()
    if (len(raw) < 100 or struct.unpack_from('>i', raw, 0)[0] != 9994 or
            struct.unpack_from('>i', raw, 24)[0] * 2 != len(raw) or
            struct.unpack_from('<i', raw, 28)[0] != 1000):
        raise ValueError('invalid SHP header/length')
    kind = struct.unpack_from('<i', raw, 32)[0]
    if kind not in (11, 13, 15):
        raise ValueError('only supplied Z layers are supported')
    result = []; offset = 100
    while offset < len(raw):
        if offset + 12 > len(raw):
            raise ValueError('truncated SHP record')
        number, words = struct.unpack_from('>ii', raw, offset)
        end = offset + 8 + words * 2
        body = raw[offset+8:end]
        if end > len(raw) or len(body) < 4 or number != len(result) + 1:
            raise ValueError('invalid SHP record bounds/order')
        if struct.unpack_from('<i', body)[0] != kind:
            raise ValueError('null/mixed SHP geometry requires review')
        if kind == 11:
            if len(body) < 28:
                raise ValueError('truncated PointZ')
            parts = [(struct.unpack_from('<3d', body, 4),)]
        else:
            if len(body) < 44:
                raise ValueError('truncated multipart header')
            part_count, point_count = struct.unpack_from('<ii', body, 36)
            start = 44 + 4 * part_count
            zstart = start + 16 * point_count + 16
            if part_count < 1 or point_count < 2 or zstart + 8 * point_count > len(body):
                raise ValueError('invalid multipart coordinate bounds')
            indices = list(struct.unpack_from('<' + 'i' * part_count, body, 44)) + [point_count]
            if indices[0] != 0 or any(b <= a for a, b in zip(indices, indices[1:])):
                raise ValueError('invalid part indices')
            points = [(*struct.unpack_from('<2d', body, start+16*i),
                       struct.unpack_from('<d', body, zstart+8*i)[0]) for i in range(point_count)]
            parts = [tuple(points[a:b]) for a, b in zip(indices, indices[1:])]
        if not all(math.isfinite(v) for part in parts for point in part for v in point):
            raise ValueError('nonfinite SHP coordinate')
        result.append(dict(parts=parts, shape_type=kind))
        offset = end
    if len(result) != len(records):
        raise ValueError('SHP/DBF record count mismatch')
    ids = set()
    for geometry, attributes in zip(result, records):
        if not attributes.get('ID') or attributes['ID'] in ids:
            raise ValueError('missing/duplicate feature ID')
        ids.add(attributes['ID']); geometry['attributes'] = attributes
    return result


def nearest(points, query):
    """Clamped nearest segment; no extension of an observed line."""
    lengths = arc(points); best = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = lengths[i+1] - lengths[i]
        if length <= 1e-9:
            continue
        tx, ty = (b[0]-a[0])/length, (b[1]-a[1])/length
        u = max(0., min(1., ((query[0]-a[0])*tx+(query[1]-a[1])*ty)/length))
        p = tuple((1-u)*x+u*y for x, y in zip(a, b))
        distance = math.dist(query[:2], p[:2])
        row = (distance, lengths[i]+u*length, tx*(query[1]-p[1])-ty*(query[0]-p[0]), p)
        if best is None or distance < best[0]:
            best = row
    if best is None:
        raise ValueError('degenerate line')
    return best


def translated(feature, origin):
    return [tuple(tuple(v-o for v, o in zip(p, origin)) for p in part) for part in feature['parts']]


def intersects(a, b, c, d):
    if (max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0]) or
            max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1])):
        return False
    def turn(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    return turn(a, b, c)*turn(a, b, d) <= 0 and turn(c, d, a)*turn(c, d, b) <= 0


def boundary_screen(center, left, right, origin):
    """Only screen original observations; no polygon/cell generation or approval."""
    result = dict(reasons=[], sampled_width_min_m=None, sampled_width_max_m=None,
                  complete_normal_support=False, sampled_support_spans_m=[], approved=False)
    if len(left) != 1 or len(right) != 1:
        result['reasons'].append('MISSING_OR_AMBIGUOUS_SIDE_MARKINGS'); return result
    if len(left[0]['parts']) != 1 or len(right[0]['parts']) != 1:
        result['reasons'].append('MULTIPART_BOUNDARY_NO_GAP_BRIDGING'); return result
    lines = [translated(x[0], origin)[0] for x in (left, right)]
    for line in lines:
        edges = list(zip(line, line[1:]))
        if any(intersects(a, b, c, d) for i, (a, b) in enumerate(edges)
               for c, d in edges[i+2:]):
            result['reasons'].append('SELF_CROSSING_BOUNDARY'); return result
    if any(intersects(a, b, c, d) for a, b in zip(lines[0], lines[0][1:])
           for c, d in zip(lines[1], lines[1][1:])):
        result['reasons'].append('CROSSING_SIDE_BOUNDARIES'); return result
    for i, line in enumerate(lines):
        if nearest(center, line[-1])[1] < nearest(center, line[0])[1]:
            lines[i] = tuple(reversed(line))  # Same observed geometry, no joining.
    indices = [PolylineIndex(line) for line in lines]
    lengths = arc(center); total = lengths[-1]
    stations = sorted(set(lengths + [float(i) for i in range(math.ceil(total))] + [total]))
    widths = []
    supported = []
    for station in stations:
        p, tangent = sample(center, station, lengths)
        sides = [idx.normal_intersection(p, tangent, side)
                 for idx, side in zip(indices, ('left', 'right'))]
        if any(row is None for row in sides):
            supported.append(False)
            continue
        supported.append(True)
        widths.append(math.dist(sides[0][1][0][:2], sides[1][1][0][:2]))
    start = end = None
    for station, valid in zip(stations, supported):
        if valid:
            if start is None: start = station
            end = station
        elif start is not None:
            result['sampled_support_spans_m'].append([start, end]); start = end = None
    if start is not None:
        result['sampled_support_spans_m'].append([start, end])
    if widths:
        result.update(sampled_width_min_m=min(widths), sampled_width_max_m=max(widths))
        if min(widths) < 2.36 or max(widths) > 12.:
            result['reasons'].append('PRELIMINARY_WIDTH_SCREEN')
    result['complete_normal_support'] = len(widths) == len(stations)
    if not result['complete_normal_support']:
        result['reasons'].append('INCOMPLETE_NORMAL_SUPPORT')
    if not result['reasons']:
        result['reasons'].append('GEOMETRY_SCREEN_ONLY_NOT_ROAD_APPROVAL')
    return result


def audit(mgeo_dir, hdmap_dir):
    from pyproj import CRS  # Offline tool only; not imported by PlanningNode.
    mgeo_dir, hdmap_dir = Path(mgeo_dir), Path(hdmap_dir)
    info = json.loads((mgeo_dir / 'global_info.json').read_text())
    origin = info['local_origin_in_global']
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError('invalid MGeo origin')
    links = json.loads((mgeo_dir / 'link_set.json').read_text())
    legacy_network = LaneNetwork(DijkstraPlanner(mgeo_dir))
    strict_network = LaneNetwork(DijkstraPlanner(mgeo_dir), require_explicit_permission=True)
    indexed = {link['idx']:link for link in links}
    if len(indexed) != len(links):
        raise ValueError('duplicate MGeo link IDs')
    layers = {path.stem:read_layer(path.with_suffix('')) for path in sorted(hdmap_dir.glob('*.shp'))}
    crs = {name:CRS.from_wkt((hdmap_dir / (name+'.prj')).read_text()).to_epsg() for name in layers}
    registered = info['global_coordinate_system'] == 'UTM52N' and set(crs.values()) == {32652}
    if not registered:
        raise ValueError('unverified horizontal CRS; no coordinate guessing')
    marks = layers['B2_SURFACELINEMARK']; shp_links = {x['attributes']['ID']:x for x in layers['A2_LINK']}
    common = sorted(set(indexed) & set(shp_links))
    errors = []; link_rows = []
    for key in common:
        center = indexed[key]['points']; shape = shp_links[key]
        if len(shape['parts']) != 1:
            raise ValueError('multipart A2 requires explicit alignment review')
        observed = translated(shape, origin)[0]
        xy_error = max(max(nearest(observed, p)[0] for p in center),
                       max(nearest(center, p)[0] for p in observed))
        z_error = max(abs(p[2] - nearest(center, p)[3][2]) for p in observed)
        errors.append(xy_error)
        left = [x for x in marks if x['attributes']['R_linkID'] == key]
        right = [x for x in marks if x['attributes']['L_linkID'] == key]
        screen = boundary_screen(center, left, right, origin)
        if any(not marking_definition(x['attributes']['Type'], x['attributes']['Kind'])[
                'definition_confirmed'] for x in left + right):
            screen['reasons'].append('UNKNOWN_OR_UNDEFINED_MARKING_CODE')
        link_rows.append(dict(link_id=key, left_marking_ids=[x['attributes']['ID'] for x in left],
            right_marking_ids=[x['attributes']['ID'] for x in right],
            link_length_m=arc(center)[-1],
            sampled_xy_alignment_max_m=xy_error, sampled_z_alignment_max_m=z_error, **screen))
    pairs = []
    for key, link in indexed.items():
        if link.get('lazy_init') or link.get('lane_ch_link_path'):
            continue
        for side in ('left', 'right'):
            target = link.get(side+'_lane_change_dst_link_idx')
            if not target or target not in indexed or key >= target:
                continue
            shared = [x for x in marks if {x['attributes']['L_linkID'], x['attributes']['R_linkID']} == {key, target}]
            reverse = 'right' if side == 'left' else 'left'
            reasons = ['ROAD_SURFACE_AND_EXCLUSIONS_NOT_VERIFIED', 'NO_REVIEWED_BOUNDARY_RELEASE']
            if link.get('can_move_'+side+'_lane') is not True or indexed[target].get('can_move_'+reverse+'_lane') is not True:
                reasons.append('UNKNOWN_OR_FORBIDDEN_LANE_CHANGE_PERMISSION')
            if not shared: reasons.append('NO_SHARED_SHP_MARKING')
            if any(indexed[k].get('link_type') not in ('6', 6) for k in (key, target)):
                reasons.append('SPECIAL_OR_UNKNOWN_LANE_TYPE_NOT_VERIFIED')
            pairs.append(dict(original=key, adjacent=target, side=side,
                mgeo_link_types=[indexed[k].get('link_type') for k in (key, target)],
                legacy_topology_geometry_passed=legacy_network.neighbor(key, side) == target,
                strict_permission_reason=strict_network.permission_reason(key, side),
                shared_markings=[dict(id=x['attributes']['ID'], type=x['attributes']['Type'],
                                      kind=x['attributes']['Kind'], **marking_definition(
                                          x['attributes']['Type'], x['attributes']['Kind'])) for x in shared],
                approved=False, reasons=reasons))
    gaps = []
    ordinary = [x for x in links if not x.get('lazy_init') and not x.get('lane_ch_link_path')]
    for a in ordinary:
        for b in ordinary:
            if a['to_node_idx'] == b['from_node_idx']:
                gap = math.dist(a['points'][-1][:2], b['points'][0][:2])
                if gap > 1e-7: gaps.append(dict(from_link=a['idx'], to_link=b['idx'], gap_m=gap))
    manifest = {str(p):sha256(p) for p in sorted(mgeo_dir.glob('*.json'))}
    manifest.update({str(p):sha256(p) for p in sorted(hdmap_dir.iterdir()) if p.is_file()})
    return dict(artifact_type='diagnostic_map_audit', approved_for_runtime=False,
        runtime_boundary_generated=False, source_sha256=manifest,
        definitions_source=dict(url=MANUAL_URL, sha256=MANUAL_SHA256),
        coordinates=dict(mgeo_crs=info['global_coordinate_system'], shp_epsg=crs,
            origin=origin, horizontal_crs_consistent=registered,
            vertical_datum_verified=False),
        inventory=dict(mgeo_links=len(links), ordinary_mgeo_links=len(ordinary),
            legacy_directed_neighbors=sum(legacy_network.neighbor(k, s) is not None
                for k in legacy_network.valid for s in ('left', 'right')),
            s4_directed_neighbors=sum(strict_network.neighbor(k, s) is not None
                for k in strict_network.valid for s in ('left', 'right')),
            shp_counts={k:len(v) for k, v in layers.items()}, common_a2_ids=len(common),
            mgeo_ids_missing_a2=sorted(set(indexed)-set(shp_links)),
            links_with_both_marking_associations=sum(bool(r['left_marking_ids'] and r['right_marking_ids']) for r in link_rows)),
        b2_types=dict(Counter(x['attributes']['Type'] for x in marks)),
        b2_kinds=dict(Counter(x['attributes']['Kind'] for x in marks)),
        b2_unknown_or_undefined_types=sorted({x['attributes']['Type'] for x in marks
                                            if x['attributes']['Type'] not in TYPE_NAMES}),
        b2_unknown_kinds=sorted({x['attributes']['Kind'] for x in marks
                                if x['attributes']['Kind'] not in KIND_NAMES}),
        c3_types=dict(Counter(x['attributes']['Type'] for x in layers['C3_VEHICLEPROTECTIONSAFETY'])),
        c3_schema_note='Delivery has Ref_ID; official 2019 manual table 67 uses Ref_FID. Version applicability requires review.',
        a3_kinds=dict(Counter(x['attributes']['Kind'] for x in layers['A3_DRIVEWAYSECTION'])),
        a3_role_note='Manual p64: limited section polygons, not a complete ordinary-road surface mask.',
        a3_local_bbox=[min(p[k] - origin[k] for f in layers['A3_DRIVEWAYSECTION']
                          for part in f['parts'] for p in part) for k in (0, 1)] +
                      [max(p[k] - origin[k] for f in layers['A3_DRIVEWAYSECTION']
                           for part in f['parts'] for p in part) for k in (0, 1)],
        c3_closed_observed_parts=sum(math.dist(part[0][:2], part[-1][:2]) <= 1e-9
            for f in layers['C3_VEHICLEPROTECTIONSAFETY'] for part in f['parts']),
        c3_low_high=dict(Counter(x['attributes']['LowHigh'] for x in layers['C3_VEHICLEPROTECTIONSAFETY'])),
        sampled_xy_alignment_max_m=max(errors, default=None), link_endpoint_gaps=gaps,
        a2_mgeo_link_type_mismatches=[k for k in common if str(indexed[k].get('link_type')) !=
                                     shp_links[k]['attributes']['LinkType']],
        complete_normal_support_count=sum(r['complete_normal_support'] for r in link_rows),
        links_with_partial_or_full_sampled_support=sum(bool(r['sampled_support_spans_m']) for r in link_rows),
        geometry_screen_only_count=sum(r['reasons']==['GEOMETRY_SCREEN_ONLY_NOT_ROAD_APPROVAL'] for r in link_rows),
        links=link_rows, pairs=pairs,
        rejection_reasons=['NO_REVIEWED_BOUNDARY_RELEASE', 'ROAD_SURFACE_AND_EXCLUSIONS_NOT_VERIFIED',
                           'MISSING_EXPLICIT_LANE_CHANGE_PERMISSIONS', 'VERTICAL_DATUM_NOT_VERIFIED'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mgeo', required=True)
    parser.add_argument('--hdmap', required=True)
    parser.add_argument('--output', required=True, help='diagnostic report; never a runtime boundary')
    args = parser.parse_args()
    destination = Path(args.output).resolve()
    for directory in (Path(args.mgeo).resolve(), Path(args.hdmap).resolve()):
        if destination == directory or directory in destination.parents:
            parser.error('diagnostic output must not be written into source map directories')
    result = audit(args.mgeo, args.hdmap)
    destination.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    summary = {k:v for k, v in result.items() if k in (
        'approved_for_runtime', 'runtime_boundary_generated', 'inventory',
        'complete_normal_support_count', 'geometry_screen_only_count', 'rejection_reasons')}
    summary['inventory'] = {k:v for k, v in result['inventory'].items() if k != 'mgeo_ids_missing_a2'}
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
