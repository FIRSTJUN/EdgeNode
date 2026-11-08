"""Offline diagnostics never authorize synthetic or incomplete map geometry."""
from pathlib import Path
import struct

import pytest

from edgenode_planning.hdmap_audit import boundary_screen, marking_definition, read_layer, audit
from edgenode_planning.avoidance_geometry import load_drivable_area


def feature(points, *, parts=None):
    return dict(parts=[tuple(points)] if parts is None else parts)


def test_even_perfect_synthetic_lines_are_diagnostic_not_drivable_area():
    center = [(0., 0., 0.), (10., 0., 0.)]
    left = [feature([(0., 2., 0.), (10., 2., 0.)])]
    right = [feature([(0., -2., 0.), (10., -2., 0.)])]
    result = boundary_screen(center, left, right, (0., 0., 0.))
    assert result['complete_normal_support']
    assert result['sampled_width_min_m'] == pytest.approx(4.)
    assert not result['approved'] and 'cells' not in result


def test_missing_middle_and_partial_end_support_are_never_bridged():
    center = [(0., 0., 0.), (10., 0., 0.)]
    right = [feature([(0., -2., 0.), (10., -2., 0.)])]
    left = [feature([], parts=[((0., 2., 0.), (4., 2., 0.)),
                              ((6., 2., 0.), (10., 2., 0.))])]
    result = boundary_screen(center, left, right, (0., 0., 0.))
    assert result['reasons'] == ['MULTIPART_BOUNDARY_NO_GAP_BRIDGING']
    left = [feature([(1., 2., 0.), (9., 2., 0.)])]
    assert not boundary_screen(center, left, right, (0., 0., 0.))['complete_normal_support']


def test_crossing_observed_boundaries_are_rejected_before_screening():
    center = [(0., 0., 0.), (10., 0., 0.)]
    left = [feature([(0., 2., 0.), (10., -2., 0.)])]
    right = [feature([(0., -2., 0.), (10., 2., 0.)])]
    assert boundary_screen(center, left, right, (0., 0., 0.))['reasons'] == ['CROSSING_SIDE_BOUNDARIES']


@pytest.mark.parametrize('type_code,kind', [('214', '503'), ('000', '503'), ('212', 'unknown')])
def test_undefined_codes_never_become_crossing_candidates(type_code, kind):
    result = marking_definition(type_code, kind)
    assert not result['definition_confirmed']
    assert not result['simple_dashed_lane_candidate'] and not result['crossing_approved']


def test_official_dashed_lane_meaning_is_still_not_permission():
    result = marking_definition('212', '503')
    assert result['definition_confirmed'] and result['simple_dashed_lane_candidate']
    assert not result['crossing_approved']
    assert not marking_definition('212', '505')['simple_dashed_lane_candidate']


def test_shp_reader_rejects_truncated_input(tmp_path):
    stem = tmp_path / 'broken'
    stem.with_suffix('.dbf').write_bytes(b'bad')
    with pytest.raises(ValueError, match='DBF'):
        read_layer(stem)


def test_reader_preserves_source_multipart_boundaries(tmp_path):
    stem = tmp_path / 'parts'
    dbf = bytearray(65 + 13)
    dbf[0] = 3; struct.pack_into('<IHH', dbf, 4, 1, 65, 13)
    dbf[32:34] = b'ID'; dbf[43] = ord('C'); dbf[48] = 12; dbf[64] = 13
    dbf[65:78] = b' ' + b'FEATURE00001'
    stem.with_suffix('.dbf').write_bytes(dbf)
    points = ((0., 0.), (1., 0.), (3., 0.), (4., 0.))
    body = struct.pack('<i4d2i2i', 13, 0., 0., 4., 0., 2, 4, 0, 2)
    body += b''.join(struct.pack('<2d', *p) for p in points)
    body += struct.pack('<2d', 0., 0.) + struct.pack('<4d', 0., 0., 0., 0.)
    header = bytearray(100); struct.pack_into('>i', header, 0, 9994)
    struct.pack_into('>i', header, 24, (108+len(body))//2)
    struct.pack_into('<ii', header, 28, 1000, 13)
    stem.with_suffix('.shp').write_bytes(header + struct.pack('>2i', 1, len(body)//2) + body)
    result = read_layer(stem)
    assert len(result) == 1 and len(result[0]['parts']) == 2
    assert result[0]['parts'][0][-1][0] == 1. and result[0]['parts'][1][0][0] == 3.


def test_cli_never_writes_diagnostics_inside_source_map_directories(tmp_path, monkeypatch):
    import sys
    from edgenode_planning.hdmap_audit import main
    source = tmp_path / 'source'; source.mkdir()
    target = source / 'not_a_boundary.json'
    monkeypatch.setattr(sys, 'argv', ['hdmap_audit', '--mgeo', str(source),
        '--hdmap', str(source), '--output', str(target)])
    with pytest.raises(SystemExit) as failure:main()
    assert failure.value.code == 2 and not target.exists()


@pytest.fixture(scope='module')
def real_audit():
    root = Path(__file__).resolve().parents[4]
    return audit(root/'local_data/kcity_mgeo', root/'local_data/kcity_hdmap')


def test_real_map_counts_and_candidates_never_generate_runtime_cells(real_audit):
    result = real_audit
    assert result['inventory']['mgeo_links'] == 352
    assert result['inventory']['shp_counts']['A2_LINK'] == 809
    assert result['inventory']['shp_counts']['B2_SURFACELINEMARK'] == 1574
    assert result['inventory']['common_a2_ids'] == 253
    assert result['inventory']['links_with_both_marking_associations'] == 131
    assert result['approved_for_runtime'] is False and result['runtime_boundary_generated'] is False
    assert 'cells' not in result and all(not p['approved'] for p in result['pairs'])


def test_known_pair_and_unmatched_pair_are_distinguished_without_guessing(real_audit):
    pairs = {(x['original'], x['adjacent']):x for x in real_audit['pairs']}
    known = pairs[('A219BS010075', 'A219BS010096')]
    assert [x['id'] for x in known['shared_markings']] == ['B219BS010377']
    assert known['shared_markings'][0]['simple_dashed_lane_candidate']
    unknown = pairs[('A219BS010624', 'A219BS010625')]
    assert not unknown['shared_markings'] and 'NO_SHARED_SHP_MARKING' in unknown['reasons']
    assert unknown['mgeo_link_types'] == ['1', '1']
    assert 'SPECIAL_OR_UNKNOWN_LANE_TYPE_NOT_VERIFIED' in unknown['reasons']
    assert not known['approved'] and not unknown['approved']


def test_diagnostic_report_cannot_be_loaded_as_a_runtime_boundary(real_audit, tmp_path):
    import json
    path = tmp_path / 'diagnostic.json'; path.write_text(json.dumps(real_audit))
    with pytest.raises(ValueError):load_drivable_area(path)
    with pytest.raises(ValueError):load_drivable_area(path, approved_sha256='')
