"""Actual live failure and guarded removal of redundant supplemental sections."""
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.corridor_diagnostics import _distance, inspect_corridor
from edgenode_planning.corridor_scene import planner_from_scene, replay_scene
from edgenode_planning.lane_geometry import LanePair, arc, corridor_covers


@pytest.fixture(scope='module')
def live_scene():
    # Exact original dump, including PRE-plan blocked/release state and all
    # remembered observations. No hand-adjusted pose, speed, target or limits.
    path = Path(__file__).parent/'fixtures/s4_corridor_vvc82ouy.json.gz'
    raw = gzip.decompress(path.read_bytes())
    assert hashlib.sha256(raw).hexdigest() == '9f53e8c6828fce5b54905b29d5ce88544d96eb316842151fd328e76f9e2c233c'
    return json.loads(raw)


def _pair(scene, mirrored=False):
    e = scene['result']['corridor_evaluation']
    original, adjacent = copy.deepcopy(e['original_lane_points']), copy.deepcopy(e['adjacent_lane_points'])
    if mirrored:
        for p in original+adjacent:
            p[0] *= -1
    return LanePair(original, adjacent, e['lane_pair_ids'], 'left' if mirrored else 'right', [arc(original)[-1]])


@pytest.mark.parametrize('mirrored', [False, True])
def test_live_and_mirrored_sections_keep_all_map_anchors_and_have_forward_order(live_scene, mirrored):
    pair = _pair(live_scene, mirrored)
    e = live_scene['result']['corridor_evaluation']
    before = live_scene['result']['corridor_diagnostics']['envelope_failure']
    assert before['reason'] == 'NONCONVEX_QUAD' and before['quad_index'] == 44
    assert before['station_range_m'] == [7.499951351758102, 7.5]
    a, b = (pair.at(s) for s in before['station_range_m'])
    assert b['adjacent_s']-a['adjacent_s'] == pytest.approx(-.0014885412712732915)
    mesh = pair.corridor_envelope(*e['station_range_m'])
    assert mesh['failure'] is None
    assert len(mesh['quads']) == 82
    assert len(mesh['normalization']) == 40
    assert all(b > a for a, b in zip(mesh['stations'], mesh['stations'][1:]))
    adjacent_stations = [pair.at(s)['adjacent_s'] for s in mesh['stations']]
    assert all(b > a for a, b in zip(adjacent_stations, adjacent_stations[1:]))
    start, end = e['station_range_m']
    anchors = {start, end} | {s for s in pair.original_arc+pair.adjacent_stations if start < s < end}
    assert anchors <= set(mesh['stations'])
    assert 7.499951351758102 in mesh['stations']
    assert 7.5 not in mesh['stations']
    assert all(s['removed_grid_station_m'] not in anchors for s in mesh['normalization'])
    points = copy.deepcopy(e['points'])
    if mirrored:
        for p in points:
            p[0] *= -1
    assert corridor_covers(points, mesh['quads'])
    report = inspect_corridor(points, mesh['quads'], pair)
    assert report['safe'] and report['failed_point_count'] == 0
    assert report['reference_cross_section']['allowed_lateral_ranges_m']
    assert max(_distance(p, mesh['quads']) for p in points) <= 3e-12
    # A genuine 1 cm excursion past the real adjacent center still fails.
    a = pair.at(10.)
    sign = 1 if mirrored else -1
    p, t = a['adjacent']
    outside = [p[0]-sign*.01*t[1], p[1]+sign*.01*t[0]]
    assert not corridor_covers([outside, outside], mesh['quads'])


def test_same_live_snapshot_evaluates_geometry_and_preserves_blocked_release_confirmation(live_scene):
    result = replay_scene(live_scene)
    diagnostic = result['corridor_diagnostics']
    assert result['drivable_safe']
    assert diagnostic['safe'] and diagnostic['reason'] == 'COVERED'
    assert diagnostic['failed_point_count'] == 0
    assert diagnostic['reference_cross_section']['allowed_lateral_ranges_m'][0] == pytest.approx([-3.4534477400790533, 0.])
    assert diagnostic['envelope']['quad_count'] == 82
    assert result['entry_length'] == pytest.approx(11.186337379314498)
    assert result['maximum_curvature'] == pytest.approx(.15689383100216267)
    assert result['minimum_clearance_m'] == pytest.approx(2.8647450168873982)
    assert result['evaluated_candidate_offset'] == pytest.approx(-3.4792014971108647)
    assert result['usable_curvature_limit'] == live_scene['result']['usable_curvature_limit']
    assert result['blocked_reason'] == 'RELEASE_PENDING'
    # Replay the exact same stationary inputs. This is offline planning only.
    planner = planner_from_scene(live_scene)
    history = [planner.plan(**live_scene['inputs']) for _ in range(3)]
    assert [r['blocked_reason'] for r in history[:2]] == ['RELEASE_PENDING', 'RELEASE_PENDING']
    assert history[2]['status'] == 'AVOIDING'
    assert history[2]['lane_change_phase'] == 'ENTRY'
    assert history[2]['drivable_safe']
    assert all(r['maximum_curvature'] <= planner.usable_curvature_limit for r in history)


@pytest.mark.parametrize('start,end', [(0., 20.), (7.499951351758102, 7.5)])
def test_real_corner_is_preserved_and_crossing_horizon_caps_fail_closed(start, end):
    vertex = 7.499951351758102
    angle = .1
    original = [[0., 0., 0.], [vertex, 0., 0.],
                [vertex+20*math.cos(angle), 20*math.sin(angle), 0.]]
    adjacent = [[0., 3.5, 0.], [30., 3.5, 0.]]
    pair = LanePair(original, adjacent, [('O', 'A')], 'left', [27.5])
    mesh = pair.corridor_envelope(start, end)
    if start==0.:
        assert mesh['failure'] is None
        vertices={tuple(p) for cell in mesh['quads'] for p in cell}
        assert tuple(original[1][:2]) in vertices
        assert corridor_covers([original[1],original[1]],mesh['quads'])
        # Past the real original corner, y=0 is outside this left corridor.
        assert not corridor_covers([[10.,0.],[10.,0.]],mesh['quads'])
    else:
        assert not mesh['quads']
        assert mesh['failure']['reason']=='NONMONOTONIC_CROSS_SECTION'


def test_small_station_gap_does_not_remove_a_real_original_map_vertex():
    vertex = 7.49995
    angle = .00044
    original = [[0., 0., 0.], [vertex, 0., 0.],
                [vertex+.00005*math.cos(angle), .00005*math.sin(angle), 0.],
                [vertex+20*math.cos(angle), 20*math.sin(angle), 0.]]
    pair = LanePair(original, [[0., 3.5, 0.], [30., 3.5, 0.]], [('O', 'A')], 'left', [27.5])
    mesh = pair.corridor_envelope(0., 20.)
    assert set(pair.original_arc[1:3]) <= set(mesh['stations'])
    assert mesh['failure'] is None
    vertices={tuple(p) for cell in mesh['quads'] for p in cell}
    assert all(tuple(p[:2]) in vertices for p in original[1:3])


def test_actual_adjacent_corner_survives_removal_of_all_supplemental_grid():
    vertex = 7.49995
    angle = .00044
    original = [[0., 0., 0.], [vertex, 0., 0.],
                [vertex+20*math.cos(angle), 20*math.sin(angle), 0.]]
    adjacent = [[0., 3.5, 0.], [7.4992, 3.5001, 0.], [30., 3.5, 0.]]
    pair = LanePair(original, adjacent, [('O', 'A')], 'left', [27.5])
    mesh = pair.corridor_envelope(0., 20.)
    assert mesh['failure'] is None
    vertices={tuple(p) for cell in mesh['quads'] for p in cell}
    assert tuple(adjacent[1][:2]) in vertices
    assert tuple(original[1][:2]) in vertices
    assert not corridor_covers([[7.5,3.51],[7.5,3.51]],mesh['quads'])


def test_actual_crossing_centerlines_are_still_rejected():
    original = [[0., 0., 0.], [20., 0., 0.]]
    adjacent = [[0., 3.5, 0.], [10., -3.5, 0.], [20., 3.5, 0.]]
    pair = LanePair(original, adjacent, [('O', 'A')], 'left', [20.])
    mesh = pair.corridor_envelope(0., 20.)
    assert not mesh['quads']
    assert mesh['failure']['reason'] == 'CROSSING_BOUNDARIES'
