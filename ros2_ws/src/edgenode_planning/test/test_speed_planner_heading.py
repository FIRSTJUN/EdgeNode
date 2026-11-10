"""Physical heading support, path artifacts and explicit curve-state regressions."""

from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.speed_planner import SpeedPlanner, _heading_evidence, _heading_trace
from test_speed_planner import curvature_profile_path


SETTINGS = dict(cruise_speed_kmh=30.0, curve_speed_kmh=15.0, sharp_curve_speed_kmh=8.0,
                preview_distance_m=20.0, curve_curvature_threshold=0.025,
                sharp_curvature_threshold=0.07)


def recorded_curve():
    return json.loads((Path(__file__).parent / 'fixtures/kcity_curve_638.json').read_text())['points']


def slice_path(points, start, horizon=20.0):
    arcs = [0.0]
    for a, b in zip(points, points[1:]):
        arcs.append(arcs[-1] + math.dist(a[:2], b[:2]))
    end = min(arcs[-1], start + horizon)
    result = []
    for position in [start] + [s for s in arcs if start < s < end] + [end]:
        for i in range(len(arcs) - 1):
            if arcs[i] <= position <= arcs[i + 1] and arcs[i] < arcs[i + 1]:
                fraction = (position - arcs[i]) / (arcs[i + 1] - arcs[i])
                result.append([(1.0 - fraction) * a + fraction * b
                               for a, b in zip(points[i], points[i + 1])])
                break
    return result


def test_recorded_638_curve_survives_median_and_persistence_suppression():
    points = recorded_curve()
    before = deepcopy(points)
    result = SpeedPlanner(**SETTINGS).plan(points)
    assert result['legacy_representative_curvature'] < 1e-8
    assert result['max_curvature'] > 0.9
    assert result['heading_curvature'] >= 0.07
    assert result['speed_mode'] == 'SHARP_CURVE'
    assert result['target_speed_kmh'] == 8.0
    assert result['quality_rejected_samples'] == 0
    assert result['curve_state']
    assert points == before


@pytest.mark.parametrize('spacing', [0.1, 0.25, 0.5])
@pytest.mark.parametrize('height', [0.5, 1.0, 2.0])
def test_one_point_excursion_is_not_a_sharp_curve(spacing, height):
    points = [[i * spacing, 0.0, 0.0] for i in range(math.ceil(18.0 / spacing) + 1)]
    points[round(7.0 / spacing)][1] = height
    result = SpeedPlanner(**(SETTINGS | {'preview_distance_m': 10.0})).plan(points)
    assert result['max_curvature'] > 0.07
    assert result['speed_mode'] == 'CRUISE'
    assert result['target_speed_kmh'] == 30.0
    assert result['curve_state'] == []


@pytest.mark.parametrize('sigma', [0.25, 0.5, 1.0])
def test_rounded_isolated_excursion_does_not_get_widened_into_a_curve(sigma):
    points = [[i * 0.1, math.exp(-0.5 * ((i * 0.1 - 7.0) / sigma) ** 2), 0.0]
              for i in range(181)]
    result = SpeedPlanner(**(SETTINGS | {'preview_distance_m': 10.0})).plan(points)
    assert result['speed_mode'] == 'CRUISE'
    assert result['heading_curvature'] == 0.0


def test_zigzag_noise_is_rejected_without_erasing_raw_diagnostic():
    points = [[i * 0.25, (0.1 if i % 2 else -0.1) if 5 < i * 0.25 < 9 else 0.0, 0.0]
              for i in range(73)]
    result = SpeedPlanner(**SETTINGS).plan(points)
    assert result['max_curvature'] > 1.0
    assert result['quality_rejected_samples'] > 0
    assert result['speed_mode'] == 'CRUISE'


@pytest.mark.parametrize('spacing', [0.1, 0.25, 0.5, 1.0])
def test_s_curve_keeps_both_turn_directions(spacing):
    points = curvature_profile_path([(0.0, 3.0, spacing), (0.08, 4.0, spacing),
                                     (-0.08, 4.0, spacing), (0.0, 5.0, spacing)])
    result = SpeedPlanner(**SETTINGS).plan(points)
    reflected = SpeedPlanner(**SETTINGS).plan([[x, -y, z] for x, y, z in points])
    assert result['speed_mode'] == reflected['speed_mode'] == 'SHARP_CURVE'
    assert result['quality_rejected_samples'] == 0
    assert result['heading_curvature'] > 0.065
    for curvature, direction in [(0.08, 1), (-0.08, -1)]:
        lobe = curvature_profile_path([(0.0, 3.0, spacing), (curvature, 4.0, spacing),
                                      (0.0, 5.0, spacing)])
        cue = _heading_evidence(_heading_trace([p[:2] for p in lobe]))
        assert cue['direction'] == direction
        # This is a mean over observed tangent context, not the instantaneous 0.08.
        assert cue['curvature'] > 0.025


@pytest.mark.parametrize('profile', [
    [(0.2, 16.0, 0.25)],
    [(0.0, 2.0, 0.25), (0.2, 4.0, 0.25), (-0.2, 8.0, 0.25), (0.2, 4.0, 0.25)],
])
def test_broad_u_turn_and_chicane_are_not_treated_as_isolated_noise(profile):
    result = SpeedPlanner(**SETTINGS).plan(curvature_profile_path(profile))
    assert result['speed_mode'] == 'SHARP_CURVE'
    assert result['quality_rejected_samples'] == 0


def test_artifact_mask_does_not_erase_a_separate_real_curve():
    points = curvature_profile_path([(0.12, 4.0, 0.5), (0.0, 14.0, 0.5)])
    points[24][0] -= 2.0 * math.sin(0.48)
    points[24][1] += 2.0 * math.cos(0.48)
    result = SpeedPlanner(**SETTINGS).plan(points)
    assert result['quality_rejected_samples'] > 0
    assert result['quality_filtered_curvature'] >= 0.07
    assert result['speed_mode'] == 'SHARP_CURVE'


@pytest.mark.parametrize('length', [0.8, 2.0, 3.9])
def test_short_tail_does_not_gain_artificial_heading_support(length):
    points = curvature_profile_path([(0.12, length, 0.1)])
    result = SpeedPlanner(**SETTINGS).plan(points)
    assert result['evaluated_path_length_m'] < 4.0
    assert result['heading_curvature'] == 0.0
    assert result['curve_state'] == []
    # A valid legacy short curve is still a curve; there is no arbitrary speed-up.
    assert result['speed_mode'] == 'SHARP_CURVE'


def test_spatial_hold_does_not_delay_entry_and_releases_on_forward_straight():
    curve = recorded_curve()
    dx, dy = curve[1][0] - curve[0][0], curve[1][1] - curve[0][1]
    norm = math.hypot(dx, dy)
    prefix = [[curve[0][0] - dx / norm * s, curve[0][1] - dy / norm * s, curve[0][2]]
              for s in reversed([i * 0.5 for i in range(1, 41)])]
    dx2, dy2 = curve[-1][0] - curve[-2][0], curve[-1][1] - curve[-2][1]
    norm2 = math.hypot(dx2, dy2)
    suffix = [[curve[-1][0] + dx2 / norm2 * s, curve[-1][1] + dy2 / norm2 * s, curve[-1][2]]
              for s in [i * 0.5 for i in range(1, 41)]]
    route = prefix + curve + suffix
    planner = SpeedPlanner(**SETTINGS)
    state, previous = None, None
    modes, instantaneous = [], []
    active_seen = False
    for start in [i * 0.25 for i in range(201)]:
        path = slice_path(route, start)
        instant = planner.plan(path)
        before = deepcopy(state)
        result = planner.plan(path, previous_speed_mode=previous, previous_curve_state=state)
        assert state == before
        state, previous = result['curve_state'], result['speed_mode']
        active_seen |= bool(state)
        assert len(state) <= 8
        assert all(len(event['anchors']) <= 256 for event in state)
        modes.append(previous)
        instantaneous.append(instant['speed_mode'])
    assert active_seen
    assert next(i for i, mode in enumerate(modes) if mode != 'CRUISE') == next(
        i for i, mode in enumerate(instantaneous) if mode != 'CRUISE')
    assert sum(a != b for a, b in zip(modes, modes[1:])) <= 4
    assert modes[-1] == 'CRUISE'
    assert state == []


@pytest.mark.parametrize('state', [False, {}, [None],
    [{'strength': True, 'direction': 1, 'anchors': [[0.0, 0.0]]}],
    [{'strength': 2, 'direction': 1, 'anchors': [[math.nan, 0.0]]}],
])
def test_malformed_optional_history_fails_closed(state):
    assert SpeedPlanner(**SETTINGS).plan(recorded_curve(), previous_curve_state=state) is None
