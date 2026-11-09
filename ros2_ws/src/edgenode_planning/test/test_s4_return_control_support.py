"""Return must remain executable until acquisition, with genuine map support."""
import math

import pytest

from avoidance_world import world
from edgenode_planning.avoidance_geometry import DrivableArea
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner


def candidates(*, road_end=140., area_end=None, speed=3.):
    network, area, point, obstacle = world()
    if road_end != 140.:
        for link in network.links.values():
            if link['idx'].endswith('1'):
                link['points'] = [p for p in link['points'] if p[0] <= road_end]
    if area_end is not None:
        area = DrivableArea((((-10., -1.75), (area_end, -1.75),
                              (area_end, 5.25), (-10., 5.25)),), 'synthetic truncated survey')
    planner = StaticObstaclePlanner(network, drivable_area=area)
    pair = network.pair('O0', 'left', ['O0', 'O1'])
    planner._remember([obstacle(55., 59.)])
    horizon = min(90., pair.length-planner.body.wheelbase_m-planner.body.front_overhang_m-1.)
    return planner, pair, planner._generate(pair, (0., 0., 0.), speed, 0., 65., 69., horizon)


def test_short_preview_uses_only_validated_real_original_tail():
    planner, pair, rows = candidates()
    accepted = [c for c in rows if not c.rejection]
    assert accepted
    assert any(c.return_preview_extension_m > 0 for c in accepted)
    for row in accepted:
        end = pair.original_index.project(row.return_path[-1])
        assert end >= row.return_end_s + row.return_control_reserve_m - 1e-7
        assert end <= pair.length - planner.body.wheelbase_m - planner.body.front_overhang_m - 1.
        tail = [p for p in row.return_path if pair.original_index.project(p) >= 90.]
        for p in tail:
            station = pair.original_index.project(p)
            assert math.dist(p[:2], (station-10., 0.)) < 1e-7
        assert planner._path_safety(row.points, planner.remembered, 3.) == ''


def test_map_shortage_is_distinct_from_preview_truncation():
    planner, pair, rows = candidates(road_end=83.)
    assert all(c.rejection for c in rows)
    assert any(c.rejection == 'RETURN_CONTROL_SUPPORT' for c in rows)
    # No real map vertex was moved to make a candidate pass.
    assert max(p[0] for p in pair.original) == 83.


def test_appended_real_tail_still_requires_whole_body_boundary_coverage():
    planner, pair, rows = candidates(area_end=84.)
    assert all(c.rejection for c in rows)
    assert any(c.return_preview_extension_m > 0 and c.rejection == 'DRIVABLE_AREA' for c in rows)


@pytest.mark.parametrize('speed,minimum', [(3., 6.), (10., 14.), (20., 25.)])
def test_control_reserve_tracks_unchanged_rollout_speed(speed, minimum):
    planner, _, _ = candidates()
    assert planner._rollout_distance(speed) > minimum
    if speed == 20.:
        assert planner._rollout_distance(speed) > planner._control_horizon_m


def test_extended_map_does_not_rescue_short_actual_control_window():
    planner, pair, _ = candidates()
    planner._control_horizon_m = 5.
    assert planner._rollout(pair.original, (0., 0., 0.), 3., 0., ()) == 'SHORT_CONTROL_HORIZON'
