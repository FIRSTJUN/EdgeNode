"""Static lane acquisition, immutable ENTRY and one-way centerline handoff."""
from dataclasses import FrozenInstanceError
import math
from unittest.mock import patch

import pytest

from edgenode_planning.lane_change_path import generate_shift_path, prepare_target_centerline
from edgenode_planning.lane_change_planner import LaneChangePlanner
from edgenode_planning.lane_change_types import (
    LaneAcquisitionConfig, LaneCenterlinePath, LaneChangeDirection,
    LaneChangeRequest, LaneChangeState,
)


def start(offset=-3.5, *, config=None, target=None):
    planner = LaneChangePlanner(acquisition_config=config)
    request = LaneChangeRequest('target', LaneChangeDirection.LEFT if offset > 0
                                else LaneChangeDirection.RIGHT, return_required=True)
    reference = [[float(x), 0.] for x in range(41)]
    target = [[float(x), offset] for x in range(41)] if target is None else target
    first = planner.start(request, reference, start_s_m=5., end_s_m=25.,
                          lateral_offset_m=offset, target_reference_points=target)
    return planner, first, reference, target


def confirm(planner, position, yaw=0.):
    for _ in range(planner.acquisition_config.confirmation_updates):
        result = planner.update(position, yaw)
    return result


@pytest.mark.parametrize('offset', [-3.5, 3.5])
def test_left_right_fixed_entry_sequence_acquires_before_nominal_endpoint(offset):
    planner, first, _, target = start(offset)
    entry = first.path
    snapshot = entry.points
    assert first.state is LaneChangeState.ENTRY
    for station in (5., 10., 15., 20., 23.5, 23.75, 24.):
        index = entry.reference_s_m.index(station)
        before, after = entry.points[index-1], entry.points[index+1]
        yaw = math.atan2(after[1]-before[1], after[0]-before[0])
        result = planner.update(entry.points[index], yaw)
        if station < 24.:
            assert result is first
            assert result.path.points is snapshot
        else:
            assert result.state is LaneChangeState.TARGET_LANE_HOLD
    assert planner.acquisition.reason == 'ACQUIRED'
    assert planner.acquisition.entry_remaining_m > 0  # Nominal end was not crossed.
    assert planner.acquisition.confirmation_count == 3
    assert planner.entry_path is entry
    assert planner.entry_path.points is snapshot
    assert isinstance(result.path, LaneCenterlinePath)
    assert result.path.points == tuple(tuple(p) for p in target)
    assert result.path is not entry
    assert result.request is first.request


@pytest.mark.parametrize('offset', [-3.5, 3.5])
@pytest.mark.parametrize('x,relative_y,yaw,reason', [
    (24., 0., math.radians(30.), 'TARGET_HEADING_ERROR'),
    (24., .3, 0., 'TARGET_LATERAL_DISTANCE'),
    (12., 0., 0., 'ENTRY_END_REGION'),
    (24., 0., math.pi, 'DIRECTION_MISMATCH'),
])
def test_geometry_failures_never_acquire(offset, x, relative_y, yaw, reason):
    planner, first, _, _ = start(offset)
    for _ in range(5):
        assert planner.update((x, offset+relative_y), yaw) is first
        assert planner.acquisition.reason == reason
        assert not planner.acquisition.eligible
        assert planner.acquisition.confirmation_count == 0


@pytest.mark.parametrize('offset', [-3.5, 3.5])
def test_reaching_entry_end_without_acquiring_target_is_not_hold(offset):
    planner, first, _, _ = start(offset)
    assert confirm(planner, (25., 0.)) is first
    assert planner.acquisition.reason == 'TARGET_LATERAL_DISTANCE'
    assert planner.acquisition.entry_remaining_m <= .01
    assert first.state is LaneChangeState.ENTRY


def test_a_transient_center_pass_and_broken_confirmation_do_not_latch_hold():
    planner, first, _, _ = start()
    assert planner.update((24., -3.5), 0.) is first
    assert planner.acquisition.confirmation_count == 1
    assert planner.update((24.1, -3.1), math.radians(20.)) is first
    assert planner.acquisition.confirmation_count == 0
    assert planner.update((24.2, -3.5), 0.) is first
    assert planner.update((24.3, -3.5), 0.) is first
    # Even two eligible samples cannot survive an intervening failed condition.
    assert planner.update((24.4, -3.5), math.radians(30.)) is first
    assert planner.acquisition.confirmation_count == 0
    assert planner.update((24.5, -3.5), 0.) is first
    assert planner.update((24.6, -3.5), 0.) is first
    assert planner.update((24.7, -3.5), 0.).state is LaneChangeState.TARGET_LANE_HOLD


@pytest.mark.parametrize('position,yaw', [
    (None, 0.), ((24., -3.5), None), ((24., -3.5), math.nan),
    ((24., math.inf), 0.), ((True, -3.5), 0.), ((24.,), 0.),
    ((1e308, -3.5), 0.),
])
def test_missing_or_invalid_sample_breaks_consecutive_confirmation(position, yaw):
    planner, first, _, _ = start()
    planner.update((24., -3.5), 0.)
    planner.update((24., -3.5), 0.)
    assert planner.acquisition.confirmation_count == 2
    assert planner.update(position, yaw) is first
    assert planner.acquisition.confirmation_count == 0
    assert planner.update((24., -3.5), 0.) is first
    assert planner.acquisition.confirmation_count == 1


def test_configured_distance_heading_progress_and_two_sample_confirmation():
    config = LaneAcquisitionConfig(max_lateral_distance_m=.05,
        max_heading_error_rad=math.radians(3.), entry_end_region_m=.5,
        min_entry_progress_fraction=.95, confirmation_updates=2)
    planner, first, _, _ = start(config=config)
    assert planner.acquisition_config is config
    assert planner.update((24., -3.5), 0.) is first
    assert planner.acquisition.reason == 'ENTRY_END_REGION'
    assert planner.update((24.75, -3.42), 0.) is first
    assert planner.acquisition.reason == 'TARGET_LATERAL_DISTANCE'
    assert planner.update((24.75, -3.48), math.radians(4.)) is first
    assert planner.acquisition.reason == 'TARGET_HEADING_ERROR'
    assert planner.update((24.75, -3.48), math.radians(2.)) is first
    assert planner.update((24.8, -3.48), math.radians(2.)).state is LaneChangeState.TARGET_LANE_HOLD


def test_minimum_fraction_prevents_early_acquisition_with_a_large_end_region():
    planner, first, _, _ = start(config=LaneAcquisitionConfig(entry_end_region_m=100.))
    assert confirm(planner, (12., -3.5)) is first
    assert planner.acquisition.entry_remaining_m < 100.
    assert planner.acquisition.entry_progress_fraction < .8
    assert planner.acquisition.reason == 'ENTRY_END_REGION'


def test_end_region_does_not_cover_the_start_of_a_short_small_offset_shift():
    planner = LaneChangePlanner()
    first = planner.start(LaneChangeRequest('close', LaneChangeDirection.LEFT),
        [(0., 0.), (10., 0.)], start_s_m=5., end_s_m=7., lateral_offset_m=.04,
        target_reference_points=[(0., .04), (10., .04)])
    assert first.state is LaneChangeState.ENTRY
    assert confirm(planner, (5., 0.)) is first
    assert planner.acquisition.reason == 'ENTRY_END_REGION'


@pytest.mark.parametrize('error', [-.05, .05])
def test_target_lateral_is_signed_and_heading_wraps(error):
    planner, first, _, _ = start()
    assert planner.update((24., -3.5+error), 2*math.pi) is first
    assert planner.acquisition.signed_lateral_distance_m == pytest.approx(error)
    assert planner.acquisition.heading_error_rad == pytest.approx(0., abs=1e-12)


def test_rotated_reference_uses_target_tangent_not_world_x():
    planner = LaneChangePlanner()
    first = planner.start(LaneChangeRequest('north', LaneChangeDirection.RIGHT),
        [(0., 0.), (0., 40.)], start_s_m=5., end_s_m=25., lateral_offset_m=-3.5,
        target_reference_points=[(3.5, 0.), (3.5, 40.)])
    assert first.state is LaneChangeState.ENTRY
    hold = confirm(planner, (3.55, 24.), math.pi/2)
    assert hold.state is LaneChangeState.TARGET_LANE_HOLD
    assert planner.acquisition.signed_lateral_distance_m == pytest.approx(-.05)
    assert planner.acquisition.heading_error_rad == pytest.approx(0.)


def test_progress_is_fixed_entry_xy_arc_not_original_reference_station():
    planner, first, _, _ = start()
    planner.update((25., -3.5), 0.)
    begin = first.path.reference_s_m.index(5.)
    end = first.path.reference_s_m.index(25.)
    expected = sum(math.dist(a, b) for a, b in
        zip(first.path.points[begin:end], first.path.points[begin+1:end+1]))
    assert expected > 20.
    assert planner.acquisition.entry_progress_m == pytest.approx(expected)
    assert planner.acquisition.entry_progress_fraction == pytest.approx(1.)
    assert planner.acquisition.entry_remaining_m == pytest.approx(0.)


def test_late_physical_acquisition_after_end_is_allowed():
    planner, _, _, _ = start()
    assert confirm(planner, (28., -3.45)).state is LaneChangeState.TARGET_LANE_HOLD
    assert planner.acquisition.entry_remaining_m < 0


def test_projection_cannot_extend_a_short_target_centerline():
    planner, first, _, _ = start(target=[(0., -3.5), (26., -3.5)])
    assert confirm(planner, (28., -3.5)) is first
    assert planner.acquisition.reason == 'TARGET_REFERENCE_RANGE'


@pytest.mark.parametrize('yaw', [0., math.pi])
def test_reversed_target_reference_cannot_override_entry_forward_direction(yaw):
    planner, first, _, _ = start(target=[(40., -3.5), (0., -3.5)])
    assert confirm(planner, (24., -3.5), yaw) is first
    assert planner.acquisition.reason == 'DIRECTION_MISMATCH'


def test_no_target_snapshot_cannot_acquire_from_pose_alone():
    planner = LaneChangePlanner()
    first = planner.start(LaneChangeRequest('target', LaneChangeDirection.RIGHT),
        [(0., 0.), (40., 0.)], start_s_m=5., end_s_m=25., lateral_offset_m=-3.5)
    assert confirm(planner, (25., -3.5)) is first
    assert planner.acquisition.reason == 'TARGET_REFERENCE_REQUIRED'


def test_entry_and_hold_paths_are_snapshots_and_never_regenerated():
    with patch('edgenode_planning.lane_change_planner.generate_shift_path',
               wraps=generate_shift_path) as entry_spy, \
         patch('edgenode_planning.lane_change_planner.prepare_target_centerline',
               wraps=prepare_target_centerline) as target_spy:
        planner, first, reference, target = start()
        entry_points = first.path.points
        target_points = tuple(tuple(p) for p in target)
        reference[0][1] = 100.
        target[24][1] = 100.
        for cycle in range(100):
            assert planner.update((10.+cycle*.01, -.02), .01) is first
            assert planner.entry_path.points is entry_points
            assert planner.start(first.request, None, end_s_m=1., lateral_offset_m=99.,
                                 target_reference_points=[]) is first
        hold = confirm(planner, (24., -3.5))
        assert hold.state is LaneChangeState.TARGET_LANE_HOLD
        assert hold.path.points == target_points
        hold_points = hold.path.points
        with pytest.raises(FrozenInstanceError):
            hold.path.points = ()
        with pytest.raises(TypeError):
            hold.path.points[0][0] = 1.
        for cycle in range(100):
            assert planner.update((24.+cycle*.01, -3.5+.01*math.sin(cycle)), .01) is hold
            assert planner.result.path.points is hold_points
            assert planner.entry_path.points is entry_points
            assert planner.start(first.request, None, end_s_m=1., lateral_offset_m=99.) is hold
        assert planner.update() is hold
        entry_spy.assert_called_once()
        target_spy.assert_called_once()
    with pytest.raises(RuntimeError):
        planner.start(LaneChangeRequest('other', LaneChangeDirection.LEFT), None,
                      end_s_m=25., lateral_offset_m=3.5)
    assert planner.result is hold


def test_reset_clears_hold_and_confirmation_for_a_new_maneuver():
    planner, _, _, _ = start()
    old_hold = confirm(planner, (24., -3.5))
    old_entry = planner.entry_path
    planner.reset()
    assert planner.result.state is LaneChangeState.IDLE
    assert planner.entry_path is None
    assert planner.acquisition.confirmation_count == 0
    new = planner.start(LaneChangeRequest('left', LaneChangeDirection.LEFT),
        [(0., 0.), (40., 0.)], start_s_m=5., end_s_m=25., lateral_offset_m=3.5,
        target_reference_points=[(0., 3.5), (40., 3.5)])
    assert new.path is not old_entry
    assert planner.update((24., 3.5), 0.) is new
    assert planner.acquisition.confirmation_count == 1
    hold = confirm(planner, (24., 3.5))
    assert hold.state is LaneChangeState.TARGET_LANE_HOLD
    assert hold.path is not old_hold.path
    assert hold.target_link_id == 'left'


@pytest.mark.parametrize('target,reason', [
    ([], 'INVALID_TARGET_REFERENCE'),
    ([(0., -3.5)], 'INVALID_TARGET_REFERENCE'),
    ([(0., -3.5), (math.nan, -3.5)], 'INVALID_TARGET_REFERENCE'),
    ([(0., -3.5), (1., -3.5), (1., -2.5)], 'TARGET_CURVATURE_LIMIT'),
])
def test_invalid_hold_geometry_blocks_without_exposing_a_partial_entry(target, reason):
    planner, result, _, _ = start(target=target)
    assert result.state is LaneChangeState.BLOCKED
    assert result.path is None
    assert planner.entry_path is None
    assert result.blocked_reason == reason
    assert planner.update((24., -3.5), 0.) is result


def test_target_xyz_centerline_is_copied_as_xy_without_new_shift_generation():
    target = [[0., -3.5, 1.], [20., -3.5, 1.], [40., -3.5, 1.]]
    planner, _, _, _ = start(target=target)
    hold = confirm(planner, (24., -3.5))
    assert hold.path.points == ((0., -3.5), (20., -3.5), (40., -3.5))


@pytest.mark.parametrize('changes', [
    dict(max_lateral_distance_m=0.), dict(max_lateral_distance_m=math.inf),
    dict(max_lateral_distance_m=True), dict(max_heading_error_rad=0.),
    dict(max_heading_error_rad=math.pi/2), dict(max_heading_error_rad=math.nan),
    dict(entry_end_region_m=-1.), dict(entry_end_region_m=math.inf),
    dict(min_entry_progress_fraction=0.), dict(min_entry_progress_fraction=1.),
    dict(confirmation_updates=1), dict(confirmation_updates=2.),
    dict(confirmation_updates=True),
])
def test_invalid_acquisition_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        LaneAcquisitionConfig(**changes)


def test_configuration_type_is_explicit():
    with pytest.raises(TypeError):
        LaneChangePlanner(acquisition_config={'max_lateral_distance_m': .15})
