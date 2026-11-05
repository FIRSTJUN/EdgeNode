"""Phase 3 gate: one fixed return, shared acquisition and latched COMPLETE."""
import math
from unittest.mock import patch

import pytest

from edgenode_planning.lane_change_path import generate_shift_path
from edgenode_planning.lane_change_planner import LaneChangePlanner
from edgenode_planning.lane_change_types import (
    LaneChangeDirection, LaneChangeRequest, LaneChangeState,
)


def lane(y):
    return [(float(x), y) for x in range(91)]


def intent(link, offset, returning=False):
    return LaneChangeRequest(link, LaneChangeDirection.LEFT if offset > 0
        else LaneChangeDirection.RIGHT, return_required=not returning, reason='RETURN' if returning else 'TEST')


def acquired_hold(offset=3.5):
    planner = LaneChangePlanner()
    entry = planner.start(intent('adjacent', offset), lane(0.), start_s_m=5.,
        end_s_m=25., lateral_offset_m=offset, target_reference_points=lane(offset))
    for x in (23.5, 23.75, 24.):
        result = planner.update((x, offset), 0.)
    assert result.state is LaneChangeState.TARGET_LANE_HOLD
    return planner, entry


def return_path(planner, offset=3.5, end=55.):
    return planner.request_return(intent('original', -offset, True), lane(offset),
        start_s_m=35., end_s_m=end, lateral_offset_m=-offset, target_reference_points=lane(0.))


@pytest.mark.parametrize('offset', [3.5, -3.5])
def test_full_primitive_lifecycle_with_one_entry_and_one_recovery(offset):
    with patch('edgenode_planning.lane_change_planner.generate_shift_path',
               wraps=generate_shift_path) as generate:
        planner, entry = acquired_hold(offset)
        recovery = return_path(planner, offset)
        assert recovery.state is LaneChangeState.RECOVERING
        assert planner.entry_path is entry.path
        assert planner.recovery_path is recovery.path
        assert recovery.path.points[-1] == (90., 0.)
        assert recovery.path.maximum_curvature_m_inv <= planner.curvature_limit_m_inv
        snapshot = recovery.path.points
        for cycle in range(100):
            assert planner.update((36.+cycle*.01, offset+.01*math.sin(cycle)), .01) is recovery
            assert planner.recovery_path.points is snapshot
            assert planner.request_return(recovery.request, None, end_s_m=1.,
                lateral_offset_m=99., target_reference_points=None) is recovery
        for x in (53.5, 53.75, 54.):
            complete = planner.update((x, 0.), 0.)
        assert complete.state is LaneChangeState.COMPLETE
        assert complete.target_link_id == 'original'
        assert complete.path.points == tuple(lane(0.))
        assert planner.recovery_path is recovery.path
        assert planner.acquisition.transition_remaining_m > 0
        for cycle in range(100):
            assert planner.update((54.+cycle*.01, .01*math.sin(cycle)), .01) is complete
            assert planner.request_return(recovery.request, None, end_s_m=1.,
                lateral_offset_m=99., target_reference_points=None) is complete
        assert generate.call_count == 2
        assert [call.kwargs['lateral_offset_m'] for call in generate.call_args_list] == [offset, -offset]


@pytest.mark.parametrize('position,yaw,reason', [
    ((54., 0.), math.radians(30.), 'TARGET_HEADING_ERROR'),
    ((54., .3), 0., 'TARGET_LATERAL_DISTANCE'),
    ((54., 0.), math.pi, 'DIRECTION_MISMATCH'),
    ((40., 0.), 0., 'ENTRY_END_REGION'),
    ((55., 3.5), 0., 'TARGET_LATERAL_DISTANCE'),
])
def test_recovery_cannot_complete_from_progress_or_distance_alone(position, yaw, reason):
    planner, _ = acquired_hold()
    recovery = return_path(planner)
    for _ in range(5):
        assert planner.update(position, yaw) is recovery
        assert planner.acquisition.reason == reason
        assert planner.acquisition.confirmation_count == 0


def test_transient_original_center_pass_and_interrupted_confirmation():
    planner, _ = acquired_hold()
    recovery = return_path(planner)
    assert planner.update((54., 0.), 0.) is recovery
    assert planner.update((54.1, .3), .3) is recovery
    assert planner.acquisition.confirmation_count == 0
    assert planner.update((54.2, 0.), 0.) is recovery
    assert planner.update((54.3, 0.), 0.) is recovery
    assert planner.update((54.4, 0.), None) is recovery
    assert planner.acquisition.confirmation_count == 0
    assert planner.update((54.5, 0.), 0.) is recovery
    assert planner.update((54.6, 0.), 0.) is recovery
    assert planner.update((54.7, 0.), 0.).state is LaneChangeState.COMPLETE


def test_short_return_blocks_and_repeated_request_cannot_retry_geometry():
    planner, entry = acquired_hold()
    with patch('edgenode_planning.lane_change_planner.generate_shift_path',
               wraps=generate_shift_path) as generate:
        blocked = return_path(planner, end=36.)
        assert blocked.state is LaneChangeState.BLOCKED
        assert blocked.blocked_reason == 'TRANSITION_TOO_SHORT'
        assert return_path(planner) is blocked
        assert planner.update((54., 0.), 0.) is blocked
        assert planner.entry_path is entry.path
        assert planner.recovery_path is None
        generate.assert_called_once()


def test_return_requires_explicit_hold_and_no_automatic_return_from_metadata():
    planner = LaneChangePlanner()
    with pytest.raises(RuntimeError, match='TARGET_LANE_HOLD'):
        return_path(planner)
    first = planner.start(intent('adjacent', 3.5), lane(0.), start_s_m=5.,
        end_s_m=25., lateral_offset_m=3.5, target_reference_points=lane(3.5))
    with pytest.raises(RuntimeError, match='TARGET_LANE_HOLD'):
        return_path(planner)
    assert planner.result is first
    for x in (23.5, 23.75, 24.):
        hold = planner.update((x, 3.5), 0.)
    for _ in range(100):
        assert planner.update((60., 3.5), 0.) is hold
    assert planner.recovery_path is None


def test_missing_return_target_blocks_and_different_request_needs_reset():
    planner, _ = acquired_hold()
    blocked = planner.request_return(intent('original', -3.5, True), lane(3.5),
        start_s_m=35., end_s_m=55., lateral_offset_m=-3.5, target_reference_points=None)
    assert blocked.state is LaneChangeState.BLOCKED
    assert blocked.blocked_reason == 'INVALID_TARGET_REFERENCE'
    with pytest.raises(RuntimeError, match='reset'):
        planner.request_return(intent('different', -3.5, True), lane(3.5),
            start_s_m=35., end_s_m=55., lateral_offset_m=-3.5, target_reference_points=lane(0.))


def test_reset_allows_an_independent_second_full_maneuver():
    planner, _ = acquired_hold()
    old_return = return_path(planner)
    for x in (53.5, 53.75, 54.):
        planner.update((x, 0.), 0.)
    assert planner.result.state is LaneChangeState.COMPLETE
    planner.reset()
    assert planner.recovery_path is None and planner.entry_path is None
    assert planner.acquisition.confirmation_count == 0
    planner.start(intent('second', -3.5), lane(0.), start_s_m=5., end_s_m=25.,
        lateral_offset_m=-3.5, target_reference_points=lane(-3.5))
    for x in (23.5, 23.75, 24.):
        planner.update((x, -3.5), 0.)
    assert return_path(planner, -3.5).path is not old_return.path
    for x in (53.5, 53.75, 54.):
        result = planner.update((x, 0.), 0.)
    assert result.state is LaneChangeState.COMPLETE
