"""One-shot constant-jerk geometry and fixed-path ownership; no ROS or S4 gates."""
from dataclasses import FrozenInstanceError
import math
from unittest.mock import patch

import pytest

from edgenode_planning.lane_change_path import (
    DEFAULT_CURVATURE_LIMIT, LaneChangePathError, constant_jerk_profile,
    generate_shift_path, steering_curvature_limit,
)
from edgenode_planning.lane_change_planner import LaneChangePlanner
from edgenode_planning.lane_change_types import (
    LaneChangeDirection, LaneChangeRequest, LaneChangeState,
)
from edgenode_planning.lattice_planner import LatticePlanner


def straight():
    return [[float(x), 0.0] for x in range(41)]


def request(direction=LaneChangeDirection.RIGHT):
    return LaneChangeRequest('target_link', direction, return_required=True, reason='unit test')


@pytest.mark.parametrize('offset', [-4.0, -3.5, -3.0, 3.0, 3.5, 4.0])
def test_left_right_zero_start_terminal_offset_and_forward_progress(offset):
    reference=straight()
    path=generate_shift_path(reference, start_s_m=5., end_s_m=25., lateral_offset_m=offset)
    assert reference==straight()
    assert path.points[0]==(0.,0.)
    assert path.points[-1]==(40.,offset)
    assert path.lateral_offsets_m[path.reference_s_m.index(5.)]==0.
    for s,xy,shift in zip(path.reference_s_m,path.points,path.lateral_offsets_m):
        assert xy==pytest.approx((s,shift))
        assert all(math.isfinite(v) for v in xy)
        if s<=5.:assert shift==0.
        if s>=25.:assert shift==offset
    assert all(b[0]>a[0] for a,b in zip(path.points,path.points[1:]))
    assert all(a<b for a,b in zip(path.reference_s_m,path.reference_s_m[1:]))
    assert path.maximum_curvature_m_inv<=path.shift_second_derivative_bound_m_inv
    assert path.shift_second_derivative_bound_m_inv<=DEFAULT_CURVATURE_LIMIT


@pytest.mark.parametrize('u,value', [(0.,0.),(.25,1/12),(.5,.5),(.75,11/12),(1.,1.)])
def test_profile_matches_autoware_base_knots(u,value):
    shift,_,_=constant_jerk_profile(u)
    assert shift==pytest.approx(value,abs=1e-14)


@pytest.mark.parametrize('u', [0.,.25,.5,.75,1.])
def test_profile_is_C2_across_all_joins_and_has_flat_end_conditions(u):
    epsilon=1e-8
    center=constant_jerk_profile(u)
    for other in (constant_jerk_profile(u-epsilon),constant_jerk_profile(u+epsilon)):
        assert other==pytest.approx(center,abs=4e-7)
    if u in (0.,1.):assert center[1:]==(0.,0.)


@pytest.mark.parametrize('u,jerk', [(.1,32.),(.4,-32.),(.6,-32.),(.9,32.)])
def test_piecewise_spatial_jerk_is_constant(u,jerk):
    h=1e-5
    measured=(constant_jerk_profile(u+h)[2]-constant_jerk_profile(u-h)[2])/(2*h)
    assert measured==pytest.approx(jerk,abs=1e-8)


def test_pose_selects_longitudinal_start_without_refitting_pose_or_yaw():
    path=generate_shift_path(straight(),start_position=[5.2,.02],end_s_m=25.2,lateral_offset_m=-3.5)
    assert path.start_s_m==pytest.approx(5.2)
    assert path.lateral_offsets_m[path.reference_s_m.index(path.start_s_m)]==0.
    assert path.points[path.reference_s_m.index(path.start_s_m)]==pytest.approx((5.2,0.))


def test_rotated_XY_reference_uses_forward_left_normal():
    path=generate_shift_path([[0.,float(y)] for y in range(41)],
        start_s_m=5.,end_s_m=25.,lateral_offset_m=-3.5)
    assert path.points[-1]==pytest.approx((3.5,40.))
    assert all(b[1]>a[1] for a,b in zip(path.points,path.points[1:]))


def test_gently_curved_polyline_and_miter_join_remain_connected():
    radius=200.
    reference=[[radius*math.sin(i/radius),radius*(1-math.cos(i/radius))] for i in range(61)]
    path=generate_shift_path(reference,start_s_m=5.,end_s_m=30.,lateral_offset_m=-3.5)
    assert path.maximum_curvature_m_inv<DEFAULT_CURVATURE_LIMIT
    assert all(0<math.dist(a,b)<.3 for a,b in zip(path.points,path.points[1:]))
    assert len(path.points)==len(path.reference_s_m)==len(path.lateral_offsets_m)


def test_xyz_mgeo_samples_are_normalized_to_immutable_XY():
    reference=[[float(x),0.,-1.] for x in range(41)]
    path=generate_shift_path(reference,start_s_m=5.,end_s_m=25.,lateral_offset_m=3.5)
    reference[-1][1]=100.
    assert path.points[-1]==(40.,3.5)
    with pytest.raises(TypeError):path.points[0][0]=1.
    with pytest.raises(FrozenInstanceError):path.start_s_m=0.


@pytest.mark.parametrize('span', [.1,1.,5.,10.])
def test_short_transition_is_rejected_even_from_two_sparse_reference_points(span):
    with pytest.raises(LaneChangePathError) as failure:
        generate_shift_path([[0.,0.],[40.,0.]],start_s_m=5.,end_s_m=5.+span,lateral_offset_m=-3.5)
    assert failure.value.reason=='TRANSITION_TOO_SHORT'


@pytest.mark.parametrize('changes', [dict(start_s_m=-1.),dict(end_s_m=41.),
    dict(end_s_m=5.),dict(lateral_offset_m=0.),dict(lateral_offset_m=math.nan),
    dict(start_s_m=math.inf),dict(sample_spacing_m=0.),dict(sample_spacing_m=1.),
    dict(curvature_limit_m_inv=math.inf),dict(curvature_limit_m_inv=False)])
def test_invalid_shift_inputs_fail_closed(changes):
    values=dict(start_s_m=5.,end_s_m=25.,lateral_offset_m=-3.5)
    values.update(changes)
    with pytest.raises(LaneChangePathError):generate_shift_path(straight(),**values)


@pytest.mark.parametrize('reference', [[],[[0.,0.]],[[0.,0.],[0.,0.]],
    [[0.,0.],[math.nan,1.]],[[0.,0.],[1.,math.inf]],[[0.,0.],[True,1.]]])
def test_invalid_reference_inputs_are_rejected(reference):
    with pytest.raises(LaneChangePathError):
        generate_shift_path(reference,start_s_m=0.,end_s_m=20.,lateral_offset_m=3.5)


def test_reversal_and_sampling_resource_limits_are_rejected():
    with pytest.raises(LaneChangePathError,match='reverses'):
        generate_shift_path([[0.,0.],[20.,0.],[0.,0.]],start_s_m=0.,end_s_m=20.,lateral_offset_m=3.5)
    with pytest.raises(LaneChangePathError) as failure:
        generate_shift_path([[0.,0.],[1e308,0.]],start_s_m=0.,end_s_m=1e308,lateral_offset_m=3.5)
    assert failure.value.reason=='RESOURCE_LIMIT'


@pytest.mark.parametrize('reference,end,reason', [
    ([[0.,0.],[20.,0.],[20.,20.],[40.,20.]],40.,'CURVATURE_LIMIT'),
    ([[0.,0.],[30.,0.],[31.,0.],[31.,1.],[31.,30.]],30.,'DIRECTION_CONTINUITY'),
])
def test_native_corners_and_offset_folding_are_not_hidden_by_a_long_shift(reference,end,reason):
    with pytest.raises(LaneChangePathError) as failure:
        generate_shift_path(reference,start_s_m=0.,end_s_m=end,lateral_offset_m=3.5)
    assert failure.value.reason==reason


def test_sonata_command_geometry_matches_existing_planning_limit():
    assert steering_curvature_limit()==pytest.approx(LatticePlanner().usable_curvature_limit)
    path=generate_shift_path(straight(),start_s_m=5.,end_s_m=25.,lateral_offset_m=4.)
    wheel_angle=math.atan(2.84*path.maximum_curvature_m_inv)
    assert wheel_angle<=.70*.610865


def test_same_maneuver_never_regenerates_or_mutates_the_path_across_cycles():
    planner=LaneChangePlanner();intent=request();reference=straight()
    from edgenode_planning.lane_change_planner import generate_shift_path as generate
    with patch('edgenode_planning.lane_change_planner.generate_shift_path',wraps=generate) as spy:
        first=planner.start(intent,reference,start_s_m=5.,end_s_m=25.,lateral_offset_m=-3.5)
        assert first.state is LaneChangeState.ENTRY
        snapshot=first.path.points
        reference[10][1]=100.
        for cycle in range(100):
            assert planner.update([5.+cycle*.1,.01*math.sin(cycle)]) is first
            assert planner.start(intent,None,start_s_m=7.,end_s_m=30.,lateral_offset_m=-4.) is first
            assert planner.result.path is first.path
            assert first.path.points is snapshot
        spy.assert_called_once()
    assert first.target_link_id=='target_link'
    assert first.request.return_required
    assert first.state is LaneChangeState.ENTRY  # No premature HOLD/completion inference.


def test_new_request_requires_explicit_reset_and_blocked_attempts_are_sticky():
    planner=LaneChangePlanner();intent=request()
    blocked=planner.start(intent,straight(),start_s_m=5.,end_s_m=6.,lateral_offset_m=-3.5)
    assert blocked.state is LaneChangeState.BLOCKED
    assert blocked.path is None
    assert blocked.blocked_reason=='TRANSITION_TOO_SHORT'
    assert planner.start(intent,straight(),start_s_m=5.,end_s_m=25.,lateral_offset_m=-3.5) is blocked
    with pytest.raises(RuntimeError):
        planner.start(request(LaneChangeDirection.LEFT),straight(),start_s_m=5.,end_s_m=25.,lateral_offset_m=3.5)
    assert planner.result is blocked
    planner.reset()
    assert planner.result.state is LaneChangeState.IDLE
    assert planner.start(intent,straight(),start_s_m=5.,end_s_m=25.,lateral_offset_m=-3.5).state is LaneChangeState.ENTRY


def test_request_direction_must_match_the_offset():
    result=LaneChangePlanner().start(request(),straight(),start_s_m=5.,end_s_m=25.,lateral_offset_m=3.5)
    assert result.state is LaneChangeState.BLOCKED
    assert result.blocked_reason=='DIRECTION_MISMATCH'


@pytest.mark.parametrize('kwargs', [dict(target_link_id=''),dict(direction='RIGHT'),dict(return_required=1)])
def test_request_types_reject_ambiguous_intent(kwargs):
    values=dict(target_link_id='target',direction=LaneChangeDirection.RIGHT)
    values.update(kwargs)
    with pytest.raises(ValueError):LaneChangeRequest(**values)
