"""Spatial smoothing preserves raw error, bounds the target and has no time state."""
import math
import pytest
from edgenode_control.pure_pursuit import PurePursuit

PP=PurePursuit()
SPIKE=[[0.,0.,0.],[3.,0.,0.],[3.2,.1,0.],[3.4,0.,0.],[12.,0.,0.]]


def test_short_geometry_excursion_is_smoothed_with_raw_projection_unchanged():
    old=PP.compute(SPIKE,0.,0.,0.,3.2)
    new=PP.compute(SPIKE,0.,0.,0.,3.2,.5,.1)
    assert abs(new['target_left_m'])<abs(old['target_left_m'])*.5
    assert new['nearest_distance_m']==old['nearest_distance_m']
    assert new['current_arc_m']==old['current_arc_m']
    assert new['raw_target_point']==old['target_point']
    assert 0<new['target_smoothing_offset_m']<=.1


def test_offset_bound_and_input_immutability():
    before=[p[:] for p in SPIKE]
    new=PP.compute(SPIKE,0.,0.,0.,3.2,.5,.02)
    assert math.dist(new['target_point'][:2],new['raw_target_point'][:2])==pytest.approx(.02)
    assert SPIKE==before


def test_moving_local_origin_cannot_jump_the_smoothed_world_target():
    first=[[0.,0.,0.],[4.,0.,0.],[5.,1.,0.],[8.,2.,0.],[20.,2.,0.]]
    shifted=[[.7,0.,0.]]+first[1:]
    a=PP.compute(first,1.,0.,0.,4.,.5,.1)
    b=PP.compute(shifted,1.,0.,0.,4.,.5,.1)
    assert a['target_point']==pytest.approx(b['target_point'],abs=1e-12)


def test_straight_high_speed_and_near_terminal_keep_original_geometry():
    for ld in [3.,6.25,7.5,10.]:
        r=PP.compute([[0.,0.,0.],[100.,0.,0.]],0.,0.,0.,ld,.5,.1)
        assert r['target_point']==pytest.approx([ld,0.,0.])
        assert r['steering_normalized']==0.
    short=[[0.,0.,0.],[2.,.5,0.]]
    assert PP.compute(short,0.,0.,0.,3.,.5,.1)['target_point']==short[-1]


def test_constant_radius_does_not_gain_large_tracking_bias():
    radius=17.7
    points=[[radius*math.sin(s/radius),radius*(1-math.cos(s/radius)),0.] for s in [i*.25 for i in range(101)]]
    a=PP.compute(points,0.,0.,0.,3.)
    b=PP.compute(points,0.,0.,0.,3.,.5,.1)
    assert abs(a['steering_normalized']-b['steering_normalized'])<.01
    assert b['nearest_distance_m']==0.


def test_duplicate_endpoint_at_smoothing_stencil_boundary_is_valid():
    path=[[0.,0.,0.],[4.5,0.,0.],[4.5,0.,2.]]
    r=PP.compute(path,0.,0.,0.,3.,.5,.1)
    assert r is not None
    assert r['target_point'][:2]==pytest.approx([3.,0.])


def test_large_steering_request_keeps_the_exact_raw_target():
    # A tight corner must retain the original response rather than round inside it.
    path=[[0.,0.,0.],[1.,0.,0.],[1.,8.,0.]]
    raw=PP.compute(path,0.,0.,0.,3.)
    assert abs(raw['steering_normalized'])>.70
    smooth=PP.compute(path,0.,0.,0.,3.,.5,.1)
    assert smooth['target_point']==raw['target_point']
    assert smooth['steering_normalized']==raw['steering_normalized']
    assert smooth['target_smoothing_offset_m']==0.


def test_support_crossing_heading_retains_raw_response_before_sign_reversal():
    path=[[0.,0.,0.],[2.,.3,0.],[3.,.1,0.],[4.,-.6,0.],[8.,-.6,0.]]
    raw=PP.compute(path,0.,0.,0.,3.)
    smooth=PP.compute(path,0.,0.,0.,3.,.5,.1)
    assert 0 < raw['steering_normalized'] < .55
    assert smooth['target_point']==raw['target_point']
    assert smooth['target_smoothing_offset_m']==0.


@pytest.mark.parametrize('sigma,bound',[(-1.,.1),(math.nan,.1),(math.inf,.1),(True,.1),(.5,-.1),(.5,.21),(.5,math.nan)])
def test_invalid_smoothing_parameters_fail_closed(sigma,bound):
    assert PP.compute(SPIKE,0.,0.,0.,3.2,sigma,bound) is None
