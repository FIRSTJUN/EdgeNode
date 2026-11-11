"""Return support is evaluated at STEP 4, independently from entry."""
import pytest
from test_static_obstacle_planner import setup,plan,enter,acquire


def test_return_tail_uses_saved_global_vertices_after_hold():
    p,point,obstacle=setup();enter(p,point,obstacle);acquire(p,point,3.5)
    plan(p,point,[],s=45.,offset=3.5,now=5.)
    result=plan(p,point,[],s=45.,offset=3.5,now=5.1)
    assert result['return_path'] and p.return_commits==1
    # Saved route begins at x=-10, so its station is x+10.
    assert result['return_path'][-1][0]+10.>=p.selected.return_end_s+p._rollout_distance(3.)-1e-7


def test_short_saved_global_path_defers_return_and_keeps_adjacent_lane():
    p,point,obstacle=setup();enter(p,point,obstacle);acquire(p,point,3.5)
    p._global_path=tuple(point(float(x),0.) for x in range(-10,55))
    plan(p,point,[],s=45.,offset=3.5,now=5.)
    result=plan(p,point,[],s=45.,offset=3.5,now=5.1)
    assert not result['blocked'] and result['state']=='RETURN' and not result['return_path']
    assert p.return_commits==0 and all(c.rejection=='RETURN_CONTROL_SUPPORT' for c in p.candidates)


@pytest.mark.parametrize('speed,minimum',[(3.,6.),(10.,14.),(20.,25.)])
def test_control_reserve_tracks_unchanged_rollout_speed(speed,minimum):
    p,_,_=setup()
    assert p._rollout_distance(speed)>minimum
