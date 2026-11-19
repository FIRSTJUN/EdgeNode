from dataclasses import replace
from test_static_obstacle_planner import setup


def test_reference_corner_does_not_replace_actual_controller_feasibility():
    planner,point,_=setup()
    planner.config=replace(planner.config,continuous_lane_follow=True,avoid_speed_kmh=10.,native_reference_rollout=True)
    path=[point(i*.5,0.) for i in range(101)]
    path[30]=point(15.,.03)
    assert planner._path_safety(path,(),10.)=='CURVATURE'
    assert planner._path_safety(path,(),10.,native_reference=True)==''
    assert planner._rollout(path,(0.,0.,0.),10.,0.,(),duration_s=9.)==''
    impossible=list(path);impossible[30]=point(15.,3.)
    assert planner._rollout(impossible,(0.,0.,0.),10.,0.,(),duration_s=9.)


def test_map_cache_context_cannot_cross_full_return_boundary_modes():
    planner,_,_=setup();before=planner._geometry_context()
    planner._complete_return_proof_active=True
    assert planner._geometry_context()!=before
