from dataclasses import replace
import math
from edgenode_planning.entry_search import matched_approach_prefix
from edgenode_planning.lattice_path import center_suffix,curvatures
from edgenode_planning.lane_geometry import sample
from test_static_obstacle_planner import setup


def test_measured_quintic_bridge_removes_native_cap_kink_without_waiving_curvature_or_obstacles():
    planner,point,obstacle=setup()
    planner.config=replace(planner.config,continuous_lane_follow=True,avoid_speed_kmh=10.,steering_slew_limit_radps=.6)
    guide=[point(i*.5,0.) for i in range(61)];guide[6]=point(3.,.03)
    native=center_suffix(guide,0.,5.555555)
    assert max(map(abs,curvatures(native)))>planner.config.curvature_limit
    pose=(0.,.04,.01)
    bridge=matched_approach_prefix(guide,pose,.003,0.,5.555555)
    assert bridge[0][:2]==pose[:2]
    assert math.dist(bridge[-1][:2],sample(guide,5.555555)[0][:2])<1e-9
    assert max(map(abs,curvatures(bridge)))<planner.config.curvature_limit
    path=bridge+tuple(point(bridge[-1][0]+i*.25,0.) for i in range(1,81))
    assert planner._path_safety(path,(),10.)==''
    assert planner._rollout(path,pose,10.,.003,(),duration_s=7.)==''
    assert planner._path_safety(path,[obstacle(2.5,3.5,-2.,2.)],10.)=='BODY_COLLISION'
