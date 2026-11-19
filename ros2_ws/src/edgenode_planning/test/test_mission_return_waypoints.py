from dataclasses import replace
from edgenode_planning.lane_geometry import PolylineIndex
from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
from test_static_obstacle_planner import setup


def scene():
 p,point,_=setup();p.config=replace(p.config,tracking_limit_m=.15)
 p._route_waypoints=(('required',(40.,0.,0.)),)
 pair=type('Pair',(),{})();pair.original=tuple(point(i,0.) for i in range(101));pair.original_index=PolylineIndex(pair.original);pair.length=100.
 return p,point,pair


def test_parallel_lane_pass_cannot_replace_a_required_route_waypoint():
 p,point,pair=scene();path=tuple(point(i,3.5) for i in range(71))
 assert LaneCandidatePlanner._mission_waypoint_reason(p,path,pair,60.)=='MISSION_WAYPOINT_BYPASS:required'


def test_return_before_the_waypoint_keeps_normal_route_following_reachable():
 p,point,pair=scene();path=tuple(point(i,3.5 if i<20 else 0.) for i in range(71))
 assert LaneCandidatePlanner._mission_waypoint_reason(p,path,pair,30.)==''
 assert LaneCandidatePlanner._mission_waypoint_reason(p,path,pair,60.)==''


def test_tracking_reserve_is_included_in_the_waypoint_tolerance():
 p,point,pair=scene();path=tuple(point(i,.9) for i in range(71))
 assert LaneCandidatePlanner._mission_waypoint_reason(p,path,pair,60.)
 p._route_waypoints=()
 assert LaneCandidatePlanner._mission_waypoint_reason(p,path,pair,60.)==''


def test_return_waypoint_cutoff_uses_world_position_independently_of_global_origin():
 p,point,_=scene();p.__class__=LaneCandidatePlanner
 pair=type('Pair',(),{})();pair.original=tuple(point(i,0.) for i in range(100,201));pair.original_index=PolylineIndex(pair.original);pair.length=100.
 path=tuple(point(i,3.5) for i in range(100,161))
 for origin in (0,100):
  p._global_path=tuple(point(i,0.) for i in range(origin,301))
  p._route_waypoints=(('future',(180.,0.,0.)),)
  assert p._mission_return_waypoint_reason(path,pair,150.-origin)==''
  p._route_waypoints=(('required',(140.,0.,0.)),)
  assert p._mission_return_waypoint_reason(path,pair,150.-origin)=='MISSION_WAYPOINT_BYPASS:required'
