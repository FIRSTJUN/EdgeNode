from types import SimpleNamespace
from edgenode_static20_experiment.natural_join import waypoint_join_proof

GLOBAL=tuple((float(x),0.,0.) for x in range(101))
# The exact native route joins at x=50; no invented lateral return edge.
FOLLOW=tuple((float(x),3.*(50-x)/30 if x<50 else 0.,0.) for x in range(20,101))
def follow():
 from edgenode_planning.lane_geometry import arc
 return SimpleNamespace(points=FOLLOW,reason='NATURAL_MISSION_JOIN',merge_station_m=arc(FOLLOW[:31])[-1],merge_link_id='native_shared_node_suffix')
ENTRY=tuple((float(x),3.*(x-10)/10,0.) for x in range(10,21))
def prove(waypoints,**kwargs):return waypoint_join_proof(ENTRY,kwargs.get('follow',follow()),GLOBAL,(10.,0.,0.),waypoints,.35)

def test_final_goal_after_exact_join_does_not_force_an_unnecessary_return():
 assert prove([('goal',(100.,0.,0.))]) is not None

def test_bypassed_mandatory_waypoint_keeps_explicit_return_required():
 assert prove([('before_join',(30.,0.,0.)),('goal',(100.,0.,0.))]) is None

def test_waypoint_on_native_witness_passes_with_original_one_meter_tracking_reserve():
 proof=prove([('merge',(50.,0.,0.)),('goal',(100.,0.,0.))]);assert proof['checked_waypoints'][0]['id']=='merge'
 assert proof['path_distance_limit_m']==.65 and proof['waypoint_distance_limit_m']==1.

def test_reversed_required_visit_order_is_rejected():
 assert prove([('later',(49.,0.,0.)),('earlier',(48.,0.,0.))]) is None

def test_nearby_geometry_is_not_an_exact_join():
 value=follow();value.points=tuple((x,y+.1,z) for x,y,z in value.points)
 assert prove([],follow=value) is None

def test_missing_directed_native_join_is_not_authorized():
 value=follow();value.reason='RETURN_REQUIRED_BEFORE_LANE_END';assert prove([],follow=value) is None

def test_entry_must_connect_to_the_native_path_without_a_gap():
 wrong=tuple((x,y+.1,z) for x,y,z in ENTRY)
 assert waypoint_join_proof(wrong,follow(),GLOBAL,(10.,0.,0.),[],.35) is None
