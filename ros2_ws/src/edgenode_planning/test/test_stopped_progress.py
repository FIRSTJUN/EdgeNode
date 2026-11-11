import math
from unittest.mock import Mock
import pytest
from std_srvs.srv import Trigger
from edgenode_planning.stopped_progress import reanchor_progress
from test_speed_parameters_ros import make_node

POINTS=[[i*.5,0.,0.] for i in range(61)]


def test_bounded_reanchor_preserves_path_and_finds_actual_segment():
    original=[p[:] for p in POINTS]
    r=reanchor_progress(POINTS,10.7,.1,0.,12.)
    assert r['progress_arc_m']==pytest.approx(10.7)
    assert r['nearest_index']==21
    assert r['retreat_m']==pytest.approx(1.3)
    assert POINTS==original


@pytest.mark.parametrize('x,y,yaw,arc',[(5.,0.,0.,12.),(12.,1.,0.,12.),
    (12.,0.,math.pi,12.),(12.,0.,0.,100.),(math.nan,0.,0.,12.),(12.,0.,0.,-1.)])
def test_teleport_distant_lane_wrong_heading_and_invalid_progress_reject(x,y,yaw,arc):
    assert reanchor_progress(POINTS,x,y,yaw,arc) is None


def test_nearby_future_crossing_cannot_steal_progress():
    pts=[[0.,0.,0.],[20.,0.,0.],[20.,2.,0.],[0.,2.,0.]]
    r=reanchor_progress(pts,10.,.2,0.,11.)
    assert r['progress_arc_m']==pytest.approx(10.)


def configure(node):
    now=node.get_clock().now().nanoseconds
    node._route_result={'points':POINTS}
    node._progress_index=24;node._progress_arc_m=12.
    node.latest_pose=(10.7,.1,0.);node._pose_received_ns=now
    node._ego_speed=(0.,now,now)
    node._long_planner=Mock()
    return now


def test_service_keeps_immutable_route_and_resets_only_query_state(make_node):
    node=make_node(enable_drive=False);configure(node)
    route=node._route_result;long_planner=node._long_planner
    result=node.reset_local_progress(Trigger.Request(),Trigger.Response())
    assert result.success
    assert node._route_result is route and node._long_planner is long_planner
    assert node._progress_arc_m==pytest.approx(10.7)
    long_planner.reset.assert_called_once()
    node.dijkstra_planner.plan.assert_not_called()
    assert node.get_parameter('enable_drive').value is False


@pytest.mark.parametrize('fault',['enabled','moving','old_speed','old_pose','future_speed','missing_pose'])
def test_service_rejects_motion_and_stale_or_future_inputs_without_mutation(make_node,fault):
    node=make_node(enable_drive=(fault=='enabled'));now=configure(node)
    if fault=='moving':node._ego_speed=(.31,now,now)
    if fault=='old_speed':node._ego_speed=(0.,now-300_000_000,now)
    if fault=='future_speed':node._ego_speed=(0.,now+1_000_000_000,now)
    if fault=='old_pose':node._pose_received_ns=now-400_000_000
    if fault=='missing_pose':node.latest_pose=None
    result=node.reset_local_progress(Trigger.Request(),Trigger.Response())
    assert not result.success
    assert node._progress_arc_m==12.
    node._long_planner.reset.assert_not_called()
