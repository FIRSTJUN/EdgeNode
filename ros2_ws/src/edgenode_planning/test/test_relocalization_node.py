import json

from rclpy.parameter import Parameter
from std_msgs.msg import String

from test_static_obstacle_node import node


def event(node,state,xy=None):
    data=dict(schema_version=1,session_id='test',seq=1,state=state,
        stamp_ns=node.get_clock().now().nanoseconds,reason='TEST_RELOCATION')
    if xy is not None:data['position_xy']=xy
    node.localization_reset_callback(String(data=json.dumps(data)))


def test_relocation_disarms_and_resets_progress_without_clearing_hard_latch(node):
    node.set_parameters([Parameter('enable_drive',value=True)])
    node._progress_index=12;node._progress_arc_m=6.
    node.avoidance_planner.stop('BODY_COLLISION')
    event(node,'PENDING')
    assert not node.get_parameter('enable_drive').value
    assert not node.local_path_pub.publish.call_args.args[0].poses
    assert node.target_speed_pub.publish.call_args.args[0].data==0.
    event(node,'CONFIRMED',[100.,200.])
    assert node._progress_index is None and node._progress_arc_m is None
    assert node.avoidance_planner.reason=='BODY_COLLISION'
    assert node.avoidance_planner.state=='BLOCKED'
    assert not node._relocation_ready()


def test_relocation_requires_new_matching_pose_frames_and_never_enables(node):
    event(node,'PENDING')
    assert not node._relocation_ready()
    event(node,'CONFIRMED',[100.,200.])
    node.latest_pose=(0.,0.,0.)
    node.latest_pose_stamp_ns=node.get_clock().now().nanoseconds
    assert not node._relocation_ready()
    node.latest_pose=(100.,200.,0.)
    for i in range(3):
        node.latest_pose_stamp_ns=node.get_clock().now().nanoseconds
        assert node._relocation_ready()==(i==2)
    assert not node.get_parameter('enable_drive').value


def test_rejected_gps_outlier_keeps_protected_progress_and_maneuver_memory(node):
    node._progress_index=12;node._progress_arc_m=6.
    node.avoidance_planner.remembered=('sentinel',)
    event(node,'PENDING');event(node,'CANCELLED')
    assert node._progress_index==12 and node._progress_arc_m==6.
    assert node.avoidance_planner.remembered==('sentinel',)
    assert not node.get_parameter('enable_drive').value
