import math

import pytest

from edgenode_perception.pose_history import PoseHistory


def test_pose_at_sensor_time_removes_translation_and_turn_latency_error():
    history=PoseHistory();history.add(1_000_000_000,(0.,0.,0.));history.add(1_100_000_000,(1.,0.,.2))
    pose=history.sample(1_050_000_000)['pose'];assert pose==pytest.approx((.5,0.,.1))
    x,y=20.-pose[0],2.-pose[1]
    local=(x*math.cos(.1)+y*math.sin(.1),-x*math.sin(.1)+y*math.cos(.1))
    def world(p):return (p[0]+local[0]*math.cos(p[2])-local[1]*math.sin(p[2]),p[1]+local[0]*math.sin(p[2])+local[1]*math.cos(p[2]))
    assert world(pose)==pytest.approx((20.,2.))
    assert math.dist(world((1.,0.,.2)),(20.,2.))>1.


def test_yaw_wrap_uses_shortest_rotation_without_extrapolation():
    history=PoseHistory();history.add(10,(0.,0.,math.radians(179)));history.add(20,(0.,0.,math.radians(-179)))
    assert abs(abs(history.sample(15)['pose'][2])-math.pi)<1e-9
    assert history.sample(9) is None and history.sample(21) is None


def test_large_gap_reordered_pose_and_relocation_never_create_interpolated_evidence():
    history=PoseHistory();assert history.add(1_000_000_000,(0.,0.,0.))
    assert not history.add(1_000_000_000,(99.,0.,0.))
    history.add(1_400_000_000,(1.,0.,0.));assert history.sample(1_200_000_000) is None
    history.add(1_410_000_000,(100.,0.,0.));assert history.sample(1_405_000_000) is None
    assert history.sample(1_410_000_000)['pose']==(100.,0.,0.)
