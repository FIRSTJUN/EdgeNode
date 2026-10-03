import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from edgenode_perception.lidar_geometry import pointcloud_xyz, corridor_points


def cloud(endian='<'):
    m = PointCloud2(height=2, width=2, point_step=16, row_step=40, is_bigendian=endian=='>')
    m.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i,k in enumerate('xyz')]
    a = bytearray(80)
    view = np.ndarray((2,2,3),dtype=endian+'f4',buffer=a,strides=(40,16,4))
    view[:] = np.arange(12).reshape(2,2,3)
    m.data = bytes(a)
    return m


def test_row_padding_and_big_endian():
    for endian in ['<','>']:
        assert np.array_equal(pointcloud_xyz(cloud(endian)), np.arange(12).reshape(4,3))


def test_truncated_or_invalid_cloud_is_not_clear():
    m=cloud();m.data=bytes(30)
    assert pointcloud_xyz(m) is None
    m=cloud();m.fields[0].datatype=3
    assert pointcloud_xyz(m) is None


def test_ground_and_side_wall_excluded_front_wall_preserved():
    p=np.array([[2,0,-.56],[5,2.5,.2],[5,.5,.2],[6,0,2.0]],dtype=float)
    assert np.array_equal(corridor_points(p,0), [[5,.5,.2]])


def test_curved_swept_corridor_follows_turn():
    k=.15;arc=7.;x=np.sin(k*arc)/k;y=(1-np.cos(k*arc))/k
    p=np.array([[x,y,.2],[x,0,.2]])
    assert np.array_equal(corridor_points(p,k), p[:1])
