"""Heading basis agrees with independently projected geodetic directions."""
import math
from unittest.mock import Mock

import pytest
import rclpy
from pyproj import Proj,Transformer,Geod
from morai_ros2_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from edgenode_localization.map_heading import grid_convergence,grid_yaw
from edgenode_localization.localization_node import LocalizationNode


@pytest.mark.parametrize('lon,lat',[(126.77515145,37.24372186),(129.,37.),(131.,45.)])
@pytest.mark.parametrize('yaw',[-2.,-.5,0.,1.,3.])
def test_conversion_matches_projected_one_metre_geodetic_displacement(lon,lat,yaw):
    projection=Proj(32652);transform=Transformer.from_crs(4326,32652,always_xy=True)
    x,y=transform.transform(lon,lat)
    next_lon,next_lat,_=Geod(ellps='WGS84').fwd(lon,lat,90.-math.degrees(yaw),1.)
    a,b=transform.transform(next_lon,next_lat)
    expected=math.atan2(b-y,a-x);actual=grid_yaw(yaw,grid_convergence(projection,lon,lat))
    assert abs(math.atan2(math.sin(actual-expected),math.cos(actual-expected)))<1e-6


def test_enu_mode_waits_for_map_basis_and_publishes_grid_yaw_with_unchanged_xy():
    rclpy.init(args=['--ros-args','-r','__ns:=/offline_map_heading','-p','imu_yaw_frame:=enu'])
    node=None
    try:
        node=LocalizationNode();node.odometry_publisher=Mock();node.grid_convergence_pub=Mock()
        imu=Imu();imu.header.stamp.sec=1;imu.orientation.w=1.
        node.imu_callback(imu)
        assert not node.ekf.initialized and node.odometry_publisher.publish.call_count==0
        gps=GPSMessage(longitude=126.77515145,latitude=37.24372186,east_offset=302459.942,north_offset=4122635.537)
        gps.header.stamp.sec=1;node.gps_callback(gps)
        imu.header.stamp.nanosec=20_000_000;node.imu_callback(imu)
        assert node.ekf.initialized
        x,y,yaw,_=node.ekf.get_state()
        expected=grid_convergence(Proj(32652),gps.longitude,gps.latitude)
        assert yaw==pytest.approx(expected)
        east,north=node.gps_transformer.transform(gps.longitude,gps.latitude)
        assert (x,y)==pytest.approx((east-gps.east_offset,north-gps.north_offset))
        assert -1.35<node.grid_convergence_pub.publish.call_args.args[0].data<-1.34
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
