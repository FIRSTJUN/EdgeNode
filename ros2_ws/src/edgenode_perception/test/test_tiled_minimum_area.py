from dataclasses import replace
import math

import numpy as np
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union

from edgenode_perception.lidar_obstacles import LidarObstacleConfig,detect_clusters


def test_diagonal_wall_tiles_contain_every_return_without_world_axis_corner_volume():
    xyz=np.array([(10.+i*.1,2.+i*.1*math.tan(.5),-.2) for i in range(70)])
    config=LidarObstacleConfig(roi_max_x=40.,roi_max_y=30.,oriented_footprints=True,
        clipped_footprints=True,footprint_tile_size_m=3.,lidar_offset_x_m=0.)
    original=detect_clusters(xyz,config,footprint_tile_pose=(0.,0.,0.))
    fitted=detect_clusters(xyz,replace(config,tiled_minimum_area_bounds=True),footprint_tile_pose=(0.,0.,0.))
    old=unary_union([Polygon(c.footprint_xy()) for c in original])
    new=unary_union([Polygon(c.footprint_xy()) for c in fitted])
    assert sum(c.point_count for c in fitted)==len(xyz)
    assert all(new.covers(Point(*p[:2])) for p in xyz)
    assert all(len(c.footprint_xy())==8 for c in fitted)
    assert new.area<old.area*.3


def test_tile_ownership_and_world_occupancy_do_not_rotate_with_sensor_heading():
    world=np.array([(10.+i*.1,2.+i*.1*math.tan(.5)) for i in range(70)])
    config=LidarObstacleConfig(roi_min_x=-40.,roi_max_x=40.,roi_min_y=-40.,roi_max_y=40.,
        oriented_footprints=True,clipped_footprints=True,footprint_tile_size_m=3.,
        tiled_minimum_area_bounds=True,lidar_offset_x_m=0.,self_mask_half_width_y=0.)
    results=[]
    for yaw in (0.,.3,-.5):
        c,s=math.cos(yaw),math.sin(yaw);rotation=np.array(((c,-s),(s,c)))
        sensor=world@rotation;xyz=np.c_[sensor,np.full(len(sensor),-.2)]
        clusters=detect_clusters(xyz,config,footprint_tile_pose=(0.,0.,yaw))
        results.append(unary_union([Polygon(np.asarray(k.footprint_xy())@rotation.T) for k in clusters]))
    assert all(results[0].symmetric_difference(g).area<1e-7 for g in results[1:])
