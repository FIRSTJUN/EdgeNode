"""Actual S-curve returns remain roadside instead of becoming a convex chord."""
import gzip
import json
import math
from pathlib import Path
from types import SimpleNamespace

from edgenode_perception.lidar_obstacles import LidarObstacleConfig,detect_clusters
from edgenode_planning.avoidance_geometry import obstacle_polygons
from edgenode_planning.static_obstacle_planner import AvoidanceConfig,StaticObstaclePlanner


def test_actual_s_curve_preserves_all_returns_and_removes_phantom_memory_intersection():
    with gzip.open(Path(__file__).parent/'fixtures/kcity_s_curve_20261117.json.gz','rt') as stream:
        data=json.load(stream)
    pose=data['pose'];config=LidarObstacleConfig(roi_min_x=-8.,roi_max_x=100.,
        roi_min_y=-40.,roi_max_y=40.,oriented_footprints=True,clipped_footprints=True,
        footprint_tile_size_m=3.,obstacle_height_filter_enabled=True,obstacle_max_z_m=.3)
    clusters=detect_clusters(data['xyz'],config,footprint_tile_pose=pose)
    assert sum(cluster.point_count for cluster in clusters)==len(data['xyz'])
    assert len(clusters)>1
    c,s=math.cos(pose[2]),math.sin(pose[2])
    groups=[[(pose[0]+c*(x+1.19)-s*y,pose[1]+s*(x+1.19)+c*y)
             for x,y in cluster.footprint_xy()] for cluster in clusters]
    planner=StaticObstaclePlanner(SimpleNamespace(boundaries=None),
        config=AvoidanceConfig(memory_merge_extent_m=3.5))
    assert planner._normal_hazards(data['path'],data['old_memory'])
    planner._remember(obstacle_polygons(groups))
    assert not planner._normal_hazards(data['path'],planner.remembered)
