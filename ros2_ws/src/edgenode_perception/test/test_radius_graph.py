"""Compiled clustering preserves the exact former radius-connected graph."""
import numpy as np
from scipy.spatial import cKDTree

from edgenode_perception.lidar_obstacles import LidarObstacleConfig, detect_clusters


def test_dense_faces_chains_and_isolates_match_reference_bfs():
    rng=np.random.default_rng(20261117)
    dense=np.c_[rng.normal(10.,.2,800),rng.normal(1.,.2,800),rng.normal(-.5,.04,800)]
    chain=np.array([(14.+i*.3,2.,-.5) for i in range(20)])
    xyz=np.r_[dense,chain,[[8.,-3.,-.5],[9.,-3.,-.5]]]
    config=LidarObstacleConfig(cluster_distance_m=.31,min_cluster_points=1)
    tree=cKDTree(xyz);visited=set();expected=[]
    for seed in range(len(xyz)):
        if seed in visited:continue
        visited.add(seed);pending=[seed];component=[]
        while pending:
            i=pending.pop();component.append(i)
            for neighbor in tree.query_ball_point(xyz[i],config.cluster_distance_m):
                if neighbor not in visited:visited.add(neighbor);pending.append(neighbor)
        expected.append(xyz[component])
    clusters=detect_clusters(xyz,config)
    assert [c.point_count for c in clusters]==[len(p) for p in expected]
    np.testing.assert_allclose([c.center_xyz for c in clusters],[p.mean(axis=0) for p in expected],atol=1e-12)
    np.testing.assert_allclose([c.min_xy for c in clusters],[p[:,:2].min(axis=0) for p in expected])
    np.testing.assert_allclose([c.max_xy for c in clusters],[p[:,:2].max(axis=0) for p in expected])
