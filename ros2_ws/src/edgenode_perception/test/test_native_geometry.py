from dataclasses import replace
import numpy as np
import pytest
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from edgenode_perception import _geometry_native
from edgenode_perception.lidar_obstacles import oriented_bounds,LidarObstacleConfig,detect_clusters


def test_radius_components_match_scipy_with_dense_isolated_and_exact_radius_points():
    rng=np.random.default_rng(921006)
    cases=[np.empty((0,3)),np.array([[0.,0.,0.],[.8,0.,0.],[1.6,0.,0.],[10.,0.,0.]]),
           rng.normal(size=(700,3)),rng.normal(size=(600,3))*.01]
    for points in cases:
        pairs=cKDTree(points).query_pairs(.8,output_type='ndarray')
        graph=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(points),len(points))).tocsr()
        expected=connected_components(graph,directed=False)[1]
        actual=np.asarray(_geometry_native.component_labels(len(points),np.ascontiguousarray(pairs,dtype=np.int64)))
        assert np.array_equal(actual,expected)


def test_native_hull_preserves_minimum_area_boxes_including_degenerate_returns():
    rng=np.random.default_rng(9233)
    clouds=[np.array([[8.,-3.]]),np.array([[8.,-3.],[8.,-3.]]),
            np.c_[np.arange(500)*.001,np.arange(500)*.002],rng.normal(size=(500,2))+[220.,1100.]]
    for xy in clouds:assert oriented_bounds(xy)==oriented_bounds(xy,native_geometry=True)


def test_all_cluster_geometry_and_order_stay_identical():
    rng=np.random.default_rng(944)
    points=np.r_[rng.normal(size=(700,3))*.2+[12.,-1.,.1],
                 rng.normal(size=(600,3))*.1+[18.,3.,.5]]
    cfg=LidarObstacleConfig(roi_max_x=30.,clipped_footprints=True,tiled_minimum_area_bounds=True,footprint_tile_size_m=3.)
    assert detect_clusters(points,cfg,footprint_tile_pose=(18.,1000.,.7))==detect_clusters(points,replace(cfg,native_geometry=True),footprint_tile_pose=(18.,1000.,.7))


def test_native_helpers_reject_invalid_geometry():
    with pytest.raises(ValueError):_geometry_native.component_labels(-1,np.empty((0,2),dtype=np.int64))
    with pytest.raises(ValueError):_geometry_native.component_labels(2,np.array([[0,2]],dtype=np.int64))
    with pytest.raises(ValueError):_geometry_native.convex_hull(np.array([[float('nan'),0.]]))
