"""Filtering, connected components and frame math with reproducible local clouds."""

from dataclasses import replace
import math

import numpy as np
import pytest

from edgenode_perception.lidar_obstacles import (
    LidarObstacleConfig, cluster_centers, detect_clusters, filter_points,
    lidar_to_vehicle_xy, vehicle_to_world_xy,
)


@pytest.fixture
def config():
    return LidarObstacleConfig()


def npc_points(center=(14.093, 0.246, -0.472)):
    """Synthetic 64-return cluster centered at the user's observed NPC centroid."""
    grid = np.array(np.meshgrid(*([[-0.12, -0.04, 0.04, 0.12]] * 3))).reshape(3, -1).T
    return grid + np.array(center)


def test_roi_removes_points_outside_each_boundary(config):
    points = [[0.99, 2, 0], [20.01, 0, 0], [6, -4.01, 0], [6, 4.01, 0], [10, 1, 0]]
    assert filter_points(points, config).tolist() == [[10, 1, 0]]


def test_roi_boundaries_are_included(config):
    points = [[1, 4, 0], [20, -4, 0], [6, -4, 0], [6, 4, 0]]
    assert filter_points(points, config).tolist() == points


def test_near_origin_removal_is_independent_of_roi_and_self_mask(config):
    config = replace(config, roi_min_x=-1.0, roi_min_y=-1.0,
                     self_mask_half_width_y=0.0)
    assert filter_points([[0, 0, 0], [0.1, 0.1, 0], [0.3, 0, 0]], config).tolist() == [
        [0.3, 0, 0]]


def test_ground_removes_equal_or_lower_threshold(config):
    assert filter_points([[10, 0, -1.4], [10, 0, -1.0], [10, 0, -0.999]],
                         config).tolist() == [[10, 0, -0.999]]


def test_self_mask_uses_strict_forward_and_half_width_limits(config):
    points = [[1.99, 0, 0], [1.5, -0.799, 0], [2, 0, 0], [1.5, 0.8, 0], [1.5, -0.8, 0]]
    assert filter_points(points, config).tolist() == points[2:]


def test_single_cluster_returns_arithmetic_xyz_centroid(config):
    centers = cluster_centers(npc_points(), config)
    np.testing.assert_allclose(centers, [[14.093, 0.246, -0.472]])


def test_too_small_cluster_is_discarded(config):
    assert cluster_centers(npc_points()[:4], config).shape == (0, 3)


def test_cluster_with_exact_minimum_size_is_kept(config):
    assert cluster_centers(npc_points()[:5], config).shape == (1, 3)


def test_two_obstacles_produce_two_centers_and_drop_isolated_noise(config):
    points = np.vstack([npc_points(), npc_points((10, -2.5, -0.3)), [[19, 3.5, 0]]])
    centers = cluster_centers(points, config)
    np.testing.assert_allclose(centers, [[14.093, 0.246, -0.472], [10, -2.5, -0.3]])


def test_clustering_connects_neighbour_chains_beyond_one_radius(config):
    centers = cluster_centers([[6 + i * 0.7, 0, 0] for i in range(5)], config)
    np.testing.assert_allclose(centers, [[7.4, 0, 0]])


def test_clustering_distance_is_three_dimensional(config):
    centers = cluster_centers(np.vstack([npc_points(), npc_points((14.093, 0.246, 2))]),
                              config)
    assert len(centers) == 2


def test_finite_check_removes_nan_inf_in_every_axis(config):
    bad = [[math.nan, 0, 0], [10, math.inf, 0], [10, 0, -math.inf], [10, 0, math.nan]]
    np.testing.assert_allclose(cluster_centers(np.vstack([npc_points(), bad]), config),
                               [[14.093, 0.246, -0.472]])


def test_empty_and_all_filtered_clouds_are_empty(config):
    for points in ([], np.empty((0, 3)), [[0, 0, 0]], [[10, 0, -1.4]]):
        assert cluster_centers(points, config).shape == (0, 3)


def test_filtering_does_not_mutate_input(config):
    points = np.vstack([npc_points(), [[0, 0, 0]]])
    original = points.copy()
    cluster_centers(points, config)
    np.testing.assert_array_equal(points, original)


def test_sonata_mount_translation(config):
    assert lidar_to_vehicle_xy(14.093, 0.246, config) == pytest.approx((15.283, 0.246))
    assert lidar_to_vehicle_xy(1, 2, replace(config, lidar_offset_y_m=-0.2)) == (2.19, 1.8)


def test_world_transform_yaw_zero(config):
    vehicle = lidar_to_vehicle_xy(14.093, 0.246, config)
    assert vehicle_to_world_xy(*vehicle, 22.55, 1115.61, 0) == pytest.approx(
        (37.833, 1115.856))


def test_world_transform_yaw_90_degrees(config):
    vehicle = lidar_to_vehicle_xy(14.093, 0.246, config)
    assert vehicle_to_world_xy(*vehicle, 22.55, 1115.61, math.pi / 2) == pytest.approx(
        (22.304, 1130.893))


def test_world_transform_yaw_negative_90_degrees():
    assert vehicle_to_world_xy(10, 2, 1, 3, -math.pi / 2) == pytest.approx((3, -7))


@pytest.mark.parametrize('override', [
    {'roi_min_x': 20.0}, {'roi_max_y': -4.0}, {'ground_z_threshold': math.nan},
    {'cluster_distance_m': 0.0}, {'cluster_distance_m': math.inf},
    {'near_origin_radius_m': -0.1}, {'self_mask_half_width_y': -1.0},
    {'min_cluster_points': 0}, {'min_cluster_points': 2.5}, {'min_cluster_points': True},
    {'lidar_offset_x_m': math.inf}, {'lidar_offset_y_m': False},
])
def test_invalid_configuration_is_rejected(config, override):
    with pytest.raises(ValueError):
        replace(config, **override)


def test_cluster_retains_bounds_and_eight_footprint_points():
    config = LidarObstacleConfig()
    points = npc_points()
    cluster, = detect_clusters(points, config)
    assert cluster.point_count == 64
    np.testing.assert_allclose(cluster.center_xyz, [14.093, 0.246, -0.472])
    np.testing.assert_allclose(cluster.min_xy, [13.973, 0.126])
    np.testing.assert_allclose(cluster.max_xy, [14.213, 0.366])
    expected = [[13.973, 0.126], [14.093, 0.126], [14.213, 0.126], [14.213, 0.246],
                [14.213, 0.366], [14.093, 0.366], [13.973, 0.366], [13.973, 0.246]]
    np.testing.assert_allclose(cluster.footprint_xy(), expected)


def test_footprint_bounds_include_extreme_return_not_only_centroid():
    points = np.array([[10, 0, -0.5], [10.7, 0, -0.5], [11.4, 0, -0.5],
                       [11.4, 0.7, -0.5], [11.4, 1.4, -0.5]])
    cluster, = detect_clusters(points, LidarObstacleConfig())
    assert cluster.min_xy == (10, 0)
    assert cluster.max_xy == (11.4, 1.4)
    assert (10, 1.4) in cluster.footprint_xy()


def test_degenerate_cluster_keeps_eight_samples_but_one_cluster():
    clusters = detect_clusters([[10, 0, -0.5]] * 5, LidarObstacleConfig())
    assert len(clusters) == 1
    assert len(clusters[0].footprint_xy()) == 8
    assert set(clusters[0].footprint_xy()) == {(10, 0)}


def test_measured_ego_return_envelope_is_fully_removed_by_self_mask(config):
    # Synthetic coverage of the measured envelope/bin counts, not raw MORAI data.
    x = np.r_[np.linspace(0.301, 0.999, 653), np.linspace(1.5, 1.934, 181)]
    ego = np.column_stack((x, np.linspace(-0.690, 0.690, 834),
                           np.linspace(-0.493, -0.024, 834)))
    # Include the measured sub-1m returns so ROI removal cannot hide a mask error.
    measured_roi = replace(config, roi_min_x=0.3)
    unmasked = replace(measured_roi, self_mask_half_width_y=0.0)
    np.testing.assert_allclose(filter_points(ego, unmasked), ego)
    assert filter_points(ego, measured_roi).shape == (0, 3)
    assert detect_clusters(ego, config) == []
    assert config.near_origin_radius_m == 0.3


def test_front_returns_beyond_two_metres_are_retained(config):
    points = [[x, 0.0, -0.5] for x in [2.000001, 2.01, 2.1, 3.0, 5.0]]
    np.testing.assert_array_equal(filter_points(points, config), points)


@pytest.mark.parametrize('distance', [3.0, 5.0])
def test_near_front_npc_keeps_cluster_and_footprint(config, distance):
    points = npc_points((distance, 0.0, -0.472))
    cluster, = detect_clusters(points, config)
    assert cluster.point_count == 64
    np.testing.assert_allclose(cluster.center_xyz, [distance, 0.0, -0.472], atol=1e-12)
    assert len(cluster.footprint_xy()) == 8



def test_oriented_wall_footprint_contains_returns_without_filling_nearby_lane():
    import numpy as np
    from edgenode_perception.lidar_obstacles import oriented_bounds,LidarCluster
    direction=np.array((.8,.6));normal=np.array((-.6,.8))
    points=np.array([s*direction+w*normal for s in range(21) for w in (-.1,.1)])
    corners=oriented_bounds(points)
    cluster=LidarCluster((0.,0.,0.),tuple(points.min(axis=0)),tuple(points.max(axis=0)),len(points),corners)
    footprint=cluster.footprint_xy()
    assert len(footprint)==8
    # The long observed wall is 0.2 m thick, independent of sensor yaw.
    local=(np.array(corners)@normal)
    assert local.max()-local.min()<.201
    for p in points:
        assert all(np.cross(np.array(b)-a,p-a)>=-1e-9 for a,b in zip(np.array(corners),np.roll(corners,-1,axis=0)))
