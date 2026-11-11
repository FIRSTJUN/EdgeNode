"""ROS-independent local LiDAR filtering, Euclidean clusters and XY transforms."""

from dataclasses import dataclass, fields
import math

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class LidarObstacleConfig:
    """Initial MORAI Sonata baseline; distances are metres in the LiDAR frame."""

    roi_min_x: float = 1.0
    roi_max_x: float = 20.0
    roi_min_y: float = -4.0
    roi_max_y: float = 4.0
    ground_z_threshold: float = -1.0
    oriented_footprints: bool = False
    near_origin_radius_m: float = 0.3
    self_mask_min_x: float = -2.4
    self_mask_max_x: float = 2.0
    self_mask_half_width_y: float = 0.8
    cluster_distance_m: float = 0.8
    min_cluster_points: int = 5
    lidar_offset_x_m: float = 1.19
    lidar_offset_y_m: float = 0.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == 'oriented_footprints':
                if type(value) is not bool:raise ValueError('oriented_footprints must be boolean')
            elif field.name == 'min_cluster_points':
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    raise ValueError('min_cluster_points must be a positive integer')
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value)):
                raise ValueError(f'{field.name} must be finite')
        if self.roi_min_x >= self.roi_max_x or self.roi_min_y >= self.roi_max_y:
            raise ValueError('ROI minima must be less than maxima')
        if self.self_mask_min_x >= self.self_mask_max_x:
            raise ValueError('self mask minima must be less than maxima')
        if self.cluster_distance_m <= 0.0:
            raise ValueError('cluster_distance_m must be positive')
        if self.near_origin_radius_m < 0.0 or self.self_mask_half_width_y < 0.0:
            raise ValueError('near-origin radius and self-mask half width must be nonnegative')


def filter_points(points, config):
    """Remove nonfinite/no-return, outside-ROI, ground and ego returns, in order."""
    xyz = np.asarray(points, dtype=float)
    if xyz.size == 0:
        return np.empty((0, 3), dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('Expected an N x 3 XYZ array')
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    xyz = xyz[np.hypot(xyz[:, 0], xyz[:, 1]) >= config.near_origin_radius_m]
    xyz = xyz[(xyz[:, 0] >= config.roi_min_x) & (xyz[:, 0] <= config.roi_max_x)
              & (xyz[:, 1] >= config.roi_min_y) & (xyz[:, 1] <= config.roi_max_y)]
    xyz = xyz[xyz[:, 2] > config.ground_z_threshold]
    self_mask = ((xyz[:, 0] > config.self_mask_min_x)
                 & (xyz[:, 0] < config.self_mask_max_x)
                 & (np.abs(xyz[:, 1]) < config.self_mask_half_width_y))
    return xyz[~self_mask]


@dataclass(frozen=True)
class LidarCluster:
    """Observed LiDAR bounds, not an estimate of an occluded object's full size."""

    center_xyz: tuple
    min_xy: tuple
    max_xy: tuple
    point_count: int
    oriented_corners_xy: tuple = ()

    def footprint_xy(self):
        """Four corners and four edge midpoints of a containing XY box."""
        if self.oriented_corners_xy:
            corners=self.oriented_corners_xy
            return tuple(p for a,b in zip(corners,corners[1:]+corners[:1])
                         for p in (a,((a[0]+b[0])/2,(a[1]+b[1])/2)))
        x0, y0 = self.min_xy
        x1, y1 = self.max_xy
        xm, ym = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        return ((x0, y0), (xm, y0), (x1, y0), (x1, ym),
                (x1, y1), (xm, y1), (x0, y1), (x0, ym))


def oriented_bounds(xy):
    """PCA box containing every observed XY point, including diagonal walls."""
    center=xy.mean(axis=0);relative=xy-center
    _,vectors=np.linalg.eigh(relative.T@relative)
    axis=vectors[:,-1];axes=np.array((axis,(-axis[1],axis[0]))).T
    local=relative@axes;low=local.min(axis=0);high=local.max(axis=0)
    corners=np.array(((low[0],low[1]),(high[0],low[1]),
                      (high[0],high[1]),(low[0],high[1])))@axes.T+center
    return tuple(tuple(float(v) for v in row) for row in corners)


def detect_clusters(points, config):
    """Return bounds and centroids of complete 3D radius-connected components."""
    xyz = filter_points(points, config)
    if len(xyz) < config.min_cluster_points:
        return []
    tree = cKDTree(xyz)
    visited = np.zeros(len(xyz), dtype=bool)
    clusters = []
    for seed in range(len(xyz)):
        if visited[seed]:
            continue
        visited[seed] = True
        pending = [seed]
        component = []
        while pending:
            index = pending.pop()
            component.append(index)
            neighbours = tree.query_ball_point(xyz[index], config.cluster_distance_m)
            for neighbour in neighbours:
                if not visited[neighbour]:
                    visited[neighbour] = True
                    pending.append(neighbour)
        if len(component) >= config.min_cluster_points:
            selected = xyz[component]
            clusters.append(LidarCluster(
                center_xyz=tuple(np.mean(selected, axis=0)),
                min_xy=tuple(np.min(selected[:, :2], axis=0)),
                max_xy=tuple(np.max(selected[:, :2], axis=0)),
                point_count=len(component),
                oriented_corners_xy=oriented_bounds(selected[:,:2]) if config.oriented_footprints else (),
            ))
    return clusters


def cluster_centers(points, config):
    """Keep the centroid helper API; Planning now receives footprint points."""
    return np.asarray([cluster.center_xyz for cluster in detect_clusters(points, config)],
                      dtype=float).reshape(-1, 3)


def lidar_to_vehicle_xy(lidar_x, lidar_y, config):
    """Apply the Sonata sensor translation; no mount rotation is assumed."""
    return lidar_x + config.lidar_offset_x_m, lidar_y + config.lidar_offset_y_m


def vehicle_to_world_xy(vehicle_x, vehicle_y, ego_x, ego_y, yaw):
    """Use ROS ENU yaw: zero along +x, positive counter-clockwise toward +y."""
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (ego_x + cosine * vehicle_x - sine * vehicle_y,
            ego_y + sine * vehicle_x + cosine * vehicle_y)
