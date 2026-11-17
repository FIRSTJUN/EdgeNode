"""ROS-independent local LiDAR filtering, Euclidean clusters and XY transforms."""

from dataclasses import dataclass, fields
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@dataclass(frozen=True)
class LidarObstacleConfig:
    """Initial MORAI Sonata baseline; distances are metres in the LiDAR frame."""

    roi_min_x: float = 1.0
    roi_max_x: float = 20.0
    roi_min_y: float = -4.0
    roi_max_y: float = 4.0
    ground_z_threshold: float = -1.0
    obstacle_height_filter_enabled: bool = False
    obstacle_max_z_m: float = 1.0
    oriented_footprints: bool = False
    clipped_footprints: bool = False
    footprint_tile_size_m: float = 0.
    ground_plane_filter_enabled: bool = False
    ground_plane_neighbor_m: float = 1.5
    ground_plane_max_slope: float = .15
    ground_plane_tolerance_m: float = .04
    ground_plane_min_seed_points: int = 5
    near_origin_radius_m: float = 0.3
    self_mask_min_x: float = -2.4
    self_mask_max_x: float = 2.0
    self_mask_half_width_y: float = 0.8
    cluster_distance_m: float = 0.8
    min_cluster_points: int = 5
    min_footprint_extent_m: float = .05
    lidar_offset_x_m: float = 1.19
    lidar_offset_y_m: float = 0.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in ('oriented_footprints','clipped_footprints','ground_plane_filter_enabled','obstacle_height_filter_enabled'):
                if type(value) is not bool:raise ValueError(field.name+' must be boolean')
            elif field.name in ('min_cluster_points','ground_plane_min_seed_points'):
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
        if self.min_footprint_extent_m<=0:raise ValueError('positive footprint extent required')
        if self.footprint_tile_size_m<0:raise ValueError('nonnegative footprint tile size required')
        if self.near_origin_radius_m < 0.0 or self.self_mask_half_width_y < 0.0:
            raise ValueError('near-origin radius and self-mask half width must be nonnegative')
        if min(self.ground_plane_neighbor_m,self.ground_plane_max_slope,self.ground_plane_tolerance_m)<=0:
            raise ValueError('positive ground continuity limits required')
        if self.obstacle_max_z_m<=self.ground_z_threshold:
            raise ValueError('obstacle height ceiling must exceed ground cutoff')


def filter_points(points, config, *, keep_ground=False):
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
    if config.obstacle_height_filter_enabled:xyz=xyz[xyz[:,2]<=config.obstacle_max_z_m]
    if not keep_ground:xyz = xyz[xyz[:, 2] > config.ground_z_threshold]
    self_mask = ((xyz[:, 0] > config.self_mask_min_x)
                 & (xyz[:, 0] < config.self_mask_max_x)
                 & (np.abs(xyz[:, 1]) < config.self_mask_half_width_y))
    return xyz[~self_mask]


def gravity_ground_heights(points,roll,pitch):
    """Level Z for ground classification; retain the verified sensor-local XY.

    Ground height must not rise with range merely because the vehicle pitches
    or rolls. Yaw has no effect on the gravity direction. The caller pairs the
    attitude to the cloud timestamp; invalid attitude is a sensor fault.
    """
    if not all(math.isfinite(v) for v in (roll,pitch)):raise ValueError('finite roll/pitch required')
    xyz=np.asarray(points,dtype=float).copy()
    if xyz.ndim!=2 or xyz.shape[1]!=3:raise ValueError('expected Nx3 cloud')
    xyz[:,2]=(-math.sin(pitch)*xyz[:,0]+math.cos(pitch)*math.sin(roll)*xyz[:,1]
              +math.cos(pitch)*math.cos(roll)*xyz[:,2])
    return xyz


@dataclass(frozen=True)
class LidarCluster:
    """Observed LiDAR bounds, not an estimate of an occluded object's full size."""

    center_xyz: tuple
    min_xy: tuple
    max_xy: tuple
    point_count: int
    oriented_corners_xy: tuple = ()
    min_footprint_extent_m: float = .05
    clipped_vertices_xy: tuple = ()
    component_id: int = -1

    def footprint_xy(self):
        """Four corners and four edge midpoints of a containing XY box."""
        if self.clipped_vertices_xy:
            return self.clipped_vertices_xy
        if self.oriented_corners_xy:
            corners=self.oriented_corners_xy
            return tuple(p for a,b in zip(corners,corners[1:]+corners[:1])
                         for p in (a,((a[0]+b[0])/2,(a[1]+b[1])/2)))
        x0, y0 = self.min_xy
        x1, y1 = self.max_xy
        dx=max(0.,self.min_footprint_extent_m-(x1-x0))/2
        dy=max(0.,self.min_footprint_extent_m-(y1-y0))/2
        x0-=dx;x1+=dx;y0-=dy;y1+=dy
        xm, ym = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        return ((x0, y0), (xm, y0), (x1, y0), (x1, ym),
                (x1, y1), (xm, y1), (x0, y1), (x0, ym))


def oriented_bounds(xy,min_extent_m=.05):
    """Minimum-area containing rectangle over observed hull edge directions.

    PCA rotates with the density of visible front/side returns and can invent
    a large corner outside a partially observed vehicle. The hull-edge box
    contains EVERY return without using point density as an object heading.
    Keep the 8-point wire format, nonzero thin-object floor and all margins.
    """
    xy=np.asarray(xy,dtype=float)
    if xy.ndim!=2 or xy.shape[1]!=2 or not len(xy) or not np.isfinite(xy).all():
        raise ValueError('finite observed XY points required')
    center=xy.mean(axis=0);relative=xy-center
    unique=sorted(set(map(tuple,relative)))
    def half(points):
        result=[]
        for p in points:
            while len(result)>1:
                a,b=result[-2:]
                if (b[0]-a[0])*(p[1]-b[1])-(b[1]-a[1])*(p[0]-b[0])>0:break
                result.pop()
            result.append(p)
        return result
    hull=np.array(half(unique)[:-1]+half(list(reversed(unique)))[:-1]) if len(unique)>1 else np.array(unique)
    edges=np.roll(hull,-1,axis=0)-hull
    angles=np.unique(np.mod(np.arctan2(edges[:,1],edges[:,0]),math.pi/2))
    best=None
    for angle in angles:
        c,s=math.cos(angle),math.sin(angle);axes=np.array(((c,-s),(s,c)))
        local=relative@axes;low=local.min(axis=0);high=local.max(axis=0)
        padding=np.maximum(0.,(min_extent_m-(high-low))/2)+1e-9
        low=low-padding;high=high+padding
        key=(float(np.prod(high-low)),float(angle))
        if best is None or key<best[0]:best=(key,axes,low,high)
    _,axes,low,high=best
    corners=np.array(((low[0],low[1]),(high[0],low[1]),
                      (high[0],high[1]),(low[0],high[1])))@axes.T+center
    return tuple(tuple(float(v) for v in row) for row in corners)


def clipped_bounds(xy,corners,min_extent_m=.05):
    """Containing octagon: clip only unobserved rectangle corners.

    The four additional support planes are computed from ALL observed points,
    without outlier rejection, guessed object heading or smaller margins.
    Each cuts at most one box corner; the existing eight-point wire format
    therefore represents the whole polygon exactly. Thin objects retain a
    support-plane padding. Inserting edge midpoints preserves the solid.
    """
    xy=np.asarray(xy,dtype=float)
    polygon=[np.asarray(p,dtype=float) for p in corners]
    axis=polygon[1]-polygon[0];axis/=np.linalg.norm(axis)
    normal=np.array((-axis[1],axis[0]))
    for a,b in ((1,1),(1,-1),(-1,1),(-1,-1)):
        direction=a*axis+b*normal
        limit=float(np.max(xy@direction))+min_extent_m+1e-9
        output=[]
        for p,q in zip(polygon,polygon[1:]+polygon[:1]):
            u=float(p@direction-limit);v=float(q@direction-limit)
            if u<=0:output.append(p)
            if (u<0<v) or (v<0<u):output.append(p+u/(u-v)*(q-p))
        polygon=output
    while len(polygon)<8:
        i=max(range(len(polygon)),key=lambda k:np.linalg.norm(polygon[k]-polygon[(k+1)%len(polygon)]))
        polygon.insert(i+1,(polygon[i]+polygon[(i+1)%len(polygon)])/2)
    if len(polygon)!=8:raise ValueError('clipped footprint exceeded eight support vertices')
    return tuple(tuple(float(v) for v in p) for p in polygon)


def _seeded_ground(selected,ground,tree,config):
    """Reject only a plane continuous with nearby, already excluded ground.

    Equal total weights keep a large flat roof from drowning out the ground
    seeds. Every seed and cluster point must fit; missing/ambiguous evidence
    retains the obstacle. No elevated-plane-only ground inference is allowed.
    """
    near=sorted({i for row in tree.query_ball_point(selected[:,:2],config.ground_plane_neighbor_m) for i in row})
    if len(near)<config.ground_plane_min_seed_points:return False
    seeds=ground[near];combined=np.concatenate((selected,seeds))
    weights=np.r_[np.full(len(selected),1/math.sqrt(len(selected))),
                  np.full(len(seeds),1/math.sqrt(len(seeds)))]
    design=np.c_[combined[:,:2],np.ones(len(combined))]
    coefficient,_,rank,_=np.linalg.lstsq(design*weights[:,None],combined[:,2]*weights,rcond=None)
    if rank<3 or np.linalg.norm(coefficient[:2])>config.ground_plane_max_slope:return False
    return bool(np.max(np.abs(combined[:,2]-design@coefficient))<=config.ground_plane_tolerance_m)


def detect_clusters(points, config, *, diagnostics=None, cluster_points=None, footprint_tile_pose=None):
    """Return bounds and centroids of complete 3D radius-connected components."""
    ground_tree=None;ground=np.empty((0,3));rejected=0
    if config.ground_plane_filter_enabled:
        base=filter_points(points,config,keep_ground=True)
        ground=base[base[:,2]<=config.ground_z_threshold]
        xyz=base[base[:,2]>config.ground_z_threshold]
        if len(ground)>=config.ground_plane_min_seed_points:ground_tree=cKDTree(ground[:,:2])
    else:xyz = filter_points(points, config)
    if diagnostics is not None:diagnostics.update(ground_seed_points=len(ground),ground_clusters_rejected=0)
    if len(xyz) < config.min_cluster_points:
        return []
    tree = cKDTree(xyz)
    # The same undirected radius graph as the former per-point Python BFS.
    # Close vehicle faces contain hundreds of thousands of neighbor pairs;
    # compiled graph traversal avoids starving live sensor callbacks there.
    pairs=tree.query_pairs(config.cluster_distance_m,output_type='ndarray')
    graph=coo_matrix((np.ones(len(pairs),dtype=np.uint8),(pairs[:,0],pairs[:,1])),
                     shape=(len(xyz),len(xyz))).tocsr()
    count,labels=connected_components(graph,directed=False)
    order=np.argsort(labels,kind='stable')
    components=np.split(order,np.flatnonzero(np.diff(labels[order]))+1)
    clusters = []
    for component_id,component in enumerate(components):
        if len(component) >= config.min_cluster_points:
            selected = xyz[component]
            if ground_tree is not None and _seeded_ground(selected,ground,ground_tree,config):
                rejected+=1
                if diagnostics is not None:diagnostics['ground_clusters_rejected']=rejected
                continue
            patches=[selected]
            if config.footprint_tile_size_m:
                xy=selected[:,:2]
                if footprint_tile_pose is not None:
                    x,y,yaw=footprint_tile_pose;c,s=math.cos(yaw),math.sin(yaw)
                    vx=xy[:,0]+config.lidar_offset_x_m;vy=xy[:,1]+config.lidar_offset_y_m
                    xy=np.c_[x+c*vx-s*vy,y+s*vx+c*vy]
                keys=np.floor(xy/config.footprint_tile_size_m).astype(np.int64)
                _,labels=np.unique(keys,axis=0,return_inverse=True)
                order=np.argsort(labels,kind='stable')
                patches=[selected[index] for index in np.split(order,np.flatnonzero(np.diff(labels[order]))+1)]
            # Confirmation/noise filtering belongs to the entire connected
            # component. Never drop a thin or sparse patch after subdivision.
            for patch in patches:
                oriented=config.oriented_footprints or config.clipped_footprints
                corners=oriented_bounds(patch[:,:2],config.min_footprint_extent_m) if oriented and not config.footprint_tile_size_m else ()
                if config.footprint_tile_size_m and oriented:
                    # Align the containing box with its fixed world tile.
                    # A minimum-area sensor box can otherwise protrude into
                    # the next tile and change association as Ego rotates.
                    angle=-footprint_tile_pose[2] if footprint_tile_pose is not None else 0.
                    c,s=math.cos(angle),math.sin(angle);axes=np.array(((c,-s),(s,c)))
                    center=patch[:,:2].mean(axis=0);local=(patch[:,:2]-center)@axes
                    low=local.min(axis=0);high=local.max(axis=0)
                    pad=np.maximum(0.,(config.min_footprint_extent_m-(high-low))/2)+1e-9
                    low-=pad;high+=pad
                    corners=tuple(map(tuple,np.array(((low[0],low[1]),(high[0],low[1]),
                        (high[0],high[1]),(low[0],high[1])))@axes.T+center))
                if cluster_points is not None:cluster_points.append(patch)
                clusters.append(LidarCluster(
                    center_xyz=tuple(np.mean(patch, axis=0)),
                    min_xy=tuple(np.min(patch[:, :2], axis=0)),
                    max_xy=tuple(np.max(patch[:, :2], axis=0)),
                    point_count=len(patch),
                    oriented_corners_xy=corners,
                    min_footprint_extent_m=config.min_footprint_extent_m,
                    clipped_vertices_xy=clipped_bounds(patch[:,:2],corners,config.min_footprint_extent_m) if config.clipped_footprints else (),
                    component_id=component_id,
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
