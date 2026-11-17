"""Observed MORAI vehicle corners must not fill unobserved rectangle wedges."""
import json
from pathlib import Path

import numpy as np
import pytest

from edgenode_perception.lidar_obstacles import LidarObstacleConfig, detect_clusters, oriented_bounds, clipped_bounds


def contains(polygon,points):
    p=np.asarray(polygon);q=np.roll(p,-1,axis=0)
    delta=np.asarray(points)[None,:,:]-p[:,None,:]
    edge=(q-p)[:,None,:]
    return np.all(edge[:,:,0]*delta[:,:,1]-edge[:,:,1]*delta[:,:,0]>=-1e-8)


def area(points):
    p=np.asarray(points);q=np.roll(p,-1,axis=0)
    return abs(np.sum(p[:,0]*q[:,1]-p[:,1]*q[:,0]))/2


def test_actual_rotated_cluster_keeps_every_return_and_existing_wire_format():
    data=json.loads((Path(__file__).parent/'fixtures/kcity_rotated_vehicle_20261117.json').read_text())
    xyz=np.asarray(data['xyz'])
    config=LidarObstacleConfig(roi_max_x=100.,roi_min_y=-40.,roi_max_y=40.,
                              oriented_footprints=True,clipped_footprints=True)
    clusters=detect_clusters(xyz,config)
    assert len(clusters)==1 and clusters[0].point_count==len(xyz)
    footprint=clusters[0].footprint_xy()
    assert len(footprint)==8 and contains(footprint,xyz[:,:2])
    assert area(footprint)<.85*area(oriented_bounds(xyz[:,:2]))


@pytest.mark.parametrize('angle',[0.,.3,1.2,-2.])
@pytest.mark.parametrize('shape',['point','line','box','triangle'])
def test_clipping_contains_all_returns_for_degenerate_and_rotated_shapes(angle,shape):
    xy={'point':[(5.,2.)], 'line':[(5.,2.),(6.,2.),(7.,2.)],
        'box':[(5.,2.),(7.,2.),(7.,4.),(5.,4.)],
        'triangle':[(5.,2.),(7.,2.),(5.,4.)]}[shape]
    rotation=np.array(((np.cos(angle),-np.sin(angle)),(np.sin(angle),np.cos(angle))))
    xy=np.asarray(xy)@rotation.T
    rectangle=oriented_bounds(xy)
    footprint=clipped_bounds(xy,rectangle)
    assert len(footprint)==8 and contains(footprint,xy)
    assert area(footprint)>0. and area(footprint)<=area(rectangle)+1e-8


def test_world_tiles_keep_sparse_patches_after_component_confirmation():
    xyz=np.array([(2.+i*.5,0.,-.5) for i in range(21)])
    config=LidarObstacleConfig(oriented_footprints=True,clipped_footprints=True,
                              footprint_tile_size_m=3.,min_cluster_points=5)
    patches=[]
    clusters=detect_clusters(xyz,config,cluster_points=patches,footprint_tile_pose=(100.,100.,np.pi/2))
    assert len(clusters)>1 and min(c.point_count for c in clusters)<5
    assert sum(c.point_count for c in clusters)==len(xyz)
    assert sorted(map(tuple,np.concatenate(patches)))==sorted(map(tuple,xyz))
    for cluster,points in zip(clusters,patches):
        assert len(cluster.footprint_xy())==8 and contains(cluster.footprint_xy(),points[:,:2])
