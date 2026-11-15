"""Partial car faces must not create PCA corners in the adjacent free lane."""
import math
import numpy as np
import pytest

from edgenode_perception.lidar_obstacles import oriented_bounds
from edgenode_planning.avoidance_geometry import convex_covers,area


@pytest.mark.parametrize('yaw',[-2.1,-.4,0.,.7,1.9])
def test_partial_front_and_side_returns_are_contained_without_density_dependent_heading(yaw):
    c,s=math.cos(yaw),math.sin(yaw);rotation=np.array(((c,-s),(s,c)))
    # The front has many more returns than the side; this changes PCA heading.
    local=np.array([(x,2.5) for x in np.linspace(-1,1,100)]+[(1.,y) for y in np.linspace(-2.5,2.5,11)])
    points=local@rotation.T+np.array((40.,-70.))
    bounds=oriented_bounds(points)
    assert convex_covers(bounds,points)
    recovered=(np.array(bounds)-np.array((40.,-70.)))@rotation
    assert recovered[:,0].min()>=-1.-1e-7 and recovered[:,0].max()<=1.+1e-7
    assert recovered[:,1].min()>=-2.5-1e-7 and recovered[:,1].max()<=2.5+1e-7
    assert area(bounds)==pytest.approx(10.,abs=1e-6)


def test_random_cloud_hull_vertices_remain_inside_the_published_rectangle():
    rng=np.random.default_rng(20261115)
    for _ in range(50):
        points=rng.normal(size=(100,2))*rng.uniform(.1,10,size=2)+rng.uniform(-100,100,size=2)
        bounds=oriented_bounds(points)
        assert convex_covers(bounds,points)


@pytest.mark.parametrize('points',[np.array([[1.,2.]]*5),np.array([[i,2.*i] for i in range(5)])])
def test_point_and_line_clouds_keep_the_nonzero_extent_floor(points):
    bounds=oriented_bounds(points)
    assert convex_covers(bounds,points) and area(bounds)>=.0025
