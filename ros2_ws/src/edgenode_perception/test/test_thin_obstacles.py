import numpy as np
import pytest
from edgenode_perception.lidar_obstacles import LidarObstacleConfig,detect_clusters
from edgenode_planning.avoidance_geometry import convex_hull,area,convex_covers,obstacle_polygons


@pytest.mark.parametrize('oriented',[False,True])
def test_vertical_pole_emits_nonzero_solid_instead_of_invalid_ready_footprint(oriented):
    xyz=np.array([(10.,2.,z) for z in (-.8,-.6,-.4,-.2,0.)])
    result=detect_clusters(xyz,LidarObstacleConfig(oriented_footprints=oriented))
    assert len(result)==1
    footprint=result[0].footprint_xy();hull=convex_hull(footprint)
    assert len(footprint)==8 and area(hull)>=.0024
    assert convex_covers(hull,[(10.,2.)])
    assert obstacle_polygons([footprint])
