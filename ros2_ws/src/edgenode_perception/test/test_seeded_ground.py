"""A real MORAI threshold-crossing road patch must not become an obstacle."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np

from edgenode_perception.lidar_obstacles import LidarObstacleConfig,detect_clusters,gravity_ground_heights


def patch():
    return np.asarray(json.loads((Path(__file__).parent/'fixtures/kcity_road_threshold_patch.json').read_text())['xyz'])


def config(**kwargs):
    return LidarObstacleConfig(roi_min_x=-8.,roi_max_x=100.,roi_min_y=-40.,roi_max_y=40.,
        oriented_footprints=True,ground_plane_filter_enabled=True,**kwargs)


def test_actual_road_patch_crosses_fixed_cut_but_is_continuous_ground():
    xyz=patch();c=config();stats={}
    assert len(detect_clusters(xyz,replace(c,ground_plane_filter_enabled=False)))==1
    assert detect_clusters(xyz,c,diagnostics=stats)==[]
    assert stats['ground_seed_points']>=5 and stats['ground_clusters_rejected']==1


def test_raised_roof_and_low_obstacle_above_same_ground_are_retained():
    for height in (.2,.7,1.4):
        # Original road height is about -0.98m here. A 20cm raised surface
        # must survive even when its own returns are perfectly planar.
        solid=np.array([(x,y,-.98+height) for x in (54.,54.3,54.6)
                        for y in (-20.,-19.7,-19.4)])
        clusters=detect_clusters(np.r_[patch(),solid],config())
        assert clusters and any(c.point_count>=len(solid) for c in clusters)


def test_no_ground_seeds_never_turn_a_flat_object_into_empty_space():
    solid=np.array([(10.+i*.1,1.+j*.1,-.5) for i in range(3) for j in range(3)])
    assert len(detect_clusters(solid,config()))==1


def test_ambiguous_plane_retains_candidate():
    xyz=patch().copy()
    selected=np.flatnonzero(xyz[:,2]>-1.)
    xyz[selected[::2],2]+=.15
    assert detect_clusters(xyz,config())


def test_actual_roll_pitch_road_patch_is_not_an_obstacle():
    data=json.loads((Path(__file__).parent/'fixtures/kcity_tilted_road_patch.json').read_text())
    raw=np.asarray(data['xyz']);leveled=gravity_ground_heights(raw,data['roll_rad'],data['pitch_rad'])
    assert len(detect_clusters(raw,config()))==3
    assert detect_clusters(leveled,config())==[]
    np.testing.assert_array_equal(raw[:,:2],leveled[:,:2])
    # An elevated object remains present in this same tilted sensor frame.
    import math
    r,p=data['roll_rad'],data['pitch_rad']
    object_points=np.array([(x,y,(-.4+math.sin(p)*x-math.cos(p)*math.sin(r)*y)/(math.cos(p)*math.cos(r)))
        for x in (54.,54.3,54.6) for y in (-20.,-19.7,-19.4)])
    assert detect_clusters(gravity_ground_heights(np.r_[raw,object_points],r,p),config())


def test_actual_overhead_structure_is_outside_vehicle_height_but_car_remains():
    data=json.loads((Path(__file__).parent/'fixtures/kcity_overhead_patch.json').read_text())
    level=gravity_ground_heights(np.asarray(data['xyz']),data['roll_rad'],data['pitch_rad'])
    assert any(c.center_xyz[2]>5. for c in detect_clusters(level,config()))
    c=replace(config(),obstacle_height_filter_enabled=True)
    assert not detect_clusters(level,c)
    vehicle=np.array([(20.+i*.1,0.+j*.1,z) for i in range(3) for j in range(3) for z in (-.6,.2,.6)])
    assert detect_clusters(np.r_[level,vehicle],c)
