"""Changing a live checking horizon must not change the committed vehicle."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from edgenode_planning.avoidance_geometry import intersects
from edgenode_planning.lattice_path import arc
from test_static_obstacle_planner import setup


class BoundaryGuard:
    def __init__(self,check):self.swept_reason=check


def scene():
    p,_,obstacle=setup()
    p.config=replace(p.config,continuous_lane_follow=True,tracking_limit_m=.15)
    p.complete_offset_search=True
    p.selected=SimpleNamespace(mode='lane',complete_return=True)
    path=((0.,0.,0.),(1.1,.05,0.),(2.8,.3,0.),(4.1,.7,0.),
          (5.9,1.4,0.),(8.7,1.8,0.),(12.1,1.9,0.),(18.2,1.9,0.),(24.,1.9,0.))
    return p,path,obstacle


def test_window_reuses_original_whole_body_sweeps_with_their_neighbour_support():
    p,path,_=scene();seen=[]
    def clear(polygon):seen.append(tuple(polygon));return ''
    p.network.boundaries=BoundaryGuard(clear)
    assert p._path_safety(path,(),3.,native_reference=True)==''
    complete=set(seen)
    for low,high in ((.1,8.),(1.25,9.3),(3.15,12.7)):
        seen.clear();p._map_sweep_cache.clear()
        assert p._path_safety(path,(),3.,native_reference=True,geometry_station_range=(low,high))==''
        assert seen and set(seen)<=complete


def test_latest_solid_inside_the_window_still_rejects_the_fixed_maneuver():
    p,path,obstacle=scene()
    solid=obstacle(5.8,6.,1.3,1.5)
    assert p._path_safety(path,(solid,),3.,native_reference=True,
        geometry_station_range=(3.,9.))=='BODY_COLLISION'


def test_forbidden_boundary_inside_the_window_remains_a_hard_gate():
    p,path,_=scene();line=((6.,-10.),(6.,10.))
    p.network.boundaries=BoundaryGuard(lambda polygon:
        'LANE_BOUNDARY_COLLISION:test' if intersects(polygon,line) else '')
    assert p._path_safety(path,(),3.,native_reference=True,
        geometry_station_range=(3.,9.))=='LANE_BOUNDARY_COLLISION:test'


@pytest.mark.parametrize('bounds',[(-1.,8.),(9.,3.),(0.,float('nan')),(0.,1000.)])
def test_invalid_checking_window_is_not_an_empty_safety_proof(bounds):
    p,path,_=scene()
    assert p._path_safety(path,(),3.,native_reference=True,geometry_station_range=bounds)=='INVALID_PATH'
