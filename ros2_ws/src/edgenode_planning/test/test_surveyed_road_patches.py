from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
from edgenode_planning.surveyed_road_patches import _chains,local_patches


def boundary(parts):
    return SimpleNamespace(directory=Path('/synthetic'),origin=(0.,0.,0.),_road_protection_parts=parts)


def test_exact_survey_endpoints_join_but_even_a_small_real_gap_stays_open():
    right=(('R',),((0.,-2.,0.),(20.,-2.,0.)))
    first=(('L1',),((0.,2.,0.),(10.,2.,0.)))
    for gap,covered in [(0.,True),(.001,False)]:
        second=(('L2',),((10.+gap,2.,0.),(20.,2.,0.)))
        with patch('edgenode_planning.surveyed_road_patches.read_layer',return_value=[]):
            patches=local_patches(boundary([first,second,right]),[(0.,0.,0.),(20.,0.,0.)])
        region=unary_union([Polygon(p['ring']) for p in patches])
        assert region.covers(Point(10.,0.))==covered


def test_another_known_physical_barrier_inside_a_strip_prevents_road_approval():
    parts=[(('L',),((0.,2.,0.),(20.,2.,0.))),
           (('R',),((0.,-2.,0.),(20.,-2.,0.))),
           (('interior',),((9.,-1.,0.),(9.,1.,0.)))]
    with patch('edgenode_planning.surveyed_road_patches.read_layer',return_value=[]):
        assert not local_patches(boundary(parts),[(0.,0.,0.),(20.,0.,0.)])
