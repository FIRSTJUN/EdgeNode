from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
from s4_partitioned_surveyed_roads import local_patches


def region(parts):
    b=SimpleNamespace(directory=Path('/synthetic'),origin=(0.,0.,0.),_road_protection_parts=parts)
    with patch('s4_partitioned_surveyed_roads.read_layer',return_value=[]):
        rows=local_patches(b,[(0.,0.,0.),(20.,0.,0.)])
    return unary_union([Polygon(r['ring']) for r in rows])


def test_short_real_curb_and_nearest_face_transition_keep_the_proved_prefix():
    outer=(('outer',),((0.,7.,0.),(20.,7.,0.)))
    inner=(('guardrail',),((8.,2.,0.),(20.,2.,0.)))
    right=(('right',),((0.,-2.,0.),(7.,-2.,0.)))
    continued=(('right2',),((7.,-2.,0.),(20.,-2.,0.)))
    road=region([outer,inner,right,continued])
    assert road.covers(Point(4.,0.)) and road.covers(Point(15.,0.))
    assert road.covers(Polygon([(6.,-1.),(10.,-1.),(10.,1.),(6.,1.)]))
    assert not road.covers(Point(15.,4.))


def test_a_real_millimetre_gap_stays_open_without_artificial_cap_trimming():
    left=(('left',),((0.,2.,0.),(10.,2.,0.)))
    second=(('left2',),((10.001,2.,0.),(20.,2.,0.)))
    right=(('right',),((0.,-2.,0.),(20.,-2.,0.)))
    road=region([left,second,right])
    assert road.covers(Point(10.,0.))
    assert not road.covers(Point(10.0005,0.))


def test_an_interior_physical_barrier_is_never_approved_as_a_road_strip():
    assert region([(('L',),((0.,2.,0.),(20.,2.,0.))),
        (('R',),((0.,-2.,0.),(20.,-2.,0.))),
        (('barrier',),((9.,-1.,0.),(9.,1.,0.)))]).is_empty
