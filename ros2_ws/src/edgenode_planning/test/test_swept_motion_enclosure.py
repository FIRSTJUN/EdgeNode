"""Independent continuous-body witnesses for the interpolation error bound."""
import json
import math
from pathlib import Path
import random

import pytest

from edgenode_planning.avoidance_geometry import convex_covers,polygon_distance,swept_body,polygon_bounds
from edgenode_planning.static_obstacle_planner import bicycle_step
from edgenode_planning.vehicle_geometry import sonata_geometry


@pytest.mark.parametrize('curvature',[-.16,-.05,-.0001,0.,.0001,.05,.16])
@pytest.mark.parametrize('distance',[.02,.25,2.,8.])
def test_full_body_at_every_intermediate_bicycle_step_is_contained(curvature,distance):
    body=sonata_geometry();start=(101.3,-57.9,.7);end=bicycle_step(start,curvature,distance)
    sweep=swept_body(body,start,end)
    for tick in range(201):
        assert convex_covers(sweep,body.polygon(*bicycle_step(start,curvature,distance*tick/200)))


def test_random_linear_translation_and_yaw_interpolations_stay_inside_the_enclosure():
    rng=random.Random(20261115);body=sonata_geometry()
    for _ in range(150):
        first=(rng.uniform(-10,10),rng.uniform(-10,10),rng.uniform(-math.pi,math.pi))
        angle=rng.uniform(-2.8,2.8);second=(first[0]+rng.uniform(-3,8),first[1]+rng.uniform(-3,8),first[2]+angle)
        sweep=swept_body(body,first,second)
        for tick in range(101):
            t=tick/100;pose=tuple((1-t)*a+t*b for a,b in zip(first,second))
            assert convex_covers(sweep,body.polygon(*pose))


def test_small_rotation_has_no_half_sample_jump_in_lateral_uncertainty():
    body=sonata_geometry();a=(0.,0.,0.)
    straight=polygon_bounds(swept_body(body,a,(.25,0.,0.)))
    turning=polygon_bounds(swept_body(body,a,(.25,0.,1e-7)))
    assert abs(turning[2]-straight[2])<1e-5 and abs(turning[3]-straight[3])<1e-5


def test_recorded_false_stop_is_clear_with_the_same_quarter_metre_safety_floor():
    data=json.loads((Path(__file__).parent/'fixtures/s4_generalized_sweep_stop.json').read_text())
    body=sonata_geometry();a=data['first_pose'];b=data['second_pose'];obstacle=data['obstacle']
    assert polygon_distance(data['swept_polygon'],obstacle)<.25
    sweep=swept_body(body,a,b)
    assert polygon_distance(sweep,obstacle)>.25
    # Independently enumerate actual rigid bodies, rather than copying the
    # enclosure formula into an expected value.
    for tick in range(1001):
        t=tick/1000;pose=tuple((1-t)*p+t*q for p,q in zip(a,b))
        solid=body.polygon(*pose)
        assert convex_covers(sweep,solid)
        assert polygon_distance(solid,obstacle)>.25
