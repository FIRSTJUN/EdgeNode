import math
import json

import pytest

from edgenode_planning.avoidance_geometry import (
    DrivableArea, intersects, load_drivable_area, obstacle_polygons,
    polygon_distance, swept_body)
from edgenode_planning.vehicle_geometry import sonata_geometry


def box(x0,y0,x1,y1):
    return ((x0,y0),(x1,y0),(x1,y1),(x0,y1))


def test_full_body_corners_and_rotation_match_approved_rear_axle():
    body=sonata_geometry()
    expected=[(-1.109,-.93),(3.790,-.93),(3.790,.93),(-1.109,.93)]
    for p,q in zip(body.polygon(0.,0.,0.),expected):assert p==pytest.approx(q)
    for p,(f,l) in zip(body.polygon(10.,20.,math.pi/2),expected):
        assert p==pytest.approx((10-l,20+f))


def test_solid_containment_rejects_hole_even_with_every_edge_and_vertex_inside():
    road=DrivableArea((box(-10,-10,10,-.2),box(-10,.2,10,10),
                       box(-10,-.2,-.2,.2),box(.2,-.2,10,.2)),'synthetic road with hole')
    assert not road.covers(box(-1,-1,1,1))
    assert road.covers(box(-3,-3,-2,-2))


def test_concave_road_rejects_edge_bridge_with_inside_vertices():
    road=DrivableArea((box(0,0,10,2),box(0,2,2,10)),'synthetic L road')
    assert not road.covers(((1,8),(8,1),(1,1)))


def test_touching_solid_collision_and_between_sample_swept_collision():
    body=sonata_geometry()
    obstacle=box(5,-.5,5.5,.5)
    assert not intersects(body.polygon(0,0,0),obstacle)
    assert not intersects(body.polygon(8,0,0),obstacle)
    assert intersects(swept_body(body,(0,0,0),(8,0,0)),obstacle)
    assert intersects(box(0,0,1,1),box(1,0,2,1))
    assert polygon_distance(box(0,0,1,1),box(2,0,3,1))==pytest.approx(1.)


@pytest.mark.parametrize('points', [[(0,0)]*8,[(0,0),(1,0),(2,0)],[(0,0),(1,1),(math.nan,2)]])
def test_incomplete_degenerate_or_nonfinite_obstacle_is_not_free_space(points):
    with pytest.raises(ValueError):obstacle_polygons([points])


def test_boundary_loader_has_no_inferred_fallback_and_requires_provenance(tmp_path):
    assert load_drivable_area('') is None
    with pytest.raises(ValueError):DrivableArea((box(0,0,1,1),),'')
    path=tmp_path/'road.json'
    path.write_text(json.dumps(dict(frame_id='odom',source='bad frame',cells=[box(0,0,1,1)])))
    with pytest.raises(ValueError):load_drivable_area(path)
    path.write_text(json.dumps(dict(frame_id='map',source='test surveyed cells',cells=[box(0,0,1,1)])))
    assert load_drivable_area(path).covers(box(.1,.1,.9,.9))


def test_cyclic_nonconvex_or_crossing_boundary_cell_is_rejected():
    with pytest.raises(ValueError):DrivableArea((((0,0),(2,2),(0,2),(2,0)),),'bad geometry')
