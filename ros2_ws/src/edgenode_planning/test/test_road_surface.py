import pytest
from edgenode_planning.road_surface import triangulate
from edgenode_planning.road_corridor import VerifiedRoadCorridor


@pytest.mark.parametrize('reverse',[False,True])
def test_concave_surface_keeps_missing_corner_outside(reverse):
    ring=[(0.,0.),(6.,0.),(6.,2.),(2.,2.),(2.,6.),(0.,6.)]
    if reverse:ring.reverse()
    corridor=VerifiedRoadCorridor(triangulate(ring),source='measured surface',link_ids=['O'],allowed_sides=['left'])
    assert corridor.covers(((.5,.5),(5.,.5),(5.,1.5),(.5,1.5)))
    assert not corridor.covers(((3.,3.),(4.,3.),(4.,4.),(3.,4.)))
    assert not corridor.covers(((1.,1.),(5.,1.),(5.,5.),(1.,5.)))


def test_self_crossing_surface_is_rejected_instead_of_filled():
    with pytest.raises(ValueError):triangulate([(0.,0.),(4.,4.),(0.,4.),(4.,0.)])


def test_exact_union_keeps_interior_holes_and_submillimetre_gaps_after_worker_pickle():
    import pickle
    cells=[((0.,0.),(4.,0.),(4.,1.),(0.,1.)),
           ((0.,3.),(4.,3.),(4.,4.),(0.,4.)),
           ((0.,1.),(1.,1.),(1.,3.),(0.,3.)),
           ((3.,1.),(4.,1.),(4.,3.),(3.,3.))]
    road=VerifiedRoadCorridor(cells,source='measured hole',link_ids=['O'],allowed_sides=['left'],exact_union_backend=True)
    assert road.covers(((.1,.1),(.9,.1),(.9,3.9),(.1,3.9)))
    assert not road.covers(((.1,.1),(3.9,.1),(3.9,3.9),(.1,3.9)))
    restored=pickle.loads(pickle.dumps(road))
    assert not restored.covers(((1.5,1.5),(2.5,1.5),(2.5,2.5),(1.5,2.5)))
    assert restored.cross_section((0.,2.),(1.,0.))==((0.,1.),(3.,4.))
    road=VerifiedRoadCorridor([((0.,0.),(1.,0.),(1.,2.),(0.,2.)),
        ((1.00001,0.),(2.,0.),(2.,2.),(1.00001,2.))],source='measured gap',
        link_ids=['O'],allowed_sides=['left'],exact_union_backend=True)
    assert not road.covers(((.5,.5),(1.5,.5),(1.5,1.5),(.5,1.5)))
    assert road.cross_section((0.,1.),(1.,0.))==((0.,1.),(1.00001,2.))


def test_oblique_shared_triangle_edges_do_not_fragment_a_continuous_section():
    import math
    origin=(111.64202916813603,1174.5740485157346)
    tangent=(.48606772784084834,.8739211428679627)
    normal=(-tangent[1],tangent[0])
    def point(x,y):return (origin[0]+x*tangent[0]+y*normal[0],origin[1]+x*tangent[1]+y*normal[1])
    ring=[point(-30.,-3.),point(30.,-3.),point(30.,15.),point(-30.,15.)]
    middle=point(3.21,2.37)
    cells=[(a,b,middle) for a,b in zip(ring,ring[1:]+ring[:1])]
    road=VerifiedRoadCorridor(cells,source='shared surveyed triangles',link_ids=['O'],allowed_sides=['left'],exact_union_backend=True)
    intervals=road.cross_section(origin,normal)
    assert len(intervals)==1
    assert intervals[0]==pytest.approx((-3.,15.))
