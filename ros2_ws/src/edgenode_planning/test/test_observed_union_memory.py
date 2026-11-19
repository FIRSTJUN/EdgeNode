from shapely.geometry import Polygon,Point
from shapely.ops import unary_union

from edgenode_planning.observed_union_memory import ObservedUnionMemory,partition
from edgenode_planning.avoidance_geometry import inflate


def test_curved_observed_wall_retains_occupancy_without_inventing_inside_corner():
    left=((0.,0.),(1.,0.),(1.,4.),(0.,4.))
    top=((0.,3.),(4.,3.),(4.,4.),(0.,4.))
    memory=ObservedUnionMemory();parts=memory.update([left,top],(),.05,0.)
    actual=unary_union([Polygon(p) for p in parts])
    expected=unary_union([Polygon(inflate(p,.05)) for p in [left,top]])
    assert actual.symmetric_difference(expected).area<1e-8
    assert not actual.covers(Point(2.,2.))
    assert actual.convex_hull.covers(Point(2.,2.))
    assert all(Polygon(p).equals(Polygon(p).convex_hull) for p in parts)


def test_occluded_observations_survive_until_explicit_geometric_retirement():
    wall=((0.,0.),(1.,0.),(1.,4.),(0.,4.))
    memory=ObservedUnionMemory();parts=memory.update([wall],(),.05,3.5)
    assert memory.update([],parts,.05,3.5)==parts
    assert memory.update([],(),.05,3.5)==()


def test_holes_are_never_convex_filled_or_lost_in_partition():
    pieces=[((0.,0.),(4.,0.),(4.,1.),(0.,1.)),((0.,3.),(4.,3.),(4.,4.),(0.,4.)),
            ((0.,0.),(1.,0.),(1.,4.),(0.,4.)),((3.,0.),(4.,0.),(4.,4.),(3.,4.))]
    shape=unary_union([Polygon(p) for p in pieces])
    result=unary_union([Polygon(p) for p in partition(shape,pieces)])
    assert result.equals(shape) and not result.covers(Point(2.,2.))


def test_touching_clear_roadside_wall_does_not_propagate_a_captured_path_hazard():
    from dataclasses import replace
    from test_static_obstacle_planner import setup,plan
    planner,point,obstacle=setup()
    planner.config=replace(planner.config,observed_union_memory=True,memory_merge_extent_m=3.5)
    blocking=obstacle(front=40.,rear=44.,low=-1.,high=1.3)
    wall=obstacle(front=0.,rear=40.,low=1.3,high=1.8)
    assert plan(planner,point,[blocking],speed=10.)['state']=='DETECT'
    result=plan(planner,point,[blocking,wall],speed=10.,now=.1)
    assert result['state']=='DETECT' and result['distance_context']['available_distance_m']>30.
    assert not planner._normal_hazards([point(i,0.) for i in range(81)],[wall])


def test_compaction_covers_all_history_and_never_accumulates_its_allowance():
    import math
    from edgenode_planning.observed_union_memory import conservative_compaction
    memory=ObservedUnionMemory();retained=();history=[]
    for frame in range(40):
        # A static thin curved boundary with millimetre measurement jitter.
        lower=[(i*.05, .015*math.sin(i*.7+frame*.2)) for i in range(61)]
        solid=tuple(lower+[(x,y+.12) for x,y in reversed(lower)])
        # These source pieces are convex, as required by the live wire format.
        from edgenode_planning.avoidance_geometry import convex_hull
        observed=convex_hull(solid);history.append(Polygon(inflate(observed,.05)))
        retained=memory.update([observed],retained,.05,3.5)
        raw=unary_union(history);actual=unary_union([Polygon(p) for p in retained])
        assert actual.covers(raw)
        assert raw.buffer(.015,resolution=4).covers(actual)
    assert conservative_compaction(raw).covers(raw)


def test_disarmed_normal_captures_latest_scene_then_arming_keeps_occluded_solids():
    from dataclasses import replace
    from test_static_obstacle_planner import setup
    planner,_,obstacle=setup();planner.config=replace(planner.config,observed_union_memory=True)
    first=obstacle(front=20.,rear=24.);second=obstacle(front=50.,rear=54.)
    planner._motion_authorized=False;planner._remember([first]);planner._remember([second])
    shape=unary_union([Polygon(p) for p in planner.remembered])
    assert not shape.intersects(Polygon(first)) and shape.covers(Polygon(second))
    planner._motion_authorized=True;planner._remember([first]);planner._remember([])
    shape=unary_union([Polygon(p) for p in planner.remembered])
    assert shape.covers(Polygon(first)) and shape.covers(Polygon(second))
    planner.state='BLOCKED';planner._motion_authorized=False;planner._remember([])
    assert unary_union([Polygon(p) for p in planner.remembered]).equals(shape)
