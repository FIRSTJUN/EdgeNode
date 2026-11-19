from dataclasses import replace
import math,random
from types import SimpleNamespace

from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from edgenode_planning.vehicle_geometry import sonata_geometry,morai_sonata_collision_envelope


def test_native_path_union_keeps_every_segment_body_and_reference_floor():
    randomizer=random.Random(2026111816)
    curves=[[(i*.5,0.,0.) for i in range(120)],
        [(30.*math.sin(i*.5/30),30.*(1.-math.cos(i*.5/30)),0.) for i in range(120)]]
    for body in (sonata_geometry(),morai_sonata_collision_envelope()):
        for points in curves:
            old=StaticObstaclePlanner(SimpleNamespace());old.body=body
            native=StaticObstaclePlanner(SimpleNamespace(),config=replace(old.config,native_obstacle_geometry=True));native.body=body
            solids=[]
            for _ in range(120):
                x=randomizer.uniform(-5.,65.);y=randomizer.uniform(-8.,35.)
                w=randomizer.uniform(.05,2.);h=randomizer.uniform(.05,2.)
                solids.append(((x,y),(x+w,y),(x+w,y+h),(x,y+h)))
            assert native._normal_hazards(points,solids)==old._normal_hazards(points,solids)
            for p in [(0.,0.,0.),(20.,3.,.4)]:
                assert native._obstacle_safety(p,p,solids)==old._obstacle_safety(p,p,solids)


def test_native_distance_keeps_body_clearance_and_does_not_replace_axle_floor():
    from edgenode_planning.native_obstacle_geometry import distance
    planner=StaticObstaclePlanner(SimpleNamespace(),config=replace(StaticObstaclePlanner(None).config,native_obstacle_geometry=True))
    for gap in (.2499,.2501):
        x=planner.body.width_m/2+gap
        obstacle=((0.,x),(3.,x),(3.,x+.1),(0.,x+.1))
        assert distance(planner.body.polygon(0.,0.,0.),obstacle)>0.
        reason=planner._obstacle_safety((0.,0.,0.),(0.,0.,0.),[obstacle])
        assert reason==('BODY_COLLISION' if gap<.25 else 'REFERENCE_CLEARANCE')
