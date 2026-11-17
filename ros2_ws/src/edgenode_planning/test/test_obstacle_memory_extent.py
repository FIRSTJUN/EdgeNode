"""Conservative local unions must not create a chord across a curved roadside."""
from dataclasses import replace
import math

from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner,AvoidanceConfig
from edgenode_planning.avoidance_geometry import convex_covers,polygon_bounds
from avoidance_world import world


def test_curved_roadside_retains_every_observation_without_bridging_road():
    network,_,_=world()
    patches=[]
    for i in range(53):
        angle=i*math.pi/104;x,y=12*math.cos(angle),12*math.sin(angle)
        patches.append(((x-.05,y-.05),(x+.05,y-.05),(x+.05,y+.05),(x-.05,y+.05)))
    reference=[(9*math.cos(a),9*math.sin(a),0.) for a in [math.pi/6+i*math.pi/600 for i in range(101)]]
    original=StaticObstaclePlanner(network);original._remember(patches)
    assert original._normal_hazards(reference,original.remembered)
    local=StaticObstaclePlanner(network,config=replace(AvoidanceConfig(),memory_merge_extent_m=3.5))
    local._remember(patches)
    assert not local._normal_hazards(reference,local.remembered)
    assert len(local.remembered)>1
    for patch in patches:assert any(convex_covers(solid,patch) for solid in local.remembered)
    for solid in local.remembered:
        x0,x1,y0,y1=polygon_bounds(solid)
        assert max(x1-x0,y1-y0)<=3.5
