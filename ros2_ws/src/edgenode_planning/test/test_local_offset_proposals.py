from dataclasses import replace
from test_strategy_dispatch import scene
from edgenode_planning.avoidance_geometry import inflate


def test_complete_search_keeps_nearby_proved_offsets_without_removing_distant_support():
    p,reference=scene('1');p.config=replace(p.config,continuous_lane_follow=True)
    road=p.network.road_corridor
    old=p._road_offsets(road,reference,'left');assert len(old)<=9
    p.complete_offset_search=True
    values=p._road_offsets(road,reference,'left')
    assert len(values)>9
    nearby=[x for x in values if x<=values[0]+2.]
    assert len(nearby)>=8 and max(b-a for a,b in zip(nearby,nearby[1:]))<=.201
    assert max(values)>=max(old)-1e-6
    for offset in values:
        solid=p.body.polygon(35.,offset,0.)
        assert road.covers(inflate(solid,2*p.config.tracking_limit_m))
