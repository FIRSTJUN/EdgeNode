from dataclasses import replace
from test_static_obstacle_planner import setup


def test_extra_observation_headroom_is_preferred_but_no_supported_alternative_is_removed():
 p,_,_=setup();p.complete_offset_search=True
 p.config=replace(p.config,tracking_limit_m=.15,observation_padding_m=.05)
 values=(2.6,2.8,3.,3.2,4.,10.)
 ordered=p._ordered_road_offsets(values)
 assert ordered[0]==3. and set(ordered)==set(values)
 assert ordered.index(2.6)<ordered.index(10.)


def test_narrow_passage_keeps_its_only_supported_offset_and_legacy_order_is_unchanged():
 p,_,_=setup();p.complete_offset_search=True
 assert p._ordered_road_offsets((2.6,))==(2.6,)
 p.complete_offset_search=False
 assert p._ordered_road_offsets((4.,2.6,3.))==(2.6,3.,4.)
