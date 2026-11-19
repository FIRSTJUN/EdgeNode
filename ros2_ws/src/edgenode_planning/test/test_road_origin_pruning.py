from types import SimpleNamespace
from unittest.mock import patch
from test_static_obstacle_planner import setup


def test_missing_origin_containment_prunes_curves_without_calling_them_impossible():
    planner,point,_=setup()
    pair=planner.network.pair('O0','left',['O0','O1'])
    pair.road_area=SimpleNamespace(covers=lambda polygon:False)
    with patch.object(planner,'_preparation',side_effect=AssertionError('must prune before curve search')):
        candidates=planner._generate_entry(pair,(0.,0.,0.),3.,0.,35.)
    assert [c.rejection for c in candidates]==['ROAD_START_BODY_UNPROVEN']
    assert planner.generated_transition_count==0
