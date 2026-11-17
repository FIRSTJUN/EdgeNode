from dataclasses import replace
from unittest.mock import patch

from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner,AvoidanceConfig
from edgenode_planning.lattice_path import LatticeCandidate
from edgenode_planning.road_corridor import VerifiedRoadCorridor
from avoidance_world import world


def scene(kind):
    network,point,obstacle=world()
    for link in network.links.values():link['link_type']=kind
    network.road_corridor=VerifiedRoadCorridor([((-15.,-8.),(150.,-8.),(150.,8.),(-15.,8.))],
        source='synthetic surveyed road',link_ids=('O0','O1'),allowed_sides=('left','right'))
    planner=StaticObstaclePlanner(network,config=replace(AvoidanceConfig(),integrated_strategy=True))
    reference=[point(i*.5,0.) for i in range(161)]
    planner.plan(reference,[obstacle()],(0.,0.,0.),current_link_id='O0',route_link_ids=['O0','O1'])
    return planner,reference


def test_native_and_offset_solvers_never_share_a_candidate_batch():
    for kind,expected in [('6','lane'),('1','road_offset')]:
        p,reference=scene(kind);modes=[]
        def generate(pair,*args,**kwargs):
            mode=getattr(pair,'mode','lane');modes.append(mode)
            return [LatticeCandidate(pair.side,(),0.,0.,0.,12.,0.,{},'TEST_REJECT',mode=mode)]
        with patch.object(p,'_generate_entry',side_effect=generate):
            p.plan(reference,[],(0.,0.,0.),current_link_id='O0',route_link_ids=['O0','O1'])
        assert modes and set(modes)=={expected}


def test_dynamic_conflict_cannot_run_either_static_solver():
    p,reference=scene('6');p._moving_conflict=True
    with patch.object(p,'_generate_entry') as search:
        result=p.plan(reference,[],(0.,0.,0.),current_link_id='O0',route_link_ids=['O0','O1'])
    search.assert_not_called()
    assert result['selected_strategy']=='DYNAMIC_TRAFFIC'
    assert result['reason']=='DYNAMIC_TRAJECTORY_SEARCH_NOT_READY'
