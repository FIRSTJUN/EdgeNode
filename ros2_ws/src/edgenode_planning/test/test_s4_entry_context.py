"""Keep the failed scene identifiable even after the vehicle is relocated."""
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from avoidance_world import world


def test_failed_entry_keeps_link_pose_and_neighbor_diagnostics_until_reset():
    network,point,obstacle=world()
    network.links['O0']['left_lane_change_dst_link_idx']=None
    planner=StaticObstaclePlanner(network)
    reference=[point(i,0.) for i in range(81)]
    args=dict(current_link_id='O0',route_link_ids=['O0','O1'],speed_kmh=3.)
    assert planner.plan(reference,[obstacle()],(0.,0.,0.),**args)['state']=='DETECT'
    result=planner.plan(reference,[],(0.,0.,0.),**args)
    assert result['reason']=='NO_VALID_ADJACENT_LANE'
    context=result['entry_context']
    assert context['current_link_id']=='O0' and context['route_occurrences']==1
    assert context['pose']==(0.,0.,0.)
    assert not context['sides']['left']['pair_available']
    assert context['sides']['left']['permission_reason']=='NO_VALID_ADJACENT_LANE'
    assert planner.plan(reference,[],(40.,0.,0.))['entry_context']==context
    planner.reset()
    result=planner.plan(reference,[],(0.,0.,0.))
    assert result['state']=='NORMAL' and result['speed_cap_kmh'] is None


def test_duplicate_route_is_reported_without_weakening_pair_rejection():
    network,point,obstacle=world();planner=StaticObstaclePlanner(network)
    reference=[point(i,0.) for i in range(81)]
    args=dict(current_link_id='O0',route_link_ids=['O0','O1','O0'],speed_kmh=3.)
    planner.plan(reference,[obstacle()],(0.,0.,0.),**args)
    result=planner.plan(reference,[],(0.,0.,0.),**args)
    assert result['reason']=='NO_VALID_ADJACENT_LANE'
    assert result['entry_context']['route_occurrences']==2
    assert result['entry_context']['sides']['left']['valid_neighbor']=='A0'
    assert not result['entry_context']['sides']['left']['pair_available']
