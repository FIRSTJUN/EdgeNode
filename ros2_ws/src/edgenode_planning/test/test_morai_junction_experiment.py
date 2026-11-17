"""Missing junction markings may be experimented with only by explicit opt-in."""
from types import SimpleNamespace
import json
from pathlib import Path
import pytest

from avoidance_world import world


def junction():
    network,_,_=world()
    for link in network.links.values():link['link_type']='1'
    network.boundaries=SimpleNamespace(permission_reason=lambda *args:'SHARED_BOUNDARY_UNAVAILABLE')
    return network


def test_default_remains_closed_and_opt_in_keeps_geometry_reciprocity():
    n=junction()
    assert not n.morai_unmarked_junction_experiment
    assert n.permission_reason('O0','left')=='SHARED_BOUNDARY_UNAVAILABLE'
    n.morai_unmarked_junction_experiment=True
    assert n.permission_reason('O0','left')=='' and n.neighbor('O0','left')=='A0'
    n.links['A0']['right_lane_change_dst_link_idx']=None
    assert n.neighbor('O0','left') is None


@pytest.mark.parametrize('reason',['LANE_BOUNDARY_FORBIDDEN','HDMAP_CENTERLINE_MISMATCH','INVALID_LANE_REFERENCE'])
def test_experiment_never_overrides_positive_denial_or_map_mismatch(reason):
    n=junction();n.morai_unmarked_junction_experiment=True
    n.boundaries.permission_reason=lambda *args:reason
    assert n.permission_reason('O0','left')==reason


def test_explicit_no_and_nonjunction_types_remain_denied():
    n=junction();n.morai_unmarked_junction_experiment=True
    n.links['O0']['can_move_left_lane']=False
    assert n.permission_reason('O0','left')=='LANE_CHANGE_FORBIDDEN'
    del n.links['O0']['can_move_left_lane'];n.links['O0']['link_type']='6'
    assert n.permission_reason('O0','left')=='SHARED_BOUNDARY_UNAVAILABLE'


def test_actual_junction_reference_corner_is_checked_by_physical_pp_only_when_opted_in():
    from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner,AvoidanceConfig
    from edgenode_planning.lane_geometry import PolylineIndex,arc
    from edgenode_planning.lattice_path import center_suffix
    fixture=json.loads((Path(__file__).parent/'fixtures/kcity_junction_hold_20261117.json').read_text())
    points=fixture['points'];pose=fixture['pose'];index=PolylineIndex(points)
    network,_,_=world()
    planner=StaticObstaclePlanner(network,config=AvoidanceConfig(avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    planner.pair=SimpleNamespace(adjacent=points,adjacent_arc=arc(points),adjacent_index=index)
    planner._control_horizon_m=20.
    start=index.project(pose[:2]);path=center_suffix(points,start,start+60.)
    planner._hold_path=path
    assert planner._validate_active(path,pose,10.,0.,{})=='CURVATURE'
    network.morai_unmarked_junction_experiment=True
    assert planner._validate_active(path,pose,10.,0.,{})==''
    # Exercise the actual corner, ~15m ahead, with the full production PP
    # model rather than merely proving its initial two-second straight.
    assert planner._rollout(path,pose,10.,0.,(),duration_s=12.)==''
    assert planner._rollout(path,pose,10.,.3,())=='ACTUAL_CURVATURE'
    class ForbiddenBoundary:
        def swept_reason(self,*args):return 'LANE_BOUNDARY_COLLISION:test'
    network.boundaries=ForbiddenBoundary()
    assert planner._validate_active(path,pose,10.,0.,{}).startswith('LANE_BOUNDARY_COLLISION:')
