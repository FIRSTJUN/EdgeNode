from types import SimpleNamespace
import pytest
from edgenode_planning.route_clearance import _permission
from unittest.mock import patch


@pytest.mark.parametrize('type_code,source,target,side,allowed',[
    ('224','R','L','left',True),('224','L','R','right',False),
    ('223','L','R','right',True),('223','R','L','left',False),
    ('221','R','L','left',False),('124','R','L','left',False),
    ('324','R','L','left',False)])
def test_only_verified_white_dashed_side_authorizes_directional_merge(type_code,source,target,side,allowed):
    row=dict(ID='surveyed',Type=type_code,Kind='506',R_linkID='R',L_linkID='L')
    boundaries=SimpleNamespace(observed_sides={(source,side):[(row,[[(0.,0.),(10.,0.)]])]})
    assert (_permission(boundaries,source,target,side) is not None)==allowed


def test_ambiguous_sources_cannot_authorize_crossing():
    row=dict(ID='surveyed',Type='224',Kind='506',R_linkID='R',L_linkID='L')
    boundaries=SimpleNamespace(observed_sides={('R','left'):[(row,[[(0.,0.),(10.,0.)]])]*2})
    assert _permission(boundaries,'R','L','left') is None


def test_static_proofs_are_reused_for_mission_suffixes_but_not_another_body():
    from edgenode_planning.route_clearance import correct_route
    from edgenode_planning.vehicle_geometry import sonata_geometry,morai_sonata_collision_envelope
    points=[(0.,0.,0.),(10.,0.,0.)]
    route=dict(link_ids=['L'],points=points,remaining_waypoints=['goal'])
    network=SimpleNamespace(boundaries=SimpleNamespace(),links={'L':dict(points=points,link_type='1')})
    certificate=dict(max_center_shift_m=.1)
    with patch('edgenode_planning.surveyed_road.evidence',return_value={}) as evidence, \
         patch('edgenode_planning.surveyed_road.centered_path',return_value=(points,certificate)) as solve:
        correct_route(route,network,sonata_geometry(),surveyed_centering=True)
        result=correct_route(dict(route,remaining_waypoints=['another']),network,sonata_geometry(),surveyed_centering=True)
        assert evidence.call_count==solve.call_count==1
        assert result['remaining_waypoints']==['another']
        correct_route(route,network,morai_sonata_collision_envelope(),surveyed_centering=True)
        assert evidence.call_count==solve.call_count==2
