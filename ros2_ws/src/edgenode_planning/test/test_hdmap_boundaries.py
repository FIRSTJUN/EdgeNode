"""Original K-City boundary evidence supplements, never replaces, topology."""
from pathlib import Path
import pytest
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.hdmap_audit import read_layer, translated

ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture(scope='module')
def boundaries():
    return HDMapBoundaries(ROOT/'local_data/kcity_hdmap', ROOT/'local_data/kcity_mgeo')


def test_new_obstacle_has_white_dashed_right_lane_but_no_left_candidate(boundaries):
    network = LaneNetwork(DijkstraPlanner(ROOT/'local_data/kcity_mgeo'))
    network.boundaries = boundaries
    assert network.neighbor('A219BS010428', 'right') == 'A219BS010427'
    assert network.neighbor('A219BS010428', 'left') is None
    assert boundaries.permission_reason('A219BS010428', 'A219BS010427') == ''
    assert boundaries.permission_reason('A219BS010428', 'unmapped') == 'HDMAP_CENTERLINE_MISMATCH'


def test_explicit_denial_wins_over_a_white_dashed_boundary(boundaries):
    planner = DijkstraPlanner(ROOT/'local_data/kcity_mgeo')
    network = LaneNetwork(planner); network.boundaries = boundaries
    planner.links_by_id['A219BS010428']['can_move_right_lane'] = False
    assert network.permission_reason('A219BS010428', 'right') == 'LANE_CHANGE_FORBIDDEN'


def test_swept_vehicle_touching_the_actual_center_divider_is_rejected(boundaries):
    import json
    origin = json.loads((ROOT/'local_data/kcity_mgeo/global_info.json').read_text())['local_origin_in_global']
    feature = next(f for f in read_layer(ROOT/'local_data/kcity_hdmap/B2_SURFACELINEMARK')
                   if f['attributes']['ID'] == 'B219BS011437')
    a, b = translated(feature, origin)[0][:2]
    x, y = (a[0]+b[0])/2., (a[1]+b[1])/2.
    polygon = ((x-.2,y-.2),(x+.2,y-.2),(x+.2,y+.2),(x-.2,y+.2))
    assert boundaries.swept_reason(polygon).startswith('LANE_BOUNDARY_COLLISION:')
    assert boundaries.swept_reason(((-500.,-500.),(-499.,-500.),(-499.,-499.),(-500.,-499.))) == ''


def test_observed_outer_markings_prove_only_their_bounded_ordinary_lane_pair(boundaries):
    import math
    from edgenode_planning.lane_geometry import sample
    from edgenode_planning.vehicle_geometry import sonata_geometry
    network=LaneNetwork(DijkstraPlanner(ROOT/'local_data/kcity_mgeo'));network.boundaries=boundaries
    pair=network.pair('A219BS010314','left',['A219BS010314'])
    road=boundaries.road_corridor_for_pair(network,pair)
    assert road is not None and road.authorizes(pair.ids,'left')
    assert not road.authorizes(pair.ids,'right')
    p,t=sample(pair.original,45.);yaw=math.atan2(t[1],t[0]);body=sonata_geometry()
    shifted=lambda offset:body.polygon(p[0]-offset*t[1],p[1]+offset*t[0],yaw)
    assert road.covers(shifted(4.))
    assert not road.covers(shifted(8.))
    p,t=sample(pair.original,100.)
    assert not road.covers(body.polygon(*p[:2],math.atan2(t[1],t[0])))


def test_missing_or_ambiguous_outer_marking_never_becomes_a_road_surface(boundaries):
    from copy import deepcopy
    network=LaneNetwork(DijkstraPlanner(ROOT/'local_data/kcity_mgeo'))
    local=deepcopy(boundaries);local._road_corridor_cache.clear();network.boundaries=local
    pair=network.pair('A219BS010314','left',['A219BS010314'])
    local.observed_sides.pop(('A219BS010315','left'))
    assert local.road_corridor_for_pair(network,pair) is None


def test_explicit_return_denial_invalidates_even_a_cached_offset_certificate(boundaries):
    network=LaneNetwork(DijkstraPlanner(ROOT/'local_data/kcity_mgeo'));network.boundaries=boundaries
    pair=network.pair('A219BS010314','left',['A219BS010314'])
    assert boundaries.road_corridor_for_pair(network,pair) is not None
    network.links['A219BS010315']['can_move_right_lane']=False
    assert boundaries.road_corridor_for_pair(network,pair) is None


def test_current_unmarked_intersection_cannot_supply_an_offset_certificate(boundaries):
    network=LaneNetwork(DijkstraPlanner(ROOT/'local_data/kcity_mgeo'));network.boundaries=boundaries
    assert network.pair('A219BS010469','right',['A219BS010469']) is None


def native_ring_world():
    from types import SimpleNamespace
    network=SimpleNamespace(valid={'lane'},links={'lane':dict(link_type='6',
        points=[(0.,0.,0.),(10.,0.,0.),(20.,0.,0.)])})
    boundary=HDMapBoundaries.__new__(HDMapBoundaries)
    boundary.aligned_links={'lane'};boundary.observed_sides={}
    for side,y in (('right',-2.5),('left',2.5)):
        boundary.observed_sides['lane',side]=[(dict(ID=side,Kind='505',Type='211'),
            [[(0.,y,0.),(10.,y,0.),(20.,y,0.)]])]
    return boundary,network


def test_single_lane_ring_keeps_measured_caps_and_solid_boundaries():
    from shapely.geometry import Polygon,Point
    boundary,network=native_ring_world()
    key,ring,ids=next(boundary.measured_lane_rings(network))
    assert key=='lane' and ids==('right','left')
    polygon=Polygon(ring)
    assert polygon.covers(Point(10.,2.)) and not polygon.covers(Point(21.,0.))
    # Geometry proves the interior; Type 211 has not become crossing permission.
    assert boundary.observed_sides['lane','left'][0][0]['Type']=='211'


@pytest.mark.parametrize('damage',['missing','ambiguous','unknown','caps','intersection','unaligned','self_crossing'])
def test_uncertain_native_rings_never_create_surface_evidence(damage):
    boundary,network=native_ring_world()
    if damage=='missing':boundary.observed_sides.pop(('lane','left'))
    elif damage=='ambiguous':boundary.observed_sides['lane','left']*=2
    elif damage=='unknown':boundary.observed_sides['lane','left'][0][0]['Type']='999'
    elif damage=='caps':boundary.observed_sides['lane','left'][0][1][0][0]=(5.,2.5,0.)
    elif damage=='intersection':network.links['lane']['link_type']='1'
    elif damage=='unaligned':boundary.aligned_links.clear()
    else:boundary.observed_sides['lane','left'][0][1][0][1]=(10.,-3.,0.)
    assert not list(boundary.measured_lane_rings(network))
