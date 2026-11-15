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
