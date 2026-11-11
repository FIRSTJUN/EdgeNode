"""Strict S4 distance arguments applied to the real Lattice/PP implementation."""
import pytest

from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from edgenode_planning.s4_distance_safety import DistancePolicy, DistanceSafety
from avoidance_world import world


def make(**kwargs):
    lanes, point, obstacle = world(**kwargs)
    planner = StaticObstaclePlanner(lanes)
    reference = [point(float(x), 0.) for x in range(21)]
    preview = [point(float(x), 0.) for x in range(81)]
    model = DistanceSafety(DistancePolicy(), planner.config, planner.body)
    return planner, reference, preview, obstacle, model


@pytest.mark.parametrize('speed', [3., 10.])
def test_exact_control_window_rollout_never_uses_full_preview(speed):
    p, reference, preview, obstacle, model = make()
    p._control_horizon_m = 20.
    assert p._rollout(preview, (0., 0., 0.), speed, 0., ()) == ''


@pytest.mark.parametrize('speed', [20., 50.])
def test_long_preview_cannot_rescue_short_control_window(speed):
    p, reference, preview, obstacle, model = make()
    p._control_horizon_m = 20.
    assert p._rollout(preview, (0., 0., 0.), speed, 0., ()) == 'SHORT_CONTROL_HORIZON'


def test_preview_does_not_change_normal_output_identity():
    p, reference, preview, obstacle, model = make()
    result = p.plan(reference, [], (0., 0., 0.), hazard_reference=preview)
    assert result['points'] is reference and result['state'] == 'NORMAL'


def test_entry_reserve_and_verified_observation_limit_reject_early_known_obstacle():
    p, reference, preview, obstacle, model = make()
    m = model.metrics(3.)
    result = p.plan(reference, [obstacle()], (0., 0., 0.), speed_kmh=3., current_link_id='O0',
        route_link_ids=['O0', 'O1'], hazard_reference=preview, entry_reserve_m=m['entry_reserve_m'],
        stopping_required_m=m['stopping_required_m'], entry_observation_m=21.19)
    result=p.plan(reference,[],(0.,0.,0.),speed_kmh=3.,current_link_id='O0',route_link_ids=['O0','O1'],hazard_reference=preview,entry_reserve_m=m['entry_reserve_m'],entry_observation_m=21.19)
    assert result['blocked'] and not result['points']
    assert any(c.rejection == 'ENTRY_OBSERVATION_DISTANCE' for c in result['candidates'])
    assert all(c.rejection for c in result['candidates'])


def test_unverified_candidate_curvature_scope_is_not_treated_as_straight_observation():
    p, reference, preview, obstacle, model = make()
    result = p.plan(reference, [obstacle()], (0., 0., 0.), speed_kmh=3., current_link_id='O0',
        route_link_ids=['O0', 'O1'], hazard_reference=preview, observation_curvature_limit=.001)
    result=p.plan(reference,[],(0.,0.,0.),speed_kmh=3.,current_link_id='O0',route_link_ids=['O0','O1'],hazard_reference=preview,observation_curvature_limit=.001)
    assert result['blocked'] and not result['points']
    assert any(c.rejection == 'OBSERVATION_CURVATURE_SCOPE' for c in result['candidates'])


def test_long_preview_never_synthesizes_missing_adjacent_support():
    p, reference, preview, obstacle, model = make()
    # Keep original road long; cut off the *actual* adjacent chain mid-maneuver.
    p.network.links['A1']['to_node_idx'] = 'DISCONNECTED'
    p.network.valid.remove('A1')
    result = p.plan(reference, [obstacle()], (0., 0., 0.), speed_kmh=3., current_link_id='O0',
        route_link_ids=['O0', 'O1'], hazard_reference=preview)
    assert result['state']=='DETECT'
    result=p.plan(reference,[],(0.,0.,0.),speed_kmh=3.,current_link_id='O0',route_link_ids=['O0','O1'],hazard_reference=preview)
    assert result['state']=='AVOID_ENTRY' and not result['return_path']


def test_future_return_support_is_not_an_entry_precondition():
    p,reference,preview,obstacle,_=make()
    kwargs=dict(speed_kmh=3.,current_link_id='O0',route_link_ids=['O0','O1'],hazard_reference=preview)
    assert p.plan(reference,[obstacle(74.,78.)],(0.,0.,0.),**kwargs)['state']=='DETECT'
    result=p.plan(reference,[],(0.,0.,0.),**kwargs)
    assert result['state']=='AVOID_ENTRY' and not result['return_path']
