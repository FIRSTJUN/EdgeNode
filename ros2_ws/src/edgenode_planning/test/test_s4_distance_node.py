"""S4 adapter tests: all outputs mocked, all visibility/map data synthetic."""
from dataclasses import replace
from unittest.mock import Mock, patch

import pytest
from std_srvs.srv import Trigger

from edgenode_planning.lane_geometry import arc
from test_static_obstacle_node import node as node_fixture, output, arm_isolated_node
from test_s4_sensor_node import clock as clock_fixture, feed, assert_stop

pytestmark = pytest.mark.parametrize('node', [True], indirect=True)
node = node_fixture
clock = clock_fixture


def test_unvalidated_roi_cannot_admit_even_valid_empty_or_reset(node, clock):
    node.distance_safety.evidence = None
    feed(node, clock); node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_DISTANCE_OBSERVATION_EVIDENCE_UNAVAILABLE'
    assert not node.reset_avoidance(Trigger.Request(), Trigger.Response()).success


def test_long_preview_preserves_normal_twenty_metre_output_and_route_progress(node, clock):
    feed(node, clock)
    original = node.avoidance_planner.plan
    node.avoidance_planner.plan = Mock(wraps=original)
    arm_isolated_node(node); node.plan()
    kwargs = node.avoidance_planner.plan.call_args.kwargs
    assert arc(kwargs['hazard_reference'])[-1] == pytest.approx(80.)
    points = node.avoidance_planner.plan.call_args.args[0]
    assert arc(points)[-1] == pytest.approx(20.)
    assert output(node, 'local_path').poses[-1].pose.position.x == pytest.approx(20.)
    assert kwargs['control_horizon_m'] == 20.
    assert output(node, 'target_speed').data == 3.
    assert node._progress_arc_m == pytest.approx(10.)  # Synthetic route starts at x=-10.
    node.plan(); node.dijkstra_planner.plan.assert_called_once()


def test_hazard_outside_control_window_is_checked_without_publishing_preview(node, clock):
    feed(node, clock, points=node.test_obstacle(32., 36.))
    node.plan()
    assert node.avoidance_planner.state=='DETECT'
    assert not node.avoidance_planner.candidates
    assert output(node, 'global_path').poses


@pytest.mark.parametrize('speed,blocked', [(3., False), (10., False), (20., True), (50., True)])
@pytest.mark.parametrize('detected', [False, True])
def test_actual_speed_envelope_before_and_after_detection(node, clock, speed, blocked, detected):
    node.latest_speed_kmh = speed
    feed(node, clock, points=node.test_obstacle(32., 36.) if detected else ())
    node.plan()
    if blocked:
        assert_stop(node)
        assert node.avoidance_planner.reason == 'S4_DISTANCE_OBSERVATION_STOPPING_DISTANCE'
    elif detected:
        assert node.avoidance_planner.state=='DETECT'
    else:
        assert node.avoidance_planner.state == 'NORMAL'


def test_short_actual_map_support_does_not_expand_to_requested_preview(node, clock):
    route = node.dijkstra_planner.plan.return_value
    route['points'] = [(float(x), 0., 0.) for x in range(6)]
    feed(node, clock); node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_DISTANCE_MAP_STOPPING_SUPPORT'


def test_unreviewed_route_scope_is_blocked(node, clock):
    node.distance_safety.evidence = replace(node.distance_safety.evidence, route_link_ids=('O0',))
    feed(node, clock); node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_DISTANCE_OBSERVATION_ROUTE_SCOPE'


def test_delayed_detection_cannot_be_repaired_by_immediately_lowering_target(node, clock):
    node.latest_speed_kmh = 10.
    feed(node, clock, points=node.test_obstacle(10., 14.))
    node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_OBSTACLE_STOPPING_DISTANCE'
    node.latest_speed_kmh = 3.
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2); node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_OBSTACLE_STOPPING_DISTANCE'


def test_source_age_is_rechecked_before_final_motion_output(node, clock, monkeypatch):
    node.latest_speed_kmh = 10.
    # At age zero range=16 m is sufficient; at age .59 s it is insufficient.
    node.distance_safety.evidence = replace(node.distance_safety.evidence, forward_range_m=16.)
    feed(node, clock)
    original = node.speed_planner.plan
    def delay(points, **kwargs):
        result = original(points, **kwargs)
        clock.ns += 590_000_000
        return result
    monkeypatch.setattr(node.speed_planner, 'plan', delay)
    node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_DISTANCE_OBSERVATION_STOPPING_DISTANCE'


def test_whole_planning_time_including_route_work_exceeding_budget_latches(node, clock, monkeypatch):
    feed(node, clock)
    elapsed = [0.]
    original = node.map_matcher.match
    def slow_match(*args):
        elapsed[0] += 1.6
        return original(*args)
    monkeypatch.setattr(node.map_matcher, 'match', slow_match)
    with patch('edgenode_planning.planning_node.time.perf_counter', side_effect=lambda: elapsed[0]):
        node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == 'S4_DISTANCE_PLANNING_BUDGET_EXCEEDED'
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2); node.plan(); assert_stop(node)


def test_increased_preview_and_healthy_recovery_do_not_erase_distance_latch(node, clock):
    node.latest_speed_kmh = 50.
    feed(node, clock); node.plan(); assert_stop(node)
    original = node.avoidance_planner.reason
    node.distance_safety.policy = replace(node.distance_safety.policy, preview_m=150.)
    node.latest_speed_kmh = 3.
    clock.ns += 1_000_000
    feed(node, clock, seq=2, frame=2); node.plan(); assert_stop(node)
    assert node.avoidance_planner.reason == original
    assert node.reset_avoidance(Trigger.Request(), Trigger.Response()).success
    node.plan(); assert node.avoidance_planner.state == 'NORMAL'


def test_future_stopping_corridor_curve_must_match_observation_scope(node, clock):
    node.distance_safety.evidence = replace(node.distance_safety.evidence, max_curvature=.001)
    route = node.dijkstra_planner.plan.return_value
    route['points'] = [(float(x), .01*x*x, 0.) for x in range(100)]
    feed(node, clock); node.plan(); assert_stop(node)
    assert node.latest_turn_curvature == 0.  # Straight now cannot approve a future curve.
    assert node.avoidance_planner.reason == 'S4_DISTANCE_OBSERVATION_CURVATURE_SCOPE'
