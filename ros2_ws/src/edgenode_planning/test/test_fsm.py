"""Mock-only proposal tests; no ROS transport, simulator, or wall-clock sleeps."""
from dataclasses import replace
import math

import pytest

from edgenode_planning.fsm import (
    LaneInput, ObstacleInput, PlanningConfig, PlanningFSM, PlanningState as S,
)


@pytest.fixture
def config():
    # Arbitrary mock speed scale, NOT an agreed physical unit.
    return PlanningConfig(cruise_speed=2.0, avoid_speed=1.0)


def inputs():
    return LaneInput(0.1, 0.9, 10.0), ObstacleInput(10.0)


@pytest.mark.parametrize('distance,left,right,state', [
    (None, False, False, S.LANE_FOLLOW),
    (5.0, True, False, S.AVOID_LEFT),
    (5.0, False, True, S.AVOID_RIGHT),
    (1.0, True, True, S.STOP),
    (5.0, False, False, S.STOP),
    (5.0, True, True, S.AVOID_LEFT),
])
def test_states_and_recovery(config, distance, left, right, state):
    fsm = PlanningFSM(config)
    lane, clear = inputs()
    out = fsm.step(lane, ObstacleInput(10.0, distance, left, right), now=10.0)
    assert out.state is state
    if state is S.STOP:
        assert (out.target_error, out.target_speed) == (0.0, 0.0)
    else:
        expected = lane.error + ({S.AVOID_LEFT: -1, S.AVOID_RIGHT: 1}.get(state, 0)
                                 * config.avoid_error_offset)
        assert out.target_error == pytest.approx(expected)
        assert out.target_speed == (config.cruise_speed if state is S.LANE_FOLLOW
                                    else config.avoid_speed)
    assert fsm.step(lane, clear, now=10.1).state is S.LANE_FOLLOW


def test_initial_and_no_input():
    fsm = PlanningFSM()
    assert fsm.state is S.LANE_FOLLOW
    for _ in range(10):
        out = fsm.step(now=10.0)
        assert out.state is S.LANE_LOST
        assert (out.target_error, out.target_speed) == (0.0, 0.0)


@pytest.mark.parametrize('lane', [None, LaneInput(0, 0.1, 10),
    LaneInput(math.nan, 1, 10), LaneInput(0, math.nan, 10),
    LaneInput(2, 1, 10), LaneInput(0, 1.1, 10),
    LaneInput(0, 1, 11), LaneInput(0, 1, math.nan)])
def test_invalid_lane_and_recovery(config, lane):
    fsm = PlanningFSM(config)
    good, clear = inputs()
    out = fsm.step(lane, clear, now=10)
    assert out.state is S.LANE_LOST and out.target_speed == 0
    assert fsm.step(good, clear, now=10).state is S.LANE_FOLLOW


@pytest.mark.parametrize('obstacle', [None, ObstacleInput(9), ObstacleInput(11),
    ObstacleInput(math.nan), ObstacleInput(10, math.nan),
    ObstacleInput(10, math.inf), ObstacleInput(10, -1)])
def test_invalid_obstacle_and_recovery(config, obstacle):
    fsm = PlanningFSM(config)
    lane, clear = inputs()
    out = fsm.step(lane, obstacle, now=10)
    assert out.state is S.STOP and out.target_speed == 0
    assert fsm.step(lane, clear, now=10).state is S.LANE_FOLLOW


@pytest.mark.parametrize('which,state', [('lane', S.LANE_LOST), ('obstacle', S.STOP)])
def test_timeout_boundary(config, which, state):
    config = replace(config, lane_timeout_sec=1, obstacle_timeout_sec=1)
    fsm = PlanningFSM(config)
    lane, obstacle = inputs()
    assert fsm.step(lane, obstacle, now=10.999).state is S.LANE_FOLLOW
    if which == 'lane':
        obstacle = replace(obstacle, received_at=11)
    else:
        lane = replace(lane, received_at=11)
    out = fsm.step(lane, obstacle, now=11)
    assert out.state is state and out.target_speed == 0
    assert fsm.step(replace(lane, received_at=11),
                    replace(obstacle, received_at=11), now=11).state is S.LANE_FOLLOW


def test_stop_priority_and_direction_changes(config):
    fsm = PlanningFSM(config)
    lane, _ = inputs()
    assert fsm.step(None, ObstacleInput(10, 1), now=10).state is S.STOP
    assert fsm.step(lane, ObstacleInput(10, 5, False, True), now=10).state is S.AVOID_RIGHT
    assert fsm.step(lane, ObstacleInput(10, 5, True, True), now=10).state is S.AVOID_RIGHT
    assert fsm.step(lane, ObstacleInput(10, 5, True, False), now=10).state is S.AVOID_LEFT
    assert fsm.step(lane, ObstacleInput(10, 1), now=10).state is S.STOP


@pytest.mark.parametrize('field,value', [('lane_timeout_sec', 0),
    ('obstacle_timeout_sec', -1), ('min_lane_confidence', 2),
    ('stop_distance', 8), ('avoid_distance', math.nan),
    ('avoid_error_offset', 2), ('cruise_speed', -1), ('avoid_speed', 3)])
def test_invalid_config(config, field, value):
    with pytest.raises(ValueError):
        replace(config, **{field: value})


def test_clamping_and_safe_defaults():
    fsm = PlanningFSM()
    out = fsm.step(LaneInput(-1, 1, 0), ObstacleInput(0, 5, True), now=0)
    assert out.target_error == -1 and out.target_speed == 0
    assert fsm.step(*inputs(), now=float('nan')).target_speed == 0


@pytest.mark.parametrize('source', list(S))
@pytest.mark.parametrize('destination', list(S))
def test_every_state_to_every_state(config, source, destination):
    lane, clear = inputs()
    samples = {
        S.LANE_FOLLOW: (lane, clear),
        S.AVOID_LEFT: (lane, ObstacleInput(10, 5, True, False)),
        S.AVOID_RIGHT: (lane, ObstacleInput(10, 5, False, True)),
        S.STOP: (lane, ObstacleInput(10, 1)),
        S.LANE_LOST: (None, clear),
    }
    fsm = PlanningFSM(config)
    assert fsm.step(*samples[source], now=10).state is source
    out = fsm.step(*samples[destination], now=10)
    assert out.state is destination
    if destination in (S.STOP, S.LANE_LOST):
        assert (out.target_error, out.target_speed) == (0, 0)
