import math

import pytest

from edgenode_planning.fsm import PlanningConfig, PlanningFSM, PlanningState as S
from edgenode_planning.input_adapter import PerceptionInputs, obstacle_input


def inputs(data=None):
    i = PerceptionInputs()
    i.error = (0.1383698, 10.0)
    i.confidence = (0.9, 10.0)
    i.obstacle = obstacle_input(
        data if data is not None else [math.nan]*3 + [math.inf, 0], 10.0)
    return i


def fsm():
    return PlanningFSM(PlanningConfig(cruise_speed=2.0, avoid_speed=1.0))


def test_clear_and_configured_speed():
    out = inputs().step(fsm(), 10.1)
    assert out.state == S.LANE_FOLLOW
    assert out.target_error == pytest.approx(0.1383698)
    assert out.target_speed == 2.0


@pytest.mark.parametrize('stream', ['error', 'confidence', 'obstacle'])
def test_each_stream_timeout(stream):
    i = inputs()
    if stream == 'obstacle':
        i.obstacle = obstacle_input([math.nan]*3 + [math.inf, 0], 9.0)
    else:
        setattr(i, stream, (getattr(i, stream)[0], 9.0))
    out = i.step(fsm(), 10.1)
    assert out.state == S.LANE_LOST
    assert out.target_speed == 0


def test_low_confidence_and_emergency_priority():
    i = inputs()
    i.confidence = (0.1, 10.0)
    assert i.step(fsm(), 10.1).state == S.LANE_LOST
    i.obstacle = obstacle_input([1, 0, -0.66, 1, 3647], 10)
    assert i.step(fsm(), 10.1).state == S.STOP


def test_reported_cluster_not_filtered_or_assumed_clear():
    i = inputs([2.9376, -0.0635, -0.6647, 2.9383, 3647])
    assert i.obstacle.front_distance == 2.9383
    assert not i.obstacle.left_clear and not i.obstacle.right_clear
    assert i.step(fsm(), 10.1).state == S.STOP


@pytest.mark.parametrize('data', [[], [0]*5, [1, 0, 0, math.inf, 8],
                                  [1, 0, 0, 1, -1], [1, 0, 0, 1, 8.5],
                                  [math.nan, 0, 0, 1, 8]])
def test_malformed_report_stops(data):
    out = inputs(data).step(fsm(), 10.1)
    assert out.state == S.STOP and out.target_speed == 0


def test_missing_and_recovery():
    i = inputs()
    i.obstacle = None
    assert i.step(fsm(), 10.1).state == S.LANE_LOST
    i.obstacle = obstacle_input([10, 0, 0, 10, 8], 10)
    assert i.step(fsm(), 10.1).state == S.LANE_FOLLOW
