"""Proposed internal interface and FSM; no ROS wire contract or MORAI dependency."""
from dataclasses import dataclass
from enum import Enum
import math


class PlanningState(str, Enum):
    LANE_FOLLOW = 'LANE_FOLLOW'
    AVOID_LEFT = 'AVOID_LEFT'
    AVOID_RIGHT = 'AVOID_RIGHT'
    STOP = 'STOP'
    LANE_LOST = 'LANE_LOST'


@dataclass(frozen=True)
class PlanningConfig:
    """Proposal only. Nonzero speed must be explicitly chosen by a mock caller.

    Distances: proposed metres; time: local monotonic seconds.
    Error: proposed normalized image error, right positive.
    Speed unit is unresolved; runtime defaults remain zero.
    """
    lane_timeout_sec: float = 0.6
    obstacle_timeout_sec: float = 0.6
    min_lane_confidence: float = 0.5
    stop_distance: float = 2.5
    avoid_distance: float = 8.0
    avoid_error_offset: float = 0.32
    cruise_speed: float = 0.0
    avoid_speed: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Configuration must be finite')
        if min(self.lane_timeout_sec, self.obstacle_timeout_sec) <= 0:
            raise ValueError('Timeouts must be positive')
        if not 0 <= self.min_lane_confidence <= 1:
            raise ValueError('Confidence threshold must be in [0, 1]')
        if not 0 <= self.stop_distance < self.avoid_distance:
            raise ValueError('Require 0 <= stop distance < avoid distance')
        if not 0 <= self.avoid_error_offset <= 1:
            raise ValueError('Error offset must be in [0, 1]')
        if not 0 <= self.avoid_speed <= self.cruise_speed:
            raise ValueError('Require 0 <= avoid speed <= cruise speed')


@dataclass(frozen=True)
class LaneInput:
    """Proposed coherent sample; separate ROS messages need a future synchronizer."""
    error: float
    confidence: float
    received_at: float


@dataclass(frozen=True)
class ObstacleInput:
    """Proposed front-corridor assessment, NOT the existing cluster array.

    None distance means an explicit fresh clear observation, not missing data.
    Left/right clearance is supplied by mocks; never inferred from one cluster.
    """
    received_at: float
    front_distance: float | None = None
    left_clear: bool = False
    right_clear: bool = False


@dataclass(frozen=True)
class PlanningOutput:
    state: PlanningState
    target_error: float = 0.0
    target_speed: float = 0.0


class PlanningFSM:
    def __init__(self, config: PlanningConfig | None = None):
        self.config = config or PlanningConfig()
        self.state = PlanningState.LANE_FOLLOW

    def step(self, lane: LaneInput | None = None,
             obstacle: ObstacleInput | None = None, *, now: float) -> PlanningOutput:
        """Proposed priority: obstacle safety, lane validity, avoidance, following.

        Initial label is LANE_FOLLOW; the first missing-lane evaluation is LANE_LOST.
        All recovery requires currently valid fresh observations, without latching.
        TODO: agree hysteresis, STOP release policy and production input adapter.
        """
        c = self.config

        def fresh(stamp, timeout):
            return math.isfinite(now) and math.isfinite(stamp) and 0 <= now - stamp < timeout

        def emit(state, error=0.0, speed=0.0):
            self.state = state
            return PlanningOutput(state, max(-1.0, min(1.0, error)), speed)

        lane_ok = (lane is not None and fresh(lane.received_at, c.lane_timeout_sec)
                   and math.isfinite(lane.error) and -1 <= lane.error <= 1
                   and math.isfinite(lane.confidence)
                   and c.min_lane_confidence <= lane.confidence <= 1)
        obstacle_ok = (obstacle is not None
                       and fresh(obstacle.received_at, c.obstacle_timeout_sec))
        if obstacle_ok:
            d = obstacle.front_distance
            if d is not None and (not math.isfinite(d) or d < 0):
                return emit(PlanningState.STOP)
            if d is not None and d <= c.stop_distance:
                return emit(PlanningState.STOP)
        if not lane_ok:
            return emit(PlanningState.LANE_LOST)
        if not obstacle_ok:
            return emit(PlanningState.STOP)
        d = obstacle.front_distance
        if d is not None and d <= c.avoid_distance:
            # Hold the previous direction when both sides are available.
            if obstacle.left_clear is True and (
                    obstacle.right_clear is not True or self.state != PlanningState.AVOID_RIGHT):
                return emit(PlanningState.AVOID_LEFT,
                            lane.error - c.avoid_error_offset, c.avoid_speed)
            if obstacle.right_clear is True:
                return emit(PlanningState.AVOID_RIGHT,
                            lane.error + c.avoid_error_offset, c.avoid_speed)
            return emit(PlanningState.STOP)
        return emit(PlanningState.LANE_FOLLOW, lane.error, c.cruise_speed)
