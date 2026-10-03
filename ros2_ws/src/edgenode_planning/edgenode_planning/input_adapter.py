"""Adapt the existing unstamped Perception messages to the planning FSM."""
import math

from edgenode_planning.fsm import LaneInput, ObstacleInput


def obstacle_input(data, received_at):
    """Use the reported centroid distance; never infer lateral clearance.

    Perception's exact no-detection sentinel is treated as no detected obstacle.
    It cannot distinguish an empty/invalid cloud from a genuinely clear scene.
    Malformed reports remain a STOP-producing invalid distance.
    """
    invalid = ObstacleInput(received_at, math.nan)
    if len(data) != 5:
        return invalid
    x, y, z, distance, count = data
    if (all(math.isnan(v) for v in (x, y, z))
            and distance == math.inf and count == 0):
        return ObstacleInput(received_at)
    if (not all(math.isfinite(v) for v in data)
            or distance < 0 or count <= 0 or not float(count).is_integer()):
        return invalid
    return ObstacleInput(received_at, distance)


class PerceptionInputs:
    def __init__(self):
        self.error = None
        self.confidence = None
        self.obstacle = None

    def lane(self):
        if self.error is None or self.confidence is None:
            return None
        # The older reception time prevents one stream hiding the other's timeout.
        # Headerless Float32 messages cannot guarantee same-camera-frame pairing.
        return LaneInput(self.error[0], self.confidence[0],
                         min(self.error[1], self.confidence[1]))

    def step(self, fsm, now):
        lane = self.lane()
        obstacle = self.obstacle
        # Runtime contract: any missing/expired perception stream is LANE_LOST.
        # Fresh emergency/invalid obstacle still keeps the FSM's STOP priority.
        if (obstacle is None or not
                0 <= now - obstacle.received_at < fsm.config.obstacle_timeout_sec):
            lane = None
        return fsm.step(lane, obstacle, now=now)
