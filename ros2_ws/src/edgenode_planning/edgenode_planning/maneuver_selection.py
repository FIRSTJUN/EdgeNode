"""One maneuver owner and one requested solver, independent of ROS and actors.

All modes share perception, trajectory execution and collision validation.
They never contribute competing paths to a blended candidate pool.
"""
from dataclasses import dataclass
from enum import Enum


class Strategy(str,Enum):
    NORMAL='NORMAL'
    LANE_STATIC='LANE_STATIC'
    OFFSET_STATIC='OFFSET_STATIC'
    DYNAMIC_TRAFFIC='DYNAMIC_TRAFFIC'
    BLOCKED='BLOCKED'


@dataclass(frozen=True)
class Situation:
    context: tuple
    sensor_valid: bool = True
    control_valid: bool = True
    current_path_valid: bool = True
    static_path_blocked: bool = False
    moving_conflict: bool = False
    lane_topology_clear: bool = False
    road_area_verified: bool = False
    replan_required: bool = False
    maneuver_complete: bool = False


@dataclass(frozen=True)
class Selection:
    strategy: Strategy
    reason: str
    generation: int


class ManeuverSelection:
    def __init__(self):
        self.owner=Strategy.NORMAL
        self.path_id=None
        self._key=None
        self._selection=None
        self._generation=0

    def select(self,scene):
        if not scene.sensor_valid or not scene.control_valid:
            strategy,reason=Strategy.BLOCKED,'SENSOR_OR_CONTROL_UNAVAILABLE'
        elif scene.moving_conflict:
            strategy,reason=Strategy.DYNAMIC_TRAFFIC,'MOVING_TRAFFIC_REQUIRES_TIME_PREDICTION'
        elif (self.owner not in (Strategy.NORMAL,Strategy.BLOCKED)
                and scene.current_path_valid and not scene.replan_required and not scene.maneuver_complete):
            strategy,reason=self.owner,'KEEP_COMMITTED_MANEUVER'
        elif scene.static_path_blocked or scene.replan_required:
            if scene.lane_topology_clear:
                strategy,reason=Strategy.LANE_STATIC,'VERIFIED_NATIVE_LANE_TOPOLOGY'
            elif scene.road_area_verified:
                strategy,reason=Strategy.OFFSET_STATIC,'AMBIGUOUS_LANES_VERIFIED_ROAD_AREA'
            else:
                strategy,reason=Strategy.BLOCKED,'NO_VERIFIED_MANEUVER_SPACE'
        elif scene.current_path_valid:
            strategy,reason=Strategy.NORMAL,'CURRENT_GUIDANCE_CLEAR'
        else:
            strategy,reason=Strategy.BLOCKED,'CURRENT_GUIDANCE_INVALID'
        # Observation timestamps are deliberately absent from this key. A
        # fresh measurement is validated, but does not restart identical work.
        key=(scene.context,strategy)
        if key!=self._key:
            self._generation+=1;self._key=key
        self._selection=Selection(strategy,reason,self._generation)
        return self._selection

    def commit(self,selection,path_id,*,path_valid):
        if selection!=self._selection or selection.strategy==Strategy.BLOCKED:
            raise ValueError('obsolete or blocked maneuver selection')
        if path_valid is not True or path_id is None:
            raise ValueError('a current validated path is required')
        self.owner=selection.strategy;self.path_id=path_id

    def dispatch(self,selection,solvers,*args,**kwargs):
        if selection!=self._selection:
            raise ValueError('obsolete maneuver selection')
        if selection.strategy in (Strategy.NORMAL,Strategy.BLOCKED):
            raise ValueError('no trajectory search requested')
        solver=solvers.get(selection.strategy)
        if solver is None:raise ValueError('selected solver is unavailable')
        return solver(*args,**kwargs)
