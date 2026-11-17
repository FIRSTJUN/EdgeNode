from dataclasses import replace
from unittest.mock import Mock
import pytest

from edgenode_planning.maneuver_selection import ManeuverSelection,Situation,Strategy


@pytest.mark.parametrize('scene,expected',[
    (Situation(('m',)),Strategy.NORMAL),
    (Situation(('m',),static_path_blocked=True,lane_topology_clear=True,road_area_verified=True),Strategy.LANE_STATIC),
    (Situation(('m',),static_path_blocked=True,road_area_verified=True),Strategy.OFFSET_STATIC),
    (Situation(('m',),static_path_blocked=True),Strategy.BLOCKED),
    (Situation(('m',),moving_conflict=True,static_path_blocked=True,lane_topology_clear=True),Strategy.DYNAMIC_TRAFFIC),
    (Situation(('m',),moving_conflict=True,sensor_valid=False),Strategy.BLOCKED),
    (Situation(('m',),control_valid=False),Strategy.BLOCKED),
])
def test_one_situation_selects_one_solver(scene,expected):
    arbiter=ManeuverSelection();selection=arbiter.select(scene)
    assert selection.strategy==expected
    solvers={mode:Mock(return_value=mode) for mode in (Strategy.LANE_STATIC,Strategy.OFFSET_STATIC,Strategy.DYNAMIC_TRAFFIC)}
    if expected not in (Strategy.NORMAL,Strategy.BLOCKED):
        assert arbiter.dispatch(selection,solvers)==expected
        assert sum(s.call_count for s in solvers.values())==1


def test_observation_refresh_and_metadata_noise_do_not_restart_committed_path():
    a=ManeuverSelection();s=Situation(('mission','obstacle'),static_path_blocked=True,lane_topology_clear=True)
    chosen=a.select(s);a.commit(chosen,'fixed-entry',path_valid=True)
    refreshed=a.select(replace(s,lane_topology_clear=False,road_area_verified=True))
    assert refreshed.strategy==Strategy.LANE_STATIC and refreshed.generation==chosen.generation
    assert a.path_id=='fixed-entry'
    requested=a.select(replace(s,moving_conflict=True))
    assert requested.strategy==Strategy.DYNAMIC_TRAFFIC
    assert a.owner==Strategy.LANE_STATIC  # Current executor retains its path until a checked handover.
    with pytest.raises(ValueError):a.commit(chosen,'stale',path_valid=True)
    a.commit(requested,'time-validated-entry',path_valid=True)
    assert a.owner==Strategy.DYNAMIC_TRAFFIC


def test_missing_dynamic_solver_cannot_fall_back_to_static_algorithm():
    a=ManeuverSelection();selection=a.select(Situation(('m',),moving_conflict=True,lane_topology_clear=True))
    static=Mock()
    with pytest.raises(ValueError,match='unavailable'):
        a.dispatch(selection,{Strategy.LANE_STATIC:static})
    static.assert_not_called()
