"""Diagnostic reads preserve maneuver state and the next planning decision."""
import copy
import math

from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner
from edgenode_planning.corridor_scene import planner_snapshot,json_value
from edgenode_planning.maneuver_diagnostics import inspect_maneuver
from test_adjacent_lane_planner import network,plan


def test_same_cycle_private_state_geometry_and_timestamps_are_read_only():
    p=AdjacentLanePlanner(network(('right',)));before=planner_snapshot(p)
    r=plan(p,speed=3.);after=planner_snapshot(p);result=copy.deepcopy(r)
    debug=inspect_maneuver(p,before,r,input_odometry_stamp_ns=100,input_receipt_stamp_ns=110,
        planning_start_stamp_ns=120,output_stamp_ns=140,current_position=[0.,0.],current_heading=0.)
    assert planner_snapshot(p)==after and r==result
    assert debug['before']['_entry_completion_progress'] is None
    assert debug['after']['_entry_completion_progress']==p._entry_completion_progress
    assert debug['candidate']['transition_length']==r['entry_length']
    assert debug['candidate_continuous_max_curvature']==r['profile_curvature']
    assert debug['decision']['rear_pending']
    assert debug['adjacent_center_distance_m']==3.5
    assert debug['adjacent_heading_error_rad']==0.
    assert debug['timestamps']==dict(input_odometry_stamp_ns=100,input_receipt_stamp_ns=110,
        planning_start_stamp_ns=120,output_stamp_ns=140)
    baseline=copy.deepcopy(p)
    assert json_value(plan(p,1.,-.01,-.01,speed=3.))==json_value(plan(baseline,1.,-.01,-.01,speed=3.))


def test_no_current_result_cannot_report_an_old_evaluation():
    p=AdjacentLanePlanner(network(('right',)));plan(p,speed=3.)
    debug=inspect_maneuver(p,None,None)
    assert not debug['evaluated']
    assert debug['decision']=={}
    assert debug['candidate']=={}
    assert debug['candidate_continuous_max_curvature'] is None
