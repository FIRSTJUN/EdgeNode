"""Read-only same-cycle lane-change decision and timestamp diagnostics."""
import math
from .corridor_scene import json_value
from .corridor_diagnostics import _projection

_FIELDS=('_entry_completion_progress','_recovery_completion_progress',
         '_transition_start_progress','_transition_end_progress','_transition_end_station',
         '_transition_last_parameter','_phase')


def inspect_maneuver(planner,before,result,*,input_odometry_stamp_ns=None,
                     input_receipt_stamp_ns=None,planning_start_stamp_ns=None,output_stamp_ns=None,
                     current_position=None,current_heading=None):
    evaluation=(result or {}).get('maneuver_evaluation') or {}
    corridor=(result or {}).get('corridor_diagnostics') or {}
    adjacent=None;adjacent_heading=None
    geometry=((result or {}).get('corridor_evaluation') or {}).get('adjacent_lane_points')
    if current_position is not None and geometry:
        adjacent=_projection(geometry,current_position)
        if current_heading is not None:
            angle=current_heading-math.atan2(adjacent['tangent'][1],adjacent['tangent'][0])
            adjacent_heading=math.atan2(math.sin(angle),math.cos(angle))
    report=dict(evaluated=result is not None,
        before={k:(before or {}).get(k) for k in _FIELDS},
        after={k:getattr(planner,k,None) for k in _FIELDS},
        decision=dict(getattr(planner,'_maneuver_context',{})) if result is not None else {},candidate=dict(evaluation),
        phase=(result or {}).get('lane_change_phase'),
        candidate_continuous_max_curvature=(result or {}).get('profile_curvature'),
        candidate_clearance=(result or {}).get('evaluated_candidate_clearance'),
        adjacent_center_distance_m=adjacent['distance_m'] if adjacent else None,
        adjacent_heading_error_rad=adjacent_heading,
        target_center_distance_m=evaluation.get('target_distance'),
        target_heading_error_rad=evaluation.get('target_heading_error_rad'),
        recovery_acquired=(result or {}).get('recovery_acquired',False),
        recovery_completion_progress=(result or {}).get('recovery_completion_progress',
            getattr(planner,'_recovery_completion_progress',None)),
        corridor=dict(safe=(result or {}).get('drivable_safe'),reason=corridor.get('reason'),
            first_failed_index=corridor.get('failed_index'),violation_m=corridor.get('max_violation_m'),
            envelope=corridor.get('envelope'),first_failure=corridor.get('first_failure'),
            first_failed_quad_index=(((corridor.get('envelope') or {}).get('failure') or {}).get('quad_index')),
            continuous_witness_violation_m=(corridor.get('continuous_transition_failure') or {}).get('violation_m')),
        timestamps=dict(input_odometry_stamp_ns=input_odometry_stamp_ns,input_receipt_stamp_ns=input_receipt_stamp_ns,
                        planning_start_stamp_ns=planning_start_stamp_ns,output_stamp_ns=output_stamp_ns))
    return json_value(report)
