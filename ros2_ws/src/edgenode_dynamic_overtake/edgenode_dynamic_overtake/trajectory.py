"""Native-lane entry/hold/return geometry with finite mission return space."""
import math
from edgenode_planning.lane_geometry import PolylineIndex,sample,arc,curvature_at
from edgenode_planning.lattice_path import transition,center_suffix
from edgenode_static20_experiment.entry_search import matched_approach_prefix

APPROACH_M=4.0

class ManeuverInfeasible(ValueError):
    def __init__(self,reason,**context):
        super().__init__(reason);self.context=context

def transition_length(pair,pose,speed_kmh):
    origin=pair.original_index.project(pose[:2]);point=sample(pair.original,origin)[0]
    native=getattr(pair,'native_adjacent',pair.adjacent);index=PolylineIndex(native)
    original=sample(native,max(0.,min(index.lengths[-1],index.project(point))))[0]
    shifted=sample(pair.adjacent,max(0.,min(pair.adjacent_index.lengths[-1],pair.adjacent_index.project(point))))[0]
    width=math.dist(point[:2],original[:2]);distance=math.dist(point[:2],shifted[:2])
    return max(22.,4.*speed_kmh/3.6)*math.sqrt(max(1.,distance/max(.1,width)))

def maneuver(pair,pose,curvature,speed_kmh,lead,body,*,return_gap_m=8.,minimum_handoff_tail_m=0.,initial_speed_kmh=None):
    v=speed_kmh/3.6;origin=pair.original_index.project(pose[:2]);lead_station=pair.original_index.project(lead['center_xy'])
    row=sample(pair.original,origin);lead_speed=sum(a*b for a,b in zip(lead['velocity_xy_mps'],row[1]))+min(.3,lead.get('motion_sigma_mps') or 0.)
    if lead_speed<.3 or v-lead_speed<1.:raise ValueError('D_NO_POSITIVE_PASSING_SPEED')
    length=transition_length(pair,pose,speed_kmh);entry_station=origin+APPROACH_M+length
    if entry_station>=pair.length:
        raise ManeuverInfeasible('D_ENTRY_SPACE_UNPROVEN',origin_s=origin,required_entry_s=entry_station,available_finish_s=pair.length)
    adjacent=pair.adjacent_index.project(sample(pair.original,entry_station)[0])
    # An estimated completion time reserves map space; the actual front gap
    # is re-proved from fresh observations before the fixed return begins.
    separation=return_gap_m+body.rear_overhang_m+lead['vehicle_length_model_m']/2+1.
    initial=v if initial_speed_kmh is None else max(0.,min(v,initial_speed_kmh/3.6))
    # Space reservation includes acceleration from the measured approach
    # speed. Constant requested speed from the first point starts RETURN too
    # early after a slow tracking warmup. This nominal .8 m/s² model grants no
    # motion permission; actual timed sweeps and the fresh >=8m gap still gate.
    acceleration_time_penalty_s=(v-initial)**2/(2*.8*v)
    duration=(lead_station-origin+separation+lead_speed*acceleration_time_penalty_s)/(v-lead_speed)
    return_station=max(origin+v*duration,entry_station+v*2.)
    end=return_station+length
    # Native tail covers the unchanged live timed/stopping horizon until the
    # measured return handoff. Normal driving then supplies its own speed plan.
    horizon=max(2.,.3+v/(2*4.)+.6)
    tail_m=max(body.wheelbase_m+body.front_overhang_m+3.,v*horizon+3.,minimum_handoff_tail_m)
    required=end+tail_m
    if required>pair.length:
        raise ManeuverInfeasible('D_RETURN_SPACE_UNPROVEN',origin_s=origin,lead_s=lead_station,
            lead_upper_speed_mps=lead_speed,ego_speed_mps=v,required_finish_s=required,
            available_finish_s=pair.length,transition_m=length,return_start_s=return_station)
    approach=matched_approach_prefix(pair.original,pose,curvature,origin,APPROACH_M)
    future,tangent=sample(pair.original,origin+APPROACH_M)
    entry=transition(future[:2],math.atan2(tangent[1],tangent[0]),curvature_at(pair.original,origin+APPROACH_M),pair.adjacent,adjacent,length)
    adjacent_return=pair.adjacent_index.project(sample(pair.original,return_station)[0]);hold=center_suffix(pair.adjacent,adjacent,adjacent_return)
    p,t=sample(pair.adjacent,adjacent_return);back=transition(p[:2],math.atan2(t[1],t[0]),curvature_at(pair.adjacent,adjacent_return),pair.original,end,length)
    tail=center_suffix(pair.original,end,end+tail_m)
    fixed=approach+entry[1:]+hold[1:]+back[1:]+tail[1:]
    return dict(points=fixed,native_handoff_tail_m=tail_m,approach_end_s=origin+APPROACH_M,entry=entry,hold=hold,back=back,entry_end_s=entry_station,return_start_s=return_station,return_end_s=end,transition_m=length,expected_passing_time_s=duration+acceleration_time_penalty_s,acceleration_time_penalty_s=acceleration_time_penalty_s,reserved_initial_speed_kmh=initial*3.6,lead_forward_mps=lead_speed,required_return_gap_m=return_gap_m)
