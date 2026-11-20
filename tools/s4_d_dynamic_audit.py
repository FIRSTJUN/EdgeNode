#!/usr/bin/env python3
"""Audit a CLOSED D trial using time-matched moving Actor boxes, never planning.

The older static mission audit's minimum actor distance uses the spawn box and
is not a moving-vehicle clearance metric. This audit uses each SDK snapshot.
"""
import argparse,json,math
from pathlib import Path
from shapely.geometry import Polygon
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import PolylineIndex
from s4_live_trial_report import actor_box_polygon

def audit(directory,tag,expect_follow=False):
    output=directory/f'dynamic-overtake-audit-{tag}.json';assert not output.exists()
    summary=json.loads((directory/'summary.json').read_text());assert summary.get('rosbag_metadata_present') and summary.get('rosbag_exit_code')==0
    assert (directory/'bag/metadata.yaml').exists()
    rows=[json.loads(line) for line in (directory/'samples-finalized.jsonl').open()]
    initial=summary['obstacle_state'];lead_id=initial['actor_info']['id']['value'];link=summary['trial_spec']['link']
    network=DijkstraPlanner('/workspace/local_data/kcity_mgeo');index=PolylineIndex(network.links_by_id[link]['points'])
    checks=[];events=[];previous=None;rpc_error=[];collisions=[];passed=False;completed=0;static_lateral=[]
    for row in rows:
        truth=row.get('all_actor_truth_audit_only');values=row['values'];debug=values.get('/planning/avoidance_debug',{});state=debug.get('state');strategy=debug.get('selected_strategy');count=debug.get('dynamic_completed',0);completed=max(completed,count)
        key=(state,strategy,count)
        if key!=previous:events.append(dict(t=row['t'],state=state,strategy=strategy,completed=count));previous=key
        if state in ('AVOID_ENTRY','AVOID_HOLD','RETURN') and strategy!='DYNAMIC_OVERTAKE':static_lateral.append(row['t'])
        if not truth:continue
        ns=int(truth['timestamp']);ns=ns*1000 if ns<10**17 else ns
        lo,hi=row['truth_rpc_interval_ns'];rpc_error.append(abs(ns-(lo+hi)/2)/1e9)
        actors={a['actor_info']['id']['value']:a for a in truth['states']}
        if 'Ego' not in actors or lead_id not in actors:continue
        convergence=values['/localization/grid_convergence_deg'];ego=actor_box_polygon(actors['Ego']['bounding_box'],convergence);lead=actor_box_polygon(actors[lead_id]['bounding_box'],convergence)
        gaps={name:Polygon(ego).distance(Polygon(actor_box_polygon(actor['bounding_box'],convergence))) for name,actor in actors.items() if name!='Ego'}
        collisions.extend(dict(t=row['t'],actor=name) for name,gap in gaps.items() if gap<=1e-8)
        front_gap=min(index.project(p) for p in ego)-max(index.project(p) for p in lead)
        if state=='AVOID_HOLD' and front_gap>0:passed=True
        follow_gap=min(index.project(p) for p in lead)-max(index.project(p) for p in ego)
        checks.append(dict(t=row['t'],source_ns=ns,state=state,strategy=strategy,completed=count,actual_front_gap_m=front_gap,actual_follow_gap_m=follow_gap,ego_speed_kmh=values.get('/ego_vehicle_status',{}).get('speed_kmh'),fsm=values.get('/planning/fsm_state'),actual_pair_clearance_m=gaps[lead_id],all_actor_clearance_m=gaps,ego_transform=actors['Ego']['transform'],lead_transform=actors[lead_id]['transform']))
    returns=[r for r in checks if r['state']=='RETURN' and r['strategy']=='DYNAMIC_OVERTAKE'];normal=[r for r in checks if r['state']=='NORMAL' and r['completed']>=1]
    dynamic=[e['state'] for e in events if e['strategy']=='DYNAMIC_OVERTAKE'];required=['AVOID_ENTRY','AVOID_HOLD','RETURN'];ordered=all(x in dynamic for x in required) and [dynamic.index(x) for x in required]==sorted(dynamic.index(x) for x in required)
    minimum_return=min((r['actual_front_gap_m'] for r in returns),default=None);sdk_hold=normal[-1]['t']-normal[0]['t'] if normal else 0.
    # NORMAL continuation is measured by fresh actual Ego status/FSM, not by
    # the cadence of audit-only SDK calls. The successful terminal sample is
    # recorded before break and need not issue another Actor RPC.
    hold=0.;since=None
    for row in rows:
        v=row['values'];d=v.get('/planning/avoidance_debug',{});ages=row.get('ages_s',{})
        valid=(d.get('dynamic_completed',0)>=1 and d.get('state')=='NORMAL' and v.get('/planning/fsm_state')=='NORMAL_DRIVE' and v.get('/ego_vehicle_status',{}).get('speed_kmh',0)>1. and v.get('/planning/s4_sensor_state',{}).get('ready') and all(ages.get(k,1.)<=.6 for k in ('/ego_vehicle_status','/planning/fsm_state','/planning/avoidance_debug','/planning/local_path')))
        if valid:
            if since is None:since=row['t']
            hold=max(hold,row['t']-since)
        else:since=None
    result=dict(scope='POSTHOC_TIMESTAMPED_MOVING_ACTOR_BOXES_NO_TRUTH_INPUT_TO_ALGORITHM',trial=directory.name,lead_audit_id=lead_id,source_frames=len(checks),events=events,actual_pass_during_hold=passed,dynamic_completed=completed,ordered_dynamic_phases=ordered,normal_after_return_hold_s=hold,minimum_actual_return_front_gap_m=minimum_return,required_actual_return_front_gap_m=8.,minimum_timestamped_actor_clearance_m=min((min(r['all_actor_clearance_m'].values()) for r in checks),default=None),actual_actor_overlap_frames=collisions,static_lateral_frames=static_lateral,maximum_truth_rpc_clock_error_s=max(rpc_error,default=None),restored=summary.get('restored'),stop_confirmed=summary.get('final_stop_confirmed'),rows=checks,note='Static mission audit minimum_actual_actor_clearance_m uses a frozen spawn box; use this time-matched metric for moving traffic.')
    result.update(normal_sdk_snapshot_hold_s=sdk_hold,normal_continuation_evidence='CONTINUOUS_FRESH_ACTUAL_EGO_SPEED_AND_NORMAL_FSM_FROM_ROS_SUPERVISION')
    result['success']=bool(len(checks)>=30 and completed>=1 and ordered and passed and returns and minimum_return>=8. and hold>=.9 and not collisions and not static_lateral and max(rpc_error,default=1.)<.15 and summary.get('restored') and summary.get('final_stop_confirmed') and not summary.get('error'))
    if expect_follow:
        moving=[r for r in checks if r['t']>=2.]
        result.update(verification='EXPECTED_NORMAL_FOLLOW_NOT_COMPLETED_OVERTAKE',minimum_actual_follow_gap_m=min((r['actual_follow_gap_m'] for r in checks),default=None),follow_observed_s=summary.get('follow_observed_s'))
        arming=[r for r in checks if r['fsm']=='READY']
        result['stationary_initial_arming_frames']=[dict(t=r['t'],ego_speed_kmh=r['ego_speed_kmh']) for r in arming]
        # The first asynchronous sample can precede the enable acknowledgement.
        # READY is allowed only while physically stopped during initial arming.
        valid_fsm=all(r['fsm']=='NORMAL_DRIVE' or (r['fsm']=='READY' and r['t']<2. and r['ego_speed_kmh']<=.15) for r in checks)
        result['success']=bool(summary.get('success') and summary.get('verification')=='MORAI_NORMAL_DYNAMIC_FOLLOW_NOT_OVERTAKE' and summary.get('follow_observed_s',0)>=19.5 and len(moving)>=30 and valid_fsm and all(r['state']=='NORMAL' and r['completed']==0 for r in checks) and all(r['fsm']=='NORMAL_DRIVE' for r in moving) and sum(r['ego_speed_kmh']>=3. for r in moving)/len(moving)>=.95 and result['minimum_actual_follow_gap_m']>=8. and not collisions and not static_lateral and max(rpc_error,default=1.)<.15 and summary.get('restored') and summary.get('final_stop_confirmed') and not summary.get('error'))
    output.write_text(json.dumps(result,indent=2));return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('--tag',default='v1');p.add_argument('--expect-follow',action='store_true');a=p.parse_args();v=audit(a.trial,a.tag,a.expect_follow);print(json.dumps({k:x for k,x in v.items() if k!='rows'},indent=2));return 0 if v['success'] else 1
if __name__=='__main__':raise SystemExit(main())
