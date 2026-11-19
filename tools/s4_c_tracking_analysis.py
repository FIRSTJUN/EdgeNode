"""Post-run C scoring against timestamped SDK truth; never a tracker input."""
import argparse,bisect,collections,json,math,pathlib
import numpy as np
from shapely.geometry import Polygon
from scipy.optimize import linear_sum_assignment
from s4_live_trial_report import actor_box_polygon

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path)
    p.add_argument('--tag',default='v1');p.add_argument('--maximum-range-m',type=float,default=50.)
    a=p.parse_args();d=a.trial;out=d/('tracking-analysis-'+a.tag+'.json');assert not out.exists()
    summary=json.loads((d/'summary.json').read_text());assert (d/'bag/metadata.yaml').exists()
    rows=[json.loads(line) for line in (d/'samples-finalized.jsonl').open()]
    actors=[summary['obstacle_state']]+[x['state'] for x in summary.get('additional_obstacles',[])]
    identifiers=[x['actor_info']['id']['value'] for x in actors];series=collections.defaultdict(list);rpc_errors=[]
    for row in rows:
        truth=row.get('all_actor_truth_audit_only')
        if not truth:continue
        ns=int(truth['timestamp']);ns=ns*1000 if ns<10**17 else ns
        interval=row['truth_rpc_interval_ns'];mid=(interval[0]+interval[1])/2
        rpc_errors.append(abs(ns-mid)/1e9)
        for actor in truth['states']:
            name=actor['actor_info']['id']['value']
            if not series[name] or ns>series[name][-1][0]:series[name].append((ns,actor))
    def at(name,ns):
        data=series[name];times=[t for t,v in data];i=bisect.bisect_left(times,ns)
        if i==0 or i==len(data) or times[i]-times[i-1]>350_000_000:return None
        lo,x=data[i-1];hi,y=data[i];u=(ns-lo)/(hi-lo);b=x['bounding_box'];c=y['bounding_box']
        center=np.array([b['location'][k]+u*(c['location'][k]-b['location'][k]) for k in ('x','y')])
        box=dict(b,location=dict(b['location'],x=float(center[0]),y=float(center[1])))
        return center,box,x
    def velocity(name,ns):
        x,y=at(name,ns-150_000_000),at(name,ns+150_000_000)
        return None if x is None or y is None else (y[0]-x[0])/.3
    stationary=bool(summary['trial_spec'].get('stationary_npc'));expected_motion='STATIC' if stationary else 'MOVING'
    placement={identifiers[0]:summary['trial_spec']['obstacle_link']}
    placement.update({item['state']['actor_info']['id']['value']:item['specification'].split(':')[0] for item in summary.get('additional_obstacles',[])})
    stats={name:dict(expected=0,matched=0,observed=0,moving=0,static=0,false_motion_all_matched=0,ids=collections.Counter(),samples=[],bins=collections.defaultdict(lambda:dict(expected=0,matched=0))) for name in identifiers}
    seen=set();source_ready=[];background=[];details=[]
    for row in rows:
        value=row['values'].get('/dynamic_tracking/tracks',{});ns=value.get('source_stamp_ns')
        if ns is None or ns in seen:continue
        seen.add(ns);source_ready.append(bool(value.get('ready')))
        if row['t']<2.:continue
        convergence=row['values']['/localization/grid_convergence_deg'];ego=at('Ego',ns)
        if ego is None:continue
        tracks=value.get('tracks',[]);truth=[]
        for name in identifiers:
            target=at(name,ns);v=velocity(name,ns)
            if target is not None and v is not None:
                center,box,state=target;distance=float(np.linalg.norm(center-ego[0]));truth.append((name,center,box,state,v,distance))
        costs=np.full((len(truth),len(tracks)),1e9)
        for i,(name,center,box,state,v,distance) in enumerate(truth):
            shape=Polygon(actor_box_polygon(box,convergence)).buffer(.35)
            for j,track in enumerate(tracks):
                error=float(np.linalg.norm(np.array(track['center_xy'])-center))
                if error<=4.5 and any(Polygon(part).intersects(shape) for part in track['footprints']):costs[i,j]=error
        matches={i:j for i,j in zip(*linear_sum_assignment(costs)) if costs[i,j]<1e9} if costs.size else {}
        used=set(matches.values());background.append(sum(t['motion']=='MOVING' for j,t in enumerate(tracks) if j not in used))
        for i,(name,center,box,state,v,distance) in enumerate(truth):
            stat=stats[name];bin_name=f'{10*int(distance//10)}-{10*(int(distance//10)+1)}'
            stat['bins'][bin_name]['expected']+=1
            if i in matches:stat['bins'][bin_name]['matched']+=1
            if i in matches:
                t=tracks[matches[i]]
                stat['false_motion_all_matched']+=int(t['motion']==('MOVING' if stationary else 'STATIC'))
            pose=value.get('ego_source_pose');relative_forward=None
            if pose is not None:
                ex,ey,yaw=pose[0];relative_forward=float((center[0]-ex)*math.cos(yaw)+(center[1]-ey)*math.sin(yaw))
            if not (2.5 if stationary else 8.)<=distance<=a.maximum_range_m or relative_forward is not None and relative_forward<(-6. if stationary else 0.):continue
            stat['expected']+=1
            if i not in matches:continue
            t=tracks[matches[i]];observed_age=(ns-t['observed_ns'])/1e9
            if observed_age>.6:continue
            stat['matched']+=1;stat['observed']+=int(not t['missed']);stat['moving']+=int(t['motion']=='MOVING');stat['static']+=int(t['motion']=='STATIC')
            key=value['tracker_session_id']+':'+str(t['id']);stat['ids'][key]+=1
            prediction=[]
            for future in t['future']:
                other=at(name,ns+int(future['seconds']*1e9))
                if other is not None:prediction.append(dict(seconds=future['seconds'],error_m=float(np.linalg.norm(np.asarray(future['center_xy'])-other[0]))))
            ego_velocity=velocity('Ego',ns)
            relative_error=(None if ego_velocity is None else float(np.linalg.norm(
                np.asarray(t['relative_velocity_xy_mps'])-(v-ego_velocity))))
            current=dict(t_s=row['t'],source_ns=ns,distance_m=distance,id=key,motion=t['motion'],missed=t['missed'],visibility=t['visibility'],position_error_m=float(costs[i,matches[i]]),speed_error_mps=abs(t['speed_mps']-float(np.linalg.norm(v))),velocity_vector_error_mps=float(np.linalg.norm(np.asarray(t['velocity_xy_mps'])-v)),relative_velocity_error_mps=relative_error,truth_speed_kmh=float(np.linalg.norm(v))*3.6,estimated_speed_kmh=t['speed_mps']*3.6,lane=t['lane']['link_id'] if t['lane'] else None,truth_lane=state.get('vehicle_state',{}).get('current_link_info',{}).get('id',{}).get('value'),predictions=prediction,source_age_s=value.get('source_age_s'))
            if not current['truth_lane'] and stationary:current['truth_lane']=placement[name]
            stat['samples'].append(current);details.append(dict(actor=name,**current))
    def quantiles(values):return np.percentile(values,[50,95,100]).tolist() if values else None
    results=[]
    for name,stat in stats.items():
        samples=stat['samples'];count=stat['matched'];expected=stat['expected'];dominant=max(stat['ids'].values(),default=0)
        lane=[s for s in samples if s['truth_lane']]
        result=dict(actor_audit_id=name,expected_source_frames=expected,matched_fresh_frames=count,match_fraction=count/max(1,expected),moving_fraction=stat['moving']/max(1,count),false_static_frames=stat['static'],dominant_id_fraction=dominant/max(1,count),ids=dict(stat['ids']),position_error_p50_p95_max_m=quantiles([s['position_error_m'] for s in samples]),speed_error_p50_p95_max_mps=quantiles([s['speed_error_mps'] for s in samples]),velocity_vector_error_p50_p95_max_mps=quantiles([s['velocity_vector_error_mps'] for s in samples]),truth_speed_p50_p95_max_kmh=quantiles([s['truth_speed_kmh'] for s in samples]),lane_match_fraction=sum(s['lane']==s['truth_lane'] for s in lane)/max(1,len(lane)),range_bins=dict(stat['bins']))
        result['static_classified_frames']=stat['static'];result['false_static_frames']=0 if stationary else stat['static']
        result['expected_motion']=expected_motion;result['correct_motion_fraction']=(stat['static'] if stationary else stat['moving'])/max(1,count)
        result['false_motion_all_matched_frames']=stat['false_motion_all_matched']
        result['relative_velocity_error_p50_p95_max_mps']=quantiles([s['relative_velocity_error_mps'] for s in samples if s['relative_velocity_error_mps'] is not None])
        result['future_position_error_by_seconds']={str(seconds):quantiles([p['error_m'] for s in samples for p in s['predictions'] if p['seconds']==seconds]) for seconds in (1.,2.,3.)}
        result['success']=bool(expected>=30 and result['match_fraction']>=.90 and result['correct_motion_fraction']>=.90 and result['dominant_id_fraction']>=.90 and result['false_motion_all_matched_frames']==0 and result['speed_error_p50_p95_max_mps'] is not None and result['speed_error_p50_p95_max_mps'][1]<=(.3 if stationary else 1.) and result['position_error_p50_p95_max_m'][1]<=1.0 and result['lane_match_fraction']>=.90 and result['relative_velocity_error_p50_p95_max_mps'] is not None and result['relative_velocity_error_p50_p95_max_mps'][1]<=1.)
        results.append(result)
    result=dict(scope='POST_RUN_SOURCE_EPOCH_SCORING_ONLY_NO_TRUTH_TO_ALGORITHM',maximum_qualified_range_m=a.maximum_range_m,warmup_s=2.,truth_clock_rpc_offset_p50_p95_max_s=quantiles(rpc_errors),actors=results,unique_source_frames=len(seen),source_ready_fraction=sum(source_ready)/max(1,len(source_ready)),background_moving_count_p50_p95_max=quantiles(background),rows=details)
    result['success']=bool(bool(results) and all(s['success'] for s in results) and result['source_ready_fraction']>=.95 and max(rpc_errors,default=1.)<.15 and (not background or np.percentile(background,95)==0))
    out.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
    return 0 if result['success'] else 1

if __name__=='__main__':raise SystemExit(main())
