"""Verify actual occlusion ID/observation-age behavior from a closed C trial."""
import argparse,collections,json,pathlib
import numpy as np
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path);p.add_argument('--tag',default='c-v3');a=p.parse_args();d=a.trial;out=d/'occlusion-analysis.json';assert not out.exists()
    summary=json.loads((d/'summary.json').read_text());analysis=json.loads((d/('tracking-analysis-'+a.tag+'.json')).read_text());event=summary['occlusion'];start=event['spawn_t_s'];end=event['destroy_t_s']
    matched=analysis['rows'];before=[r for r in matched if 2<=r['t_s']<start-.1];after=[r for r in matched if end+.3<=r['t_s']<=end+2.5]
    pre=collections.Counter(r['id'] for r in before).most_common(1);post=collections.Counter(r['id'] for r in after).most_common(1);assert pre and post
    key=pre[0][0];session,identifier=key.rsplit(':',1);identifier=int(identifier);trace=[];seen=set()
    for line in (d/'samples-finalized.jsonl').open():
        row=json.loads(line);value=row['values'].get('/dynamic_tracking/tracks',{});stamp=value.get('source_stamp_ns')
        if stamp in seen or value.get('tracker_session_id')!=session:continue
        seen.add(stamp)
        if not start-1.<=row['t']<=end+2.:continue
        t=next((t for t in value.get('tracks',[]) if t['id']==identifier),None)
        if t:trace.append(dict(t_s=row['t'],source_ns=stamp,observed_ns=t['observed_ns'],missed=t['missed'],visibility=t['visibility'],confidence=t['confidence_score'],position_variance=float(np.linalg.eigvalsh(np.asarray(t['covariance'])[:2,:2]).max())))
    missing=[r for r in trace if r['missed']];visible=[r for r in trace if not r['missed'] and r['t_s']<start]
    same=key==post[0][0];held=any(a['observed_ns']==b['observed_ns'] and a['source_ns']!=b['source_ns'] for a,b in zip(missing,missing[1:]))
    result=dict(scope='POST_RUN_REAL_SPAWNED_OCCLUDER_AND_RAW_C_SOURCE_STAMPS',before_id=key,after_id=post[0][0],same_id=same,physical_occlusion_event=event,missed_new_source_frames=len(missing),observation_timestamp_not_refreshed=held,missed_duration_s=missing[-1]['t_s']-missing[0]['t_s'] if len(missing)>1 else 0.,confidence_reduces=bool(missing and visible and min(r['confidence'] for r in missing)<max(r['confidence'] for r in visible)),trace=trace)
    result['success']=same and len(missing)>=2 and held and result['missed_duration_s']>=.15 and result['confidence_reduces']
    out.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2));return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
