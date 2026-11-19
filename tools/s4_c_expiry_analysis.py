"""Audit a parked actor leaving the actual rear LiDAR ROI; post-run only."""
import argparse,collections,json,pathlib

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path);p.add_argument('--tag',default='c-v3');a=p.parse_args();d=a.trial;out=d/'expiry-analysis.json';assert not out.exists()
    scored=json.loads((d/('tracking-analysis-'+a.tag+'.json')).read_text());identity=max(scored['actors'][0]['ids'],key=scored['actors'][0]['ids'].get);session,identifier=identity.rsplit(':',1);identifier=int(identifier)
    records=[]
    for line in (d/'samples-finalized.jsonl').open():
        row=json.loads(line);v=row['values'].get('/dynamic_tracking/tracks',{})
        if v.get('tracker_session_id')!=session:continue
        t=next((t for t in v.get('tracks',[]) if t['id']==identifier),None)
        records.append(dict(t_s=row['t'],published_ns=v['published_ns'],track=t))
    observed=[r for r in records if r['track'] and not r['track']['missed']];assert observed
    last=max(r['track']['observed_ns'] for r in observed);missed=[r for r in records if r['track'] and r['track']['missed'] and r['track']['observed_ns']==last]
    after=[r for r in records if (r['published_ns']-last)/1e9>1.3];stale=[r for r in missed if (r['published_ns']-last)/1e9>.6]
    result=dict(scope='POST_RUN_SENSOR_ROI_EXIT_EXPIRY_NO_ACTOR_FEEDBACK',identity=identity,last_observed_ns=last,missed_samples=len(missed),retained_age_max_s=max(((r['published_ns']-last)/1e9 for r in missed),default=0.),stale_samples=len(stale),stale_usable_count=sum(bool(r['track']['usable']) for r in stale),after_expiry_samples=len(after),remaining_after_1_3_s=sum(r['track'] is not None for r in after),source_observation_unchanged=all(r['track']['observed_ns']==last for r in missed))
    result['success']=bool(missed and stale and len(after)>=5 and result['stale_usable_count']==0 and result['remaining_after_1_3_s']==0 and result['retained_age_max_s']<=1.2 and result['source_observation_unchanged'])
    out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
