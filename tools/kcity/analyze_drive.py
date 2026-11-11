"""Compare bounded-runner JSONL on the same map geometry, regardless of route origin.

Only ACTIVE, positive-target, moving samples enter driving metrics. Stops remain
in the original logs/results. Steering rates use CtrlCmd header time if present,
otherwise receive time. All signals join backward in time, with a 0.3s limit.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(paths):
    groups=defaultdict(list)
    for path in paths:
        with Path(path).open() as f:
            for line in f:
                r=json.loads(line);groups[r['topic']].append(r)
    for values in groups.values():values.sort(key=lambda r:r['t'])
    return groups


def join(groups,topic,times,default=None):
    rows=groups.get(topic,[])
    ts=np.array([r['t'] for r in rows]);out=[]
    for t in times:
        i=np.searchsorted(ts,t,side='right')-1
        out.append(rows[i]['value'] if i>=0 and t-ts[i]<.3 else default)
    return out


def map_projection(points,xy):
    """Closest original XY segment near 8 nearest vertices, including corners."""
    pts=np.asarray(points)[:,:2];delta=np.diff(pts,axis=0)
    lengths=np.linalg.norm(delta,axis=1);arcs=np.r_[0,np.cumsum(lengths)]
    _,near=cKDTree(pts).query(xy,k=min(8,len(pts)))
    result=[]
    for q,vertices in zip(xy,near):
        indices=np.unique(np.clip(np.r_[vertices,vertices-1],0,len(delta)-1))
        ds=delta[indices];aa=pts[indices];squared=(ds*ds).sum(axis=1)
        fraction=np.clip(((q-aa)*ds).sum(axis=1)/np.maximum(squared,1e-20),0,1)
        distances=np.linalg.norm(q-aa-fraction[:,None]*ds,axis=1)
        i=np.argmin(distances);idx=indices[i]
        result.append((arcs[idx]+fraction[i]*lengths[idx],distances[i],np.arctan2(delta[idx,1],delta[idx,0])))
    return np.array(result)


def stats(values):
    values=np.asarray(values,dtype=float);values=values[np.isfinite(values)]
    if not len(values):return None
    return dict(count=len(values),mean=float(np.mean(values)),rms=float(np.sqrt(np.mean(values**2))),
                max=float(np.max(values)),p95=float(np.percentile(values,95)),min=float(np.min(values)))


def steering_metrics(t,u,cap):
    if len(t)<2:return None
    dt=np.diff(t);valid=(dt>.005)&(dt<.15)
    rates=np.abs(np.diff(u)[valid]/dt[valid]);duration=float(np.sum(dt[valid]))
    sat=(np.abs(u[:-1])>=cap-.001)&valid
    longest=running=0.
    for saturated,elapsed in zip(sat,dt):
        running=running+elapsed if saturated else 0.;longest=max(longest,running)
    last=0;last_time=None;reversals=0
    for stamp,command in zip(t,u):
        side=1 if command>.05 else -1 if command<-.05 else 0
        if side:
            if last and last!=side and last_time is not None and stamp-last_time<=.5:reversals+=1
            last=side;last_time=stamp
    return dict(rate_abs_per_sec=stats(rates),max_step=float(np.max(np.abs(np.diff(u)[valid]))) if valid.any() else None,
        saturation_ratio=float(np.sum(dt[sat])/duration) if duration else None,
        longest_saturation_sec=longest,total_variation=float(np.sum(np.abs(np.diff(u)[valid]))),
        rapid_direction_reversals=reversals,duration_sec=duration,configured_cap=cap)


def after_startup(groups,times,seconds):
    starts=[];previous=None
    for row in groups['/planning/drive_status']:
        if row['value']=='ACTIVE' and previous!='ACTIVE':starts.append(row['t'])
        previous=row['value']
    starts=np.array(starts)
    if not len(starts):return np.zeros(len(times),dtype=bool)
    index=np.searchsorted(starts,times,side='right')-1
    return (index>=0)&(times-starts[np.maximum(index,0)]>=seconds)


def analyze(paths,points,cap,startup_sec=0.):
    g=load(paths);ego=g['/ego_vehicle_status'];times=np.array([r['t'] for r in ego])
    speed=np.array([r['value']['speed'] for r in ego]);xy=np.array([[r['value']['x'],r['value']['y']] for r in ego])
    projected=map_projection(points,xy);arc,error,heading=projected.T
    # A restarted planner can prepend another legal approach to the same saved
    # waypoints. Distance to the old reference there is not tracking error.
    routes={}
    for path in paths:
        metadata=Path(path).with_name(Path(path).stem+'_result.json')
        if metadata.exists():
            snapshot=json.loads(metadata.read_text()).get('global_route',{}).get('points_file')
            if snapshot and Path(snapshot).exists():routes[snapshot]=json.loads(Path(snapshot).read_text())
    own_error=np.min([map_projection(route,xy)[:,1] for route in routes.values()],axis=0) if routes else error
    vehicle_yaw=np.radians([r['value']['heading'] for r in ego])
    common=(error<.75)&(np.abs(np.arctan2(np.sin(vehicle_yaw-heading),np.cos(vehicle_yaw-heading)))<np.pi/6)
    drive=join(g,'/planning/drive_status',times,'');target=np.array(join(g,'/planning/target_speed',times,np.nan))
    active=(np.array(drive)=='ACTIVE')&(target>.05)&(speed>2)
    active &= after_startup(g,times,startup_sec)
    control_error=np.array(join(g,'/control/path_error',times,np.nan))
    mode=np.array(join(g,'/planning/speed_mode',times,''))
    cmd=g['/ctrl_cmd'];ct=np.array([r['t'] for r in cmd]);u=np.array([r['value']['steer'] for r in cmd])
    hdr=np.array([r['value'].get('stamp',r['t']) for r in cmd])
    cs=np.array(join(g,'/ego_vehicle_status',ct,{}),dtype=object)
    valid=np.array([bool(v) for v in cs]);cxy=np.array([[v.get('x',0),v.get('y',0)] for v in cs]);cp=map_projection(points,cxy);ca=cp[:,0]
    cyaw=np.radians([v.get('heading',0) for v in cs]);ccommon=(cp[:,1]<.75)&(np.abs(np.arctan2(np.sin(cyaw-cp[:,2]),np.cos(cyaw-cp[:,2])))<np.pi/6)
    enabled=np.array(join(g,'/planning/drive_status',ct,''))=='ACTIVE'
    ctarget=np.array(join(g,'/planning/target_speed',ct,np.nan))
    moving=np.array([v.get('speed',0)>2 for v in cs]);cactive=valid&enabled&moving&(ctarget>.05)
    cactive &= after_startup(g,ct,startup_sec)
    ranges={'all':(0,1e9),'tight_335':(315,340),'roundabout':(960,1090),
        'roundabout_entry':(960,990),'roundabout_ring':(990,1045),'roundabout_exit':(1045,1090),
        'straight_south':(450,650),'straight_north':(1500,1850),'gentle_bend':(415,445)}
    result={}
    for name,(lo,hi) in ranges.items():
        m=active&(arc>=lo)&(arc<=hi);c=cactive&(ca>=lo)&(ca<=hi)
        if name!='all':m=m&common;c=c&ccommon
        # Remove cross-file and disjoint-window differences from derivative metrics.
        result[name]=dict(speed_kmh=stats(speed[m]),target_kmh=stats(target[m]),
            reference_arc_coverage_m=[float(arc[m].min()),float(arc[m].max())] if m.any() else None,
            speed_tracking_error_kmh=stats(np.abs(speed[m]-target[m])),
            global_path_error_m=stats(own_error[m] if name=='all' else error[m]),
            control_path_error_m=stats(control_error[m]),
            steering=steering_metrics(hdr[c],u[c],cap),
            steering_above_old_cap=steering_metrics(hdr[c],u[c],.7))
    for name in ('CRUISE','CURVE','SHARP_CURVE'):
        result['local_mode_'+name]=dict(speed_kmh=stats(speed[active&(mode==name)]))
    result['long_status_counts']=dict(Counter(r['value']['status'] for r in g['/planning/long_speed_plan']))
    result['map_match_counts']=dict(Counter(r['value'] for r in g['/planning/map_match_status']))
    result['localization_status_counts']=dict(Counter(r['value'] for r in g.get('/localization/status',[])))
    result['ego_header_age_sec']=stats([r['value']['age'] for r in ego])
    result['map_match_distance_m']=stats([r['value'] for r in g.get('/planning/map_match_distance',[])])
    result['gps_speed_mps']=stats([r['value'] for r in g.get('/localization/gps_speed',[])])
    result['odometry_speed_mps']=stats([r['value']['speed_mps'] for r in g.get('/localization/odometry',[])])
    return result,dict(common=common,ccommon=ccommon,arc=arc,xy=xy,speed=speed,target=target,error=control_error,active=active,ca=ca,u=u,cactive=cactive,ct=hdr)


def main():
    p=argparse.ArgumentParser();p.add_argument('--reference-path',type=Path,required=True)
    p.add_argument('--case',action='append',nargs=3,metavar=('LABEL','CAP','GLOB'),required=True)
    p.add_argument('--exclude-startup-sec',type=float,default=0.)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    points=json.loads(a.reference_path.read_text());summaries={};data={}
    import glob
    for label,cap,pattern in a.case:
        files=[f for f in glob.glob(pattern,recursive=True) if Path(f).name.startswith('drive_') and f.endswith('.jsonl')]
        if not files:raise ValueError('no driving files: '+pattern)
        summaries[label],data[label]=analyze(files,points,float(cap),a.exclude_startup_sec)
    (a.output/'metrics.json').write_text(json.dumps(summaries,indent=2,allow_nan=False))
    for name,limits in [('tight_335',(315,340)),('roundabout',(960,1090)),
                        ('straight_south',(430,680)),('straight_north',(1500,2050))]:
        fig,axs=plt.subplots(4,1,figsize=(12,11),sharex=True)
        for label,v in data.items():
            m=v['common']&v['active']&(v['arc']>=limits[0])&(v['arc']<=limits[1]);c=v['ccommon']&v['cactive']&(v['ca']>=limits[0])&(v['ca']<=limits[1])
            if not m.any() and not c.any():continue
            line,=axs[0].plot(v['arc'][m],v['error'][m],label=label)
            color=line.get_color()
            axs[1].plot(v['ca'][c],v['u'][c],label=label,color=color)
            axs[2].plot(v['arc'][m],v['speed'][m],label=label,color=color)
            axs[2].plot(v['arc'][m],v['target'][m],':',alpha=.5,color=color)
            idx=np.flatnonzero(c);dt=np.diff(v['ct'][idx]);valid=(dt>.005)&(dt<.15)
            axs[3].plot(v['ca'][idx][1:][valid],np.diff(v['u'][idx])[valid]/dt[valid],label=label,color=color)
        for ax,ylabel in zip(axs,['Control error (m)','MORAI steering','Speed (km/h), dotted target','Steering rate (/s)']):
            ax.set_ylabel(ylabel);ax.grid();ax.legend(loc='upper right')
        axs[-1].set_xlabel('Arc on original reference route (m)');fig.suptitle(name);fig.tight_layout();fig.savefig(a.output/(name+'.png'),dpi=150);plt.close(fig)
    for label,v in summaries.items():
        print(label,json.dumps({k:v[k] for k in ['tight_335','roundabout','straight_south']}))

if __name__=='__main__':main()
