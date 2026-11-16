#!/usr/bin/env python3
"""Reproducible offline S4 position/approach matrix; never connects to MORAI."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/edgenode_planning/test'))
from offline_static_avoidance import run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--avoidance-speed-kmh',type=float,default=10.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    velocity=args.avoidance_speed_kmh/3.6
    horizon=max(20.,5.*math.ceil((4.*velocity+max(2.5,min(12.,.8*velocity))+.6+2.)/5.))
    results=[]
    for i,(side,front,radius,start_speed) in enumerate([
            ('left',24.,math.inf,10.),('right',24.,math.inf,10.),
            ('left',43.,300.,20.),('right',43.,300.,20.),
            ('left',58.,150.,20.),('right',58.,150.,20.)]):
        start_speed=max(start_speed,args.avoidance_speed_kmh)
        scenario=dict(side=side,front_m=front,radius_m=None if math.isinf(radius) else radius,
                      approach_kmh=start_speed,origin=(100.*i,-30.*i),yaw=.21*i)
        result=run(start_speed,side,radius=radius,origin=scenario['origin'],yaw=scenario['yaw'],
            lateral_error=.05,heading_error=.005,brake_to_default=True,periods=(.1,),
            duration=65.,reference_horizon_m=80.,obstacle_front_m=front,control_horizon_m=horizon,
            config_overrides=dict(avoid_speed_kmh=args.avoidance_speed_kmh,steering_slew_limit_radps=.6,
                normal_obstacle_confirmations=2,normal_detection_latency_s=1.6,
                lookahead_max_m=12.,lookahead_preview_time_sec=.8,lookahead_heading_budget_rad=1.6))
        (args.output/f'scenario-{i+1}.json').write_text(json.dumps(dict(scenario=scenario,**result),indent=2))
        compact=dict(scenario=scenario,**{k:v for k,v in result.items() if k!='rows'})
        results.append(compact);print(json.dumps(compact),flush=True)
    summary=dict(model='causal no-slip bicycle + production PP/slew; declared constant deceleration, no ROS/PID/MORAI claim',
        count=len(results),successes=sum(r['success'] for r in results),results=results)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        figure,axes=plt.subplots(2,3,figsize=(15,8))
        for i,(ax,result) in enumerate(zip(axes.flat,results)):
            rows=json.loads((args.output/f'scenario-{i+1}.json').read_text())['rows']
            scenario=result['scenario'];ox,oy=scenario['origin'];yaw=scenario['yaw']
            def local(p):
                x,y=p[0]-ox,p[1]-oy
                return x*math.cos(yaw)+y*math.sin(yaw),-x*math.sin(yaw)+y*math.cos(yaw)
            xy=[local(row['pose']) for row in rows]
            ax.plot([p[0] for p in xy],[p[1] for p in xy],label='bicycle model')
            radius=scenario['radius_m'];stations=[j*.5 for j in range(180)]
            center=[(s,0.) if radius is None else (radius*math.sin(s/radius),radius*(1-math.cos(s/radius))) for s in stations]
            ax.plot([p[0] for p in center],[p[1] for p in center],'k--',label='global reference')
            front=scenario['front_m'];s=front+2.;theta=0. if radius is None else s/radius
            x,y=(s,0.) if radius is None else (radius*math.sin(theta),radius*(1-math.cos(theta)))
            ax.scatter([x],[y],c='red',marker='s',label='obstacle center')
            ax.set_title(f"{scenario['side']}, front {front:g}m, {scenario['approach_kmh']:g}→{args.avoidance_speed_kmh:g} km/h: {'PASS' if result['success'] else 'FAIL'}")
            ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid();ax.legend(fontsize=7)
        figure.tight_layout();figure.savefig(args.output/'model-trajectories.png',dpi=150);plt.close(figure)
    except ImportError:pass
    return 0 if summary['successes']==summary['count'] else 1


if __name__=='__main__':raise SystemExit(main())
