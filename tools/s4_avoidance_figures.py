#!/usr/bin/env python3
"""Standalone figures from closed-bag analysis; no runtime or Actor inputs."""
import argparse,json,pathlib

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=pathlib.Path);a=p.parse_args();d=a.trial;data=json.loads((d/'avoidance-speed-analysis.json').read_text());rows=data['rows'];reference=json.loads((d/'global-path.json').read_text())
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    colors={'NORMAL':'#6b7280','DETECT':'#d97706','AVOID_ENTRY':'#2563eb','AVOID_HOLD':'#059669','RETURN':'#9333ea'}
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for phase in data['timeline']:
        for ax in axes:ax.axvspan(phase['first_t_s'],phase['last_t_s'],color=colors.get(phase['stage'],'#aaa'),alpha=.10)
    times=[r['t'] for r in rows];axes[0].plot(times,[r['speed_kmh'] for r in rows],label='Measured Ego speed');axes[0].plot(times,[r['target_kmh'] for r in rows],label='Published target',alpha=.7);axes[0].axhline(data['requested_kmh'],color='black',linestyle=':',label='Requested avoidance speed');axes[0].set_ylabel('km/h');axes[0].legend()
    axes[1].plot(times,[r['wheel_rad'] for r in rows]);axes[1].set_ylabel('Measured front wheel [rad]')
    values=list(colors);axes[2].step(times,[values.index(r['stage']) if r['stage'] in values else -1 for r in rows],where='post');axes[2].set_yticks(range(len(values)),values);axes[2].set_xlabel('Seconds from trial authorization');axes[2].set_ylabel('Recorded phase')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle(d.name+' | measured speed, wheel and actual phase');fig.tight_layout()
    for ext in ['png','svg']:fig.savefig(d/('avoidance-speed-phases.'+ext),dpi=150)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,8));ax.plot([p[0] for p in reference],[p[1] for p in reference],color='#bbb',label='Recorded normal global path')
    for phase,color in colors.items():
        points=[r for r in rows if r['stage']==phase]
        if points:ax.scatter([r['x'] for r in points],[r['y'] for r in points],s=3,color=color,label=phase)
    xs=[r['x'] for r in rows];ys=[r['y'] for r in rows];ax.set_xlim(min(xs)-15,max(xs)+15);ax.set_ylim(min(ys)-15,max(ys)+15);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('Map x [m]');ax.set_ylabel('Map y [m]');ax.legend();ax.set_title(d.name+' | measured trajectory by phase');fig.tight_layout()
    for ext in ['png','svg']:fig.savefig(d/('avoidance-trajectory.'+ext),dpi=150)
    plt.close(fig)
if __name__=='__main__':main()
