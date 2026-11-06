"""ROS diagnostic rendering only: rejected candidates are never control paths."""
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)):
        return [json_safe(v) for v in value]
    return value


def diagnostic(result):
    values={k:v for k,v in result.items() if k not in (
        'points','candidates','original_lane','left_lane','right_lane','return_path')}
    values['candidates']=[dict(side=c.side,entry_end_s=c.entry_end_s,return_start_s=c.return_start_s,
        return_end_s=c.return_end_s,transition_length_m=c.transition_length_m,cost=c.cost,
        cost_terms=c.cost_terms,rejection=c.rejection) for c in result.get('candidates',())]
    return json_safe(values)


def candidate_markers(result, stamp, previous_ids=()):
    markers=[];active=set()
    def marker(index, kind):
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp
        m.ns='static_avoidance';m.id=index;m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color.a=1.
        # Input watchdog + finite render lifetime removes diagnostics if the
        # planner dies. IDs remain stable; no DELETEALL/ADD flicker per cycle.
        m.lifetime.sec=2
        active.add(index);return m
    for i,c in enumerate(result.get('candidates',())):
        m=marker(i+1,Marker.LINE_STRIP);m.scale.x=.08;m.pose.position.z=.10
        m.color.r,m.color.g,m.color.b=(.95,.2,.1) if c.rejection else (.15,.6,1.)
        m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in c.points]
        if m.points:markers.append(m)
        else:active.discard(i+1)
        text=marker(i+1001,Marker.TEXT_VIEW_FACING);text.scale.z=.55
        if c.points:
            p=c.points[min(len(c.points)-1,len(c.points)//3)]
            text.pose.position.x,text.pose.position.y,text.pose.position.z=float(p[0]),float(p[1]),float(p[2])+1.+i*.2
        text.color.r=text.color.g=text.color.b=1.
        cost=f'{c.cost:.3f}' if math.isfinite(c.cost) else 'invalid'
        text.text=f'{c.side} L={c.transition_length_m:g} J={cost} {c.rejection or "SAFE"}'
        markers.append(text)
    state=marker(0,Marker.TEXT_VIEW_FACING);state.scale.z=.7
    reference=result.get('original_lane') or result.get('points') or ()
    if reference:
        p=reference[0];state.pose.position.x=float(p[0]);state.pose.position.y=float(p[1]);state.pose.position.z=2.
    state.color.r,state.color.g,state.color.b=(1.,.3,.1) if result['blocked'] else (.2,1.,.4)
    state.text=f'{result["state"]} lane={result.get("selected_lane","")} {result.get("reason","")}'
    markers.append(state)
    for index in set(previous_ids)-active:
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp
        m.ns='static_avoidance';m.id=index;m.action=Marker.DELETE;markers.append(m)
    return MarkerArray(markers=markers),active
