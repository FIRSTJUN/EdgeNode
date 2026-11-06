"""Synthetic ground-truth road, not a production MGeo boundary inference."""
from types import SimpleNamespace
import math

from edgenode_planning.avoidance_geometry import DrivableArea
from edgenode_planning.lane_geometry import LaneNetwork


def world(side='left', radius=math.inf, origin=(0.,0.), yaw=0., two_sides=False):
    def point(s, lateral):
        theta=0. if math.isinf(radius) else s/radius
        x=s if math.isinf(radius) else radius*math.sin(theta)
        y=0. if math.isinf(radius) else radius*(1-math.cos(theta))
        x,y=x-lateral*math.sin(theta),y+lateral*math.cos(theta)
        return (origin[0]+x*math.cos(yaw)-y*math.sin(yaw),
                origin[1]+x*math.sin(yaw)+y*math.cos(yaw),0.)
    offsets={'O':0.,'A':3.5 if side=='left' else -3.5}
    if two_sides:offsets={'O':0.,'A':3.5,'B':-3.5}
    links={}
    for name,offset in offsets.items():
        for index,(start,end) in enumerate(((-10.,40.),(40.,140.))):
            record=dict(idx=name+str(index),from_node_idx=name+str(index)+'node',
                to_node_idx=name+str(index+1)+'node',max_speed=30.,
                points=[point(start+i*.5,offset) for i in range(round((end-start)*2)+1)],
                left_lane_change_dst_link_idx=None,right_lane_change_dst_link_idx=None)
            if name=='O':
                for target,delta in offsets.items():
                    if target!='O':
                        record[('left' if delta>0 else 'right')+'_lane_change_dst_link_idx']=target+str(index)
            else:record[('right' if offset>0 else 'left')+'_lane_change_dst_link_idx']='O'+str(index)
            links[record['idx']]=record
    lanes=LaneNetwork(SimpleNamespace(links_by_id=links,
        graph={'x':[dict(link_id=name) for name in links]}))
    low,high=min(offsets.values())-1.75,max(offsets.values())+1.75
    # These are scene-defined true road edges; the planner receives them via
    # the same injected DrivableArea interface future surveyed maps will use.
    if math.isinf(radius):
        cells=((point(-10,low)[:2],point(140,low)[:2],point(140,high)[:2],point(-10,high)[:2]),)
    else:
        cells=tuple((point(s,low)[:2],point(s+2,low)[:2],point(s+2,high)[:2],point(s,high)[:2])
                    for s in range(-10,140,2))
    area=DrivableArea(cells,'synthetic ground-truth road (offline only)')
    def obstacle(front=35.,rear=39.,low=-.5,high=.5):
        mid=(front+rear)/2
        return [point(s,d)[:2] for s,d in ((front,low),(mid,low),(rear,low),(rear,(low+high)/2),
            (rear,high),(mid,high),(front,high),(front,(low+high)/2))]
    return lanes,area,point,obstacle
