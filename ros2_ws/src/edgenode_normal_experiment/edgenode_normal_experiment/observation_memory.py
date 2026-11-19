"""NORMAL observations: immediate risk plus separately confirmed static history.

Current returns always participate in safety. Confirmed static occupancy
survives occlusion. Provisional single-frame returns need six-tenths of a
second and three distinct, unobstructed fresh views before retirement. An
absent/stale frame never constitutes a clear view or advances confirmation.
"""
import math
from shapely.geometry import Polygon,LineString,GeometryCollection
from shapely.ops import unary_union
from shapely.strtree import STRtree
from shapely.prepared import prep
from edgenode_planning.avoidance_geometry import inflate


class NormalObservationMemory:
    def __init__(self):
        self.tracks=[];self.stamp=None;self.geometry=None;self.sequence=0;self.debug={}
    @property
    def cells(self):return self.tracks # count compatibility only, no grid occupancy
    def update(self,observed,index,ego,rear,padding,*,pose,stamp,now):
        if stamp is None or stamp==self.stamp:
            return self.geometry if self.geometry is not None else unary_union([])
        self.stamp=stamp;self.sequence+=1
        current=[Polygon(inflate(p,padding)) for p in observed];union=unary_union(current)
        tree=STRtree(current);identifiers={id(s):i for i,s in enumerate(current)}
        centers=[s.centroid for s in current];prepared=prep(union)
        matches=set();retained=[];threshold=ego-rear-.5
        origin=(pose[0]+1.19*math.cos(pose[2]),pose[1]+1.19*math.sin(pose[2]))
        for track in self.tracks:
            shape=track['shape'];x0,y0,x1,y1=shape.bounds
            corners=((x0,y0),(x0,y1),(x1,y0),(x1,y1))
            if (all(math.cos(pose[2])*(x-pose[0])+math.sin(pose[2])*(y-pose[1]) < -rear-.5 for x,y in corners)
                and all(index.project(p)<threshold for p in corners)):continue
            found=[]
            for solid in tree.query(shape):
                i=identifiers[id(solid)]
                if i in matches:continue
                a,b,c,d=solid.bounds
                if c<x0 or x1<a or d<y0 or y1<b:continue
                if (shape.centroid.distance(centers[i])>.75 or
                    max(shape.area,solid.area)>4*min(shape.area,solid.area)):continue
                if shape.intersection(solid).area<.5*min(shape.area,solid.area):continue
                found.append(i)
            if found:
                i=min(found,key=lambda v:shape.centroid.distance(centers[v]));matches.add(i)
                track['count']=track['count']+1 if track['seen_sequence']==self.sequence-1 else 1
                track['seen_sequence']=self.sequence;track['confirmed']=track['confirmed'] or track['count']>=2
                track['shape']=shape.union(current[i]);track['last_seen']=now;track['clear_frames']=0
            else:
                center=shape.centroid;dx=center.x-pose[0];dy=center.y-pose[1]
                forward=math.cos(pose[2])*dx+math.sin(pose[2])*dy
                lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
                visible=(-8<=forward<=100 and abs(lateral)<=40)
                distance=math.hypot(center.x-origin[0],center.y-origin[1])
                end=(origin[0]+(center.x-origin[0])*max(0.,distance-.25)/max(distance,1e-12),
                     origin[1]+(center.y-origin[1])*max(0.,distance-.25)/max(distance,1e-12))
                hidden=not union.is_empty and prepared.intersects(LineString((origin,end)))
                if visible and not hidden:track['clear_frames']+=1
                else:track['clear_frames']=0
                if not track['confirmed'] and now-track['last_seen']>=.6 and track['clear_frames']>=3:continue
            retained.append(track)
        for i,solid in enumerate(current):
            if i not in matches:retained.append(dict(shape=solid,count=1,seen_sequence=self.sequence,confirmed=False,last_seen=now,clear_frames=0))
        self.tracks=retained;shapes=[t['shape'] for t in retained]
        self.tree=STRtree(shapes);self.geometry=GeometryCollection(shapes)
        self.debug=dict(confirmed=sum(t['confirmed'] for t in retained),provisional=sum(not t['confirmed'] for t in retained),source_sequence=self.sequence)
        return self.geometry
