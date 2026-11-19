"""Strict source-epoch Ego pose/speed interpolation, with relocation rejection."""
from bisect import bisect_left
from collections import deque
import math

class EgoHistory:
    def __init__(self,max_gap_s=.25):self.rows=deque(maxlen=500);self.max_gap_ns=round(max_gap_s*1e9)
    def add(self,stamp,pose,speed):
        if type(stamp) is not int or stamp<=0 or len(pose)!=3 or not all(math.isfinite(v) for v in (*pose,speed)):return False
        if self.rows and stamp<=self.rows[-1][0]:return False
        reset=False
        if self.rows:
            dt=(stamp-self.rows[-1][0])/1e9
            if math.dist(pose[:2],self.rows[-1][1][:2])>5.+80.*dt:self.rows.clear();reset=True
        self.rows.append((stamp,tuple(pose),speed));return reset
    def sample(self,stamp):
        rows=list(self.rows);i=bisect_left([r[0] for r in rows],stamp)
        if i<len(rows) and rows[i][0]==stamp:return rows[i][1],rows[i][2]
        if i==0 or i==len(rows):return None
        a,p,v=rows[i-1];b,q,u=rows[i]
        if b-a>self.max_gap_ns:return None
        f=(stamp-a)/(b-a);yaw=p[2]+f*math.atan2(math.sin(q[2]-p[2]),math.cos(q[2]-p[2]))
        return ((1-f)*p[0]+f*q[0],(1-f)*p[1]+f*q[1],yaw),(1-f)*v+f*u
