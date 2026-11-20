"""D-owned anisotropic road motion model; C tracking source stays unchanged."""
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from edgenode_dynamic_tracking.kinematics import Track

def measurement(observation,state):
    axes=getattr(observation,'point_axes',None)
    if axes is None:return np.array(((1.,0.,0.,0.),(0.,1.,0.,0.))),np.asarray(observation.center_xy)
    rows=[];values=[]
    for axis in axes:rows.append((*axis,0.,0.));values.append(float(np.asarray(axis)@np.asarray(observation.center_xy)))
    for axis,low,high in observation.interval_axes:
        predicted=float(np.asarray(axis)@state[:2])
        if predicted<low or predicted>high:rows.append((*axis,0.,0.));values.append(max(low,min(high,predicted)))
    return np.asarray(rows,dtype=float).reshape((-1,4)),np.asarray(values,dtype=float)

class RoadObstacleTracking:
    def __init__(self,measurement_std_m=.3,acceleration_std_mps2=1.5,max_missed_s=1.2,*,lateral_std_mps2=.15,heading_provider=None):
        self.lateral_var=lateral_std_mps2**2;self.heading_provider=heading_provider
        if any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in
               (measurement_std_m,acceleration_std_mps2,max_missed_s)):raise ValueError('positive finite tracking limits required')
        self.measurement_var=measurement_std_m**2;self.acceleration_var=acceleration_std_mps2**2
        self.max_missed_ns=round(max_missed_s*1e9);self.reset()

    def reset(self):self.tracks={};self.source_ns=None;self._next_id=1;self.process_modes={}

    def update(self,source_ns,observations):
        if type(source_ns) is not int or source_ns<=0 or (self.source_ns is not None and source_ns<=self.source_ns):
            raise ValueError('strictly ordered source observations required')
        if any(len(o.center_xy)!=2 or not all(math.isfinite(v) for v in o.center_xy) for o in observations):
            raise ValueError('finite map-frame object centers required')
        dt=(source_ns-self.source_ns)/1e9 if self.source_ns is not None else 0.
        self.source_ns=source_ns
        F=np.eye(4);F[0,2]=F[1,3]=dt
        G=np.array(((dt*dt/2,0.),(0.,dt*dt/2),(dt,0.),(0.,dt)))
        self.tracks={key:t for key,t in self.tracks.items() if source_ns-t.observed_ns<=self.max_missed_ns}
        tracks=list(self.tracks.values())
        for t in tracks:
            process=np.eye(2)*self.acceleration_var;conditioned=False
            heading=self.heading_provider(t.state[:2]) if self.heading_provider is not None else None
            if heading is not None and t.motion=='MOVING' and t.speed_mps>.7:
                forward=np.array((math.cos(heading),math.sin(heading)));left=np.array((-forward[1],forward[0]))
                velocity=t.state[2:];alignment=float(velocity@forward)/max(.1,t.speed_mps)
                if alignment>math.cos(.15) and abs(float(velocity@left))<.3:
                    process=self.acceleration_var*np.outer(forward,forward)+self.lateral_var*np.outer(left,left);conditioned=True
            self.process_modes[t.identifier]=conditioned
            t.state=F@t.state;t.covariance=F@t.covariance@F.T+G@process@G.T
        cost=np.full((len(tracks),len(observations)),1e9);systems={}
        for i,t in enumerate(tracks):
            for j,o in enumerate(observations):
                if getattr(o,'point_axes',None) is not None and np.linalg.norm(np.asarray(o.center_xy)-t.state[:2])>3.:continue
                H,z=measurement(o,t.state);systems[i,j]=(H,z)
                if len(z):
                    R=np.eye(len(z))*self.measurement_var;S=H@t.covariance@H.T+R;delta=z-H@t.state;distance=float(delta@np.linalg.inv(S)@delta)
                    if distance<=(6.635 if len(z)==1 else 9.21):cost[i,j]=distance
                else:cost[i,j]=float(np.linalg.norm(np.asarray(o.center_xy)-t.state[:2]))*.01
        matched=set()
        if cost.size:
            for i,j in zip(*linear_sum_assignment(cost)):
                if cost[i,j]>=1e9:continue
                t=tracks[i];o=observations[j];H,z=systems[i,j]
                if len(z):
                    R=np.eye(len(z))*self.measurement_var;K=t.covariance@H.T@np.linalg.inv(H@t.covariance@H.T+R)
                    t.state=t.state+K@(z-H@t.state)
                    A=np.eye(4)-K@H;t.covariance=A@t.covariance@A.T+K@R@K.T
                t.observed_ns=source_ns;t.frames+=1;t.observation=o;matched.add(j)
                if len(z)==2:
                    t.history.append((source_ns,tuple(np.linalg.solve(H[:,:2],z))));t.history=t.history[-20:]
                    t.fit_motion(self.measurement_var)
        for j,o in enumerate(observations):
            if j in matched:continue
            key=self._next_id;self._next_id+=1
            covariance=np.diag((self.measurement_var,self.measurement_var,9.,9.))
            for axis,low,high in getattr(o,'interval_axes',()):
                a=np.asarray(axis);covariance[:2,:2]+=max(0.,((high-low)/2)**2-self.measurement_var)*np.outer(a,a)
            observed=getattr(o,'point_axes',None)
            history=[(source_ns,o.center_xy)] if observed is None or len(observed)==2 else []
            self.tracks[key]=Track(key,np.array((*o.center_xy,0.,0.),dtype=float),covariance,source_ns,source_ns,1,o,history=history)
        return tuple(self.tracks.values())
