"""Bounded source-time replay for GPS/IMU and measured vehicle speed.

The original4-state EKF equations/noise contracts are reused unchanged. GPS
and vehicle-speed callbacks queue observations. Only a new IMU source epoch
advances the published state; delayed observations replay the recent history.
No Actor API or simulator object position enters this filter.
"""
from copy import deepcopy
import math
import numpy as np
from edgenode_localization.ekf import EKF


class SourceTimeEKF(EKF):
    def __init__(self,baseline,*,history_sec=.8,source_velocity_std_mps=.1,max_predict_dt_sec=.2):
        self.__dict__.update(deepcopy(baseline.__dict__))
        if not all(math.isfinite(x) and x>0 for x in (history_sec,source_velocity_std_mps,max_predict_dt_sec)):
            raise ValueError('positive finite source-time contracts required')
        self.history_ns=round(history_sec*1e9)
        self.source_velocity_noise=np.array([[source_velocity_std_mps**2]])
        self.max_predict_dt_sec=max_predict_dt_sec
        self.context=None;self.events={};self.snapshots={};self.anchor=None
        self.dirty=None;self.last_output_ns=None;self.dropped_before_history=0

    def begin(self,kind,stamp_ns,*,gyro=0.):
        if kind not in ('gps','imu') or type(stamp_ns) is not int or stamp_ns<=0:
            raise ValueError('real positive source timestamp required')
        self.context=(kind,stamp_ns,gyro)

    def end(self):self.context=None

    def initialize(self,*args,**kwargs):
        if self.context is None:raise ValueError('initialization needs a sensor epoch')
        EKF.initialize(self,*args,**kwargs)
        stamp=self.context[1]
        self.events={};self.snapshots={};self.dirty=None
        self.anchor=(stamp,self._state.copy(),self._covariance.copy(),0.)
        self.last_output_ns=stamp

    def _record(self,stamp,key,value):
        if self.anchor is None:return
        if stamp<self.anchor[0]:
            self.dropped_before_history+=1;return
        event=self.events.setdefault(stamp,{})
        if key in event:return
        event[key]=value;self.dirty=stamp if self.dirty is None else min(self.dirty,stamp)

    def predict(self,gyro_z,dt):
        # The original node still validates IMU dt. Store its control sample;
        # update_yaw completes the same-source epoch before any publication.
        if self.context is None or self.context[0]!='imu':raise ValueError('IMU context required')
        if not math.isfinite(gyro_z) or not math.isfinite(dt) or dt<=0:raise ValueError('invalid prediction')

    def update_yaw(self,yaw):
        if self.context is None or self.context[0]!='imu' or not math.isfinite(yaw):raise ValueError('valid IMU context required')
        _,stamp,gyro=self.context
        if not math.isfinite(gyro):raise ValueError('finite gyro required')
        if self.anchor is not None and stamp<self.anchor[0]:return
        self._record(stamp,'imu',(yaw,gyro));self._replay(stamp)

    def update_position(self,x,y):
        if self.context is None or self.context[0]!='gps' or not all(math.isfinite(v) for v in (x,y)):
            raise ValueError('valid GPS context required')
        self._record(self.context[1],'gps',(x,y))

    def update_velocity(self,v_mps):
        if self.context is None or self.context[0]!='gps' or not math.isfinite(v_mps) or v_mps<0:
            raise ValueError('valid GPS speed context required')
        self._record(self.context[1],'gps_velocity',v_mps)

    def vehicle_velocity(self,stamp_ns,v_mps):
        if type(stamp_ns) is not int or stamp_ns<=0 or not math.isfinite(v_mps) or not 0<=v_mps<=60:
            raise ValueError('valid source-stamped vehicle velocity required')
        self._record(stamp_ns,'vehicle_velocity',v_mps)

    def _replay(self,until):
        if self.anchor is None or until<self.anchor[0]:return
        dirty=min(until,self.dirty if self.dirty is not None else until)
        before=[t for t in self.snapshots if t<dirty]
        if before:
            previous=max(before);state,covariance,last_gyro=self.snapshots[previous]
        else:previous,state,covariance,last_gyro=self.anchor
        self._state=state.copy();self._covariance=covariance.copy()
        keys=sorted(t for t in self.events if previous<t<=until or t==self.anchor[0] and not before)
        # A GPS sample inside an IMU interval splits that same measured control
        # interval. Replaying it later uses the identical IMU control, so arrival
        # order cannot double-count vehicle travel.
        for i,stamp in enumerate(keys):
            event=self.events[stamp]
            next_gyro=next((self.events[t]['imu'][1] for t in keys[i:] if 'imu' in self.events[t]),last_gyro)
            dt=(stamp-previous)/1e9
            if 0<dt<=self.max_predict_dt_sec:EKF.predict(self,next_gyro,dt)
            if 'imu' in event:
                yaw,last_gyro=event['imu'];EKF.update_yaw(self,yaw)
            if 'gps' in event:EKF.update_position(self,*event['gps'])
            if 'gps_velocity' in event:EKF.update_velocity(self,event['gps_velocity'])
            if 'vehicle_velocity' in event:
                innovation=np.array([event['vehicle_velocity']-self._state[3]])
                self._update(innovation,np.array([[0.,0.,0.,1.]]),self.source_velocity_noise)
            self.snapshots[stamp]=(self._state.copy(),self._covariance.copy(),last_gyro)
            previous=stamp
        self.last_output_ns=until
        pending=[t for t in self.events if t>until]
        self.dirty=min(pending) if pending else None
        old=[t for t in self.snapshots if t<=until-self.history_ns]
        if old:
            stamp=max(old);state,covariance,gyro=self.snapshots[stamp]
            self.anchor=(stamp,state.copy(),covariance.copy(),gyro)
            self.events={t:v for t,v in self.events.items() if t>stamp}
            self.snapshots={t:v for t,v in self.snapshots.items() if t>stamp}
