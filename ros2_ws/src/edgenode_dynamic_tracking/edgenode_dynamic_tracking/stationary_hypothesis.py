"""Bounded static-object hypothesis under changing partial visibility.

A fixed template may explain newly exposed body surfaces. This hypothesis is
never a collision-space exemption. Raw obstacle solids remain authoritative.
Translation of the same visible face or support outside the fixed template
invalidates the hypothesis; it cannot follow a moving object indefinitely.
"""
import math
import numpy as np

class StationaryHypothesis:
    def __init__(self,track,ego,heading):
        self.center=np.asarray(track.state[:2]).copy();self.heading=heading
        self.forward=np.array((math.cos(heading),math.sin(heading)));self.left=np.array((-self.forward[1],self.forward[0]))
        self.length=track.observation.model_length_m;self.width=track.observation.model_width_m
        self.bounds=self.project(track.observation);self.sector=self.view(ego)
    def project(self,observation):
        vertices=np.array([p for part in observation.footprints for p in part])-self.center
        a=vertices@self.forward;b=vertices@self.left
        return np.array((a.min(),a.max(),b.min(),b.max()))
    def view(self,ego):
        eye=np.asarray(ego[:2])-self.center;a=eye@self.forward;b=eye@self.left
        return (-1 if a<-self.length/2-1 else 1 if a>self.length/2+1 else 0,
                -1 if b<-self.width/2-1 else 1 if b>self.width/2+1 else 0)
    def consistent(self,observation,ego):
        bounds=self.project(observation);half=np.array((self.length/2,self.length/2,self.width/2,self.width/2))
        # Two-sigma observation tolerance belongs to this partial-body model,
        # not the vehicle/road/collision safety margins.
        if np.any(np.abs(bounds)>half+.6):return False
        sector=self.view(ego)
        # Compare the same exposed end when the longitudinal view remains
        # the same. The current endpoint changing inside a side view alone
        # is not evidence of world translation.
        if sector[0] and sector[0]==self.sector[0]:
            index=0 if sector[0]<0 else 1
            if abs(bounds[index]-self.bounds[index])>.45:return False
        return True
