"""The unchanged v61 atomic admission contract for D's auxiliary perception."""
from edgenode_planning.sensor_readiness import SensorReadiness
from edgenode_planning.planning_node import TRANSIENT_SENSOR_REASONS

class AuxiliaryAdmission:
    def __init__(self):
        self.guard=SensorReadiness(allow_transient_history=True);self.latched='';self.last_key=None
    def accept(self,payload,now_ns):
        reason=self.guard.frame(payload,now_ns)
        if reason and reason not in TRANSIENT_SENSOR_REASONS:self.latched=reason
        matched=self.guard.matched;key=(self.guard.session_id,matched[0]['frame_seq'] if matched else None,self.guard.fault_seq,self.latched,self.guard.reason(now_ns))
        changed=key!=self.last_key;self.last_key=key;return changed
    def snapshot(self,now_ns):
        reason=self.latched or self.guard.reason(now_ns);matched=self.guard.matched
        return dict(ready=not reason,reason=reason,source_ns=matched[0]['source_stamp_ns'] if matched else None,groups=self.guard.groups() if matched else [],status_state=self.guard.status.get('state') if self.guard.status else None)
