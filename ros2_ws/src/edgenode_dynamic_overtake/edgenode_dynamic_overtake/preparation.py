"""Non-executable geometry proposals in a separate process.

Only immutable map proofs are returned. Motion permission requires all current
sensor, body, PP and timed traffic checks in the owner, after worker completion.
"""
from types import SimpleNamespace
import time
from copy import copy

_PROTOTYPE=None

def initialize(prototype):
    global _PROTOTYPE
    _PROTOTYPE=prototype

def prepare_current(snapshot,config,static_records,reference,groups,pose,lead,value,kwargs,generation):
    if _PROTOTYPE is None:raise RuntimeError("D immutable map worker is not initialized")
    probe=copy(_PROTOTYPE);probe._lane_snapshot=snapshot;probe.config=config
    return prepare(probe,static_records,reference,groups,pose,lead,value,kwargs,generation)

def prepare(probe,static_records,reference,groups,pose,lead,value,kwargs,generation):
    probe._proposal_executor=None;probe._proposal_future=None;probe.dynamic=None;probe.dynamic_debug={}
    probe._lane_bank=None;probe._preview_executor=None;probe._preview_future=None
    probe.normal_guard=SimpleNamespace(occupancy=SimpleNamespace(static=SimpleNamespace(tracks=static_records)))
    accepted=probe._build_entry(reference,groups,pose,lead,value,kwargs,geometry_only=True)
    # Strip map objects from cache keys. The owner accepts these exact sweep
    # certificates only if its original map/body/config identity is unchanged.
    map_proofs=[(first,second,result) for (context,first,second),result in probe._map_sweep_cache.items()]
    return dict(executable=False,accepted_geometry=accepted,dynamic=probe.dynamic,pair=probe.pair,selected=probe.selected,
        map_proofs=map_proofs,fixed_sweeps=list(probe._fixed_sweep_cache.items()),config=probe.config,body=probe.body,
        checks=probe.dynamic_debug.get('candidates',[]),generation=generation,current_link=kwargs.get('current_link_id'),
        tracker_session=value['tracker_session_id'],lead_id=lead['track']['id'],prepared_source_ns=value['source_stamp_ns'],finished_ns=time.time_ns())
