#!/usr/bin/env python3
"""Replay identical CLOSED sensor sources through C fitting and D view fitting.

Actor snapshots are used only AFTER filter updates for error measurement.
No ROS node, motion command, scene change, or motion permission is produced.
"""
import argparse,json,math,pickle,sys,importlib.util,hashlib
from pathlib import Path
import numpy as np
import yaml
from s4_failure_forensics import messages
from edgenode_dynamic_tracking.tracker import TrafficTracker
from edgenode_dynamic_tracking.vehicle_observation import nearest_native
from edgenode_dynamic_overtake.view_tracker import ViewConsistentTracker
from edgenode_dynamic_overtake.road_kinematics import RoadObstacleTracking

def replay(directory,output,baseline_view=None,baseline_model=None):
    assert not output.exists();summary=json.loads((directory/'summary.json').read_text());assert summary.get('rosbag_metadata_present') and summary.get('rosbag_exit_code')==0
    rows=[json.loads(x) for x in (directory/'samples-finalized.jsonl').open()];lead=summary['obstacle_state']['actor_info']['id']['value'];truth=[]
    for row in rows:
        inventory=row.get('all_actor_truth_audit_only')
        if not inventory:continue
        actor=next((a for a in inventory['states'] if a['actor_info']['id']['value']==lead),None)
        if not actor:continue
        ns=int(inventory['timestamp']);ns=ns*1000 if ns<10**17 else ns
        box=actor['bounding_box']['location'];velocity=actor.get('global_velocity',{});truth.append((ns,[box['x'],box['y']],[velocity.get('x',0.)/3.6,velocity.get('y',0.)/3.6]))
    assert len(truth)>10
    root=directory.parent;cache=pickle.loads((root/'offline-surveyed-network-cache-d15.pkl').read_bytes());aligned=cache['network'].boundaries.aligned_links
    config=next(iter(yaml.safe_load(Path('/workspace/config/s4_d/tracking.yaml').read_text()).values()))['ros__parameters']
    def make(view):
        base=TrafficTracker(config['mgeo_dir'],measurement_std_m=config['measurement_std_m'],acceleration_std_mps2=config['acceleration_std_mps2'],max_missed_s=config['max_missed_sec'],vehicle_length_prior_m=config['vehicle_length_prior_m'],vehicle_width_prior_m=config['vehicle_width_prior_m'])
        base.matcher.links=[link for link in base.matcher.links if link['idx'] in aligned]
        tracker=ViewConsistentTracker.from_existing(base) if view else base
        def heading(center):
            row=nearest_native(tracker.matcher,center);return math.radians(row[1]) if row is not None and row[0]<=2.2 else None
        tracker.filter=RoadObstacleTracking(config['measurement_std_m'],config['acceleration_std_mps2'],config['max_missed_sec'],heading_provider=heading)
        return tracker
    old,new=make(False),make(True);
    baseline_hashes={}
    if baseline_model is not None:
        assert baseline_view is None
        import types
        package_name='edgenode_dynamic_overtake._frozen_replay_model'
        package=types.ModuleType(package_name);package.__path__=[str(baseline_model)]
        sys.modules[package_name]=package
        loaded={}
        for name in ('vehicle_observation','road_kinematics','view_tracker'):
            filename=baseline_model/(name+'.py');baseline_hashes[name]=hashlib.sha256(filename.read_bytes()).hexdigest()
            spec=importlib.util.spec_from_file_location(package_name+'.'+name,filename);module=importlib.util.module_from_spec(spec)
            sys.modules[spec.name]=module;spec.loader.exec_module(module);loaded[name]=module
        old=loaded['view_tracker'].ViewConsistentTracker.from_existing(old)
        old.filter=loaded['road_kinematics'].RoadObstacleTracking(config['measurement_std_m'],config['acceleration_std_mps2'],config['max_missed_sec'],heading_provider=old.filter.heading_provider)
    if baseline_view is not None:
        spec=importlib.util.spec_from_file_location('edgenode_dynamic_overtake._source_replay_baseline',baseline_view);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);old=module.ViewConsistentTracker.from_existing(old)
    frames={};outputs=[];session=None;last=None;missing=[]
    start=summary['drive_started_ros_ns'];end=summary['supervisor_stop_requested_ros_ns']
    stamps=np.asarray([t[0] for t in truth],dtype=float);centers=np.asarray([t[1] for t in truth]);velocities=np.asarray([t[2] for t in truth])
    for topic,ns,msg in messages(directory,('/perception/lidar_frame','/dynamic_tracking/tracks'),0,2**63-1):
        value=json.loads(msg.data)
        if topic.endswith('lidar_frame'):
            if value['status']['state'].startswith('READY_'):frames[value['status']['source_stamp_ns']]=value
            continue
        source=value.get('source_stamp_ns')
        if not value.get('ready') or type(source)is not int or source==last:continue
        if value['tracker_session_id']!=session:old.reset();new.reset();session=value['tracker_session_id'];last=None
        envelope=frames.get(source)
        if envelope is None:missing.append(source);continue
        last=source;ego=value['ego_source_pose'][0];before=old.update(envelope,ego);after=new.update(envelope,ego)
        # Posthoc association and truth interpolation occur only below this line.
        if not start<=source<=end or not truth[0][0]<=source<=truth[-1][0]:continue
        center=np.array([np.interp(source,stamps,centers[:,k]) for k in (0,1)]);velocity=np.array([np.interp(source,stamps,velocities[:,k]) for k in (0,1)])
        result=dict(source_ns=source,t=(source-start)/1e9,actual_bbox_center_xy=center.tolist(),actual_velocity_xy_mps=velocity.tolist())
        for name,tracks in [('before',before),('after',after)]:
            track=min(tracks,key=lambda t:float(np.linalg.norm(t.state[:2]-center)),default=None)
            if track is None or np.linalg.norm(track.state[:2]-center)>5.:continue
            result[name]=dict(id=track.identifier,frames=track.frames,motion=track.motion,near_actual_vehicle_ids=[t.identifier for t in tracks if np.linalg.norm(t.state[:2]-center)<3.],center_xy=track.state[:2].tolist(),velocity_xy_mps=track.state[2:].tolist(),position_error_m=float(np.linalg.norm(track.state[:2]-center)),velocity_error_mps=float(np.linalg.norm(track.state[2:]-velocity)),lateral_x_error_m=float(track.state[0]-center[0]))
        outputs.append(result)
    result=dict(scope='OFFLINE_IDENTICAL_RECORDED_SENSOR_SOURCES_NOT_MORAI_DRIVING_QUALIFICATION_ACTOR_ONLY_POSTHOC',trial=directory.name,sources=len(outputs),missing_recorded_atomic_frames=sorted(set(missing)),baseline_model_source_sha256=baseline_hashes,baseline_view_source_sha256=hashlib.sha256(baseline_view.read_bytes()).hexdigest() if baseline_view else None,rows=outputs)
    for name in ('before','after'):
        values=[x[name] for x in outputs if name in x]
        result[name]=dict(matched_sources=len(values),position_error_p95_m=float(np.percentile([x['position_error_m'] for x in values],95)),velocity_error_p95_mps=float(np.percentile([x['velocity_error_mps'] for x in values],95)),maximum_abs_lateral_x_error_m=max(abs(x['lateral_x_error_m']) for x in values),maximum_near_actual_vehicle_id_count=max(len(x['near_actual_vehicle_ids']) for x in values))
        mature=[x for x in values if x['frames']>=25]
        result[name]['mature_sources']=len(mature)
        if mature:result[name].update(mature_position_error_p95_m=float(np.percentile([x['position_error_m'] for x in mature],95)),mature_velocity_error_p95_mps=float(np.percentile([x['velocity_error_mps'] for x in mature],95)))
    output.write_text(json.dumps(result,indent=2));return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--baseline-view-tracker',type=Path);p.add_argument('--baseline-model-dir',type=Path);a=p.parse_args();v=replay(a.trial,a.output,a.baseline_view_tracker,a.baseline_model_dir);print(json.dumps({k:x for k,x in v.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
