#!/usr/bin/env python3
"""Verify CLOSED D bags preserve inline rays matching admitted source hashes.

No scratch files or Actor information are used. Full inline archive bytes are
validated against their stamped pointer hashes after the scratch ring expires.
"""
import argparse,base64,json,math
import numpy as np
from pathlib import Path
from s4_failure_forensics import messages
from edgenode_dynamic_overtake.ray_transport import verify_inline

def audit(directory,tag):
    output=directory/f'rear-ray-transport-audit-{tag}.json';assert not output.exists()
    summary=json.loads((directory/'summary.json').read_text())
    assert summary.get('rosbag_metadata_present') and summary.get('rosbag_exit_code')==0
    archive={};pointers={};required=set();frames=0;inline_bytes=0;small_bytes=[];durable=[]
    start=summary['drive_started_ros_ns'];stop=summary['supervisor_stop_requested_ros_ns']
    for topic,ns,msg in messages(directory,('/dynamic_overtake/rear_ray_evidence','/dynamic_overtake/rear_scene','/planning/avoidance_debug'),0,2**63-1):
        value=json.loads(msg.data)
        if topic.endswith('/rear_ray_evidence'):
            frames+=1
            for row in value.get('sensors',[]):
                raw=base64.b64decode(row['xyz_f64_le'],validate=True);verify_inline(raw,row['source_ns'],row['xyz_file']);inline_bytes+=len(raw)
                archive[(row['name'],row['source_ns'],row['xyz_file']['sha256'])]=len(raw)
        elif topic.endswith('/rear_scene'):
            small_bytes.append(len(msg.data.encode()))
            for row in value.get('sensors',[]):pointers[(row['name'],row['source_ns'],row['xyz_file']['sha256'])]=row
        elif start<=ns<=stop:
            for row in value.get('dynamic_debug',{}).get('admitted_rear_sources',[]):
                if row.get('sha256'):required.add((row['name'],row['source_ns'],row['sha256']))
    preserved=directory/'ray-source-archive';manifest=preserved/'manifest.json'
    if manifest.exists():
        import hashlib,struct
        for row in json.loads(manifest.read_text())['files']:
            filename=(preserved/row['path']).resolve();assert filename.is_relative_to(preserved.resolve())
            data=filename.read_bytes();assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
            magic,source=struct.unpack_from('<8sQ',data);assert magic==b'EDNDRAY1' and source==row['source_ns'] and (len(data)-16)%24==0
            for key in required-set(archive):
                if key[1]==source and key[2]==row['sha256']:
                    archive[key]=len(data)-16;durable.append(dict(sensor=key[0],source_ns=source,sha256=key[2],artifact=str(filename.relative_to(directory.resolve()))))
    missing=sorted(required-set(archive));reconstructed=[]
    if missing:
        from edgenode_perception.lidar_obstacle_node import pointcloud_xyz
        from edgenode_perception.lidar_obstacles import gravity_ground_heights
        wanted={key[1] for key in missing};raw_frames={};attitudes=[]
        # Bag timestamps are receipt times. A raw source admitted within the
        # unchanged .6 s deadline can arrive more than .1 s after its stamp.
        # Read the full closed-bag interval; select only exact source stamps
        # below, and still require byte-identical stamped content hashes.
        for topic,ns,msg in messages(directory,('/lidar/points','/lidar_rear/points','/Imu'),0,2**63-1):
            source=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if topic=='/Imu':
                q=msg.orientation;norm=math.hypot(q.x,q.y,q.z,q.w)
                if norm<1e-12 or msg.orientation_covariance[0]==-1:continue
                x,y,z,w=(v/norm for v in (q.x,q.y,q.z,q.w));attitudes.append((source,math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1.,min(1.,2*(w*y-z*x))))))
            elif source in wanted:raw_frames[('original_main_lidar' if topic=='/lidar/points' else 'd_roof_hdl64',source)]=msg
        for key in missing:
            row=pointers.get(key);msg=raw_frames.get(key[:2])
            if row is None or msg is None:continue
            xyz=pointcloud_xyz(msg)
            candidates=([tuple(row[k] for k in ('gravity_attitude_source_ns','gravity_roll_rad','gravity_pitch_rad'))] if 'gravity_attitude_source_ns' in row else [a for a in attitudes if abs(a[0]-key[1])<=50_000_000])
            for attitude in candidates:
                leveled=gravity_ground_heights(xyz,attitude[1],attitude[2]);leveled=leveled[np.isfinite(leveled).all(axis=1)&(leveled[:,0]<=0.)&(leveled[:,2]<=row['maximum_belt_z'])]
                raw=np.asarray(leveled,dtype='<f8').tobytes()
                try:verify_inline(raw,key[1],row['xyz_file'])
                except ValueError:continue
                archive[key]=len(raw);reconstructed.append(dict(sensor=key[0],source_ns=key[1],sha256=key[2],recorded_imu_source_ns=attitude[0],exact_stamped_content_hash_match=True));break
    missing=sorted(required-set(archive));result=dict(scope='CLOSED_BAG_AND_PRESERVED_EXACT_SOURCE_BYTES_MATCH_STAMPED_HASH_NO_SCRATCH_OR_ACTOR',trial=directory.name,archive_messages=frames,archive_unique_sensor_sources=len(archive),inline_coordinate_bytes_validated=inline_bytes,maximum_planning_scene_message_bytes=max(small_bytes,default=None),admitted_sensor_sources=len(required),missing_admitted_sources=missing,archive_independent_of_expired_scratch=True,preserved_exact_source_artifacts=durable,reconstructed_from_original_raw_and_imu=reconstructed)
    result['success']=bool(frames>=20 and required and not missing and max(small_bytes,default=10**9)<100000)
    output.write_text(json.dumps(result,indent=2));return result

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('trial',type=Path);parser.add_argument('--tag',default='v1');a=parser.parse_args();result=audit(a.trial,a.tag);print(json.dumps(result,indent=2));return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
