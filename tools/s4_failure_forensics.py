#!/usr/bin/env python3
"""Read closed MORAI bags; separate sensor transport from route/body failures.

Writes only to a new output directory. Actor dimensions are audit evidence,
never inputs to the running planner. Sample counts are not collision counts.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import sqlite3

from s4_live_trial_report import measured_body_for_audit
from edgenode_planning.avoidance_geometry import swept_body
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.lane_geometry import PolylineIndex, sample
from edgenode_planning.vehicle_geometry import sonata_geometry


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def messages(directory, topics, start, end):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    for database in sorted((directory / 'bag').glob('*.db3')):
        with sqlite3.connect('file:' + str(database) + '?mode=ro', uri=True) as connection:
            kinds = {i: (name, get_message(kind)) for i, name, kind in
                     connection.execute('select id,name,type from topics') if name in topics}
            if not kinds:
                continue
            placeholders = ','.join('?' for _ in kinds)
            query = ('select topic_id,timestamp,data from messages where topic_id in (' +
                     placeholders + ') and timestamp between ? and ? order by timestamp')
            for tid, ns, data in connection.execute(query, (*kinds, start, end)):
                name, kind = kinds[tid]
                yield name, ns, deserialize_message(data, kind)


def coasting_evidence(directory, summary, rows, boundaries, body, path, index):
    """Resolve endpoint-only STOPPING_AUDIT warnings with all recorded Ego poses."""
    drive_start=summary['drive_started_ros_ns']
    end=drive_start+round(rows[-1]['t']*1e9)
    stop=summary.get('before_restore',{}).get('values',{})
    stop_ns=stop.get('/ego_vehicle_status',{}).get('source_stamp_ns')
    if not stop_ns or stop_ns<=end:
        return dict(available=False,reason='no recorded post-supervision stopping interval')
    convergence=rows[-1]['values'].get('/localization/grid_convergence_deg')
    previous=None;contacts=[];checked=0;command=None
    topics=('/ego_vehicle_status','/localization/grid_convergence_deg','/ctrl_cmd')
    for name,ns,message in messages(directory,topics,end-200_000_000,stop_ns+300_000_000):
        if name=='/localization/grid_convergence_deg':
            convergence=message.data;continue
        if name=='/ctrl_cmd':
            command=dict(accel=message.accel,brake=message.brake,steer=message.front_steer);continue
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp>stop_ns or convergence is None:
            continue
        pose=(message.position.x,message.position.y,math.radians(message.heading+convergence))
        if stamp>=end:
            polygon=swept_body(body,previous or pose,pose)
            reason=boundaries.swept_reason(polygon);checked+=1
            if reason:
                station=max(0.,min(index.lengths[-1],index.project(pose[:2])))
                point,tangent=sample(path,station,index.lengths)
                path_pose=(*point[:2],math.atan2(tangent[1],tangent[0]))
                contacts.append(dict(t_s=(stamp-drive_start)/1e9,reason=reason,
                    actual_pose_intersects=bool(boundaries.swept_reason(body.polygon(*pose))),
                    body_on_global_path_contact_reason=boundaries.swept_reason(body.polygon(*path_pose)),
                    lateral_error_m=(pose[0]-point[0])*-tangent[1]+(pose[1]-point[1])*tangent[0],
                    command=command,actual_steer_deg=message.front_steer_angle,pose=pose,path_pose=path_pose))
        previous=pose
    return dict(available=checked>0,checked_ego_frames=checked,contacts=contacts,
                contact_samples=len(contacts),scope='All recorded Ego poses from supervision end through confirmed stop; no teleport')


def analyze(directory, output):
    if not (directory / 'bag/metadata.yaml').exists():
        raise ValueError('closed bag required')
    output.mkdir(exist_ok=False, parents=True)
    summary = json.loads((directory / 'summary.json').read_text())
    rows = [json.loads(line) for line in (directory / 'samples.jsonl').read_text().splitlines()]
    start = summary['drive_started_ros_ns']
    end = start + round(rows[-1]['t'] * 1e9)
    first = None
    topics = ('/planning/avoidance_debug', '/planning/s4_sensor_state',
              '/planning/diagnostics', '/planning/fsm_state', '/perception/lidar_status')
    series = defaultdict(list)
    for name, ns, message in messages(directory, topics, start, end):
        try:
            value = json.loads(message.data)
        except ValueError:
            value = message.data
        t = (ns - start) / 1e9
        series[name].append((t, value))
        if first is None and name == '/planning/avoidance_debug' and (
                value.get('state') in ('SENSOR_HOLD', 'BLOCKED')):
            first = dict(t_s=t, state=value['state'], reason=value.get('reason'),
                         active=value.get('active'),stage=value.get('stage'))
    sensor = dict(first_stop=first)
    if first:
        t = first['t_s']
        window = {k: [(s, v) for s, v in items if t - 2 <= s <= t + 1]
                  for k, items in series.items()}
        dump(output / 'sensor-event-window.json', window)
        status = [(s, v) for s, v in window['/perception/lidar_status'] if s <= t]
        ready = [(s, v) for s, v in status if v['state'] in ('READY_EMPTY', 'READY_OBSTACLES')]
        states = [(s, v) for s, v in window['/planning/s4_sensor_state'] if s <= t]
        timings = [v['planning_elapsed_ms'] for s, v in window['/planning/diagnostics'] if s <= t]
        sensor.update(last_producer_status=ready[-1] if ready else None,
                      last_planning_sensor_state=states[-1] if states else None,
                      planning_max_ms=max(timings, default=None),
                      planning_over_100ms=sum(v > 100 for v in timings))
        if ready:
            sensor['producer_source_age_at_stop_s'] = (
                start + round(t * 1e9) - ready[-1][1]['source_stamp_ns']) / 1e9
        sensor['classification'] = ('SHARED_SENSOR_GATE' if 'S4_SENSOR_' in (first['reason'] or '')
                                    else 'PLANNER_REJECTION_REQUIRES_SEPARATE_ANALYSIS')
    path = json.loads((directory / 'global-path.json').read_text())
    index = PolylineIndex(path)
    boundaries = HDMapBoundaries('/workspace/local_data/kcity_hdmap', '/workspace/local_data/kcity_mgeo')
    actual_body = None
    previous = None
    contacts = []
    for row in rows:
        values = row['values']
        ego = values.get('/ego_vehicle_status')
        convergence = values.get('/localization/grid_convergence_deg')
        if not ego or convergence is None:
            continue
        if actual_body is None:
            actual_body = measured_body_for_audit(summary, convergence)
            if actual_body is None:
                raise ValueError('measured body dimensions required')
        pose = (ego['x'], ego['y'], math.radians(ego['heading_deg'] + convergence))
        polygon = swept_body(actual_body, previous or pose, pose)
        previous = pose
        reason = boundaries.swept_reason(polygon)
        if not reason:
            continue
        from shapely.geometry import Polygon, LineString
        identifier = reason.split(':', 1)[1]
        lines = [LineString((a, b)) for a, b, _, key in boundaries.forbidden if key == identifier]
        station = index.project(pose[:2])
        point, tangent = sample(path, station, index.lengths)
        path_pose = (*point[:2], math.atan2(tangent[1], tangent[0]))
        pose_now = actual_body.polygon(*pose)
        planned = actual_body.polygon(*path_pose)
        lateral = (pose[0] - point[0]) * -tangent[1] + (pose[1] - point[1]) * tangent[0]
        record = dict(t_s=row['t'], reason=reason, stage=values.get('/planning/avoidance_debug', {}).get('stage'),
                      link=values.get('/planning/current_link'), station_m=station, pose=pose,
                      path_pose=path_pose, lateral_error_m=lateral,
                      heading_error_deg=math.degrees(math.atan2(math.sin(pose[2]-path_pose[2]),
                                                               math.cos(pose[2]-path_pose[2]))),
                      actual_body_intersects=any(Polygon(pose_now).intersects(line) for line in lines),
                      body_on_global_path_intersects=any(Polygon(planned).intersects(line) for line in lines),
                      configured_body_intersects=bool(boundaries.swept_reason(sonata_geometry().polygon(*pose))),
                      planned_body_boundary_gap_m=min(Polygon(planned).distance(line) for line in lines),
                      speed_kmh=ego['speed_kmh'], actual_steer_deg=ego.get('front_steer_deg'),
                      command=values.get('/ctrl_cmd'), control_error_m=values.get('/control/path_error'))
        contacts.append(record)
    dump(output / 'boundary-contacts.json', contacts)
    if contacts:
        local_paths=[]
        for _,ns,message in messages(directory,('/planning/local_path',),
                start+round((contacts[0]['t_s']-2)*1e9),start+round(contacts[-1]['t_s']*1e9)):
            if len(message.poses)>1:
                local_paths.append(((ns-start)/1e9,PolylineIndex([
                    (p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in message.poses])))
        for record in contacts:
            preceding=[p for t,p in local_paths if t<=record['t_s']]
            if not preceding:
                continue
            local=preceding[-1];station=max(0.,min(local.lengths[-1],local.project(record['pose'][:2])))
            point,tangent=sample(local.points,station,local.lengths)
            pose=(*point[:2],math.atan2(tangent[1],tangent[0]))
            global_station=max(0.,min(index.lengths[-1],index.project(point[:2])))
            global_point,_=sample(path,global_station,index.lengths)
            record['local_path_pose']=pose
            record['body_on_local_path_contact_reason']=boundaries.swept_reason(actual_body.polygon(*pose))
            record['local_reference_distance_to_global_curve_m']=math.dist(pose[:2],global_point[:2])
            record['local_nearest_station_ahead_of_ego_m']=global_station-record['station_m']
        dump(output / 'boundary-contacts.json',contacts)
    coasting=coasting_evidence(directory,summary,rows,boundaries,actual_body,path,index) if actual_body else None
    dump(output/'coasting-boundary-audit.json',coasting)
    result = dict(source=str(directory), source_sha=summary.get('code_sha'),
                  runtime_fingerprint=summary.get('runtime_implementation_sha256'),
                  sensor=sensor, measured_body=actual_body.__dict__ if actual_body else None,
                  configured_body=sonata_geometry().__dict__,
                  coasting_boundary_audit=coasting,
                  boundary=dict(swept_contact_samples=len(contacts),
                                actual_pose_contact_samples=sum(r['actual_body_intersects'] for r in contacts),
                                global_path_body_contact_samples=sum(r['body_on_global_path_intersects'] for r in contacts),
                                local_path_body_contact_samples=sum(bool(r.get('body_on_local_path_contact_reason')) for r in contacts),
                                max_local_reference_distance_to_global_curve_m=max((r.get('local_reference_distance_to_global_curve_m',0.) for r in contacts),default=None),
                                stages=sorted({r['stage'] for r in contacts}),
                                links=sorted({r['link'] for r in contacts}),
                                max_contact_lateral_error_m=max((abs(r['lateral_error_m']) for r in contacts), default=None)),
                  note='Sample contacts are not independent collisions. Global path test uses measured full body at tangent heading.')
    dump(output / 'forensics.json', result)
    if contacts:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 9))
        lo, hi = min(r['station_m'] for r in contacts)-10, max(r['station_m'] for r in contacts)+10
        part = [p for p, station in zip(path, index.lengths) if lo <= station <= hi]
        ax.plot(*zip(*[p[:2] for p in part]), label='Stored global path')
        ax.plot([r['pose'][0] for r in contacts], [r['pose'][1] for r in contacts], label='Measured Ego')
        identifiers = {r['reason'].split(':', 1)[1] for r in contacts}
        for a, b, _, key in boundaries.forbidden:
            if key in identifiers:
                ax.plot([a[0], b[0]], [a[1], b[1]], 'r-', alpha=.5)
        for row in contacts[::max(1, len(contacts)//7)]:
            p = actual_body.polygon(*row['pose']);p.append(p[0])
            ax.plot(*zip(*p), 'k-', alpha=.4)
        ax.axis('equal');ax.legend();ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)')
        ax.set_xlim(min(p[0] for p in part)-4,max(p[0] for p in part)+4)
        ax.set_ylim(min(p[1] for p in part)-4,max(p[1] for p in part)+4)
        fig.tight_layout();fig.savefig(output/'boundary-evidence.png');plt.close(fig)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.trial, args.output), indent=2))
