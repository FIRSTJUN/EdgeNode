"""Recorded-input stateful replay; no ROS publishers and no per-cycle state reset."""
import gzip
import json
from pathlib import Path

from edgenode_planning.corridor_scene import planner_from_scene


def live_sequences():
    path=Path(__file__).parent/'fixtures/s4_stateful_live_sequences.json.gz'
    return json.loads(gzip.decompress(path.read_bytes()))


def restored_sequence(fixture,run):
    return planner_from_scene(dict(schema='edgenode_s4_corridor_scene_v1',
        network=fixture['network'],planner_before=run['seed']))


def recorded_inputs(run,frame):
    observations=run['obstacle_messages'][:frame['obstacle_count']]
    # Historical footprint samples are a conservative collision superset.
    # Association uses only the latest recorded observation, not all history.
    points=[p for group in observations for p in group]
    latest=observations[-1] if observations else []
    return dict(reference_points=frame['reference'],obstacles=points,
        current_position=frame['position'],current_heading=frame['yaw'],
        obstacle_groups=[latest[i:i+8] for i in range(0,len(latest),8)],
        road_progress_m=frame['progress'],current_link_id='',
        route_link_ids=run['route'],speed_kmh=frame['speed'])


def replay_recorded(fixture,run,planner=None):
    planner=restored_sequence(fixture,run) if planner is None else planner
    history=[]
    for frame in run['frames']:
        result=planner.plan(**recorded_inputs(run,frame))
        history.append((frame,result))
    return planner,history
