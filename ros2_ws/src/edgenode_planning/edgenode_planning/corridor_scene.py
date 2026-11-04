"""One-cycle JSON snapshots and offline replay; never publishes ROS commands."""
import argparse
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace

from .adjacent_lane_planner import AdjacentLanePlanner, WorldTransition
from .lane_geometry import LaneNetwork, LanePair


def json_value(value):
    """Strict JSON: unknown/unbounded measurements are null, never NaN/Infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    return value


def _encode(value):
    # Preserve state types and nonfinite sentinels exactly, including release
    # confirmation tuple keys and the remembered obstacle set during HOLD.
    if isinstance(value, float) and not math.isfinite(value):
        return {'type': 'float', 'value': str(value)}
    if isinstance(value, (set, tuple)):
        return {'type': type(value).__name__, 'items': [_encode(v) for v in
                (sorted(value) if isinstance(value, set) else value)]}
    if isinstance(value, list):
        return [_encode(v) for v in value]
    if isinstance(value, dict):
        return {k: _encode(v) for k, v in value.items()}
    return value


def _decode(value):
    if isinstance(value, list):
        return [_decode(v) for v in value]
    if isinstance(value, dict):
        if value.get('type') == 'float':
            return float(value['value'])
        if value.get('type') in ('set', 'tuple'):
            return (set if value['type'] == 'set' else tuple)(_decode(v) for v in value['items'])
        return {k: _decode(v) for k, v in value.items()}
    return value


def planner_snapshot(planner):
    """Capture PRE-plan history as well as parameters; no planner mutation."""
    values = {k: v for k, v in vars(planner).items()
              if k not in ('network', '_lane_pair', '_world_transition', '_corridor_debug_cache', '_maneuver_context')}
    pair = planner._lane_pair
    values['_lane_pair'] = (dict(original=pair.original, adjacent=pair.adjacent, ids=pair.ids,
                                side=pair.side, section_ends=pair.section_ends) if pair else None)
    values['_world_transition'] = vars(planner._world_transition) if planner._world_transition else None
    return _encode(copy.deepcopy(values))


def capture_scene(planner, before, inputs, result, ros_time_ns, frame_id):
    if result is None:
        return None
    ids = set(inputs['route_link_ids']) | {inputs['current_link_id']}
    old_pair = _decode(before)['_lane_pair']
    if old_pair:
        ids.update(link_id for pair in old_pair['ids'] for link_id in pair)
    for link_id in list(ids):
        record = planner.network.links.get(link_id, {})
        ids.update(record.get(side+'_lane_change_dst_link_idx') for side in ('left', 'right'))
    ids.discard(None)
    # Map records are immutable planner inputs. Only this route and its
    # declared neighbors are needed, including an active frozen lane pair.
    network = dict(links=[planner.network.links[i] for i in sorted(ids) if i in planner.network.links],
                   valid_link_ids=sorted(ids & planner.network.valid))
    return dict(schema='edgenode_s4_corridor_scene_v1', ros_time_ns=ros_time_ns, frame_id=frame_id,
                provenance='Live Planning timer input snapshot; no MORAI rollout',
                inputs=copy.deepcopy(inputs), planner_before=before, network=network,
                result=json_value(result),
                canonical_local_path_points=json_value(result['points'] if not result['blocked'] else []))


def write_scene(scene, directory):
    if scene is None:
        raise ValueError('No current adjacent-lane planning input; wait for a valid Planning cycle')
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    path = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=directory,
                                         prefix='s4_corridor_', suffix='.json', delete=False) as stream:
            path = Path(stream.name)
            json.dump(scene, stream, allow_nan=False, ensure_ascii=False, indent=2)
            stream.write('\n')
    except Exception:
        if path is not None:
            path.unlink(missing_ok=True)
        raise
    return str(path.resolve())


def planner_from_scene(scene):
    """Restore the PRE-plan state for offline one-cycle or repeated-input replay."""
    if scene.get('schema') != 'edgenode_s4_corridor_scene_v1':
        raise ValueError('Unsupported corridor scene schema')
    links = {record['idx']: record for record in scene['network']['links']}
    network = LaneNetwork(SimpleNamespace(links_by_id=links, graph={
        'snapshot': [{'link_id': i} for i in scene['network']['valid_link_ids']]}))
    planner = AdjacentLanePlanner(network)
    state = _decode(scene['planner_before'])
    pair, transition = state.pop('_lane_pair'), state.pop('_world_transition')
    if set(state)-set(vars(planner)):
        raise ValueError('Snapshot contains unknown planner state fields')
    vars(planner).update(state)
    planner._lane_pair = LanePair(**pair) if pair else None
    if transition:
        restored = WorldTransition.__new__(WorldTransition)
        vars(restored).update(transition)
        planner._world_transition = restored
    return planner


def replay_scene(scene):
    planner = planner_from_scene(scene)
    inputs = scene['inputs']
    return planner.plan(inputs['reference_points'], inputs['obstacles'],
                        current_position=inputs['current_position'], current_heading=inputs['current_heading'],
                        obstacle_groups=inputs['obstacle_groups'], road_progress_m=inputs['road_progress_m'],
                        current_link_id=inputs['current_link_id'], route_link_ids=inputs['route_link_ids'],
                        speed_kmh=inputs['speed_kmh'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    scene = json.loads(args.scene.read_text())
    result = json_value(replay_scene(scene))
    report = dict(matches_recorded=result == scene['result'], result=result)
    text = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
