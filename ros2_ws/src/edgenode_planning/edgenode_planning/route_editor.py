"""ROS-independent waypoint editing and persistence over the existing MGeo graph."""

import math
import os
from pathlib import Path
import re
import tempfile

import yaml

from edgenode_planning.dijkstra_planner import DijkstraPlanner, _finite_number


class RouteEditorError(ValueError):
    """An expected, user-correctable editing error with a status code."""

    def __init__(self, code, detail):
        self.code = code
        super().__init__(f'{code}: {detail}')


class RouteEditor:
    """Own the draft only. This class never writes Planning parameters.

    Node positions are already in the map's local coordinates: no origin
    offset is added. Each preview delegates to DijkstraPlanner.plan_route.
    """

    def __init__(self, mgeo_dir, planner=None):
        self.mgeo_dir = str(Path(mgeo_dir).resolve())
        self.planner = planner if planner is not None else DijkstraPlanner(mgeo_dir)
        self.node_points = {}
        for node_id, record in self.planner.nodes_by_id.items():
            point = record.get('point')
            if (isinstance(point, (list, tuple)) and len(point) == 3
                    and all(_finite_number(value) for value in point)):
                self.node_points[node_id] = tuple(float(value) for value in point)
        self.valid_link_ids = {
            edge['link_id'] for edges in self.planner.graph.values() for edge in edges
        }
        self._waypoints = []
        self.start_link_id = ''
        self.preview = None
        self.preview_status = 'WAITING_FOR_CURRENT_LINK'
        self.revision = 0

    @property
    def waypoint_node_ids(self):
        return list(self._waypoints)

    def snap(self, x, y, max_distance_m=5.0):
        if not all(_finite_number(v) for v in (x, y, max_distance_m)) or max_distance_m <= 0:
            raise RouteEditorError('INVALID_SNAP_INPUT', 'finite XY and positive snap distance required')
        if not self.node_points:
            raise RouteEditorError('MAP_UNAVAILABLE', 'no valid MGeo node coordinates')
        node_id = min(self.node_points, key=lambda key: (
            math.hypot(self.node_points[key][0] - x, self.node_points[key][1] - y), key,
        ))
        point = self.node_points[node_id]
        distance = math.hypot(point[0] - x, point[1] - y)
        if not math.isfinite(distance) or distance > max_distance_m:
            raise RouteEditorError('SNAP_TOO_FAR', f'nearest node is {distance:.2f} m away')
        return {'node_id': node_id, 'point': point, 'distance_m': distance}

    def append_click(self, x, y, max_distance_m=5.0):
        snapped = self.snap(x, y, max_distance_m)
        if self._waypoints and self._waypoints[-1] == snapped['node_id']:
            raise RouteEditorError('DUPLICATE_WAYPOINT', 'consecutive duplicate ignored')
        self._waypoints.append(snapped['node_id'])
        self._changed()
        return snapped

    def undo(self):
        if not self._waypoints:
            raise RouteEditorError('EMPTY_WAYPOINTS', 'nothing to undo')
        removed = self._waypoints.pop()
        self._changed()
        return removed

    def clear(self):
        self._waypoints.clear()
        self._changed()

    def set_start_link(self, link_id):
        valid = link_id if isinstance(link_id, str) and link_id in self.valid_link_ids else ''
        if valid == self.start_link_id:
            return False
        self.start_link_id = valid
        self._changed()
        return True

    def _changed(self):
        self.revision += 1
        self.refresh_preview()

    def refresh_preview(self):
        # Clear the previous result even on failure; stale previews must not
        # remain visible or become eligible for activation.
        self.preview = None
        if not self.start_link_id:
            self.preview_status = 'WAITING_FOR_CURRENT_LINK'
        elif not self._waypoints:
            self.preview_status = 'READY'
        else:
            self.preview = self.planner.plan_route(self.start_link_id, self._waypoints)
            self.preview_status = 'PREVIEW_READY' if self.preview is not None else 'ROUTE_NOT_FOUND'
        return self.preview

    def activation_node_ids(self):
        self._validate_waypoints(self._waypoints)
        # Revalidate against the draft's captured start link before activation.
        if self.refresh_preview() is None:
            raise RouteEditorError(self.preview_status, 'a reachable preview is required')
        return self.waypoint_node_ids

    def _validate_waypoints(self, node_ids):
        if not isinstance(node_ids, list) or not node_ids:
            raise RouteEditorError('EMPTY_WAYPOINTS', 'at least one waypoint is required')
        for node_id in node_ids:
            if not isinstance(node_id, str) or node_id not in self.node_points:
                raise RouteEditorError('INVALID_WAYPOINT', f'unknown or invalid node: {node_id!r}')

    @staticmethod
    def route_file(save_dir, route_name):
        if not isinstance(route_name, str) or re.fullmatch(r'[\w-]{1,80}', route_name) is None:
            raise RouteEditorError('INVALID_ROUTE_NAME', 'use 1–80 letters, numbers, underscores or hyphens')
        return Path(save_dir) / f'{route_name}.yaml'

    def save(self, save_dir, route_name):
        self._validate_waypoints(self._waypoints)
        destination = self.route_file(save_dir, route_name)
        data = {
            'route_name': route_name,
            'frame_id': 'map',
            'waypoint_node_ids': self.waypoint_node_ids,
            'created_from': {'mgeo_dir': self.mgeo_dir},
        }
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=destination.parent, suffix='.tmp',
                                             delete=False) as stream:
                temporary = Path(stream.name)
                yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        return destination

    def load(self, save_dir, route_name, start_link_id=None):
        """Restore a draft, optionally capturing a new session start on success."""
        source = self.route_file(save_dir, route_name)
        try:
            with source.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise RouteEditorError('INVALID_ROUTE_FILE', 'malformed YAML') from error
        if not isinstance(data, dict) or data.get('frame_id') != 'map':
            raise RouteEditorError('INVALID_ROUTE_FILE', 'expected a mapping with frame_id: map')
        node_ids = data.get('waypoint_node_ids')
        self._validate_waypoints(node_ids)
        origin = data.get('created_from', {})
        if not isinstance(origin, dict):
            raise RouteEditorError('INVALID_ROUTE_FILE', 'created_from must be a mapping')
        saved_map = origin.get('mgeo_dir')
        if saved_map is not None and (not isinstance(saved_map, str)
                                      or str(Path(saved_map).resolve()) != self.mgeo_dir):
            raise RouteEditorError('MAP_MISMATCH', 'saved route references another MGeo directory')
        # Validate before replacing the draft. Only consecutive duplicates are
        # removed; deliberate revisits to a node later in the route are kept.
        normalized = []
        for node_id in node_ids:
            if not normalized or normalized[-1] != node_id:
                normalized.append(node_id)
        if start_link_id is not None:
            self.start_link_id = (start_link_id if isinstance(start_link_id, str)
                                  and start_link_id in self.valid_link_ids else '')
        self._waypoints = normalized
        self._changed()
        return source
