"""ROS-independent shortest routes over directed MGeo node connections."""

import heapq
import json
import math
from pathlib import Path


def _finite_number(value):
    """Accept finite JSON numbers, excluding booleans and overflowing integers."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _index_records(records):
    """Index records with string IDs; omit ambiguous duplicate IDs."""
    indexed = {}
    duplicates = set()
    if not isinstance(records, list):
        return indexed
    for record in records:
        if not isinstance(record, dict):
            continue
        record_id = record.get('idx')
        if not isinstance(record_id, str) or not record_id:
            continue
        if record_id in indexed:
            duplicates.add(record_id)
        indexed[record_id] = record
    for record_id in duplicates:
        del indexed[record_id]
    return indexed


def _link_cost(link):
    """Validate XYZ geometry and prefer MGeo length, with an XY fallback."""
    points = link.get('points')
    if not isinstance(points, list) or len(points) < 2:
        return None
    for point in points:
        if not isinstance(point, list) or len(point) != 3:
            return None
        if not all(_finite_number(value) for value in point):
            return None

    length = link.get('link_length')
    if _finite_number(length) and length > 0:
        return float(length)

    length = sum(
        math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        for a, b in zip(points, points[1:])
    )
    return length if math.isfinite(length) and length > 0 else None


class DijkstraPlanner:
    """Load MGeo once and plan using only actual from-node/to-node edges.

    ``graph[node_id]`` holds outgoing edges with ``to_node_idx``, ``link_id``
    and ``cost_m``. Lane-change metadata never contributes edges. Invalid
    links are excluded; unreadable or malformed map files leave an empty
    planner whose ``plan`` method returns None.

    The search starts at the current link's to-node. Returned link IDs,
    node IDs, XYZ points and cost cover the entire current link as well as
    the shortest continuation. Thus ``node_ids[0]`` is the current link's
    from-node, while ``start_node_id`` is its to-node. This is the full-link
    cost, not the remaining distance from the vehicle's matched position.
    """

    def __init__(self, mgeo_dir):
        self.nodes_by_id = {}
        self.links_by_id = {}
        self.graph = {}
        self._link_costs = {}

        try:
            directory = Path(mgeo_dir)
            with (directory / 'node_set.json').open('r', encoding='utf-8') as stream:
                raw_nodes = json.load(stream)
            with (directory / 'link_set.json').open('r', encoding='utf-8') as stream:
                raw_links = json.load(stream)
        except (OSError, UnicodeError, ValueError):
            return

        self.nodes_by_id = _index_records(raw_nodes)
        self.links_by_id = _index_records(raw_links)
        self.graph = {node_id: [] for node_id in self.nodes_by_id}

        for link_id, link in self.links_by_id.items():
            from_node = link.get('from_node_idx')
            to_node = link.get('to_node_idx')
            if not isinstance(from_node, str) or not isinstance(to_node, str):
                continue
            if from_node not in self.nodes_by_id or to_node not in self.nodes_by_id:
                continue
            cost = _link_cost(link)
            if cost is None:
                continue
            self._link_costs[link_id] = cost
            self.graph[from_node].append({
                'to_node_idx': to_node,
                'link_id': link_id,
                'cost_m': cost,
            })

    def plan(self, start_link_id, goal_node_id):
        """Return a full route dictionary, or None for invalid/unreachable input.

        Each returned point is a copy of the original MGeo [x, y, z] value.
        Consecutive identical points, including shared link endpoints, appear
        once. ``total_cost_m`` includes the current link's complete cost.
        """
        if not isinstance(start_link_id, str) or not isinstance(goal_node_id, str):
            return None
        if start_link_id not in self._link_costs or goal_node_id not in self.nodes_by_id:
            return None

        start_link = self.links_by_id[start_link_id]
        start_node_id = start_link['to_node_idx']
        distances = {start_node_id: 0.0}
        predecessors = {}
        queue = [(0.0, start_node_id)]

        while queue:
            cost, node_id = heapq.heappop(queue)
            if cost > distances[node_id]:
                continue
            if node_id == goal_node_id:
                break
            for edge in self.graph[node_id]:
                next_node = edge['to_node_idx']
                next_cost = cost + edge['cost_m']
                if not math.isfinite(next_cost):
                    continue
                if next_cost < distances.get(next_node, math.inf):
                    distances[next_node] = next_cost
                    predecessors[next_node] = (node_id, edge['link_id'])
                    heapq.heappush(queue, (next_cost, next_node))
        else:
            return None

        node_ids = [goal_node_id]
        link_ids = []
        node_id = goal_node_id
        while node_id != start_node_id:
            previous_node, link_id = predecessors[node_id]
            link_ids.append(link_id)
            node_ids.append(previous_node)
            node_id = previous_node
        node_ids.reverse()
        link_ids.reverse()
        node_ids.insert(0, start_link['from_node_idx'])
        link_ids.insert(0, start_link_id)

        total_cost = sum(self._link_costs[link_id] for link_id in link_ids)
        if not math.isfinite(total_cost):
            return None
        points = []
        for link_id in link_ids:
            for point in self.links_by_id[link_id]['points']:
                if not points or points[-1] != point:
                    points.append(point.copy())

        return {
            'start_link_id': start_link_id,
            'start_node_id': start_node_id,
            'goal_node_id': goal_node_id,
            'node_ids': node_ids,
            'link_ids': link_ids,
            'points': points,
            'total_cost_m': total_cost,
        }
