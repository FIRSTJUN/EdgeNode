"""ROS-independent MGeo link matching ported from test_link_matching_v02.py."""

import json
import math
from pathlib import Path


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def angle_difference_deg(a, b):
    return abs(normalize_angle_deg(a - b))


def point_to_segment(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy

    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0

    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    nearest_x = ax + t * dx
    nearest_y = ay + t * dy
    distance = math.hypot(px - nearest_x, py - nearest_y)

    # ROS ENU: East = 0 degrees, counterclockwise positive.
    heading = math.degrees(math.atan2(dy, dx))
    return distance, heading


def closest_point_on_link(px, py, link):
    points = link['points']
    best_distance = float('inf')
    best_heading = 0.0

    for i in range(len(points) - 1):
        ax, ay = points[i][0], points[i][1]
        bx, by = points[i + 1][0], points[i + 1][1]
        distance, heading = point_to_segment(px, py, ax, ay, bx, by)

        if distance < best_distance:
            best_distance = distance
            best_heading = heading

    return best_distance, best_heading


class MapMatcher:
    """Match poses in MGeo local ENU coordinates, retaining the last match.

    The map is read once during construction. A successful match returns a
    dictionary containing link_id, distance (m), link_heading (degrees),
    heading_diff (degrees), and score. A rejected pose returns None and keeps
    the last successful link for continuity, as in v0.2.
    """

    def __init__(
        self,
        mgeo_dir,
        search_radius_m=6.0,
        max_match_distance_m=4.0,
        max_heading_diff_deg=80.0,
        heading_weight=0.025,
        same_link_bonus=0.4,
        connected_link_bonus=0.7,
        unrelated_link_penalty=1.0,
    ):
        self.search_radius_m = search_radius_m
        self.max_match_distance_m = max_match_distance_m
        self.max_heading_diff_deg = max_heading_diff_deg
        self.heading_weight = heading_weight
        self.same_link_bonus = same_link_bonus
        self.connected_link_bonus = connected_link_bonus
        self.unrelated_link_penalty = unrelated_link_penalty
        self.previous_link_id = None

        with (Path(mgeo_dir) / 'link_set.json').open(
            'r', encoding='utf-8',
        ) as stream:
            raw_links = json.load(stream)

        self.links = []
        self.links_by_id = {}
        self.outgoing = {}

        for link in raw_links:
            points = link.get('points', [])
            if len(points) < 2:
                continue

            xs = [float(p[0]) for p in points]
            ys = [float(p[1]) for p in points]
            link['_bbox'] = (min(xs), max(xs), min(ys), max(ys))
            self.links.append(link)
            self.links_by_id[link['idx']] = link
            self.outgoing.setdefault(link.get('from_node_idx'), set()).add(
                link['idx'],
            )

    def bbox_candidate(self, px, py, bbox):
        min_x, max_x, min_y, max_y = bbox
        radius = self.search_radius_m
        return (
            min_x - radius <= px <= max_x + radius
            and min_y - radius <= py <= max_y + radius
        )

    def get_connected_links(self):
        previous = self.links_by_id.get(self.previous_link_id)
        if previous is None:
            return set()

        connected = {self.previous_link_id}
        connected.update(self.outgoing.get(previous.get('to_node_idx'), set()))

        left = previous.get('left_lane_change_dst_link_idx')
        right = previous.get('right_lane_change_dst_link_idx')
        if left:
            connected.add(left)
        if right:
            connected.add(right)
        return connected

    def match(self, x, y, heading_deg):
        """Return the lowest-scoring eligible link, or None if none qualifies."""
        # NaN must not bypass the distance/heading rejection comparisons.
        if not all(math.isfinite(value) for value in (x, y, heading_deg)):
            return None

        connected = self.get_connected_links()
        best = None

        for link in self.links:
            if not self.bbox_candidate(x, y, link['_bbox']):
                continue

            distance, link_heading = closest_point_on_link(x, y, link)
            if distance > self.max_match_distance_m:
                continue

            heading_diff = angle_difference_deg(heading_deg, link_heading)
            if heading_diff > self.max_heading_diff_deg:
                continue

            score = distance + self.heading_weight * heading_diff
            link_id = link['idx']
            if self.previous_link_id is not None:
                if link_id == self.previous_link_id:
                    score -= self.same_link_bonus
                elif link_id in connected:
                    score -= self.connected_link_bonus
                else:
                    score += self.unrelated_link_penalty

            if best is None or score < best['score']:
                best = {
                    'link_id': link_id,
                    'distance': distance,
                    'link_heading': link_heading,
                    'heading_diff': heading_diff,
                    'score': score,
                }

        if best is not None:
            self.previous_link_id = best['link_id']
        return best
