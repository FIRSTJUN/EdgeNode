"""Directed MGeo link tracking in MORAI local ENU coordinates (metres).

No global nearest-link rematching after acquisition: progress follows topology.
Steering is a normalized wheel angle, positive left, independent of image PID.
"""
from dataclasses import dataclass
import json
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class TrackingResult:
    steering: float
    curvature: float
    cross_track: float
    heading_error: float
    remaining: float
    link_id: str


class MapTracker:
    def __init__(self, map_file, wheelbase=1.04, max_wheel_angle=0.49,
                 turn_preference='straight'):
        if wheelbase <= 0 or max_wheel_angle <= 0:
            raise ValueError('Vehicle dimensions must be positive')
        if turn_preference not in ('straight', 'left', 'right'):
            raise ValueError('turn_preference must be straight, left or right')
        self.wheelbase, self.max_wheel_angle = wheelbase, max_wheel_angle
        self.preference = turn_preference
        self.links = {}
        self.outgoing = {}
        for item in json.load(open(map_file, encoding='utf-8')):
            if item.get('lazy_init') or item.get('opp_traffic'):
                continue
            p = np.asarray(item['points'], dtype=float)[:, :2]
            if len(p) < 2 or not np.isfinite(p).all():
                continue
            keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-5]
            p = p[keep]
            if len(p) < 2:
                continue
            item = dict(item, xy=p)
            self.links[item['idx']] = item
            self.outgoing.setdefault(item['from_node_idx'], []).append(item['idx'])
        if not self.links:
            raise ValueError('Map has no drivable links')
        self.current = None
        self.progress = 0.0
        self.next_links = []

    @staticmethod
    def project(p, position):
        v = np.diff(p, axis=0)
        lengths = np.linalg.norm(v, axis=1)
        t = np.clip(np.sum((position-p[:-1])*v, axis=1)/(lengths*lengths), 0, 1)
        q = p[:-1] + t[:, None]*v
        i = int(np.argmin(np.linalg.norm(q-position, axis=1)))
        yaw = math.atan2(v[i, 1], v[i, 0])
        # Signed displacement: left of route is positive.
        d = position-q[i]
        cross = -math.sin(yaw)*d[0] + math.cos(yaw)*d[1]
        arc = float(lengths[:i].sum()+t[i]*lengths[i])
        return float(np.linalg.norm(d)), float(cross), yaw, arc

    def acquire(self, position, yaw):
        options = []
        for key, link in self.links.items():
            dist, cross, tangent, arc = self.project(link['xy'], position)
            heading = abs(wrap(tangent-yaw))
            if dist <= 1.5 and heading < math.radians(55):
                options.append((dist+1.5*heading, key, arc))
        if not options:
            return False
        _, self.current, self.progress = min(options)
        return True

    def successor(self, key):
        link = self.links[key]
        p = link['xy']
        vin = p[-1]-p[max(0, len(p)-12)]
        angle = math.atan2(vin[1], vin[0])
        choices = []
        for nxt in self.outgoing.get(link['to_node_idx'], []):
            q = self.links[nxt]['xy']
            if np.linalg.norm(q[0]-p[-1]) > .75:
                continue
            vout = q[min(len(q)-1, 16)]-q[0]
            delta = wrap(math.atan2(vout[1], vout[0])-angle)
            if abs(delta) > math.radians(135):
                continue
            choices.append((delta, nxt))
        if not choices:
            return None
        if len(choices) > 1 and self.preference != 'straight':
            selected = [c for c in choices if c[0] > .3] if self.preference == 'left' else [c for c in choices if c[0] < -.3]
            if selected:
                return min(selected, key=lambda c: abs(abs(c[0])-math.pi/2))[1]
        return min(choices, key=lambda c: (abs(c[0]), c[1]))[1]

    def track(self, x, y, yaw, speed_mps, min_lookahead=2.5,
              max_cross_track=1.5):
        if not all(math.isfinite(v) for v in (x, y, yaw, speed_mps)):
            return None
        pos = np.array([x, y])
        if self.current is None and not self.acquire(pos, yaw):
            return None
        p = self.links[self.current]['xy']
        dist, cross, tangent, arc = self.project(p, pos)
        length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        if length-arc < 1.0:
            nxt = self.successor(self.current)
            if nxt:
                nd, nc, nt, na = self.project(self.links[nxt]['xy'], pos)
                # Change links only at the connected endpoint, never at crossings.
                if nd <= dist+.25:
                    self.current = nxt; p = self.links[nxt]['xy']
                    dist, cross, tangent, arc = nd, nc, nt, na
                    length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        if dist > max_cross_track or abs(wrap(tangent-yaw)) > math.radians(75):
            return None
        if arc < self.progress-2.0 and self.current == getattr(self, '_previous_link', None):
            return None
        self.progress = arc
        self._previous_link = self.current
        points = p.copy()
        key = self.current
        self.next_links = []
        # Enough connected geometry for lookahead and curve speed anticipation.
        for _ in range(8):
            key = self.successor(key)
            if key is None:
                break
            self.next_links.append(key)
            points = np.vstack((points, self.links[key]['xy'][1:]))
            if np.linalg.norm(np.diff(points, axis=0), axis=1).sum()-arc > 50:
                break
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.r_[0., np.cumsum(lengths)]
        remaining = float(cumulative[-1]-arc)
        lookahead = max(min_lookahead, min(5.0, min_lookahead+.6*speed_mps))
        target_arc = min(arc+lookahead, cumulative[-1])
        target = np.array([np.interp(target_arc, cumulative, points[:, k]) for k in (0, 1)])
        delta = target-pos
        forward = math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1]
        left = -math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]
        if forward <= .1:
            return None
        curvature = 2*left/max(.1, float(delta@delta))
        steering = math.atan(self.wheelbase*curvature)/self.max_wheel_angle
        return TrackingResult(float(np.clip(steering, -.85, .85)), curvature,
                              cross, wrap(tangent-yaw), remaining, self.current)
