"""Original NGII boundary evidence for S4, without editing MGeo permissions."""
import json
from pathlib import Path

from .hdmap_audit import read_layer, translated, nearest
from .avoidance_geometry import intersects, polygon_bounds


class HDMapBoundaries:
    def __init__(self, directory, mgeo_directory):
        origin = json.loads((Path(mgeo_directory) / 'global_info.json').read_text())['local_origin_in_global']
        mgeo = {row['idx']: row for row in json.loads((Path(mgeo_directory) / 'link_set.json').read_text())}
        self.aligned_links = set()
        for feature in read_layer(Path(directory) / 'A2_LINK'):
            identifier = feature['attributes']['ID']
            if identifier not in mgeo or len(feature['parts']) != 1:
                continue
            raw = translated(feature, origin)[0]
            center = mgeo[identifier]['points']
            if (all(nearest(raw, p)[0] <= .05 for p in center)
                    and all(nearest(center, p)[0] <= .05 for p in raw)):
                self.aligned_links.add(identifier)
        self.shared = {}
        self.forbidden = []
        self.cells = {}
        # Only lane/road boundaries are used. Stop lines and guide lines are not
        # lateral restrictions. Mixed, unknown and special boundaries stay closed.
        lateral_kinds = {'501', '5011', '503', '504', '505', '506', '531', '535', '599'}
        for feature in read_layer(Path(directory) / 'B2_SURFACELINEMARK'):
            attr = feature['attributes']
            left, right = attr['L_linkID'], attr['R_linkID']
            allowed = attr['Type'] == '212' and attr['Kind'] == '503'
            if left and right:
                self.shared.setdefault(frozenset((left, right)), []).append((allowed, attr['ID']))
            if attr['Kind'] in lateral_kinds and not allowed:
                for part in translated(feature, origin):
                    for a, b in zip(part, part[1:]):
                        bounds = polygon_bounds((a, b))
                        identifier = len(self.forbidden)
                        self.forbidden.append((a[:2], b[:2], bounds, attr['ID']))
                        x0, x1, y0, y1 = bounds
                        for x in range(int(x0 // 10), int(x1 // 10) + 1):
                            for y in range(int(y0 // 10), int(y1 // 10) + 1):
                                self.cells.setdefault((x, y), set()).add(identifier)

    def permission_reason(self, original, adjacent):
        if original not in self.aligned_links or adjacent not in self.aligned_links:
            return 'HDMAP_CENTERLINE_MISMATCH'
        rows = self.shared.get(frozenset((original, adjacent)), ())
        if not rows:
            return 'SHARED_BOUNDARY_UNAVAILABLE'
        if not all(allowed for allowed, _ in rows):
            return 'LANE_BOUNDARY_FORBIDDEN'
        return ''

    def swept_reason(self, polygon):
        lo_x, hi_x, lo_y, hi_y = polygon_bounds(polygon)
        nearby = set()
        for x in range(int(lo_x // 10), int(hi_x // 10) + 1):
            for y in range(int(lo_y // 10), int(hi_y // 10) + 1):
                nearby.update(self.cells.get((x, y), ()))
        for index in nearby:
            a, b, (x0, x1, y0, y1), identifier = self.forbidden[index]
            if x1 < lo_x or hi_x < x0 or y1 < lo_y or hi_y < y0:
                continue
            if intersects(polygon, (a, b)):
                return 'LANE_BOUNDARY_COLLISION:' + identifier
        return ''
