"""Original NGII boundary evidence for S4, without editing MGeo permissions."""
import json
import hashlib
from pathlib import Path

from .hdmap_audit import read_layer, translated, nearest
from .avoidance_geometry import intersects, polygon_bounds


class HDMapBoundaries:
    def __init__(self, directory, mgeo_directory):
        origin = json.loads((Path(mgeo_directory) / 'global_info.json').read_text())['local_origin_in_global']
        self.directory=Path(directory);self.origin=origin
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
        self.observed_sides = {}
        self._road_corridor_cache = {}
        self.forbidden = []
        self.cells = {}
        # Only lane/road boundaries are used. Stop lines and guide lines are not
        # lateral restrictions. Mixed, unknown and special boundaries stay closed.
        lateral_kinds = {'501', '5011', '503', '504', '505', '506', '531', '535', '599'}
        for feature in read_layer(Path(directory) / 'B2_SURFACELINEMARK'):
            attr = feature['attributes']
            left, right = attr['L_linkID'], attr['R_linkID']
            for link,side in ((left,'right'),(right,'left')):
                if link:self.observed_sides.setdefault((link,side),[]).append(
                    (attr,translated(feature,origin)))
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

    def install_physical_segments(self, sources):
        """Keep physical C3 separate from unioned road-surface permission.

        Two proved surfaces can meet on opposite sides of the same curb.
        Their union must never remove the curb's whole-body collision gate.
        The v2 pinned survey loader supplies unchanged physical source lines.
        """
        from shapely.geometry import LineString
        from shapely.ops import unary_union
        self._physical_lines=tuple((key,LineString([p[:2] for p in part]))
            for key,parts in sources.items() for part in parts if len(part)>1)
        self._physical_geometry=unary_union([line for key,line in self._physical_lines])
        for name in ('_physical_prepared','_physical_tree','_physical_by_geometry'):
            self.__dict__.pop(name,None)

    def __getstate__(self):
        state=dict(self.__dict__)
        for name in ('_physical_prepared','_physical_tree','_physical_by_geometry'):state.pop(name,None)
        return state

    def swept_reason(self, polygon):
        if hasattr(self,'_physical_geometry'):
            from shapely.geometry import Polygon
            from shapely.prepared import prep
            shape=Polygon(polygon)
            if not shape.is_valid:return 'INVALID_PHYSICAL_BOUNDARY_SWEEP'
            if not hasattr(self,'_physical_prepared'):self._physical_prepared=prep(self._physical_geometry)
            if self._physical_prepared.intersects(shape):
                # Common clear sweeps finish in one exact native query. Only
                # a real contact needs the source identifier lookup.
                if not hasattr(self,'_physical_tree'):
                    from shapely.strtree import STRtree
                    self._physical_tree=STRtree([g for key,g in self._physical_lines])
                    self._physical_by_geometry={id(g):key for key,g in self._physical_lines}
                for candidate in self._physical_tree.query(shape):
                    if hasattr(candidate,'geom_type'):edge=candidate;identifier=self._physical_by_geometry[id(edge)]
                    else:identifier,edge=self._physical_lines[int(candidate)]
                    if shape.intersects(edge):return 'PHYSICAL_BOUNDARY_COLLISION:'+identifier
                return 'PHYSICAL_BOUNDARY_COLLISION:SOURCE_LOOKUP_UNAVAILABLE'
        lo_x, hi_x, lo_y, hi_y = polygon_bounds(polygon)
        nearby = set()
        for x in range(int(lo_x // 10), int(hi_x // 10) + 1):
            for y in range(int(lo_y // 10), int(hi_y // 10) + 1):
                nearby.update(self.cells.get((x, y), ()))
        for index in nearby:
            # Do not filter by L/R_linkID: a center divider may be recorded
            # only against the opposite-direction link and still touch Ego.
            a, b, (x0, x1, y0, y1), identifier = self.forbidden[index]
            if x1 < lo_x or hi_x < x0 or y1 < lo_y or hi_y < y0:
                continue
            if intersects(polygon, (a, b)):
                return 'LANE_BOUNDARY_COLLISION:' + identifier
        return ''

    def road_corridor_for_pair(self, network, pair):
        """Bound ONLY ordinary, bidirectionally permitted lane-pair strips.

        Both outer sides must be actual unambiguous B2 observations. Exact
        piecewise-linear strips are triangulated, never a centerline buffer or
        a convex hull across a junction. Unobserved caps/gaps remain outside.
        Internal regulatory markings still use swept_reason() on every body.
        """
        from .hdmap_audit import marking_definition
        from .lane_geometry import LanePair,PolylineIndex,arc
        from .road_corridor import VerifiedRoadCorridor
        reverse='right' if pair.side=='left' else 'left'
        if any(network.permission_reason(a,pair.side) or network.permission_reason(b,reverse)
               for a,b in pair.ids):return None
        key=(tuple(pair.ids),pair.side)
        if key in self._road_corridor_cache:return self._road_corridor_cache[key]
        cells=[];links=[];sources=set()
        for original,adjacent in pair.ids:
            outer_left=adjacent if pair.side=='left' else original
            outer_right=original if pair.side=='left' else adjacent
            lines=[];identifiers=[]
            for link,side in ((outer_right,'right'),(outer_left,'left')):
                rows=self.observed_sides.get((link,side),())
                if len(rows)!=1:break
                attr,parts=rows[0]
                if (len(parts)!=1 or attr['Kind'] not in ('501','503','505','506')
                        or attr['Type']=='999' or not marking_definition(attr['Type'],attr['Kind'])['definition_confirmed']):break
                line=parts[0];center=PolylineIndex(network.links[link]['points'])
                if center.project(line[-1])<center.project(line[0]):line=tuple(reversed(line))
                lines.append(line);identifiers.append(attr['ID'])
            if len(lines)!=2:continue
            right,left=lines;index=PolylineIndex(right)
            # Trim strictly inside actual measured caps; never extrapolate a
            # side boundary to the MGeo endpoint or fill the next section gap.
            start=max(0.,index.project(left[0]))+.25
            end=min(arc(right)[-1],index.project(left[-1]))-.25
            if end<=start:continue
            observed=LanePair(right,left,[(outer_right,outer_left)],'left',[arc(right)[-1]])
            envelope=observed.corridor_envelope(start,end)
            if envelope['failure'] or not envelope['quads']:continue
            cells.extend(envelope['quads']);links.extend((original,adjacent));sources.update(identifiers)
        result=(VerifiedRoadCorridor(cells,source='HDMAP_B2_ORDINARY_LANE_STRIPS:'+','.join(sorted(sources)),
                    link_ids=links,allowed_sides=(pair.side,)) if cells else None)
        if result is not None:
            result.crossings=tuple(row for a,b in pair.ids for row in
                ((a,pair.side,b),(b,reverse,a)))
        self._road_corridor_cache[key]=result
        return result

    def road_surface_corridor(self,network):
        """MORAI experiment: actual A3 surfaces plus proved ordinary strips.

        NGII 2019 p61-64 identifies Kind 1 as a driving section. Only supplied
        single rings are triangulated; absent areas, multipart rings and caps
        are never filled. This is local surface evidence, not a whole-map mask.
        B2 forbidden boundaries and current LiDAR solids remain separate gates.
        """
        from .road_surface import triangulate
        from .avoidance_geometry import area
        from .road_corridor import VerifiedRoadCorridor
        cells=[];sources=[]
        sections=read_layer(self.directory/'A3_DRIVEWAYSECTION')
        exclusions=[tuple(p[:2] for p in part) for f in sections if f['attributes']['Kind']=='7'
                    for part in translated(f,self.origin)]
        for feature in sections:
            attr=feature['attributes']
            if attr['Kind']!='1' or attr['RoadType']!='1' or len(feature['parts'])!=1:continue
            ring=translated(feature,self.origin)[0]
            try:triangles=triangulate(ring)
            except ValueError:continue
            cells.extend(triangles);sources.append(attr['ID'])
        # Preserve the exact observed lane strips before/after a junction.
        for key in sorted(network.valid & self.aligned_links):
            if network.links[key].get('link_type') not in ('6',6):continue
            for side in ('left','right'):
                pair=network.pair(key,side,[key])
                if pair is None:continue
                corridor=self.road_corridor_for_pair(network,pair)
                if corridor is not None:
                    cells.extend(cell for cell,bounds in corridor.cells)
                    sources.append(corridor.source)
        # A single ordinary lane needs no adjacent-lane crossing permission
        # to prove its own interior. Keep both surveyed sides and their actual
        # end caps; the exact union does not extend them across a junction.
        for key,ring,identifiers in self.measured_lane_rings(network):
            try:triangles=triangulate(ring)
            except ValueError:continue
            cells.extend(triangles)
            sources.append('HDMAP_B2_NATIVE_LANE:'+key+':'+','.join(identifiers))
        # Omit sub-tolerance slivers conservatively; never replace them by a
        # convex patch. The same lower area floor is enforced by the corridor.
        cells=[cell for cell in cells if area(cell)>1e-8 and not any(intersects(cell,excluded) for excluded in exclusions)]
        if not cells:return None
        provenance=tuple(sorted(set(sources)))
        digest=hashlib.sha256(json.dumps(provenance).encode()).hexdigest()
        result=VerifiedRoadCorridor(cells,source='MORAI_HDMap_SURFACES:sha256:'+digest,
            link_ids=network.valid & self.aligned_links,allowed_sides=('left','right'),exact_union_backend=True)
        result.provenance=provenance
        return result

    def measured_lane_rings(self,network):
        """Exact ordinary-lane interiors, without inferring road width.

        This is geometric evidence only. A solid boundary, explicit crossing
        denial or exclusion still rejects the maneuver independently.
        Ambiguous, reversed, missing or self-intersecting sides prove nothing.
        """
        from .hdmap_audit import marking_definition
        from .lane_geometry import PolylineIndex
        from shapely.geometry import Polygon,Point
        for key in sorted(network.valid & self.aligned_links):
            link=network.links[key]
            if link.get('link_type') not in ('6',6):continue
            index=PolylineIndex(link['points']);sides=[]
            for side in ('right','left'):
                options=[]
                for attributes,parts in self.observed_sides.get((key,side),()):
                    if (len(parts)!=1 or attributes['Kind'] not in ('501','503','505','506')
                            or attributes['Type']=='999'
                            or not marking_definition(attributes['Type'],attributes['Kind'])['definition_confirmed']):continue
                    line=parts[0]
                    if index.project(line[-1])<index.project(line[0]):line=tuple(reversed(line))
                    mismatch=abs(index.project(line[0]))+abs(index.project(line[-1])-index.lengths[-1])
                    options.append((mismatch,attributes['ID'],line))
                options.sort(key=lambda row:(row[0],row[1]))
                if (not options or options[0][0]>2. or
                        len(options)>1 and abs(options[0][0]-options[1][0])<=1e-9):break
                sides.append(options[0])
            if len(sides)!=2:continue
            right,left=sides
            ring=[]
            for p in list(right[2])+list(reversed(left[2])):
                xy=tuple(p[:2])
                if not ring or xy!=ring[-1]:ring.append(xy)
            if ring and ring[0]==ring[-1]:ring.pop()
            polygon=Polygon(ring)
            if (polygon.is_empty or not polygon.is_valid or
                    not polygon.covers(Point(link['points'][len(link['points'])//2][:2]))):continue
            yield key,tuple(ring),(right[1],left[1])
