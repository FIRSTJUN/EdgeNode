"""Load a pinned map-derived road artifact; no simulator actor data is accepted."""
import hashlib
import json
from pathlib import Path

from .road_surface import triangulate,cells_from_geometry
from .road_corridor import VerifiedRoadCorridor
from .avoidance_geometry import area

PHYSICAL_LOWER_TYPES=frozenset(('2','3','4','5','7','8','99'))


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_artifact(path,digest,network):
    if not path or not digest or file_sha(path)!=digest:raise ValueError('surveyed road artifact pin mismatch')
    data=json.loads(Path(path).read_text())
    version=data.get('schema')
    if version not in ('edgenode.surveyed-local-road.v1','edgenode.surveyed-local-road.v2') or data.get('actor_data_used') is not False:
        raise ValueError('invalid surveyed road provenance')
    for name,expected in data['map_files_sha256'].items():
        if file_sha(name)!=expected:raise ValueError('surveyed road input map changed: '+name)
    base=network.road_corridor
    if base is None:raise ValueError('surveyed roads require the original conservative road mask')
    from shapely.geometry import Polygon,LineString
    from shapely.ops import unary_union
    from .hdmap_audit import read_layer,translated
    boundaries=network.boundaries
    permitted=PHYSICAL_LOWER_TYPES if version.endswith('v2') else frozenset(('2','3','4'))
    raw={f['attributes']['ID']:translated(f,boundaries.origin)
        for f in read_layer(boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')
        if f['attributes']['Type'] in permitted and f['attributes']['LowHigh']=='2'}
    lines={k:[LineString([p[:2] for p in part]) for part in parts if len(part)>1] for k,parts in raw.items()}
    negatives=[]
    for island in data.get('negative_islands',[]):
        if version.endswith('v1'):raise ValueError('survey exclusions require v2 provenance')
        ids=island['source_ids']
        if not ids or any(k not in lines for k in ids):raise ValueError('unsurveyed exclusion boundary')
        shape=Polygon(island['ring'],island.get('holes',[]))
        if not shape.is_valid or shape.is_empty:raise ValueError('invalid survey exclusion')
        surveyed=unary_union([line for k in ids for line in lines[k]])
        if shape.boundary.difference(surveyed.buffer(1e-7)).length>1e-5:
            raise ValueError('exclusion perimeter not observed in physical C3')
        if island.get('driving_center_intersections')!=0:raise ValueError('unproved non-driving enclosure')
        if any(shape.relate_pattern(LineString([p[:2] for p in link['points']]),'T********')
               for key,link in network.links.items() if not link.get('lazy_init') and len(link['points'])>1):
            raise ValueError('survey exclusion contains a native driving center')
        negatives.append(shape)
    exclusion_union=unary_union(negatives)
    cells=[];surfaces=[]
    for cell,bounds in base.cells:
        shape=Polygon(cell)
        if negatives and shape.intersects(exclusion_union):
            # Exact clipping can produce a numerically collinear fragment.
            # Retain the corridor's existing nondegenerate-cell floor; the
            # exact source geometry below still preserves every map boundary.
            cells.extend(c for c in cells_from_geometry(shape.difference(exclusion_union)) if area(c)>1e-8)
        else:cells.append(cell)
    surfaces.append(base._exact_geometry if base._exact_geometry is not None else unary_union([Polygon(c) for c in cells]))
    for patch in data['patches']:
        if not patch['source_ids'] or any(k not in raw for k in patch['source_ids']):
            raise ValueError('unsurveyed road boundary source')
        shape=Polygon(patch['ring'],patch.get('holes',[]))
        if negatives:shape=shape.difference(exclusion_union)
        if not shape.is_valid or shape.is_empty:raise ValueError('invalid surveyed road ring')
        # A source face never approves a known additional barrier in its
        # interior. The caps remain those archived by the exact generator.
        if version.endswith('v2'):
            numerical_band=shape.boundary.buffer(1e-7)
            if any(shape.relate_pattern(line,'T********') and
                   shape.intersection(line).difference(numerical_band).length>1e-6
                   for row in lines.values() for line in row):raise ValueError('surveyed surface contains a physical barrier')
        elif any(k not in patch['source_ids'] and shape.relate_pattern(line,'T********')
                 for k,row in lines.items() for line in row):raise ValueError('surveyed strip contains another physical barrier')
        for feature in read_layer(boundaries.directory/'A3_DRIVEWAYSECTION'):
            if feature['attributes']['Kind']!='7':continue
            for part in translated(feature,boundaries.origin):
                exclusion=Polygon([p[:2] for p in part])
                if not exclusion.is_valid or shape.intersects(exclusion):raise ValueError('surveyed strip intersects exclusion')
        cells.extend(c for c in cells_from_geometry(shape) if area(c)>1e-8)
        surfaces.append(shape)
    corridor=VerifiedRoadCorridor(cells,source=base.source+';SURVEYED_C3_ARTIFACT:sha256:'+digest,
        link_ids=base.link_ids,allowed_sides=base.allowed_sides,exact_union_backend=True)
    # Preserve the exact surveyed polygons and holes, rather than introducing
    # triangulation-roundoff cracks between adjacent measured source caps.
    corridor._exact_geometry=unary_union(surfaces).difference(exclusion_union)
    if not corridor._exact_geometry.is_valid:raise ValueError('invalid surveyed road/exclusion union')
    corridor.provenance=tuple(getattr(base,'provenance',()))+('SURVEYED_C3_ARTIFACT:'+digest,)
    if version.endswith('v2'):boundaries.install_physical_segments(raw)
    return corridor
