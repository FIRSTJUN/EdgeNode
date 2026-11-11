"""Delivery diagnostics must preserve gaps, defects, and the approval boundary."""
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import zipfile

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'tools'))
from kcity_full_map_audit import (adjacent_chains, load_layers, member_name,
    coordinate_comparison,read_archive,read_dbf,read_shapes,read_workbook,scan_candidates,validate_output)


def headers(kind, body):
    header=bytearray(100);struct.pack_into('>i',header,0,9994)
    struct.pack_into('>i',header,24,(108+len(body))//2)
    struct.pack_into('<ii',header,28,1000,kind)
    shp=bytes(header)+struct.pack('>ii',1,len(body)//2)+body
    shx=bytearray(header);struct.pack_into('>i',shx,24,54)
    return shp,bytes(shx)+struct.pack('>ii',50,len(body)//2)


def dbf():
    data=bytearray(65+13);data[0]=3
    struct.pack_into('<IHH',data,4,1,65,13)
    data[32:34]=b'ID';data[43]=ord('C');data[48]=12;data[64]=13
    data[65:]=b' '+b'FEATURE00001'
    return bytes(data)


def test_archive_korean_legacy_name_and_path_traversal_are_distinguished():
    text='SEC01_첨단주행시험로/HDMap_UTM52N_타원체고/A2_LINK.shp'
    legacy=text.encode('cp949').decode('cp437')
    assert member_name(SimpleNamespace(filename=legacy,flag_bits=0))==text
    with pytest.raises(ValueError,match='unsafe'):
        member_name(SimpleNamespace(filename='../source.shp',flag_bits=0x800))


def test_duplicate_archive_members_are_never_silently_overwritten(tmp_path):
    path=tmp_path/'duplicate.zip'
    with zipfile.ZipFile(path,'w') as z:
        z.writestr('same.txt','first')
        with pytest.warns(UserWarning):z.writestr('same.txt','second')
    with pytest.raises(ValueError,match='duplicate'):read_archive(path)


def test_dbf_deleted_rows_and_bad_layout_are_not_removed_to_force_counts():
    raw=dbf();fields,rows=read_dbf(raw)
    assert fields[0]['name']=='ID' and rows[0]['ID']=='FEATURE00001'
    deleted=bytearray(raw);deleted[65]=42
    with pytest.raises(ValueError,match='deleted'):read_dbf(deleted)
    with pytest.raises(ValueError,match='layout'):read_dbf(raw[:-1])


def test_shp_shx_offsets_are_validated_independently():
    shp,shx=headers(11,struct.pack('<i3d',11,1.,2.,3.))
    assert read_shapes(shp,shx)[0]['parts'][0][0]==(1.,2.,3.)
    bad=bytearray(shx);struct.pack_into('>i',bad,100,51)
    with pytest.raises(ValueError,match='offset'):read_shapes(shp,bad)


def test_forensic_trailer_read_retains_strict_failure_without_source_repair():
    shp,shx=headers(11,struct.pack('<i3d',11,1.,2.,3.));damaged=shp+b'UNINDEXED!'
    with pytest.raises(ValueError,match='header'):read_shapes(damaged,shx)
    members={'HDMap/test.shp':damaged,'HDMap/test.shx':shx,'HDMap/test.dbf':dbf()}
    layer=load_layers(members)['HDMap/test']
    assert layer['features'][0]['parts'][0][0]==(1.,2.,3.)
    assert layer['source_format_issues'][0]['code']=='SHP_HEADER_LENGTH_MISMATCH'
    assert layer['source_format_issues'][0]['unindexed_trailing_bytes']==10
    assert members['HDMap/test.shp']==damaged


def test_missing_control_point_crs_remains_missing():
    shp,shx=headers(11,struct.pack('<i3d',11,1.,2.,3.))
    layer=load_layers({'ControlPoint/point.shp':shp,'ControlPoint/point.shx':shx,
                       'ControlPoint/point.dbf':dbf()})['ControlPoint/point']
    assert layer['wkt']==''


@pytest.mark.parametrize('offset',[0.,1.])
def test_vertex_count_difference_is_compared_without_claiming_geometry_identity(offset):
    from pyproj import CRS
    def layer(points):
        return dict(wkt=CRS.from_epsg(32652).to_wkt(),features=[dict(attributes={'ID':'curb'},parts=[points])])
    a={'C3':layer([(300000.,4100000.,30.),(300010.,4100000.,30.)])}
    b={'C3':layer([(300000.,4100000.,30.),(300005.,4100000.+offset,30.),(300010.,4100000.,30.)])}
    row=coordinate_comparison(a,{'variant':b})[0]
    assert row['identical_ids'] and row['part_layout_mismatch_ids']==['curb']
    assert row['part_layout_mismatch_diagnostics'][0]['vertex_to_segment_xy_max_m']==pytest.approx(offset)


def test_workbook_reader_keeps_all_sheets_hidden_cells_and_cached_formulas():
    data=io.BytesIO()
    with zipfile.ZipFile(data,'w') as book:
        book.writestr('xl/workbook.xml','''<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"
          xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>
          <sheet name="도화데이터" sheetId="1" r:id="r1" state="hidden"/></sheets></workbook>''')
        book.writestr('xl/_rels/workbook.xml.rels','''<Relationships><Relationship Id="r1" Target="worksheets/sheet1.xml"/></Relationships>''')
        book.writestr('xl/sharedStrings.xml','''<sst xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><si><t>705</t></si></sst>''')
        book.writestr('xl/worksheets/sheet1.xml','''<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData><row r="4">
          <c r="CF4" t="s"><v>0</v></c><c r="CG4"><f>1+2</f><v>3</v></c></row></sheetData></worksheet>''')
    sheets=read_workbook(data.getvalue())
    assert sheets[0]['state']=='hidden'
    assert sheets[0]['cells']==[dict(address='CF4',value='705',formula=None),dict(address='CG4',value='3',formula='1+2')]


def link(name,start,end,y,from_node,to_node,left='',right=''):
    return dict(shape_type=13,parts=[((start,y,0.),(end,y,0.))],attributes=dict(
        ID=name,FromNodeID=from_node,ToNodeID=to_node,LinkType='6',L_LinkID=left,R_LinkID=right))


def paired_sections(gap=0.):
    return [link('a0',0.,40.,0.,'na0','na1',left='b0'),
            link('b0',0.,40.,3.5,'nb0','nb1',right='a0'),
            link('a1',40.+gap,100.,0.,'na1','na2',left='b1'),
            link('b1',40.+gap,100.,3.5,'nb1','nb2',right='a1')]


def test_real_endpoints_join_only_when_the_actual_geometry_is_exact():
    chains,meta=adjacent_chains(paired_sections())
    assert len(chains)==1 and len(chains[0])==2
    chains,meta=adjacent_chains(paired_sections(.001))
    assert len(chains)==2 and all(len(c)==1 for c in chains)
    assert meta['endpoint_gap_transitions'] and meta['join_tolerance_m']==0.


def test_topology_and_nearby_parallel_lines_do_not_create_undeclared_neighbors():
    features=paired_sections()
    features[1]['attributes']['R_LinkID']=''
    chains,meta=adjacent_chains(features)
    assert all('a0' not in (a,b) for c in chains for a,b,_ in c)


def marking(name,y,left='',right='',parts=None):
    return dict(shape_type=13,parts=parts or [((0.,y,0.),(100.,y,0.))],
                attributes=dict(ID=name,Type='212',Kind='503',L_linkID=left,R_linkID=right))


@pytest.mark.parametrize('split',[False,True])
def test_complete_observed_lane_markings_never_authorize_a_surface_or_crossing(split,tmp_path):
    lanes=[link('a',0.,100.,0.,'n0','n1',left='b'),link('b',0.,100.,3.5,'m0','m1',right='a')]
    parts=[((0.,1.75,0.),(49.,1.75,0.)),((51.,1.75,0.),(100.,1.75,0.))] if split else None
    marks=[marking('divider',1.75,left='b',right='a',parts=parts),
           marking('outer_left',5.25,right='b'),marking('outer_right',-1.75,left='a')]
    rows,top,profiles,meta,assumptions=scan_candidates({'A2_LINK':{'features':lanes},
        'B2_SURFACELINEMARK':{'features':marks}},{},(0.,0.,0.))
    assert len(rows)==1 and rows[0]['length_sufficient']
    assert rows[0]['classification']=='B' and not rows[0]['approved_for_runtime']
    assert 'NO_EXPLICIT_OUTWARD_AND_RETURN_PERMISSION' in rows[0]['reasons']
    if split:
        assert rows[0]['max_sampled_support_span_m']<assumptions['required_actual_map_support_m']
        assert len(rows[0]['sampled_support_spans_m'])==2
    else:assert rows[0]['sampled_support_ratio']==1.
    path=tmp_path/'diagnostic.json';path.write_text(json.dumps(rows[0]))
    assert 'cells' not in rows[0]


def test_outputs_cannot_write_into_source_or_symlinked_source(tmp_path):
    source=tmp_path/'map';source.mkdir();alias=tmp_path/'alias';alias.symlink_to(source)
    with pytest.raises(ValueError,match='outside'):validate_output(alias/'audit',[source])


def test_shared_marking_id_is_insufficient_when_geometry_is_outside_both_lanes():
    lanes=[link('a',0.,100.,0.,'n0','n1',left='b'),link('b',0.,100.,3.5,'m0','m1',right='a')]
    marks=[marking('wrong_divider',6.,left='b',right='a'),
           marking('outer_left',5.25,right='b'),marking('outer_right',-1.75,left='a')]
    rows,_,_,_,_=scan_candidates({'A2_LINK':{'features':lanes},
        'B2_SURFACELINEMARK':{'features':marks}},{},(0.,0.,0.))
    assert rows[0]['sampled_support_ratio']==0.
    assert rows[0]['max_sampled_support_span_m']==0.
