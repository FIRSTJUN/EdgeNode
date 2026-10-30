"""Draft, snap, directed preview and atomic persistence tests without ROS."""

import math
from unittest.mock import Mock

import pytest
import yaml

from edgenode_planning.route_editor import RouteEditor, RouteEditorError


@pytest.fixture
def editor(route_editor_map):
    return RouteEditor(route_editor_map)


def test_nearest_node_and_snap_diagnostics(editor):
    result = editor.append_click(19, 2)
    assert result['node_id'] == 'C'
    assert result['point'] == (20, 0, 0)
    assert result['distance_m'] == pytest.approx(math.sqrt(5))


def test_snap_limit_inclusive_and_rejection_does_not_edit(editor):
    editor.append_click(20, 5)
    with pytest.raises(RouteEditorError, match='SNAP_TOO_FAR'):
        editor.append_click(30, 5.01)
    assert editor.waypoint_node_ids == ['C']


@pytest.mark.parametrize('x,y,limit', [(math.nan, 0, 5), (0, math.inf, 5), (0, 0, -1),
                                      (True, 0, 5), (0, 0, math.nan), ('bad', 0, 5)])
def test_invalid_snap_values_fail_closed(editor, x, y, limit):
    with pytest.raises(RouteEditorError, match='INVALID_SNAP_INPUT'):
        editor.append_click(x, y, limit)
    assert editor.waypoint_node_ids == []


def test_duplicate_rejected_order_and_nonconsecutive_revisits_preserved(editor):
    editor.append_click(20, 0)
    with pytest.raises(RouteEditorError, match='DUPLICATE_WAYPOINT'):
        editor.append_click(20.1, 0)
    editor.append_click(30, 0)
    editor.append_click(20, 0)
    assert editor.waypoint_node_ids == ['C', 'D', 'C']
    returned = editor.waypoint_node_ids
    returned.clear()
    assert editor.waypoint_node_ids == ['C', 'D', 'C']


def test_undo_and_clear_update_preview(editor):
    editor.set_start_link('AB')
    editor.append_click(20, 0)
    editor.append_click(30, 0)
    assert editor.undo() == 'D'
    assert editor.preview['goal_node_id'] == 'C'
    editor.clear()
    assert editor.waypoint_node_ids == []
    assert editor.preview is None
    with pytest.raises(RouteEditorError, match='EMPTY_WAYPOINTS'):
        editor.undo()


def test_single_and_multiple_waypoint_preview_reuse_existing_planner(editor):
    editor.planner.plan_route = Mock(wraps=editor.planner.plan_route)
    editor.set_start_link('AB')
    editor.append_click(20, 0)
    assert editor.preview['link_ids'] == ['AB', 'BC']
    editor.append_click(30, 0)
    assert editor.preview['link_ids'] == ['AB', 'BC', 'CD']
    assert editor.preview['node_ids'] == ['A', 'B', 'C', 'D']
    assert editor.preview['total_cost_m'] == 30
    editor.planner.plan_route.assert_called_with('AB', ['C', 'D'])


def test_unreachable_waypoint_removes_old_preview_and_refuses_activation(editor):
    editor.set_start_link('AB')
    editor.append_click(20, 0)
    editor.append_click(50, 0)
    assert editor.preview is None
    assert editor.preview_status == 'ROUTE_NOT_FOUND'
    with pytest.raises(RouteEditorError, match='ROUTE_NOT_FOUND'):
        editor.activation_node_ids()


def test_missing_changed_and_lost_start_link(editor):
    editor.append_click(30, 0)
    assert editor.preview is None
    assert editor.preview_status == 'WAITING_FOR_CURRENT_LINK'
    assert editor.set_start_link('AB')
    assert editor.preview['link_ids'] == ['AB', 'BC', 'CD']
    revision = editor.revision
    assert not editor.set_start_link('AB')
    assert editor.revision == revision
    assert editor.set_start_link('BC')
    assert editor.preview['link_ids'] == ['BC', 'CD']
    assert editor.set_start_link('unknown')
    assert editor.preview is None


def test_activation_requires_nonempty_draft_and_current_link(editor):
    with pytest.raises(RouteEditorError, match='EMPTY_WAYPOINTS'):
        editor.activation_node_ids()
    editor.append_click(20, 0)
    with pytest.raises(RouteEditorError, match='WAITING_FOR_CURRENT_LINK'):
        editor.activation_node_ids()
    editor.set_start_link('AB')
    assert editor.activation_node_ids() == ['C']


def test_save_load_roundtrip_preserves_order_and_metadata(editor, tmp_path):
    editor.set_start_link('AB')
    editor.append_click(20, 0)
    editor.append_click(30, 0)
    saved = editor.save(tmp_path / 'routes', 'demo')
    data = yaml.safe_load(saved.read_text())
    assert data['frame_id'] == 'map'
    assert data['created_from']['mgeo_dir'] == editor.mgeo_dir
    assert data['waypoint_node_ids'] == ['C', 'D']
    editor.clear()
    editor.load(saved.parent, 'demo')
    assert editor.waypoint_node_ids == ['C', 'D']
    assert editor.preview['goal_node_id'] == 'D'
    assert list(saved.parent.glob('*.tmp')) == []


def test_load_without_start_link_restores_draft_only(editor, tmp_path):
    editor.append_click(20, 0)
    editor.save(tmp_path, 'demo')
    editor.clear()
    editor.load(tmp_path, 'demo')
    assert editor.waypoint_node_ids == ['C']
    assert editor.preview_status == 'WAITING_FOR_CURRENT_LINK'


@pytest.mark.parametrize('contents', [
    'frame_id: map\nwaypoint_node_ids: [missing]',
    'frame_id: odom\nwaypoint_node_ids: [C]',
    'frame_id: map\nwaypoint_node_ids: []',
    'frame_id: map\nwaypoint_node_ids: C',
    'frame_id: map\nwaypoint_node_ids: [C]\ncreated_from: {mgeo_dir: /another/map}',
    'frame_id: map\nwaypoint_node_ids: [C]\ncreated_from: bad',
    '[broken', 'null', '!!python/object:bad {}',
])
def test_bad_load_preserves_previous_draft_and_preview(editor, tmp_path, contents):
    editor.set_start_link('AB')
    editor.append_click(20, 0)
    previous = editor.preview
    (tmp_path / 'bad.yaml').write_text(contents)
    with pytest.raises(RouteEditorError):
        editor.load(tmp_path, 'bad', start_link_id='BC')
    assert editor.waypoint_node_ids == ['C']
    assert editor.start_link_id == 'AB'
    assert editor.preview is previous


@pytest.mark.parametrize('name', ['../escape', '/tmp/absolute', '', '.', 'a/b', 'a.b', 'x' * 81])
def test_route_names_cannot_escape_save_directory(editor, tmp_path, name):
    editor.append_click(20, 0)
    with pytest.raises(RouteEditorError, match='INVALID_ROUTE_NAME'):
        editor.save(tmp_path, name)


def test_load_preserves_revisits_but_normalizes_boundary_duplicates(editor, tmp_path):
    (tmp_path / 'demo.yaml').write_text('frame_id: map\nwaypoint_node_ids: [C, C, D, C]')
    editor.load(tmp_path, 'demo')
    assert editor.waypoint_node_ids == ['C', 'D', 'C']


def test_missing_map_and_missing_route_fail_safely(tmp_path):
    editor = RouteEditor(tmp_path / 'missing')
    with pytest.raises(RouteEditorError, match='MAP_UNAVAILABLE'):
        editor.append_click(0, 0)
    with pytest.raises(OSError):
        editor.load(tmp_path, 'missing')


@pytest.mark.parametrize('start_link,expected_start', [('BC', 'BC'), ('', ''), ('missing', '')])
def test_load_captures_new_start_before_calculating_preview(editor, tmp_path, start_link, expected_start):
    editor.set_start_link('AB')
    editor.append_click(20, 0)
    editor.append_click(30, 0)
    editor.save(tmp_path, 'demo')
    editor.planner.plan_route = Mock(wraps=editor.planner.plan_route)
    editor.load(tmp_path, 'demo', start_link_id=start_link)
    assert editor.start_link_id == expected_start
    assert editor.waypoint_node_ids == ['C', 'D']
    if expected_start:
        editor.planner.plan_route.assert_called_once_with('BC', ['C', 'D'])
        assert editor.preview['link_ids'] == ['BC', 'CD']
        assert editor.preview['total_cost_m'] == 20.0
    else:
        editor.planner.plan_route.assert_not_called()
        assert editor.preview is None
        assert editor.preview_status == 'WAITING_FOR_CURRENT_LINK'
