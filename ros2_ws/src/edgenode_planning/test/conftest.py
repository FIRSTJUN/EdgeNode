"""Small directed map shared by Route Editor tests; no real map files are changed."""

import json

import pytest


@pytest.fixture
def route_editor_map(tmp_path):
    nodes = [{'idx': name, 'point': [x, 0, 0]}
             for name, x in (('A', 0), ('B', 10), ('C', 20), ('D', 30), ('E', 50))]
    links = [
        {'idx': start + end, 'from_node_idx': start, 'to_node_idx': end,
         'points': [[x, 0, 0] for x in range(first, last + 1)], 'link_length': 10.0}
        for start, end, first, last in (('A', 'B', 0, 10), ('B', 'C', 10, 20), ('C', 'D', 20, 30))
    ]
    directory = tmp_path / 'mgeo'
    directory.mkdir()
    (directory / 'node_set.json').write_text(json.dumps(nodes))
    (directory / 'link_set.json').write_text(json.dumps(links))
    return directory
