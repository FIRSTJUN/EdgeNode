"""Verify startup geometry wiring without launching Control or MORAI."""

import importlib.util
import math
from pathlib import Path
from unittest.mock import Mock

from launch.actions import LogInfo
import pytest
import yaml

from edgenode_planning.lattice_planner import LatticePlanner


SRC = Path(__file__).resolve().parents[2]


def test_standalone_planner_geometry_and_cap_match_control_baseline():
    planner = yaml.safe_load((SRC / 'edgenode_planning/config/planning.yaml').read_text())[
        'planning_node']['ros__parameters']
    control = yaml.safe_load((SRC / 'edgenode_control/config/control.yaml').read_text())[
        'control_node']['ros__parameters']
    for name in ('wheelbase_m', 'max_wheel_angle_rad', 'max_front_steer_normalized'):
        assert planner['lattice_' + name] == control[name]
    limit = math.tan(control['max_front_steer_normalized'] * control['max_wheel_angle_rad'])
    limit /= control['wheelbase_m']
    assert LatticePlanner().usable_curvature_limit == pytest.approx(limit)
    assert planner['obstacle_avoid_speed_cap_kmh'] == 3.0
    assert planner['cruise_speed_kmh'] == 50.0
    assert planner['max_drive_speed_kmh'] == 50.0


def test_bringup_reads_geometry_from_same_control_configuration(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        's4_autonomy_launch', SRC / 'edgenode_bringup/launch/autonomy.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(SRC / package))
    node = Mock(side_effect=lambda **kwargs: LogInfo(msg='test only: no nodes started'))
    monkeypatch.setattr(module, 'Node', node)
    module.generate_launch_description()
    kwargs = [c.kwargs for c in node.call_args_list]
    control = next(k for k in kwargs if k['name'] == 'control_node')
    planning = next(k for k in kwargs if k['name'] == 'planning_node')
    geometry = yaml.safe_load(Path(control['parameters'][0]).read_text())[
        'control_node']['ros__parameters']
    assert planning['parameters'][1] == {
        'lattice_' + name: geometry[name]
        for name in ('wheelbase_m', 'max_wheel_angle_rad', 'max_front_steer_normalized')}
