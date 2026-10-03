"""Control math and shutdown lifecycle; no actuator publishers are created."""
from unittest.mock import Mock

import pytest

from edgenode_control import control_node as module


def test_small_error_is_not_clipped_to_zero():
    pid = module.PID(0.8, 0.0, 0.12, 0.5)
    for _ in range(10):
        steer = module.clamp(-pid.update(-0.056458566, 1 / 30), -0.7, 0.7)
        assert steer == pytest.approx(0.0451668528)


def test_pid_reset_removes_derivative_history():
    pid = module.PID(0.8, 0.0, 0.12, 0.5)
    pid.update(0.5, 1 / 30)
    pid.reset()
    assert pid.update(-0.1, 1 / 30) == pytest.approx(-0.08)


def test_shutdown_brakes_before_context_shutdown(monkeypatch):
    events = []
    node = Mock()
    node.publish_safe_stop.side_effect = lambda: events.append('brake')
    node.destroy_node.side_effect = lambda: events.append('destroy')
    monkeypatch.setattr(module, 'ControlNode', lambda: node)
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'spin', Mock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: events.append('shutdown'))
    module.main()
    assert events == ['brake', 'destroy', 'shutdown']
    assert module.rclpy.init.call_args.kwargs['signal_handler_options'] == module.SignalHandlerOptions.NO


def test_already_closed_context_does_not_publish(monkeypatch):
    node = Mock()
    monkeypatch.setattr(module, 'ControlNode', lambda: node)
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'spin', Mock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    module.main()
    node.publish_safe_stop.assert_not_called()
    node.destroy_node.assert_called_once()
