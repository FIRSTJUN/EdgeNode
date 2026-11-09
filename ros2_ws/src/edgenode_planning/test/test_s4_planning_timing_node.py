"""Actual disarmed Planning callback; real elapsed freshness, no motion output."""
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import pytest
from rclpy.time import Time
from std_msgs.msg import String

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'tools'))
from s4_timing_audit import NodeCase, load, make_case


@pytest.fixture
def adapter(tmp_path):
    root=Path(__file__).resolve().parents[4]
    loop,modules=load(root)
    fixture=make_case(loop,modules,dict(name='node_timing',radius=0.))
    value=NodeCase(fixture,modules,tmp_path/'synthetic_map')
    try:yield value
    finally:value.close()


@pytest.mark.parametrize('paused_ros',[False,True])
def test_fresh_start_can_never_publish_motion_after_real_expiry(adapter,paused_ros):
    node=adapter.node
    context=patch.object(node,'get_clock') if paused_ros else None
    if context:
        clock=context.start();constant=node._clock.now().nanoseconds
        from types import SimpleNamespace
        clock.return_value=SimpleNamespace(now=lambda:Time(nanoseconds=constant))
    try:
        adapter.feed(.01)
        assert node.sensor_reason(node.get_clock().now().nanoseconds)==''
        original=node.speed_planner.plan
        def slow(*args,**kwargs):
            result=original(*args,**kwargs);time.sleep(.65);return result
        with patch.object(node.speed_planner,'plan',slow):node.plan()
        assert node.avoidance_planner.state=='BLOCKED'
        assert node.avoidance_planner.reason.startswith('S4_SENSOR_')
        assert not node.local_path_pub.last.poses and node.target_speed_pub.last.data==0.
        assert node.get_parameter('enable_drive').value is False
    finally:
        if context:context.stop()


@pytest.mark.parametrize('event',['sensor_error','restart','recovered_fault'])
def test_delivered_error_during_callback_is_rechecked_and_latched(adapter,event):
    node=adapter.node;adapter.feed(.01);original=node.speed_planner.plan
    def injected(*args,**kwargs):
        result=original(*args,**kwargs)
        status=dict(node.sensor_guard.status);status['status_seq']+=1
        status['published_ns']=node.get_clock().now().nanoseconds
        if event=='sensor_error':status.update(state='PROCESSING_ERROR',fault_seq=1)
        elif event=='restart':status.update(session_id='d'*32,status_seq=1)
        else:status['fault_seq']=1
        node.sensor_status_callback(String(data=json.dumps(status)))
        return result
    with patch.object(node.speed_planner,'plan',injected):node.plan()
    assert node.avoidance_planner.state=='BLOCKED'
    assert not node.local_path_pub.last.poses and node.target_speed_pub.last.data==0.
