#!/usr/bin/env python3
"""A experiment runner with bounded RPC acknowledgement logging.

Planning's default callback group also services parameters. Under recorded
load, a valid parameter request can be applied after the old observer's 4 s
wait. Wait up to 15 s for its acknowledgement so the observer does not exit
while an authorized arming request remains in flight. Sensor/Control source
freshness and all vehicle-side watchdog limits stay unchanged.
"""
import json,pathlib,runpy,sys,time
import rclpy
from morai_drive import Monitor


def acknowledged_call(self,client,request):
    if not client.wait_for_service(timeout_sec=3.):
        raise RuntimeError('ROS service unavailable: '+client.srv_name)
    sent=time.time_ns();began=time.monotonic();future=client.call_async(request)
    rclpy.spin_until_future_complete(self,future,timeout_sec=15.)
    done=future.done() and future.result() is not None
    output=pathlib.Path(sys.argv[sys.argv.index('--output')+1])
    with (output/'observer-rpc-acknowledgements.jsonl').open('a') as stream:
        stream.write(json.dumps(dict(service=client.srv_name,sent_utc_ns=sent,
            elapsed_s=time.monotonic()-began,acknowledged=done,observer_deadline_s=15.,
            sensor_freshness_limits_unchanged=True))+'\n')
    if not done:raise RuntimeError('ROS service acknowledgement timeout (15 s observer bound)')
    return future.result()

if __name__=='__main__':
    Monitor.call=acknowledged_call
    runpy.run_path(str(pathlib.Path(__file__).with_name('s4_morai_generalized_trial.py')),run_name='__main__')
