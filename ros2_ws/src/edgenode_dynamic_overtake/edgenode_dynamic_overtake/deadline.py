"""Start expensive validation only when original input deadlines have slack.

Deferral publishes no new path or speed and never renews any input clock.
The existing watchdog and Control expiry remain active during the idle gap.
"""
from collections import deque

class ValidationBudget:
    """Recent measured cost, expiring independently of successful execution.

    A slow earlier callback cannot make future validation permanently idle.
    This is scheduling only; all original deadline checks still grant or
    reject the resulting command using its unchanged source clocks.
    """
    def __init__(self,initial,minimum,window_s=2.):
        self.initial=initial;self.minimum=minimum;self.window_s=window_s;self.costs=deque(maxlen=8)
    def estimate(self,now):
        while self.costs and self.costs[0][0]<now-self.window_s:self.costs.popleft()
        return max(self.minimum,max((cost for _,cost in self.costs),default=self.initial)+.02)
    def record(self,now,cost):self.costs.append((now,cost))

def remaining_seconds(node,now_ns):
    guard=node.sensor_guard;status,_,receipt,steady=guard.matched
    deadlines=[status['source_stamp_ns']+guard.max_age_ns,status['frame_received_ns']+guard.timeout_ns,receipt+guard.timeout_ns,guard.status['published_ns']+guard.timeout_ns,node.latest_pose_stamp_ns+guard.max_age_ns]
    remaining=min((deadline-now_ns)/1e9 for deadline in deadlines)
    remaining=min(remaining,(steady+guard.timeout_ns-guard.steady_time_ns())/1e9)
    value=node.traffic;source=value.get('source_stamp_ns');coverage=node.coverage()
    if not value.get('ready') or type(source)is not int or not coverage.get('auxiliary_ready'):return None
    remaining=min(remaining,(source+guard.max_age_ns-now_ns)/1e9,(coverage['auxiliary_source_ns']+guard.max_age_ns-now_ns)/1e9,(coverage['evidence_published_ns']+guard.timeout_ns-now_ns)/1e9)
    for row in coverage.get('rear_sensors',[]):remaining=min(remaining,(row['ray'].source_ns+guard.max_age_ns-now_ns)/1e9)
    return remaining

def should_defer(node,now_ns,budget_s):
    # A fault/expired input must reach the unchanged stop logic immediately.
    if node.sensor_reason(now_ns) or node._sensor_pause is not None:return False,None
    remaining=remaining_seconds(node,now_ns)
    return remaining is not None and 0.<remaining<budget_s,remaining
