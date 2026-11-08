"""Versioned LiDAR health snapshots; source stamps must use the ROS clock."""
import hashlib
import json
import math
import time
import uuid


def footprint_digest(frame_id, stamp_ns, points):
    payload = [frame_id, stamp_ns, [[float(x), float(y)] for x, y in points]]
    return hashlib.sha256(json.dumps(payload, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


class LidarStatus:
    def __init__(self, timeout_sec=.6, max_age_sec=.6, future_tolerance_sec=.05,
                 steady_time_ns=time.monotonic_ns):
        for value in (timeout_sec, max_age_sec, future_tolerance_sec):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('positive finite sensor time limits required')
        self.timeout_ns = int(timeout_sec * 1e9)
        self.max_age_ns = int(max_age_sec * 1e9)
        self.future_ns = int(future_tolerance_sec * 1e9)
        self.session_id = uuid.uuid4().hex
        self.status_seq = self.frame_seq = 0
        self.last_clock_ns = self.last_stamp_ns = self.receipt_ns = None
        self.clock_fault = False
        self.state = 'WAITING_FOR_INPUT'
        self.source_stamp_ns = 0
        self.cluster_count = 0
        self.digest = ''
        self.steady_time_ns = steady_time_ns
        self.frame_steady_ns = None
        self.fault_seq = 0
        self.last_fault = None

    def _clock(self, now_ns):
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.clock_fault = True
        self.last_clock_ns = now_ns

    def begin(self, stamp_ns, now_ns):
        self._clock(now_ns)
        self.frame_seq += 1
        self.source_stamp_ns = stamp_ns
        self.receipt_ns = now_ns
        self.frame_steady_ns = self.steady_time_ns()
        self.digest = ''
        self.cluster_count = 0
        if self.clock_fault:
            self.state = 'CLOCK_REGRESSION'
        elif stamp_ns <= 0:
            self.state = 'INVALID_STAMP'
        elif now_ns - stamp_ns > self.max_age_ns:
            self.state = 'STALE_INPUT'
        elif stamp_ns - now_ns > self.future_ns:
            self.state = 'FUTURE_STAMP'
        elif self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.state = 'DUPLICATE_FRAME' if stamp_ns == self.last_stamp_ns else 'OUT_OF_ORDER'
        else:
            self.last_stamp_ns = stamp_ns
            self.state = 'PROCESSING'
        return self.state == 'PROCESSING'

    def finish(self, count, digest, now_ns, failure=''):
        self._clock(now_ns)
        if self.clock_fault:
            self.state = 'CLOCK_REGRESSION'
        elif self.state == 'PROCESSING':
            self.state = failure or ('READY_OBSTACLES' if count else 'READY_EMPTY')
            self.cluster_count = count
            self.digest = digest if not failure else ''

    def snapshot(self, now_ns, *, localization_valid=True):
        self._clock(now_ns)
        state = self.state
        if self.clock_fault:
            state = 'CLOCK_REGRESSION'
        elif self.receipt_ns is not None and (now_ns - self.receipt_ns > self.timeout_ns or
                self.steady_time_ns() - self.frame_steady_ns > self.timeout_ns):
            state = 'INPUT_TIMEOUT'
        elif state.startswith('READY_') and now_ns - self.source_stamp_ns > self.max_age_ns:
            state = 'STALE_INPUT'
        elif state.startswith('READY_') and not localization_valid:
            state = 'LOCALIZATION_UNAVAILABLE'
        if state.startswith('READY_'):
            self.last_fault = None
        elif state != 'WAITING_FOR_INPUT' and self.last_fault != (self.frame_seq, state):
            self.fault_seq += 1
            self.last_fault = (self.frame_seq, state)
        self.status_seq += 1
        return dict(schema_version=1, clock='ros', session_id=self.session_id,
                    status_seq=self.status_seq, frame_seq=self.frame_seq,
                    fault_seq=self.fault_seq, state=state,
                    source_stamp_ns=self.source_stamp_ns,
                    frame_received_ns=self.receipt_ns if self.receipt_ns is not None else 0,
                    published_ns=now_ns, cluster_count=self.cluster_count,
                    obstacle_digest=self.digest)
