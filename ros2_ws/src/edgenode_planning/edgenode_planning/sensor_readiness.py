"""Strict S4 sensor admission; independent of legacy obstacle-free following."""
import hashlib
import json
import math
import time


READY = ('READY_EMPTY', 'READY_OBSTACLES')
STATES = READY + ('WAITING_FOR_INPUT', 'INVALID_INPUT', 'PROCESSING_ERROR',
                 'LOCALIZATION_UNAVAILABLE', 'INPUT_TIMEOUT', 'STALE_INPUT',
                 'INVALID_STAMP', 'FUTURE_STAMP', 'DUPLICATE_FRAME',
                 'OUT_OF_ORDER', 'CLOCK_REGRESSION', 'PROCESSING')


def footprint_digest(frame_id, stamp_ns, points):
    payload = [frame_id, stamp_ns, [[float(x), float(y)] for x, y in points]]
    return hashlib.sha256(json.dumps(payload, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


class SensorReadiness:
    """Pair separate DDS streams without assuming cross-topic arrival order.

    A heartbeat never renews the age of a frame. A new good frame may restore
    readiness, but the owner alone controls the latched avoidance STOP/reset.
    """
    def __init__(self, timeout_sec=.6, max_age_sec=.6, future_tolerance_sec=.05,
                 steady_time_ns=time.monotonic_ns):
        for value in (timeout_sec, max_age_sec, future_tolerance_sec):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('positive finite S4 sensor time limits required')
        self.timeout_ns = int(timeout_sec * 1e9)
        self.max_age_ns = int(max_age_sec * 1e9)
        self.future_ns = int(future_tolerance_sec * 1e9)
        self.session_id = None
        self.retired_sessions = set()
        self.status_seq = 0
        self.fault_seq = 0
        self.last_clock_ns = None
        self.clock_fault = False
        self.status = self.status_received_ns = self.matched = None
        self.frames = {}
        self.failure = ''
        self.steady_time_ns = steady_time_ns
        self.status_steady_ns = None

    def _clock(self, now_ns):
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.clock_fault = True
        self.last_clock_ns = now_ns

    def observation(self, stamp_ns, points, now_ns):
        self._clock(now_ns)
        try:
            if (stamp_ns <= 0 or len(points) % 8 or
                    any(not all(math.isfinite(v) for v in p) for p in points)):
                raise ValueError('invalid footprint')
            key = (stamp_ns, footprint_digest('map', stamp_ns, points))
        except (ValueError, TypeError, OverflowError):
            self.failure = 'INVALID_OBSTACLE_FRAME'
            return self.failure
        # Replayed arrays do not refresh their local receipt age.
        if key not in self.frames:
            self.frames[key] = (tuple(tuple(p) for p in points), now_ns, self.steady_time_ns())
        while len(self.frames) > 16:
            del self.frames[next(iter(self.frames))]
        self._match()
        return ''

    def receive(self, payload, now_ns):
        self._clock(now_ns)
        try:
            if len(payload) > 8192:
                raise ValueError('oversized status')
            status = json.loads(payload)
            if not isinstance(status, dict):
                raise ValueError('status object required')
            for key in ('schema_version', 'status_seq', 'frame_seq', 'fault_seq', 'source_stamp_ns',
                        'frame_received_ns', 'published_ns', 'cluster_count'):
                if type(status[key]) is not int or status[key] < 0:
                    raise ValueError('invalid ' + key)
            if (status['schema_version'] != 1 or status['clock'] != 'ros' or
                    status['state'] not in STATES or status['status_seq'] < 1 or
                    not isinstance(status['session_id'], str) or
                    len(status['session_id']) != 32 or
                    any(c not in '0123456789abcdef' for c in status['session_id'])):
                raise ValueError('invalid status protocol')
            if not -self.future_ns <= now_ns - status['published_ns'] <= self.timeout_ns:
                raise ValueError('stale/future status')
            if status['state'] in READY:
                digest = status['obstacle_digest']
                if (not isinstance(digest, str) or len(digest) != 64 or
                        any(c not in '0123456789abcdef' for c in digest) or
                        status['frame_seq'] < 1 or status['source_stamp_ns'] <= 0 or
                        status['frame_received_ns'] > status['published_ns'] or
                        (status['state'] == 'READY_EMPTY') != (status['cluster_count'] == 0)):
                    raise ValueError('invalid ready frame')
        except (ValueError, TypeError, KeyError, OverflowError):
            self.failure = 'INVALID_SENSOR_STATUS'
            self.status = None
            return self.failure
        session = status['session_id']
        restarted = False
        if session != self.session_id:
            if session in self.retired_sessions:
                self.failure = 'RETIRED_SENSOR_SESSION'
                return self.failure
            restarted = self.session_id is not None
            if restarted:
                self.retired_sessions.add(self.session_id)
                self.frames.clear()
            self.session_id = session
            self.status_seq = 0
            self.fault_seq = 0
            self.matched = None
        if status['status_seq'] <= self.status_seq:
            self.failure = 'SENSOR_STATUS_REPLAY'
            return self.failure
        if status['fault_seq'] < self.fault_seq:
            self.failure = 'SENSOR_FAULT_HISTORY_REGRESSION'
            return self.failure
        recovered_fault = status['fault_seq'] > self.fault_seq and status['state'] in READY
        self.fault_seq = status['fault_seq']
        if self.status is not None and not restarted and status['frame_seq'] < self.status['frame_seq']:
            self.failure = 'SENSOR_FRAME_REGRESSION'
            return self.failure
        if (self.status is not None and not restarted and status['state'] in READY and
                self.status['state'] in READY and status['frame_seq'] == self.status['frame_seq'] and
                any(status[key] != self.status[key] for key in (
                    'source_stamp_ns', 'frame_received_ns', 'cluster_count', 'obstacle_digest'))):
            self.failure = 'SENSOR_FRAME_IDENTITY_CHANGED'
            return self.failure
        self.status_seq = status['status_seq']
        self.status, self.status_received_ns = status, now_ns
        self.status_steady_ns = self.steady_time_ns()
        if status['state'] not in READY:
            self.failure = status['state']
        else:
            self._match()
        return ('SENSOR_RESTARTED' if restarted else
                'SENSOR_FAULT_HISTORY' if recovered_fault else self.failure)

    def _match(self):
        status = self.status
        if status is None or status['state'] not in READY:
            return
        found = self.frames.get((status['source_stamp_ns'], status['obstacle_digest']))
        if found is None:
            return
        points, received_ns, steady_ns = found
        if len(points) != status['cluster_count'] * 8:
            self.failure = 'SENSOR_CLUSTER_COUNT_MISMATCH'
            return
        if self.matched is not None:
            previous = self.matched[0]
            if status['frame_seq'] == previous['frame_seq']:
                if (status['source_stamp_ns'], status['obstacle_digest']) != (
                        previous['source_stamp_ns'], previous['obstacle_digest']):
                    self.failure = 'SENSOR_FRAME_IDENTITY_CHANGED'
                return
            if status['source_stamp_ns'] <= previous['source_stamp_ns']:
                self.failure = 'SENSOR_SOURCE_REPLAY'
                return
        self.matched = (status, points, received_ns, steady_ns)
        self.failure = ''

    def reason(self, now_ns):
        self._clock(now_ns)
        if self.clock_fault:
            return 'PLANNING_CLOCK_REGRESSION'
        if self.failure:
            return self.failure
        if self.status is None:
            return 'SENSOR_STATUS_NEVER_RECEIVED'
        if (not 0 <= now_ns - self.status_received_ns <= self.timeout_ns or
                self.steady_time_ns() - self.status_steady_ns > self.timeout_ns):
            return 'SENSOR_STATUS_TIMEOUT'
        if not -self.future_ns <= now_ns - self.status['published_ns'] <= self.timeout_ns:
            return 'SENSOR_STATUS_STALE'
        if self.matched is None:
            return 'SENSOR_FRAME_UNPAIRED'
        status, _, received_ns, steady_ns = self.matched
        if (not 0 <= now_ns - received_ns <= self.timeout_ns or
                self.steady_time_ns() - steady_ns > self.timeout_ns):
            return 'OBSTACLE_FRAME_TIMEOUT'
        if not -self.future_ns <= now_ns - status['frame_received_ns'] <= self.timeout_ns:
            return 'SENSOR_INPUT_TIMEOUT'
        if not -self.future_ns <= now_ns - status['source_stamp_ns'] <= self.max_age_ns:
            return 'SENSOR_SOURCE_STALE'
        return ''

    def groups(self):
        points = self.matched[1]
        return [list(points[i:i+8]) for i in range(0, len(points), 8)]
