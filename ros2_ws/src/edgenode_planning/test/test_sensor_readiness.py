"""DDS ordering, replay and restart tests with deterministic ROS/steady clocks."""
import json

import pytest

from edgenode_planning.sensor_readiness import SensorReadiness, footprint_digest


NOW = 10_000_000_000


def payload(points=(), *, stamp=NOW, seq=1, frame=1, session='a' * 32,
            state=None, published=None):
    return dict(schema_version=1, clock='ros', session_id=session,
                status_seq=seq, frame_seq=frame, fault_seq=0,
                state=state or ('READY_OBSTACLES' if points else 'READY_EMPTY'),
                source_stamp_ns=stamp, frame_received_ns=stamp,
                published_ns=stamp if published is None else published,
                cluster_count=len(points) // 8,
                obstacle_digest=footprint_digest('map', stamp, points))


def send(guard, data, now=NOW):
    return guard.receive(json.dumps(data), now)


@pytest.mark.parametrize('status_first', [False, True])
def test_pairing_works_in_either_dds_arrival_order(status_first):
    guard = SensorReadiness()
    data = payload()
    first, second = ((lambda: send(guard, data)),
                     (lambda: guard.observation(NOW, [], NOW)))
    if not status_first:
        first, second = second, first
    first()
    assert guard.reason(NOW)
    second()
    assert guard.reason(NOW) == '' and guard.groups() == []


def test_never_received_and_stale_status_are_not_clear_space():
    guard = SensorReadiness()
    assert guard.reason(NOW) == 'SENSOR_STATUS_NEVER_RECEIVED'
    assert send(guard, payload(published=NOW - 10**9)) == 'INVALID_SENSOR_STATUS'


def test_fresh_heartbeat_and_replayed_array_do_not_renew_old_data():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW)
    send(guard, payload())
    later = NOW + 610_000_000
    send(guard, payload(seq=2, published=later), later)
    guard.observation(NOW, [], later)
    assert guard.reason(later) == 'OBSTACLE_FRAME_TIMEOUT'


def test_steady_watchdog_detects_stream_loss_with_frozen_ros_clock():
    steady = [0]
    guard = SensorReadiness(steady_time_ns=lambda: steady[0])
    guard.observation(NOW, [], NOW)
    send(guard, payload())
    steady[0] = 601_000_000
    assert guard.reason(NOW) == 'SENSOR_STATUS_TIMEOUT'


@pytest.mark.parametrize('state', ['INVALID_INPUT', 'INPUT_TIMEOUT', 'STALE_INPUT',
                                  'DUPLICATE_FRAME', 'OUT_OF_ORDER', 'CLOCK_REGRESSION'])
def test_failure_status_overrides_previously_valid_empty(state):
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW)
    send(guard, payload())
    assert send(guard, payload(seq=2, frame=2, state=state)) == state
    assert guard.reason(NOW) == state


@pytest.mark.parametrize('change', [dict(schema_version=2), dict(clock='unknown'),
    dict(state='UNKNOWN'), dict(status_seq=True), dict(session_id='bad'),
    dict(cluster_count=1), dict(obstacle_digest='bad')])
def test_malformed_status_fails_closed(change):
    guard = SensorReadiness()
    data = payload(); data.update(change)
    assert send(guard, data) == 'INVALID_SENSOR_STATUS'


def test_status_replay_and_wrong_digest_cannot_authorize_a_frame():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW)
    send(guard, payload())
    assert send(guard, payload()) == 'SENSOR_STATUS_REPLAY'
    data = payload(seq=2, frame=2, stamp=NOW + 1)
    data['obstacle_digest'] = 'f' * 64
    send(guard, data, NOW + 1)
    assert guard.reason(NOW + 1)


def test_restart_invalidates_prior_pair_and_rejects_retired_producer():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW); send(guard, payload())
    assert send(guard, payload(session='b' * 32)) == 'SENSOR_RESTARTED'
    assert guard.reason(NOW) == 'SENSOR_FRAME_UNPAIRED'
    guard.observation(NOW + 1, [], NOW + 1)
    send(guard, payload(session='b' * 32, stamp=NOW + 1, seq=2, frame=2), NOW + 1)
    assert not guard.reason(NOW + 1)
    assert send(guard, payload(seq=2), NOW + 1) == 'RETIRED_SENSOR_SESSION'


def test_pending_good_frame_preserves_last_complete_pair_until_deadline():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW); send(guard, payload())
    send(guard, payload(stamp=NOW + 1, seq=2, frame=2), NOW + 1)
    assert not guard.reason(NOW + 1)
    assert guard.matched[0]['source_stamp_ns'] == NOW
    guard.observation(NOW + 1, [], NOW + 1)
    assert guard.matched[0]['source_stamp_ns'] == NOW + 1


def test_ros_clock_rewind_cannot_be_healed_by_new_messages():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW); send(guard, payload())
    assert guard.reason(NOW - 1) == 'PLANNING_CLOCK_REGRESSION'
    send(guard, payload(seq=2), NOW)
    assert guard.reason(NOW) == 'PLANNING_CLOCK_REGRESSION'


def test_heartbeat_cannot_change_the_identity_of_an_existing_frame():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW); send(guard, payload())
    assert send(guard, payload(stamp=NOW + 1, seq=2), NOW + 1) == 'SENSOR_FRAME_IDENTITY_CHANGED'


def test_wrong_obstacle_count_cannot_match_a_successful_status():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW)
    data = payload(state='READY_OBSTACLES'); data['cluster_count'] = 1
    assert send(guard, data) == 'SENSOR_CLUSTER_COUNT_MISMATCH'


def test_recovered_snapshot_still_exposes_an_unseen_failure():
    guard = SensorReadiness()
    guard.observation(NOW, [], NOW); send(guard, payload())
    guard.observation(NOW + 1, [], NOW + 1)
    data = payload(seq=3, frame=3, stamp=NOW + 1)
    data['fault_seq'] = 1
    assert send(guard, data, NOW + 1) == 'SENSOR_FAULT_HISTORY'
    assert not guard.reason(NOW + 1)  # Owner must latch the returned stop event.


def test_invalid_status_cannot_be_healed_by_an_array_for_an_older_success():
    guard = SensorReadiness()
    send(guard, payload())  # Success status exists but the array is pending.
    assert guard.receive('{invalid', NOW) == 'INVALID_SENSOR_STATUS'
    guard.observation(NOW, [], NOW)
    assert guard.reason(NOW) == 'INVALID_SENSOR_STATUS'
