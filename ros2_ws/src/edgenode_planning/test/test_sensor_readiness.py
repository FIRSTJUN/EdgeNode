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


def atomic(guard,data,points=(),now=NOW):
    return guard.frame(json.dumps(dict(schema_version=1,frame_id='map',
                                      status=data,points=points)),now)


def test_atomic_frames_can_skip_samples_without_losing_pairing_or_fault_history():
    guard=SensorReadiness()
    assert not atomic(guard,payload())
    data=payload(stamp=NOW+100_000_000,seq=9,frame=5,published=NOW+100_000_000)
    assert not atomic(guard,data,now=NOW+100_000_000)
    assert not guard.reason(NOW+100_000_000)
    assert guard.matched[0]['frame_seq']==5
    data=payload(stamp=NOW+200_000_000,seq=11,frame=7,published=NOW+200_000_000)
    data['fault_seq']=1
    assert atomic(guard,data,now=NOW+200_000_000)=='SENSOR_FAULT_HISTORY'


def test_atomic_heartbeat_cannot_renew_observation_and_digest_mismatch_fails_closed():
    guard=SensorReadiness()
    assert not atomic(guard,payload())
    later=NOW+610_000_000
    atomic(guard,payload(seq=2,published=later),now=later)
    assert guard.reason(later)=='OBSTACLE_FRAME_TIMEOUT'
    guard=SensorReadiness();data=payload();data['obstacle_digest']='f'*64
    assert atomic(guard,data)=='INVALID_SENSOR_FRAME'
    assert guard.reason(NOW)=='INVALID_SENSOR_FRAME'


def test_atomic_restart_invalidates_old_producer_and_pairs_new_enclosed_frame():
    guard=SensorReadiness();atomic(guard,payload())
    assert atomic(guard,payload(session='b'*32))=='SENSOR_RESTARTED'
    assert not guard.reason(NOW) and guard.matched[0]['session_id']=='b'*32
    assert atomic(guard,payload(seq=2))=='RETIRED_SENSOR_SESSION'


def test_morai_alignment_gap_keeps_only_last_complete_frame_until_unchanged_deadline():
    guard=SensorReadiness(allow_transient_history=True)
    atomic(guard,payload())
    later=NOW+100_000_000
    data=payload(stamp=later,seq=2,frame=2,state='LOCALIZATION_UNAVAILABLE')
    data['fault_seq']=1
    assert not atomic(guard,data,now=later)
    assert not guard.reason(later) and guard.matched[0]['source_stamp_ns']==NOW
    later=NOW+610_000_000
    data=payload(stamp=later,seq=3,frame=3,state='LOCALIZATION_UNAVAILABLE')
    data['fault_seq']=2
    assert atomic(guard,data,now=later)=='LOCALIZATION_UNAVAILABLE'
    assert guard.reason(later)=='LOCALIZATION_UNAVAILABLE'


@pytest.mark.parametrize('fault,expected',[('LOCALIZATION_UNAVAILABLE',''),('INVALID_INPUT','SENSOR_FAULT_HISTORY')])
def test_morai_fault_ledger_distinguishes_skipped_alignment_gap_from_bad_content(fault,expected):
    guard=SensorReadiness(allow_transient_history=True);atomic(guard,payload())
    later=NOW+100_000_000;data=payload(stamp=later,seq=3,frame=3)
    data.update(fault_seq=1,fault_history=[dict(seq=1,state=fault,source_stamp_ns=later)])
    assert atomic(guard,data,now=later)==expected
    guard=SensorReadiness(allow_transient_history=True);atomic(guard,payload())
    data['fault_seq']=2  # Missing history is never permission to ignore a fault.
    assert atomic(guard,data,now=later)=='SENSOR_FAULT_HISTORY'


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
