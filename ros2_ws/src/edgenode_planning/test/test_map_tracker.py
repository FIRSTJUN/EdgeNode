import math
from pathlib import Path
import pytest
from edgenode_planning.map_tracker import MapTracker

MAP = Path('/workspace/local_data/c_track_mgeo/link_set.json')


def test_local_c_track_pose_acquires_correct_directed_lane():
    tracker = MapTracker(MAP)
    result = tracker.track(91.602783, -82.436241, math.radians(16.41685), .67)
    assert result.link_id == 'A222CC001925'
    assert abs(result.cross_track) < .05
    assert abs(result.steering) < .05


def test_turn_choices_follow_connected_topology():
    for preference, expected in [('straight', 'A222CC001939'),
                                 ('left', 'A222CC001937'),
                                 ('right', 'A222CC001934')]:
        tracker = MapTracker(MAP, turn_preference=preference)
        assert tracker.successor('A222CC001001') == expected
        outgoing = tracker.links[expected]
        assert outgoing['from_node_idx'] == tracker.links['A222CC001001']['to_node_idx']


@pytest.mark.parametrize('preference', ['straight', 'left', 'right'])
def test_closed_loop_bicycle_tracks_real_c_track_turns(preference):
    tracker = MapTracker(MAP, turn_preference=preference)
    x, y, yaw = 91.602783, -82.436241, math.radians(16.41685)
    dt, speed = .05, 1.0
    links, errors, steering = set(), [], []
    filtered = 0.
    for _ in range(6000):
        result = tracker.track(x, y, yaw, speed)
        assert result is not None, (preference, x, y, yaw, tracker.current)
        links.add(result.link_id); errors.append(abs(result.cross_track)); steering.append(result.steering)
        filtered += dt/(.1+dt)*(result.steering-filtered)
        wheel_angle = filtered*.49
        yaw += speed/1.04*math.tan(wheel_angle)*dt
        x += speed*math.cos(yaw)*dt; y += speed*math.sin(yaw)*dt
        if result.remaining < 1.5:
            break
    assert len(links) >= 4
    assert max(errors) < .65
    assert any(s > .15 for s in steering) or any(s < -.15 for s in steering)


def test_invalid_pose_and_off_route_do_not_rematch():
    tracker = MapTracker(MAP)
    assert tracker.track(float('nan'), 0., 0., 0.) is None
    assert tracker.track(10000., 10000., 0., 0.) is None
    assert tracker.track(91.602783, -82.436241, math.radians(16.41685), .67)
    current = tracker.current
    assert tracker.track(0., 0., 0., 0.) is None
    assert tracker.current == current
