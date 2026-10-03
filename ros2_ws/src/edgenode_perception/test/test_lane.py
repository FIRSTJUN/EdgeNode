"""Camera-only regression tests; no ROS publishers or LiDAR changes."""
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
import yaml
from edgenode_perception.perception_node import PerceptionNode


class Detector:
    make_lane_mask = PerceptionNode.make_lane_mask
    sliding_window_lane = PerceptionNode.sliding_window_lane

    def __init__(self):
        self.params = yaml.safe_load((Path(__file__).parents[1] / 'config/perception.yaml').read_text())['perception_node']['ros__parameters']
        # Reference regression fixtures draw paint starting at y=264.
        # Keep their original ROI/lookahead; live C-track tuning is tested separately.
        self.params.update(roi_top_y_ratio=.55, lookahead_y_ratio=.60, min_lane_span_ratio=.10)
        self.previous_lane_center = None

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])


def lane_image(left=True, right=True, shift=0):
    image = np.full((480, 640, 3), 45, np.uint8)
    if left:
        cv2.line(image, (220+shift, 264), (100+shift, 470), (210, 210, 210), 7)
    if right:
        cv2.line(image, (420+shift, 264), (540+shift, 470), (210, 210, 210), 7)
    return image


def test_blank_and_upper_background():
    d = Detector()
    im = np.zeros((480, 640, 3), np.uint8)
    im[:260] = 255
    mask = d.make_lane_mask(im)
    assert not mask[:264].any()
    assert d.sliding_window_lane(mask) is None


def test_two_lanes_and_binary_full_size():
    d = Detector()
    mask = d.make_lane_mask(lane_image())
    assert mask.shape == (480, 640)
    assert set(np.unique(mask)) <= {0, 255}
    error, confidence, _, _ = d.sliding_window_lane(mask)
    assert abs(error) < .03
    assert .5 < confidence <= 1


@pytest.mark.parametrize('left', [True, False])
def test_single_lane_uses_default_width(left):
    d = Detector()
    result = d.sliding_window_lane(d.make_lane_mask(lane_image(left=left, right=not left)))
    assert result is not None
    assert 0 < result[1] <= .55
    fit = d.lane_debug['left' if left else 'right']
    expected = np.polyval(fit, result[3]) + (1 if left else -1)*d.params['default_lane_width_px_640']/2
    assert result[2] == pytest.approx(expected)


def test_temporal_jump_reduces_confidence_and_can_recover():
    d = Detector()
    mask = d.make_lane_mask(lane_image())
    baseline = d.sliding_window_lane(mask)[1]
    d.previous_lane_center = 0
    assert d.sliding_window_lane(mask)[1] < baseline * .5
    assert d.sliding_window_lane(mask)[1] == pytest.approx(baseline)
    assert d.sliding_window_lane(np.zeros_like(mask)) is None
    assert d.previous_lane_center is None


def test_short_horizontal_marks_are_not_confident_lanes():
    d = Detector()
    im = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(im, (150, 282), (200, 294), (255, 255, 255), -1)
    cv2.rectangle(im, (440, 282), (490, 294), (255, 255, 255), -1)
    result = d.sliding_window_lane(d.make_lane_mask(im))
    assert result is None or result[1] < .5


def test_extrapolation_beyond_visible_lanes_is_not_confident():
    d = Detector()
    d.params['lookahead_y_ratio'] = .95
    im = lane_image()
    im[330:] = 0
    result = d.sliding_window_lane(d.make_lane_mask(im))
    assert result is not None
    assert result[1] < .5
    assert result[3] <= 329
    for support in (d.lane_debug['left_support'], d.lane_debug['right_support']):
        assert support[0] <= result[3] <= support[1]


def test_stronger_outer_curb_does_not_replace_lane():
    d = Detector()
    im = lane_image()
    # Bright pavement and its edge have substantially more pixels than paint.
    cv2.fillPoly(im, [np.array([[545,264],[600,470],[639,470],[639,264]])], (180,180,180))
    cv2.line(im,(545,264),(600,470),(235,235,235),14)
    result = d.sliding_window_lane(d.make_lane_mask(im))
    assert result is not None
    assert abs(result[0]) < .05
    assert np.polyval(d.lane_debug['right'], result[3]) < 470


def test_curved_pair_tracks_observed_pixels():
    d = Detector()
    im = np.full((480,640,3),45,np.uint8)
    ys = np.arange(275,435)
    bend = .001*(ys-320)**2
    left = 220-.5*(ys-275)+bend
    right = 420+.5*(ys-275)+bend
    for xs in (left,right):
        cv2.polylines(im,[np.column_stack([xs,ys]).astype(np.int32)],False,(230,230,230),7)
    result = d.sliding_window_lane(d.make_lane_mask(im))
    assert result is not None
    expected = 320+.001*(result[3]-320)**2
    assert abs(result[2]-expected) < 4
    assert result[1] > .5


def test_requested_lookahead_moves_inside_near_curve_support():
    d = Detector()
    im = lane_image()
    im[:305] = 45
    result = d.sliding_window_lane(d.make_lane_mask(im))
    assert result is not None
    assert 305 <= result[3] <= 470
    assert result[3] > d.lane_debug['requested_y']
    assert abs(result[0]) < .03


def test_recent_width_is_used_at_effective_y_and_expires():
    d = Detector()
    for _ in range(3):
        both = d.sliding_window_lane(d.make_lane_mask(lane_image()))
    assert both[1] >= d.params['lane_width_update_confidence']
    old_width = d._lane_tracker.width_fit.copy()
    one = d.sliding_window_lane(d.make_lane_mask(lane_image(right=False)))
    assert one is not None
    assert d.lane_debug['reason'] == 'single/recent width'
    assert d.lane_debug['width'] == pytest.approx(np.polyval(old_width,one[3]))
    assert abs(one[0]) < .03
    # One-sided and rejected measurements must never train the width prior.
    assert np.array_equal(d._lane_tracker.width_fit, old_width)
    blank = np.zeros((480,640),np.uint8)
    for _ in range(d.params['lane_width_max_age_frames']+1):
        d.sliding_window_lane(blank)
    d.sliding_window_lane(d.make_lane_mask(lane_image(right=False)))
    assert d.lane_debug['reason'] == 'single/default width'


def test_false_candidate_does_not_lock_tracker_or_update_width():
    d = Detector()
    im = lane_image()
    initial = d.sliding_window_lane(d.make_lane_mask(im))
    width = d._lane_tracker.width_fit.copy()
    false = im.copy()
    cv2.rectangle(false,(575,264),(635,470),(255,255,255),-1)
    observed = d.sliding_window_lane(d.make_lane_mask(false))
    assert observed is not None
    assert abs(observed[0]-initial[0]) < .1
    recovered = d.sliding_window_lane(d.make_lane_mask(im))
    assert abs(recovered[0]) < .03
    assert recovered[1] > .5
    # A distant, unambiguous new pair can recover instead of staying locked.
    moved = lane_image(shift=65)
    d.sliding_window_lane(d.make_lane_mask(moved))
    new = d.sliding_window_lane(d.make_lane_mask(moved))
    assert new[2] > 375
    assert new[1] > .5


def test_width_update_rejects_temporal_jump_frame():
    d = Detector()
    d.sliding_window_lane(d.make_lane_mask(lane_image()))
    old = d._lane_tracker.width_fit.copy()
    d.previous_lane_center = 0
    result = d.sliding_window_lane(d.make_lane_mask(lane_image()))
    assert result[1] < d.params['lane_width_update_confidence']
    assert np.array_equal(old,d._lane_tracker.width_fit)


def test_yellow_left_context_preserves_white_right_across_center():
    d = Detector()
    im = lane_image()
    cv2.line(im,(220,264),(100,470),(20,220,240),7)
    d.sliding_window_lane(d.make_lane_mask(im))
    assert d._lane_tracker.yellow_left_age == 0
    turning = np.full_like(im,45)
    cv2.line(turning,(180,275),(80,460),(230,230,230),9)
    result = d.sliding_window_lane(d.make_lane_mask(turning))
    assert result is not None
    assert d.lane_debug['right'] is not None
    assert d.lane_debug['left'] is None
    assert result[1] <= .55


def test_all_bright_background_is_not_a_lane_pair():
    d = Detector()
    im = np.full((480,640,3),235,np.uint8)
    assert d.sliding_window_lane(d.make_lane_mask(im)) is None


def test_color_context_recovers_on_clear_white_pair():
    d = Detector()
    yellow = lane_image()
    cv2.line(yellow,(220,264),(100,470),(20,220,240),7)
    d.sliding_window_lane(d.make_lane_mask(yellow))
    assert d._lane_tracker.yellow_left_age == 0
    clear = d.sliding_window_lane(d.make_lane_mask(lane_image()))
    assert clear[1] > .6
    assert d._lane_tracker.yellow_left_age >= d.params['lane_side_memory_frames']
