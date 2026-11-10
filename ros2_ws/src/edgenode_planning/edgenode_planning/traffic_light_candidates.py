"""Small image-space association tracker for the explicit route/ROI fallback.

Scores are heuristics, not calibrated lane ownership. Route heading supplies an
optional forward image hint; no map-to-camera calibration is claimed.
"""
import math


COLORS = {'red': 'RED', 'yellow': 'YELLOW', 'green': 'GREEN'}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def center(box):
    return [(box[0] + box[2]) / 2., (box[1] + box[3]) / 2.]


def iou(a, b):
    overlap = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
    total = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - overlap
    return overlap / total if total > 0 else 0.


class TrafficLightCandidateSelector:
    """Use distinct camera frames only; expire tracks after 0.5 seconds."""
    required_margin = .06
    max_center_step = .08  # image-width/height normalized, not vehicle geometry

    def __init__(self):
        self.box = None
        self.state = None
        self.stamp = None
        self.track_id = 0

    def select(self, detections, association, source, *, near_stop=False, expected_x=.5):
        w, h = association['image_size']
        roi = association['roi_xyxy_norm']
        expected_x = min(roi[2], max(roi[0], expected_x))
        tracked = self.box is not None and 0 <= source - self.stamp <= 500_000_000
        old_center = center(self.box) if tracked else None
        diagnostics = dict(detection_count=len(detections), candidate_count=0, candidates=[],
            selected_candidate=None, selected_score=None, second_best_score=None,
            ambiguity_margin=None, required_ambiguity_margin=self.required_margin,
            ambiguous=False, expected_center_x=expected_x * w, near_stop_hysteresis=near_stop,
            selected_track_id=None, target_switched=False)
        eligible = []
        for index, detection in enumerate(detections):
            if not isinstance(detection,dict):
                diagnostics['candidates'].append(dict(index=index,center=None,score=None,rejection='INVALID_DETECTION'))
                continue
            label=detection.get('class_name')
            raw_confidence=detection.get('confidence')
            row = dict(index=index, class_name=label if isinstance(label,str) else None,
                confidence=raw_confidence if finite(raw_confidence) else None, center=None, score=None, rejection=None)
            diagnostics['candidates'].append(row)
            box = detection.get('box_xyxy')
            conf = row['confidence']
            if (not isinstance(box, (list, tuple)) or len(box) != 4 or not all(finite(v) for v in box)
                    or not finite(conf) or not 0 <= conf <= 1
                    or not 0 <= box[0] < box[2] <= w or not 0 <= box[1] < box[3] <= h):
                row['rejection'] = 'INVALID_DETECTION'
                continue
            xy = center(box)
            row.update(center=xy, box_xyxy=list(box))
            if conf < association.get('min_confidence', .25) or row['class_name'] not in COLORS:
                row['rejection'] = 'BELOW_CONFIDENCE_OR_UNKNOWN_CLASS'
                continue
            nx, ny = xy[0] / w, xy[1] / h
            # Use center containment: a large head straddling the ROI border is
            # still a candidate. Numeric ROI/confidence settings stay unchanged.
            if not roi[0] <= nx <= roi[2] or not roi[1] <= ny <= roi[3]:
                row['rejection'] = 'OUTSIDE_EGO_ROI'
                continue
            horizontal = max(0., 1. - abs(nx - expected_x) / .25)
            upper = max(0., 1. - ny / max(roi[3], .01))
            area = min(1., math.sqrt((box[2]-box[0]) * (box[3]-box[1]) / (w*h)) / .06)
            overlap = iou(box, self.box) if tracked else 0.
            step = math.hypot((xy[0]-old_center[0])/w, (xy[1]-old_center[1])/h) if tracked else None
            continuity = max(overlap, max(0., 1.-step/self.max_center_step)) if tracked else 0.
            persistence = float(tracked and COLORS[row['class_name']] == self.state)
            score = .40*horizontal + .10*upper + .08*area + .12*conf
            if tracked:
                score += .26*continuity + .04*persistence
                if step > self.max_center_step and overlap < .05:
                    score -= .30  # a sudden side jump cannot inherit the track
            else:
                score /= .70
            row.update(score=score, iou=overlap, center_step=step, continuity=continuity,
                components=dict(horizontal=horizontal, upper=upper, size=area,
                    confidence=conf, continuity=continuity, state_persistence=persistence))
            eligible.append(row)
        # Same-class duplicates on one lamp should not become two competitors.
        unique = []
        for row in sorted(eligible, key=lambda r: (-r['confidence'], r['index'])):
            if any(row['class_name'] == other['class_name'] and iou(row['box_xyxy'], other['box_xyxy']) >= .65 for other in unique):
                row['rejection'] = 'DUPLICATE_SAME_HEAD'
            else:
                unique.append(row)
        diagnostics['candidate_count'] = len(unique)
        incumbent = None
        if tracked and unique:
            matches = [r for r in unique if r['iou'] >= .05 or r['center_step'] <= self.max_center_step]
            if matches:
                incumbent = max(matches, key=lambda r: (r['continuity'], r['score']))
                incumbent['score'] += .16 if near_stop else .04
                incumbent['retention_bonus'] = .16 if near_stop else .04
        ranked = sorted(unique, key=lambda r: (-r['score'], r['index']))
        if not ranked:
            return None, dict(diagnostics, selection_reason='NO_ELIGIBLE_EGO_ROI_DETECTION')
        best = ranked[0]
        second = ranked[1]['score'] if len(ranked) > 1 else None
        margin = best['score'] - second if second is not None else None
        diagnostics.update(selected_score=best['score'], second_best_score=second, ambiguity_margin=margin)
        if second is not None and margin < self.required_margin:
            return None, dict(diagnostics, ambiguous=True, selection_reason='AMBIGUOUS_CANDIDATE_SCORES')
        if tracked and incumbent is None:
            return None, dict(diagnostics, selection_reason='TARGET_JUMP_REJECTED')
        switched = not tracked or (incumbent is not None and best is not incumbent)
        # Repeated Planning ticks of a camera frame do not refresh or move it.
        if self.stamp != source:
            if switched:
                self.track_id += 1
            self.box = best['box_xyxy']
            self.state = COLORS[best['class_name']]
            self.stamp = source
        diagnostics.update(selected_candidate=dict(best), selected_track_id=self.track_id,
            target_switched=switched, selection_reason='SCORED_EGO_ROUTE_TARGET')
        return COLORS[best['class_name']], diagnostics
