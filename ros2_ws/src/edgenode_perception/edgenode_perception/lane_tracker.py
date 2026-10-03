"""Camera-only lane candidates, supported fits and bounded temporal priors.

Coordinates remain in the full image. No ROS, actuator or LiDAR dependencies.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class LaneCandidate:
    fit: np.ndarray
    low: float
    high: float
    residual: float
    thickness: float
    coverage: float
    pixels: int
    base: float
    quality: float
    yellow: float = 0.


class LaneTracker:
    def __init__(self):
        self.previous = None
        self.width_fit = None
        self.width_age = 0
        self.misses = 0
        self.yellow_left_age = 1000
        self.last_single = None
        # Also used by the offline ablation evaluator; not ROS parameters.
        self.support_aware = True
        self.adaptive_width = True

    @staticmethod
    def option(params, name, default):
        # Offline evaluators may load older parameter snapshots.
        try:
            return params(name)
        except KeyError:
            return default

    @staticmethod
    def candidates(binary, params, image=None, rejected=None):
        """Track several bounded row runs, never average disjoint bright regions.

        Row windows retain individual peaks. A wide filled background is not a
        paint stripe; gaps allow dashed paint and intersections to be crossed.
        """
        h, w = binary.shape
        gray = None if image is None else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
        step = max(2, round(h / 160))
        tracks = []
        top = int(h * params('roi_top_y_ratio'))
        margin = params('sliding_margin_px') * w / 640
        for y in range(top, h, step):
            row = np.pad(binary[y] > 0, (1, 1)).astype(np.int8)
            edges = np.diff(row)
            starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
            runs = [(float((a+b-1)/2), float(b-a)) for a,b in zip(starts,ends)
                    if 2 <= b-a <= max(24, LaneTracker.option(params, 'lane_max_run_width_ratio', .11)*w) and a > 0 and b < w]
            assigned = set()
            possibilities = []
            for ti, tr in enumerate(tracks):
                if y-tr[-1][0] > step*5:
                    continue
                pred = tr[-1][1]
                if len(tr) >= 3:
                    slope = (tr[-1][1]-tr[-3][1]) / (tr[-1][0]-tr[-3][0])
                    pred += np.clip(slope, -8, 8)*(y-tr[-1][0])
                for ri,(x,width) in enumerate(runs):
                    dist = abs(x-pred)
                    if dist < min(margin, 12*w/640 + 3*(y-tr[-1][0])):
                        possibilities.append((dist, ti, ri))
            used = set()
            for _,ti,ri in sorted(possibilities):
                if ti in used or ri in assigned:
                    continue
                x,width = runs[ri]; tracks[ti].append((y,x,width))
                used.add(ti); assigned.add(ri)
            for ri,(x,width) in enumerate(runs):
                if ri not in assigned:
                    tracks.append([(y,x,width)])
        candidates = []
        for tr in tracks:
            if len(tr) < 5:
                continue
            ys,xs,widths = np.array(tr).T
            if ys[-1]-ys[0] < h*.025:
                continue
            fit = np.polyfit(ys, xs, 2)
            # Robust center fit rejects crossing marks, not just their confidence.
            for _ in range(2):
                resid = np.abs(np.polyval(fit,ys)-xs)
                keep = resid <= max(3*w/640, float(np.median(resid))*2.5)
                if np.count_nonzero(keep) < 5:
                    break
                ys,xs,widths = ys[keep],xs[keep],widths[keep]
                fit = np.polyfit(ys,xs,2)
            span = ys[-1]-ys[0]
            residual = float(np.sqrt(np.mean((np.polyval(fit,ys)-xs)**2)))
            slopes = np.polyval(np.polyder(fit),ys)
            thickness = float(np.median(widths/np.sqrt(1+slopes**2)))
            # Thick ribbons/filled areas and almost horizontal crosswalks are
            # not longitudinal paint candidates. This is image-resolution scaled.
            if thickness > (45 if image is not None else 18)*w/640 or thickness < 1.2*w/640:
                continue
            contrast_score = 1.
            yellow = 0.
            if gray is not None:
                ix,iy=xs.astype(int),ys.astype(int)
                flank=max(3,int(4*w/640))
                il=np.clip((xs-widths/2-flank).astype(int),0,w-1)
                ir=np.clip((xs+widths/2+flank).astype(int),0,w-1)
                contrast=np.minimum(gray[iy,ix]-gray[iy,il],gray[iy,ix]-gray[iy,ir])
                # Paint is a bright ridge on road on BOTH sides. A curb at the
                # edge of a bright pavement typically has only one dark flank.
                contrast_score=float(np.mean(contrast>LaneTracker.option(params, 'lane_min_contrast', 45.0)))
                colors=image[iy,ix].astype(float)
                yellow=float(np.mean((colors[:,2]>1.25*colors[:,0]) & (colors[:,1]>1.25*colors[:,0]) & (colors[:,1]-colors[:,0]>40)))
                if contrast_score < LaneTracker.option(params, 'lane_min_paint_fraction', .6):
                    if rejected is not None:
                        rejected.append(dict(base=float(xs[-1]),y=float(ys[-1]),reason='low bilateral contrast'))
                    continue
            pixels = int(np.sum(widths)*step)
            if pixels < params('min_lane_pixels'):
                continue
            coverage = min(1., len(ys)*step/max(step,span))
            support = min(1., span / max(1,h*params('min_lane_span_ratio')))
            quality = support*coverage*math.exp(-residual/(4*w/640))
            quality *= contrast_score
            candidates.append(LaneCandidate(fit,float(ys[0]),float(ys[-1]),residual,
                thickness,coverage,pixels,float(xs[-1]),quality,yellow))
        return sorted(candidates,key=lambda c:c.quality,reverse=True)[:max(2,int(LaneTracker.option(params,'lane_max_candidates',12)))]

    def detect(self, binary, params, previous_center=None, image=None):
        h,w = binary.shape
        requested = float(int(h*params('lookahead_y_ratio')))
        default_width = params('default_lane_width_px_640')*w/640
        self.width_age += 1
        self.yellow_left_age += 1
        rejected=[]
        candidates = self.candidates(binary,params,image,rejected)
        if self.misses>15:
            self.previous=None
        detail = dict(reason='no supported paint candidates', requested_y=requested,
                      candidates=candidates, rejected=rejected, left=None, right=None)
        self.debug = detail
        pairs = []
        for i,a in enumerate(candidates):
            for b in candidates[i+1:]:
                low,high = max(a.low,b.low),min(a.high,b.high)
                if high-low < h*.025:
                    continue
                y = float(np.clip(requested,low,high)) if self.support_aware else requested
                if not low <= y <= high:
                    continue
                left,right = sorted((a,b),key=lambda c:np.polyval(c.fit,y))
                yy = np.linspace(low,high,9)
                lx,rx = np.polyval(left.fit,yy),np.polyval(right.fit,yy)
                widths=rx-lx
                if np.any(widths < w*.12) or np.any(widths > w*1.15):
                    continue
                lx0,rx0 = float(np.polyval(left.fit,y)),float(np.polyval(right.fit,y))
                if not (0<=lx0<w and 0<=rx0<w):
                    continue
                center=(lx0+rx0)/2; width=rx0-lx0
                if not lx0-w*.08 < w/2 < rx0+w*.08:
                    continue
                if right.yellow > .6 and left.yellow < .3:
                    continue
                # Perspective width is locally linear; abrupt narrowing or
                # asymmetric bending distinguishes a curb/stripe pairing.
                linear=np.polyval(np.polyfit(yy,widths,1),yy)
                bend=float(np.max(abs(widths-linear))/np.mean(widths))
                narrowing=max(0.,float(widths[0]-widths[-1]))/float(np.mean(widths))
                curvature=abs(left.fit[0]-right.fit[0])*(high-low)**2/float(np.mean(widths))
                geometry=math.exp(-(bend+narrowing+curvature)/params('geometry_tolerance_ratio'))
                width_score=math.exp(-abs(width-default_width)/max(default_width,1))
                support=min(1.,(high-low)/(h*params('min_lane_span_ratio')))
                temporal=1.
                if self.previous is not None:
                    old_left,old_right,old_y=self.previous
                    prev=(np.polyval(old_left,y)+np.polyval(old_right,y))/2
                    temporal=math.exp(-abs(center-prev)/(w*.25))
                quality=math.sqrt(left.quality*right.quality)*geometry*support
                # Width/temporal are ranking priors, never an excuse to retain
                # a bad fit. A clear new geometry can win after camera motion.
                score=quality*(.8+.2*width_score)*(.85+.15*temporal)
                pairs.append((score,left,right,y,quality,width_score,temporal,geometry,support))
        # A geometrically poor pair is worse evidence than a supported single
        # stripe. Do not select any pair merely because two fits exist.
        best_single=max((c.quality for c in candidates), default=0.)
        pairs=[pair for pair in pairs if pair[7]>=.4 and pair[0]>=.45*best_single]
        pairs.sort(key=lambda p:p[0],reverse=True)
        left=right=None; ambiguity=1.; geometry=1.; temporal=1.; width_score=1.
        if pairs:
            score,left,right,y,quality,width_score,temporal,geometry,support=pairs[0]
            if len(pairs)>1:
                # Only penalize distinct lane alternatives, not a duplicated fit.
                next_score=pairs[1][0]
                ambiguity=max(.4,min(1.,(score-next_score)/max(.15*score,1e-6)))
            confidence=quality*(.8+.2*width_score)*ambiguity
            center=float((np.polyval(left.fit,y)+np.polyval(right.fit,y))/2)
            width=float(np.polyval(right.fit,y)-np.polyval(left.fit,y))
            low,high=max(left.low,right.low),min(left.high,right.high)
            self.previous=(left.fit.copy(),right.fit.copy(),y)
            self.misses=0
            if confidence>=.6:
                self.yellow_left_age = 0 if left.yellow>.6 and right.yellow<.3 else 1000
            reason='pair'
        elif candidates:
            def single_score(candidate):
                look=float(np.clip(requested,candidate.low,candidate.high))
                score=candidate.quality*math.exp(-abs(look-requested)/(h*.15))
                if self.last_single is not None:
                    prior=self.last_single
                    low_common=max(candidate.low,prior.low)
                    high_common=min(candidate.high,prior.high)
                    if low_common<=high_common:
                        shared=float(np.clip(requested,low_common,high_common))
                        distance=abs(np.polyval(candidate.fit,shared)-np.polyval(prior.fit,shared))
                        score*=.8+.2*math.exp(-distance/(w*.2))
                return score
            lane=max(candidates,key=single_score)
            low,high=lane.low,lane.high
            y=float(np.clip(requested,low,high)) if self.support_aware else requested
            if not low<=y<=high:
                detail['reason']='outside support'; self.misses+=1
                return None
            x=float(np.polyval(lane.fit,y))
            if not 0<=x<w:
                detail['reason']='outside image';self.misses+=1
                return None
            # Side matching is allowed across the image center during a turn.
            side='left' if lane.yellow>.6 or x<w/2 else 'right'
            if lane.yellow<.3 and self.yellow_left_age<int(self.option(params,'lane_side_memory_frames',90)):
                side='right'
            elif lane.yellow<.6 and self.previous is not None and self.misses<15:
                dist=[abs(x-np.polyval(f,y)) for f in self.previous[:2]]
                if min(dist)<w*.25:side=('left','right')[int(np.argmin(dist))]
            width=default_width;reason='single/default width'
            if self.adaptive_width and self.width_fit is not None and self.width_age<=int(self.option(params,'lane_width_max_age_frames',30)):
                if self.width_support[0]<=y<=self.width_support[1]:
                    estimated=float(np.polyval(self.width_fit,y))
                    if w*.12<=estimated<=w*1.15:width=estimated;reason='single/recent width'
            center=x+(width/2 if side=='left' else -width/2)
            confidence=min(.55,.55*lane.quality)
            if not 0<=center<w:
                # The unseen mate cannot justify a confident off-image center.
                confidence *= .5
            left=lane if side=='left' else None;right=lane if side=='right' else None
            self.last_single=lane
            self.misses+=1
        else:
            self.misses+=1
            if self.misses>15:
                self.previous=None
                self.last_single=None
            return None
        movement=abs(y-requested)
        # Clamping removes extrapolation but a major lookahead move still means
        # a less comparable steering observation.
        extrapolation=math.exp(-movement/(h*.2))
        jump=1.
        if previous_center is not None:
            jump=min(1.,w*params('max_center_jump_ratio')/max(1.,abs(center-previous_center)))
        pixel_support=min(1., sum(c.pixels for c in [left,right] if c)/
                          max(1., 3*params('min_lane_pixels')))
        confidence*=extrapolation*jump*pixel_support
        if self.adaptive_width and left is not None and right is not None and confidence>=self.option(params,'lane_width_update_confidence',.7) and ambiguity>=.9:
            fitted=right.fit-left.fit
            alpha=float(np.clip(self.option(params,'lane_width_ema_alpha',.2),0,1))
            self.width_fit=fitted.copy() if self.width_fit is None else (1-alpha)*self.width_fit+alpha*fitted
            self.width_support=(low,high); self.width_age=0
        components=dict(paint_fit=float(min(c.quality for c in [left,right] if c)),
                        pixels=pixel_support,geometry=geometry,width=width_score,
                        ambiguity=ambiguity,temporal=jump,lookahead=extrapolation)
        detail.update(reason=reason,left=None if left is None else left.fit,
                      right=None if right is None else right.fit,low=low,high=high,
                      left_support=None if left is None else (left.low,left.high),
                      right_support=None if right is None else (right.low,right.high),
                      effective_y=y,width=width,recent_width=None if self.width_fit is None else float(np.polyval(self.width_fit,y)),
                      components=components,selected_bases=[c.base for c in [left,right] if c])
        return float(np.clip((center-w/2)/(w/2),-1,1)),float(np.clip(confidence,0,1)),center,y


def draw_lane_debug(image, mask, polygon, detail, result):
    """Draw the actual support, candidate decisions and unsmoothed evidence."""
    out = image.copy()
    h, w = image.shape[:2]
    cv2.polylines(out, polygon, True, (255, 255, 0), 1)
    for rejected in detail.get('rejected',[]):
        cv2.drawMarker(out,(int(rejected['base']),int(rejected['y'])),(0,0,200),cv2.MARKER_TILTED_CROSS,7,1)
    selected = detail.get('selected_bases', [])
    for candidate in detail.get('candidates', []):
        point = (int(np.clip(candidate.base, 0, w-1)), int(candidate.high))
        chosen = any(abs(candidate.base-base)<.01 for base in selected)
        color = (0, 220, 0) if chosen else (60, 60, 255)
        cv2.circle(out, point, 4, color, 1)
    for side, color in [('left', (0, 255, 255)), ('right', (255, 128, 0))]:
        fit = detail.get(side)
        if fit is None:
            continue
        low, high = detail.get(side+'_support') or (detail['low'],detail['high'])
        yy = np.linspace(low, high, 60)
        xx = np.polyval(fit, yy)
        inside = (xx>=0)&(xx<w)
        points = np.column_stack((xx[inside],yy[inside])).astype(np.int32)
        if len(points)>1:
            cv2.polylines(out,[points],False,color,2)
            for point in [points[0],points[-1]]:
                cv2.circle(out,tuple(point),4,color,-1)
    requested=detail.get('requested_y', h*.6)
    cv2.line(out,(0,int(requested)),(w-1,int(requested)),(255,0,255),1)
    cv2.line(out,(w//2,100),(w//2,h-1),(0,0,255),1)
    status='LANE LOST'
    if result is not None:
        error,confidence,cx,y=result
        status=f'error={error:+.3f} confidence={confidence:.3f}'
        cv2.line(out,(0,int(y)),(w-1,int(y)),(0,140,0),1)
        cv2.circle(out,(int(cx),int(y)),6,(0,255,0),-1)
    def value(key):
        v=detail.get(key)
        return '-' if v is None else f'{v:.1f}'
    lines=[status+'  '+detail.get('reason',''),
           f"lookahead requested={requested:.0f} effective={value('effective_y')} candidates={len(detail.get('candidates',[]))}",
           f"width={value('width')} recent={value('recent_width')} L={detail.get('left_support')} R={detail.get('right_support')}",
           ' '.join(f'{k}={v:.2f}' for k,v in list(detail.get('components',{}).items())[:4]),
           ' '.join(f'{k}={v:.2f}' for k,v in list(detail.get('components',{}).items())[4:])]
    cv2.rectangle(out,(0,0),(w-1,113),(25,25,25),-1)
    for i,line in enumerate(lines):
        cv2.putText(out,line,(6,19+22*i),cv2.FONT_HERSHEY_SIMPLEX,.43,(230,240,230),1)
    out[h-h//3:, :w//3]=cv2.cvtColor(cv2.resize(mask,(w//3,h//3)),cv2.COLOR_GRAY2BGR)
    return out
