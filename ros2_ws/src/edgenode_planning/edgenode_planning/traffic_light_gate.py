"""Opt-in route/stop-line gate; evidence is never a drive-enable command."""
import hashlib,json,math
from pathlib import Path
from .stop_line_geometry import resolve,stop_speed_cap
from .route_stopline_fallback import MODE, associations_for_route, entry_key
from .traffic_light_candidates import TrafficLightCandidateSelector
from .crosswalk_stop_correction import CrosswalkStopCorrection


def finite(x):
    try:return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)
    except OverflowError:return False


def route_digest(points):return hashlib.sha256(json.dumps(points,separators=(',',':'),allow_nan=False).encode()).hexdigest()


class TrafficLightGate:
    def __init__(self,config=None):
        self.config=config;self.payload=None;self.received=None;self.observed_state="UNKNOWN";self.state_received=None;self._fallback_cache=None;self._route_digest=None
        self.crosswalk=CrosswalkStopCorrection()
        self.reset()

    def reset(self):
        self.candidate_selector=TrafficLightCandidateSelector();self.selection_diagnostics={};self._selected_track_id=None
        self.cleared=set();self.approached=False;self.crossed=False;self.late_pass=False
        self.last_source=0;self.green_since=None;self.green_count=0;self.entry_authorized=False
        self.active_signal=None;self.red_since=None;self.red_count=0;self.red_last=None;self.stable_red=False;self.sample_source=None;self.sample_color=None
        self.stop_latched=False;self.stop_requested=False;self.comfort_stop_selected=False;self.last_progress=None;self.last_now=None

    @classmethod
    def from_file(cls,path):
        try:return cls(json.loads(Path(path).read_text())) if path else cls()
        except (OSError,ValueError):return cls()

    def observe(self,raw,now):
        try:self.payload=json.loads(raw) if isinstance(raw,str) else raw
        except (ValueError,TypeError):self.payload=None
        self.received=now

    def observe_state(self,raw,now):
        self.observed_state=raw if raw in ('RED','GREEN','YELLOW','UNKNOWN') else 'UNKNOWN'
        self.state_received=now

    def observe_detector(self,raw,now,camera_topic='/camera/image/compressed'):
        """Adapt the active YOLO detector without inventing stamp/ROI ownership."""
        try:
            p=json.loads(raw);stamp=p['stamp']
            rows=[]
            for d in p['predictions']:
                x,y,w,h=(d[k] for k in ('x','y','w','h'))
                rows.append(dict(class_name=d['label'],confidence=d['confidence'],
                    box_xyxy=[x-w/2,y-h/2,x+w/2,y+h/2]))
            self.observe(dict(schema=1,status='OK',
                source_stamp_ns=stamp['sec']*1_000_000_000+stamp['nanosec'],
                source_topic=camera_topic,frame_id=p['frame_id'],
                image_size=[p['image_width'],p['image_height']],detections=rows,
                state=p.get('state','UNKNOWN')),now)
        except (KeyError,TypeError,ValueError):self.observe(None,now)

    def reset_green(self):self.green_since=None;self.green_count=0

    def signal(self,a,now,*,near_stop=False,expected_x=.5):
        p=self.payload
        self.selection_diagnostics={}
        try:
            source=p['source_stamp_ns']
            if not (p['schema']==1 and p['status']=='OK' and isinstance(source,int) and source>0
                and 0<=(now-source)/1e9<=.5 and self.received is not None and 0<=(now-self.received)/1e9<=.5
                and source>=self.last_source):return 'UNKNOWN',None,'MISSING_STALE_OR_UNORDERED_EVIDENCE'
            if p['source_topic']!=a['camera_topic'] or p['frame_id']!=a['camera_frame_id'] or p['image_size']!=a['image_size']:return 'UNKNOWN',None,'CAMERA_BINDING_MISMATCH'
            if a.get('association_id')==MODE:
                state,diagnostics=self.candidate_selector.select(p['detections'],a,source,near_stop=near_stop,expected_x=expected_x)
                self.selection_diagnostics=diagnostics
                return state or 'UNKNOWN',source,diagnostics['selection_reason']
            w,h=p['image_size'];x1,y1,x2,y2=a['roi_xyxy_norm'];states=[];boxes=[]
            if not all(finite(v) and v>0 for v in (w,h)):return 'UNKNOWN',None,'INVALID_IMAGE_SIZE'
            for d in p['detections']:
                b=d['box_xyxy'];conf=d['confidence']
                if len(b)!=4 or not all(finite(v) for v in b) or not finite(conf) or not 0<=conf<=1:continue
                if not 0<=b[0]<b[2]<=w or not 0<=b[1]<b[3]<=h:continue
                if x1*w<=b[0] and y1*h<=b[1] and b[2]<=x2*w and b[3]<=y2*h:
                    boxes.append(b)
                    states.append({'red':'RED','yellow':'YELLOW','green':'GREEN'}.get(d['class_name'],'UNKNOWN') if conf>=a.get('min_confidence',.25) else 'UNKNOWN')
            state='RED' if 'RED' in states else 'YELLOW' if 'YELLOW' in states else 'GREEN' if states and set(states)=={'GREEN'} else 'UNKNOWN'
            return state,source,'ROI_EVIDENCE' if states else 'NO_TARGET_DETECTION'
        except (KeyError,TypeError,ValueError):return 'UNKNOWN',None,'INVALID_EVIDENCE'

    def decision(self,enabled,route,progress,speed,now,heading_diff=0.,route_node_ids=None,ego_yaw=None):
        context=dict(signal_id=None,state='UNKNOWN',distance_to_stopline_m=None,front_distance_to_stopline_m=None,
            distance_to_stop_m=None,waiting=False,phase='DISABLED',binding_failure_reason=None,
            route_association_verified=False,stop_feasible=None,required_stop_distance_m=None)
        context.update(self.crosswalk.diagnostics(now))
        def result(reason,hold=False,cap=None,phase=None):
            return dict(context,enabled=enabled,hold=hold,speed_cap_kmh=cap,signal_speed_cap_kmh=cap,
                reason=reason,phase=phase or context['phase'],waiting=hold,green_frames=self.green_count,
                stop_requested=cap is not None,evidence_count=context.get('red_frames',0),
                stopline=context.get('stop_line_id'),association=context.get('association_id'),
                green_stable_sec=(now-self.green_since)/1e9 if self.green_since is not None else 0.)
        def fault(reason):
            self.reset();context['binding_failure_reason']=reason
            return result(reason,False,None,'UNAVAILABLE')
        if not enabled:
            self.reset()
            if not isinstance(self.config,dict) or self.config.get('verified') is not True:context['binding_failure_reason']='ASSOCIATION_NOT_VERIFIED'
            return result('DISABLED')
        c=self.config
        fallback=isinstance(c,dict) and c.get('mode')==MODE
        context.update(fallback_used=fallback,association_mode=MODE if fallback else 'VERIFIED_SIGNAL_BINDING')
        if not isinstance(c,dict) or (not fallback and c.get('verified') is not True) or not route:return fault('ASSOCIATION_NOT_VERIFIED')
        try:
            digest=route_digest(route['points'])
            if self._route_digest!=digest:
                self.reset();self._route_digest=digest
            if not fallback and c['route_sha256']!=digest:return fault('ROUTE_BINDING_MISMATCH')
            policy=c.get('stop_policy',{})
            if not fallback and policy.get('braking_verified') is not True:return fault('STOPPING_POLICY_NOT_VERIFIED')
            decel=policy.get('deceleration_mps2');reaction=policy.get('reaction_sec')
            if not finite(decel) or not 0<decel<=1.5 or not finite(reaction) or not 0<reaction<=5:return fault('INVALID_STOPPING_POLICY')
            if fallback:
                cache_key=(digest,tuple(route_node_ids or []))
                if self._fallback_cache is None or self._fallback_cache[0]!=cache_key:
                    self._fallback_cache=(cache_key,associations_for_route(c,route,route_node_ids))
                entries=self._fallback_cache[1]
            else:entries=c['associations']
            if not entries or any(not isinstance(a,dict) for a in entries):return fault('NO_VERIFIED_STOP_LINES')
            ordered=sorted(entries,key=lambda a:a['stop_arc_m'])
            if len({entry_key(a) for a in ordered})!=len(ordered):return fault('AMBIGUOUS_SIGNAL_BINDING')
            for a in ordered:
                keys=['stop_arc_m','front_offset_m','buffer_m','clearance_m','approach_m','max_heading_error_deg']
                if (not all(finite(a[k]) and a[k]>=0 for k in keys) or a['buffer_m']<=0 or (not fallback and a['clearance_m']<=0)
                    or a['approach_m']<=0 or not 0<a['max_heading_error_deg']<=45
                    or a['controlled_link_id'] not in route['link_ids'] or a['movement']!=('route_color_fallback' if fallback else 'straight')):return fault('BINDING_GEOMETRY_OR_MOVEMENT_UNVERIFIED')
                if not fallback and any(a.get(k) is not True for k in ['stop_line_binding_verified','front_offset_verified','camera_binding_verified']):return fault('BINDING_EVIDENCE_NOT_VERIFIED')
                resolve(route['points'],a['stop_line_xy'],a['stop_arc_m'])
                roi=a['roi_xyxy_norm']
                if len(roi)!=4 or not all(finite(v) and 0<=v<=1 for v in roi) or not roi[0]<roi[2] or not roi[1]<roi[3]:return fault('INVALID_CAMERA_ROI')
            if not all(finite(v) for v in [progress,speed,now,heading_diff]) or progress<0 or speed<0:return fault('POSE_OR_SPEED_INVALID')
            if self.last_progress is not None:
                dt=(now-self.last_now)/1e9
                if not 0<=dt<=.6 or progress<self.last_progress-1 or progress-self.last_progress>max(2.,speed*dt+1.):return fault('PROGRESS_DISCONTINUITY')
            self.last_progress=progress;self.last_now=now
            # Never brake for an earlier route crossing, including startup after it.
            for entry in ordered:
                if entry['stop_arc_m']-progress-entry['front_offset_m']<=0:
                    self.cleared.add(entry_key(entry))
            a=next((a for a in ordered if entry_key(a) not in self.cleared),None)
            if a is None:return result('ROUTE_STOPLINES_PASSED' if fallback else 'VERIFIED_SIGNALS_CLEARED',phase='CLEARED')
            if self.active_signal!=entry_key(a):
                self.active_signal=entry_key(a);self.reset_green()
                self.candidate_selector=TrafficLightCandidateSelector();self._selected_track_id=None
                self.red_since=None;self.red_count=0;self.red_last=None;self.stable_red=False
                self.sample_source=None;self.sample_color=None;self.last_source=0;self.stop_latched=False
            stop=resolve(route['points'],a['stop_line_xy'],a['stop_arc_m'])['arc_m']
            front_distance=stop-progress-a['front_offset_m'];distance=front_distance-a['buffer_m']
            required=speed*reaction+speed*speed/(2*decel)
            context.update(signal_id=a['signal_id'],route_association_verified=not fallback,
                distance_to_stopline_m=stop-progress,front_distance_to_stopline_m=front_distance,
                distance_to_stop_m=distance,required_stop_distance_m=required,stop_feasible=required<=max(0.,distance),stop_arc_m=stop,
                assumed_deceleration_mps2=decel,assumed_reaction_sec=reaction)
            expected_x=.5
            if fallback and finite(ego_yaw):
                delta=a['route_heading_rad']-ego_yaw
                delta=math.atan2(math.sin(delta),math.cos(delta))
                # Forward-image hint assumes nominal 90-degree horizontal FOV;
                # this is not a calibrated map-signal projection.
                expected_x=.5-.5*math.tan(max(-math.pi/4,min(math.pi/4,delta)))
            state,source,evidence_reason=self.signal(a,now,
                near_stop=distance<=max(3*a['buffer_m'],required),expected_x=expected_x)
            context.update(state=state,evidence_reason=evidence_reason,**self.selection_diagnostics)
            selected_id=self.selection_diagnostics.get('selected_track_id')
            if selected_id is not None:
                if self._selected_track_id is not None and selected_id!=self._selected_track_id:
                    # Different signal heads cannot pool three-frame evidence.
                    self.red_since=None;self.red_count=0;self.reset_green()
                    self.sample_source=None;self.sample_color=None
                self._selected_track_id=selected_id
            context.update(stop_line_id=a.get('stop_line_id',a['signal_id']),
                association_id=a.get('association_id',a['signal_id']),
                map_signal_candidates=a.get('map_signal_candidates',[]),
                route_heading_rad=a.get('route_heading_rad'),
                front_offset_measured=not fallback,stop_margin_m=a['buffer_m'])
            if distance>a['approach_m']:
                self.reset_green();self.red_since=None;self.red_count=0;self.red_last=None
                self.stable_red=False;self.stop_latched=False
                return result('BEFORE_VERIFIED_APPROACH',phase='CRUISE_BEFORE_APPROACH')
            if abs(heading_diff)>a['max_heading_error_deg']:return fault('CAMERA_POSE_OUTSIDE_CALIBRATION')
            # Count distinct camera frames, never repeated Planning ticks. A long
            # gap breaks confirmation even if both surviving samples are RED.
            if source is not None and source!=self.sample_source:
                gap=None if self.sample_source is None else (source-self.sample_source)/1e9
                if state=='RED':
                    if self.sample_color!='RED' or gap is None or not 0<gap<=.5:
                        self.red_since=source;self.red_count=0
                    self.red_count+=1;self.red_last=source;self.reset_green()
                    if self.red_count>=3 and source-self.red_since>=300_000_000:self.stable_red=True
                elif state=='GREEN':
                    self.red_since=None;self.red_count=0
                    if self.sample_color!='GREEN' or gap is None or not 0<gap<=.5:
                        self.reset_green();self.green_since=source
                    self.green_count+=1
                    if self.green_count>=3 and source-self.green_since>=300_000_000:
                        self.stable_red=False;self.stop_latched=False
                else:
                    self.red_since=None;self.red_count=0;self.reset_green()
                self.sample_source=source;self.sample_color=state
            elif source is None:
                self.red_since=None;self.red_count=0;self.reset_green()
            if source is not None:self.last_source=source
            # Brief missing/yellow/green evidence preserves a confirmed RED,
            # but stale RED cannot remain latched indefinitely.
            if self.red_last is None or not 0<=now-self.red_last<=800_000_000:
                self.stable_red=False;self.stop_latched=False
            context.update(red_frames=self.red_count,red_confirmed=self.stable_red)
            correction=self.crosswalk.apply(now=now,state=state,red_confirmed=self.stable_red,
                reference_distance=stop-progress,front_offset=a['front_offset_m'],buffer=a['buffer_m'])
            context.update(correction)
            if not self.stable_red:
                reason=('GREEN_RELEASE' if fallback else 'VERIFIED_GREEN') if state=='GREEN' and self.green_count>=3 else 'RED_CONFIRMING' if state=='RED' else 'NO_CONFIRMED_RED'
                if fallback and state=='UNKNOWN':reason=evidence_reason
                return result(reason,phase='GO_ON_GREEN' if reason in ('VERIFIED_GREEN','GREEN_RELEASE') else 'OBSERVING')
            final_distance=context['final_stop_distance']
            cap=0. if self.stop_latched or final_distance<=.15 else stop_speed_cap(final_distance,decel,reaction)
            if cap<=.05:self.stop_latched=True
            return result(('RED_ACTIVE_STOPLINE' if fallback else 'STOP_ENVELOPE_RED') if state=='RED' else 'RED_HOLD_BRIEF_DROPOUT',
                self.stop_latched,cap,'WAITING_SIGNAL' if self.stop_latched else 'APPROACH_STOP')
        except (KeyError,TypeError,ValueError,OverflowError,StopIteration) as exc:
            reason=str(exc) if isinstance(exc,ValueError) else 'INVALID_ASSOCIATION_OR_EVIDENCE'
            return fault(reason)
