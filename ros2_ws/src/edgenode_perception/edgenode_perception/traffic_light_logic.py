"""YOLO evidence only: image colors never imply route ownership or drive permission."""
import math


COLORS={'red':'RED','yellow':'YELLOW','green':'GREEN'}


def detections_from_result(result,names,width,height,threshold):
    if result.boxes is None:return []
    rows=[]
    for box,cls,score in zip(result.boxes.xyxy.cpu().tolist(),result.boxes.cls.cpu().tolist(),result.boxes.conf.cpu().tolist()):
        if not all(math.isfinite(v) for v in [*box,cls,score]) or not 0<=score<=1 or score<threshold:continue
        name=names.get(int(cls),str(int(cls)))
        x1,y1,x2,y2=box
        if not 0<=x1<x2<=width or not 0<=y1<y2<=height:continue
        rows.append(dict(class_id=int(cls),class_name=name,confidence=score,box_xyxy=box))
    return rows


def image_state(detections):
    states={COLORS.get(d['class_name'],'UNKNOWN') for d in detections}
    if not states or 'UNKNOWN' in states:return 'UNKNOWN'
    if 'RED' in states:return 'RED'
    if 'YELLOW' in states:return 'YELLOW'
    return 'GREEN'


def fresh(source_ns,receive_ns,now_ns,timeout_s):
    return (isinstance(source_ns,int) and source_ns>0 and isinstance(receive_ns,int)
            and all(isinstance(v,(int,float)) and math.isfinite(v) for v in (now_ns,timeout_s))
            and timeout_s>0 and 0<=(now_ns-source_ns)/1e9<=timeout_s
            and 0<=(now_ns-receive_ns)/1e9<=timeout_s)
