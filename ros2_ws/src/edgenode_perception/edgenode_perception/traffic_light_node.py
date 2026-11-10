"""Independent camera subscriber; never publishes control commands/parameters."""
import concurrent.futures
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from edgenode_perception.traffic_light_logic import detections_from_result,image_state,fresh


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        for name,value in [('model_path','/workspace/models/traffic_light_yolo/weights.pt'),
            ('camera_topic','/camera/image/compressed'),('device','cpu'),('confidence',.4),
            ('image_size',640),('max_image_age_sec',.5),('inference_hz',5.),('publish_debug',True)]:
            self.declare_parameter(name,value)
        self.pub=self.create_publisher(String,'/perception/traffic_light_detections',10)
        self.debug=self.create_publisher(CompressedImage,'/perception/traffic_light/debug/compressed',10)
        self.pending=None;self.future=None;self.latest=None;self.dropped=0;self.model=None;self.model_error=None
        self.last_source=0;self.sequence=0;self.last_submit=0.;self.names={};self.started=time.monotonic();self.completed=0
        self.pool=concurrent.futures.ThreadPoolExecutor(max_workers=1)
        try:
            path=Path(self.get_parameter('model_path').value)
            if not path.is_file():raise ValueError('MODEL_FILE_MISSING')
            from ultralytics import YOLO
            self.model=YOLO(str(path))
            self.names=dict(self.model.names)
            if self.model.task!='detect':raise ValueError('MODEL_IS_NOT_DETECTION')
            self.get_logger().info('YOLO traffic classes: '+json.dumps(self.names))
        except Exception as exc:
            self.model=None
            self.model_error=type(exc).__name__+': '+str(exc)
            self.get_logger().error('TRAFFIC_MODEL_UNAVAILABLE '+self.model_error)
        self.sub=self.create_subscription(CompressedImage,self.get_parameter('camera_topic').value,self.camera_callback,qos_profile_sensor_data)
        self.timer=self.create_timer(.1,self.tick)

    def camera_callback(self,msg):
        if self.pending is not None:self.dropped+=1
        self.pending=(msg,self.get_clock().now().nanoseconds)

    def infer(self,msg,receive_ns):
        begin=time.monotonic()
        frame=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if frame is None:raise ValueError('IMAGE_DECODE_FAILED')
        h,w=frame.shape[:2]
        result=self.model.predict(frame,imgsz=self.get_parameter('image_size').value,
            conf=self.get_parameter('confidence').value,device=self.get_parameter('device').value,
            verbose=False,save=False)[0]
        rows=detections_from_result(result,self.names,w,h,self.get_parameter('confidence').value)
        infer_ms=(time.monotonic()-begin)*1000
        debug=None
        if self.get_parameter('publish_debug').value:
            for d in rows:
                a,b,c,e=map(int,d['box_xyxy']);color={'red':(0,0,255),'green':(0,255,0),'yellow':(0,255,255)}.get(d['class_name'],(255,255,255))
                cv2.rectangle(frame,(a,b),(c,e),color,2)
                cv2.putText(frame,f"{d['class_name']} {d['confidence']:.2f}",(a,max(15,b-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
            cv2.putText(frame,f'{image_state(rows)} infer={infer_ms:.0f}ms (not route permission)',(5,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
            ok,buf=cv2.imencode('.jpg',frame)
            if ok:debug=CompressedImage(header=msg.header,format='jpeg',data=buf.tobytes())
        return dict(source_stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
            receive_stamp_ns=receive_ns,frame_id=msg.header.frame_id,image_size=[w,h],
            detections=rows,state=image_state(rows),status='OK' if rows else 'NO_DETECTIONS',inference_ms=infer_ms),debug

    def tick(self):
        now=self.get_clock().now().nanoseconds
        if self.future is not None and self.future.done():
            try:
                self.latest,debug=self.future.result();self.completed+=1
                if debug is not None and fresh(self.latest['source_stamp_ns'],self.latest['receive_stamp_ns'],now,self.get_parameter('max_image_age_sec').value):self.debug.publish(debug)
            except Exception as exc:
                self.latest=dict(source_stamp_ns=0,receive_stamp_ns=now,state='UNKNOWN',status='INFERENCE_ERROR',detections=[],error=type(exc).__name__+': '+str(exc))
            self.future=None
        payload=dict(self.latest or dict(source_stamp_ns=0,receive_stamp_ns=now,detections=[],state='UNKNOWN',status='WAITING_FOR_IMAGE'))
        if self.model_error is not None:payload.update(state='UNKNOWN',status='MODEL_UNAVAILABLE',error=self.model_error,detections=[])
        elif not fresh(payload['source_stamp_ns'],payload['receive_stamp_ns'],now,self.get_parameter('max_image_age_sec').value):
            payload.update(state='UNKNOWN',detections=[])
            if payload.get('status')!='INFERENCE_ERROR':payload['status']='STALE_OR_MISSING_IMAGE'
        payload.update(schema=1,publish_stamp_ns=now,source_topic=self.get_parameter('camera_topic').value,
            model_classes=self.names,dropped_frames=self.dropped,route_association_verified=False,
            image_age_ms=(now-payload['source_stamp_ns'])/1e6 if payload['source_stamp_ns'] else None,
            completed_inferences=self.completed,inference_fps=self.completed/max(.001,time.monotonic()-self.started))
        self.pub.publish(String(data=json.dumps(payload,allow_nan=False)))
        hz=self.get_parameter('inference_hz').value
        if self.model is not None and self.future is None and self.pending is not None and hz>0 and time.monotonic()-self.last_submit>=1/hz:
            msg,received=self.pending;self.pending=None
            source=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if fresh(source,received,now,self.get_parameter('max_image_age_sec').value) and source>self.last_source:
                self.last_source=source;self.last_submit=time.monotonic();self.future=self.pool.submit(self.infer,msg,received)

    def destroy_node(self):
        self.pool.shutdown(wait=False,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=TrafficLightNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
