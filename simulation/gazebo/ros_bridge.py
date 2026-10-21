"""ROS2 plant boundary. Carries raw Xbox/UART reports and native sensor products.
No navigation policy, hidden obstacle map, motor commands or mission endpoint writes.
"""
import argparse,csv,json,math,subprocess,time
from pathlib import Path
from multiprocessing.connection import Client
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import UInt8MultiArray,Bool,String
from runtime_paths import ffmpeg as ffmpeg_path

ROOT=Path(__file__).resolve().parents[2]
def stamp(t):
    sec=int(t);return sec,int((t-sec)*1e9)
class Boundary(Node):
 def __init__(self):
    super().__init__('gazebo_sensor_and_uart_boundary');self.hid=[];self.uart=[];self.done=False;self.mission='';self.pilot='';self.semantics=0;self.metric_frames=0
    self.create_subscription(UInt8MultiArray,'pilot/xbox_report',lambda m:self.hid.append(bytes(m.data)),10)
    self.create_subscription(UInt8MultiArray,'fc/companion_tx',lambda m:self.uart.append(bytes(m.data)),10)
    self.create_subscription(Bool,'pilot/completed',lambda m:setattr(self,'done',m.data),10)
    self.create_subscription(String,'mission/status',lambda m:setattr(self,'mission',m.data),10)
    self.create_subscription(String,'pilot/state',lambda m:setattr(self,'pilot',m.data),10)
    from qcopter_autonomy.msg import SemanticDetections,MetricObstacles
    self.trace=None;self.boxes=[];self.box_time=-99.
    self.create_subscription(SemanticDetections,'perception/semantics',self.semantic,10)
    self.create_subscription(MetricObstacles,'perception/metric_obstacles',self.metric,10)
    self.clock=self.create_publisher(Clock,'/clock',10);self.odom=self.create_publisher(Odometry,'odometry',10)
    self.fc=self.create_publisher(UInt8MultiArray,'fc/status',10);self.contact=self.create_publisher(Bool,'simulation/ground_contact',10)
    self.rgb=self.create_publisher(Image,'camera/image',10);self.cam=self.create_publisher(CameraInfo,'camera/camera_info',10)
 def semantic(self,m):
    self.semantics+=1;self.box_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    self.boxes=[dict(class_id=int(d.class_id),confidence=float(d.confidence),xyxy=list(map(float,d.xyxy_pixels))) for d in m.detections]
    if self.trace:self.trace.write(json.dumps(dict(kind='YOLO',image_time_s=self.box_time,detections=self.boxes))+'\n');self.trace.flush()
 def metric(self,m):
    self.metric_frames+=1
    if self.trace:self.trace.write(json.dumps(dict(kind='PnP',image_time_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,image_processed=m.image_processed,frame=m.header.frame_id,centers=[[p.x,p.y,p.z] for p in m.centers],radii=list(map(float,m.radii_m))))+'\n');self.trace.flush()
 def sensors(self,state,rgb=None):
    t=state['sim_time_s'];sec,nsec=stamp(t);clock=Clock();clock.clock.sec=sec;clock.clock.nanosec=nsec;self.clock.publish(clock)
    odom=Odometry();odom.header.stamp=clock.clock;odom.header.frame_id=odom.child_frame_id='map'
    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=state['position']
    q=odom.pose.pose.orientation;q.x,q.y,q.z,q.w=state['quaternion'];odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=state['velocity'];self.odom.publish(odom)
    self.fc.publish(UInt8MultiArray(data=list(state['fc'])));self.contact.publish(Bool(data=state['ground_contact']))
    if rgb is not None:
        image=Image();image.header.stamp=clock.clock;image.header.frame_id='camera_optical';image.height,image.width=rgb.shape[:2];image.encoding='rgb8';image.step=image.width*3;image.data=rgb.tobytes();self.rgb.publish(image)
        info=CameraInfo();info.header=image.header;info.height=image.height;info.width=image.width;f=240/(2*math.tan(math.radians(35)));info.k=[f,0.,160.,0.,f,120.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob';self.cam.publish(info)

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--port',type=int,default=9251);p.add_argument('--timeout',type=float,default=100);args,ros=p.parse_known_args()
 rclpy.init(args=ros);node=Boundary();client=Client(('127.0.0.1',args.port),authkey=b'intelqcopter-esp32')
 out=ROOT/'artifacts/gazebo-suite'/args.run;out.mkdir(parents=True,exist_ok=True)
 node.trace=(out/'perception.jsonl').open('w',encoding='utf8')
 wire=(out/'wire-inputs.jsonl').open('w',encoding='utf8')
 client.send({'op':'state'});state=client.recv();start=state['sim_time_s'];last_camera=last_video=-99.;next_wall=time.monotonic();rows=[];done_at=None;count=0;hid_count=uart_count=0;success=False
 ffmpeg=ffmpeg_path()
 recorder=subprocess.Popen([str(ffmpeg),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','5','-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','27','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'gazebo.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(out/'ffmpeg.log').open('w'))
 rgb=np.zeros((240,320,3),np.uint8);overview=np.zeros((360,640,3),np.uint8)
 print('BOUNDARY READY: real ROS2 DDS; native Gazebo sensors; raw HID and companion UART input only',flush=True)
 try:
  while state['sim_time_s']-start<args.timeout:
    t=state['sim_time_s']
    if t-last_camera>=.099:
        client.send({'op':'camera','topic':'/drone/rgb'});w,h,data=client.recv();rgb=np.frombuffer(data,np.uint8).reshape(h,w,3).copy();last_camera=t;image=rgb
    else:image=None
    node.sensors(state,image)
    # Allow independent DDS processes to handle this clock and publish their inputs.
    deadline=time.monotonic()+.006
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.001)
    hid,uart=node.hid,node.uart;node.hid=[];node.uart=[];hid_count+=len(hid);uart_count+=len(uart)
    wire.write(json.dumps(dict(time_s=t,xbox=[p.hex() for p in hid],companion=[p.hex() for p in uart]))+'\n');wire.flush()
    client.send({'op':'step','hid':hid,'companion':uart});state=client.recv();count+=1
    rows.append([state['sim_time_s'],*state['position']])
    if t-last_video>=.199:
        client.send({'op':'camera','topic':'/overview/rgb'});w,h,data=client.recv();overview=np.frombuffer(data,np.uint8).reshape(h,w,3)
        canvas=np.full((540,960,3),[20,28,40],np.uint8);canvas[:360,:640]=overview;canvas[:240,640:]=rgb
        if 0<=t-node.box_time<.3:
            for box in node.boxes:
                x1,y1,x2,y2=map(int,box['xyxy']);cv2.rectangle(canvas,(640+x1,y1),(640+x2,y2),(70,245,130),1)
                cv2.putText(canvas,f"YOLO panel {box['confidence']:.2f}",(640+x1,max(12,y1)),cv2.FONT_HERSHEY_SIMPLEX,.32,(70,245,130),1)
        texts=[f'Gazebo | {args.run} | t={t-start:.1f}s',f'Xbox mode={state["mode"]} | disarm={state["disarm"]}',f'DDS HID packets={hid_count} | UART={uart_count}',node.mission[:90],f'YOLO frames={node.semantics} | PnP frames={node.metric_frames}']
        for i,text in enumerate(texts):cv2.putText(canvas,text,(15,382+27*i),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,240,245),1,cv2.LINE_AA)
        recorder.stdin.write(canvas.tobytes());last_video=t
        if count%250<10:print(f'PROGRESS t={t-start:.1f}s position={state["position"]} mode={state["mode"]} {node.mission}',flush=True)
    if node.done and done_at is None:done_at=t
    if done_at is not None and t-done_at>.3:break
    next_wall+=.02;time.sleep(max(0,min(.02,next_wall-time.monotonic())))
  success=bool(node.done and state['disarm'] and state['ground_contact'])
  summary=dict(completed=success,timeout=not node.done,simulated_seconds=state['sim_time_s']-start,hid_packets=hid_count,companion_packets=uart_count,yolo_frames=node.semantics,pnp_frames=node.metric_frames,mission_status=node.mission,final_state={k:v for k,v in state.items() if k!='fc'},scope='Native Gazebo physics and cameras; real ROS2 DDS; ESP32 C core host build. C++ mission/YOLO/PnP enabled only for navigation cases. Not Xtensa emulation or physical BLE.')
  (out/'transport.json').write_text(json.dumps(summary,indent=2)+'\n');print('BOUNDARY RESULT',json.dumps(summary),flush=True)
 finally:
  wire.close();node.trace.close()
  recorder.stdin.close();recorder.wait(timeout=30)
  client.send({'op':'close'});client.recv();client.close();node.destroy_node();rclpy.shutdown()
 raise SystemExit(0 if success else 2)
if __name__=='__main__':main()
