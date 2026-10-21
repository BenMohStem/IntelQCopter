"""Actual ROS2 C++ mission process fault injection; no flight physics or hardware."""
import argparse,os,json,subprocess,time,struct,zlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray,String,Bool
from qcopter_autonomy.msg import MetricObstacles
from qcopter_autonomy.srv import RequestRoute
from geometry_msgs.msg import Point

ROOT=Path(__file__).resolve().parents[2]
def main():
 p=argparse.ArgumentParser();p.add_argument('--name',required=True);a=p.parse_args()
 out=ROOT/'artifacts/validation'/a.name;out.mkdir(parents=True,exist_ok=False)
 os.environ['ROS_DOMAIN_ID']='94';rclpy.init();n=Node('mission_fault_injection');log=(out/'mission-original.log').open('w',encoding='utf8')
 process=subprocess.Popen([str(ROOT/'build/ros2-install/lib/qcopter_autonomy/pilot_mission_node.exe'),'--ros-args','-r','__ns:=/drone','-p','debug_level:=3'],stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
 checks=[];frames=[];profiles=[];wire=[]
 op=n.create_publisher(Odometry,'/drone/odometry',10);sp=n.create_publisher(UInt8MultiArray,'/drone/fc/status',10);mp=n.create_publisher(MetricObstacles,'/drone/perception/metric_obstacles',10);cp=n.create_publisher(Bool,'/drone/simulation/ground_contact',10)
 n.create_subscription(UInt8MultiArray,'/drone/fc/companion_tx',lambda m:frames.append((time.monotonic(),bytes(m.data))),10)
 n.create_subscription(String,'/drone/debug/profile',lambda m:profiles.append(m.data),10)
 client=n.create_client(RequestRoute,'/drone/mission/route')
 def check(name,value):checks.append(dict(name=name,passed=bool(value)));print(('PASS ' if value else 'FAIL ')+name,flush=True)
 def status(mode=1):
  b=bytearray([81,83,1,mode,0,1,0,1,0,0,0,0]);b+=struct.pack('<I',zlib.crc32(b));sp.publish(UInt8MultiArray(data=list(b)))
 def pose(kind='valid'):
  m=Odometry();m.header.frame_id=m.child_frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.position.z=.082;m.pose.pose.orientation.w=1.
  if kind=='NaN':m.pose.pose.position.x=float('nan')
  if kind=='nonunit':m.pose.pose.orientation.w=2.
  if kind=='future':m.header.stamp.sec+=10
  if kind=='old':m.header.stamp.sec-=10
  op.publish(m);wire.append(dict(wall_s=time.time(),pose_case=kind));return m
 def pump(seconds,kind=None,fc=True,camera=False,mode=1):
  end=time.monotonic()+seconds;last=0
  while time.monotonic()<end:
   if time.monotonic()-last>.02:
    last=time.monotonic()
    if fc:status(mode)
    if kind:pose(kind)
    if camera:
     m=MetricObstacles();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='map';m.image_processed=True;mp.publish(m);cp.publish(Bool(data=True))
   rclpy.spin_once(n,timeout_sec=.005)
 def request():
  r=RequestRoute.Request();r.points=[Point(x=0.,y=0.,z=.082),Point(x=3.,y=0.,z=.082)];r.cruise_height_m=1.25
  f=client.call_async(r);end=time.monotonic()+3
  while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
  if not f.done():raise RuntimeError('Mission service timed out')
  res=f.result();wire.append(dict(wall_s=time.time(),accepted=res.accepted,reason=res.reason));return res.accepted
 try:
  if not client.wait_for_service(timeout_sec=20):raise RuntimeError('Production C++ mission service not ready')
  pump(.5,fc=True)
  check('Route rejected without odometry',not request())
  for kind in ['NaN','nonunit','future','old']:
   pump(.15,kind);check('Route rejects '+kind+' odometry',not request())
  pump(.2,'valid',camera=True);check('Fresh valid route accepted',request())
  pump(.5,'valid',camera=True);check('Actual C++ mission publishes UART commands',len(frames)>5)
  check('All published command frames pass CRC and finite payload',all(len(b)==28 and zlib.crc32(b[:24])==struct.unpack('<I',b[24:])[0] and all(__import__('math').isfinite(x) for x in struct.unpack('<4f',b[8:24])) for _,b in frames))
  pump(1.0,'valid',fc=False,camera=True);cut=len(frames);pump(.3,'valid',fc=False,camera=True)
  check('Stale FC status stops companion commands',len(frames)==cut)
  pump(.2,'valid',mode=0,camera=True);cut=len(frames);pump(.2,'valid',mode=0,camera=True)
  check('Stable mode cancels mission and sends no commands',len(frames)==cut)
  pump(.2,'valid',camera=True);pump(.35,kind=None,camera=True);check('Stale last pose cannot start a mission',not request())
  pump(2.2,'NaN',camera=True);check('Wall-clock diagnostics published with rejection counts',any('clock=steady_wall' in x and 'rejected=0,' not in x for x in profiles))
 finally:
  process.terminate();process.wait(timeout=10);log.close();n.destroy_node();rclpy.shutdown()
 (out/'injection-journal.json').write_text(json.dumps(wire,indent=2),encoding='utf8')
 (out/'profiles.json').write_text(json.dumps(profiles,indent=2),encoding='utf8')
 (out/'uart-frames.json').write_text(json.dumps([dict(wall_monotonic=t,frame=b.hex()) for t,b in frames]),encoding='utf8')
 result=dict(passed=all(x['passed'] for x in checks),checks=checks,scope=__doc__);(out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf8');return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
