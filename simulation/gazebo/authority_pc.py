"""One route request and neutral pilot keepalive. No pose subscription or steering."""
import argparse,csv,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray,String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from qcopter_autonomy.srv import RequestRoute
from pc_station import packet

class PC(Node):
 def __init__(self,a):
  super().__init__('proof_ground_pc');self.a=a;self.begin=None;self.mode=0;self.sent=False;self.last=-1
  self.pub=self.create_publisher(UInt8MultiArray,'pilot/xbox_report',10)
  self.client=self.create_client(RequestRoute,'mission/route')
  self.create_subscription(UInt8MultiArray,'fc/status',self.status,10)
  self.create_subscription(Clock,'/clock',self.tick,10)
  self.f=Path(a.out,'proof-pc.jsonl').open('w',encoding='utf8')
  self.log(dict(role='ground PC',subscriptions=['/clock','fc/status'],policy='Down+B until FC acknowledges; then identical neutral HID forever; one approved route; no pose/camera/motor/setpoint access'))
  print('PC READY',flush=True)
 def log(self,d):self.f.write(json.dumps(d)+'\n');self.f.flush()
 def status(self,m):
  if len(m.data)==16:self.mode=m.data[3]
 def tick(self,m):
  t=m.clock.sec+m.clock.nanosec*1e-9
  if t<=self.last:return
  self.last=t
  if self.begin is None:self.begin=t
  selecting=t-self.begin<5.4 or (self.mode!=1 and not self.sent)
  raw=packet(down=True,b=True) if selecting else packet()
  self.pub.publish(UInt8MultiArray(data=list(raw)));self.log(dict(time_s=t,hid=raw.hex(),selecting=selecting))
  if not selecting and not self.sent and self.client.service_is_ready():
   self.sent=True;req=RequestRoute.Request();route=json.loads(Path(self.a.route).read_text());req.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in route['points']];req.cruise_height_m=float(route['cruise'])
   self.log(dict(route_request=route));future=self.client.call_async(req)
   def done(f):
    r=f.result();self.log(dict(accepted=r.accepted,reason=r.reason));print('ROUTE RESPONSE',r.accepted,r.reason,flush=True)
   future.add_done_callback(done)
  if t-self.begin>8 and int(t*50)%250==0:
   self.log(dict(time_s=t,publishers=[dict(node=p.node_name,namespace=p.node_namespace) for p in self.get_publishers_info_by_topic('fc/companion_tx')]))

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--route',required=True);a,ros=p.parse_known_args();rclpy.init(args=ros);n=PC(a)
 try:rclpy.spin(n)
 finally:n.f.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
