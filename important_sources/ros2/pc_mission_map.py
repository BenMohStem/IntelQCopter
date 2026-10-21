"""Local PC map selector -> real ROS2 mission/request service.
Run in the ROS environment with namespace /drone; open http://127.0.0.1:8086.
"""
import json,threading,math,argparse,time,zlib,urllib.request,urllib.error
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from qcopter_autonomy.srv import RequestMission,RequestRoute
from geometry_msgs.msg import Point
from qcopter_autonomy.msg import MetricObstacles
from std_msgs.msg import String,UInt8MultiArray

HTML=Path(__file__).with_name('route_planner.html').read_text(encoding='utf8')

class GroundStation(Node):
 def __init__(self,simulation_controls=False):
    super().__init__('pc_map_mission_client');self.simulation_controls=simulation_controls;self.mission='';self.position=[0,0,.082];self.path=[];self.observed=[];self.fc={'mode':'unknown','pilot_fresh':False,'disarmed':False};self.client=self.create_client(RequestMission,'mission/request');self.route_client=self.create_client(RequestRoute,'mission/route')
    self.create_subscription(Odometry,'odometry',lambda m:setattr(self,'position',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),10)
    self.create_subscription(String,'mission/status',lambda m:setattr(self,'mission',m.data),10)
    self.create_subscription(String,'mission/path',self.planned_path,10)
    self.create_subscription(MetricObstacles,'perception/metric_obstacles',self.observations,10)
    self.create_subscription(UInt8MultiArray,'fc/status',self.fc_status,10)
 def fc_status(self,m):
    p=bytes(m.data)
    if len(p)==16 and p[:3]==b'QS\x01' and zlib.crc32(p[:12])==int.from_bytes(p[12:],'little'):
        self.fc=dict(mode='navigation' if p[3]==1 else 'stable',pilot_fresh=bool(p[5]),disarmed=bool(p[4]),companion_enabled=bool(p[7]&1))
 def planned_path(self,m):
    try:
        points=[[float(v) for v in point.split(',')] for point in m.data.split(';') if point]
        if len(points)<=1000 and all(len(p)==2 and all(math.isfinite(v) for v in p) for p in points):self.path=points
    except ValueError:pass
 def observations(self,m):
    if m.header.frame_id!='map' or len(m.centers)!=len(m.radii_m):return
    for p,r in zip(m.centers,m.radii_m):
        if not all(math.isfinite(v) for v in [p.x,p.y,r]) or not 0<r<3:continue
        match=next((i for i,o in enumerate(self.observed) if math.hypot(o[0]-p.x,o[1]-p.y)<.75),None)
        value=[p.x,p.y,float(r)]
        if match is not None:self.observed[match]=value
        elif len(self.observed)<100:self.observed.append(value)
 def request(self,data):
    if 'points' in data:
        points=data['points']
        if not isinstance(points,list) or not 2<=len(points)<=64:return {'accepted':False,'reason':'Choose 2 to 64 nodes; first is start, last is finish'}
        for i,p in enumerate(points):
            if not isinstance(p,list) or len(p)!=3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in p):return {'accepted':False,'reason':f'Node {i+1}: finite x,y,z required'}
            if not -1<=p[0]<=9 or abs(p[1])>3:return {'accepted':False,'reason':f'Node {i+1}: x={p[0]}, y={p[1]} outside x=-1..9 m, y=-3..3 m'}
            if i and math.hypot(p[0]-points[i-1][0],p[1]-points[i-1][1])<.4:return {'accepted':False,'reason':f'Nodes {i} and {i+1} must be at least 0.4 m apart'}
        if math.hypot(points[0][0]-self.position[0],points[0][1]-self.position[1])>.25:return {'accepted':False,'reason':'First node is not at the drone; use drone as start'}
        client=self.route_client;req=RequestRoute.Request()
        req.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points]
    else:
        client=self.client;req=RequestMission.Request();req.start.x,req.start.y,req.start.z=map(float,data['start']);req.finish.x,req.finish.y,req.finish.z=map(float,data['finish'])
    if data.get('select_navigation'):
        if not self.simulation_controls:return {'accepted':False,'reason':'PC mode selection is disabled outside the simulator'}
        if self.fc.get('mode')!='navigation':
            try:
                request=urllib.request.Request('http://127.0.0.1:8087/action',data=b'{"action":"navigation"}',headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(request,timeout=3) as response:answer=json.load(response)
                if not answer.get('accepted'):return answer
            except (OSError,urllib.error.URLError) as error:
                reason=error.read().decode() if isinstance(error,urllib.error.HTTPError) else str(error)
                return {'accepted':False,'reason':'Cannot select simulator navigation: '+reason}
            end=time.monotonic()+25
            while self.fc.get('mode')!='navigation' and time.monotonic()<end:time.sleep(.05)
            if self.fc.get('mode')!='navigation':return {'accepted':False,'reason':'FC did not acknowledge navigation selection; inspect Xbox and gesture progress'}
    if not client.service_is_ready():return {'accepted':False,'reason':'Drone mission service is unavailable'}
    req.cruise_height_m=float(data['cruise'])
    event=threading.Event();future=client.call_async(req);future.add_done_callback(lambda f:event.set())
    if not event.wait(5):return {'accepted':False,'reason':'ROS2 service response timed out'}
    reply=future.result();return {'accepted':reply.accepted,'reason':reply.reason}
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--log');parser.add_argument('--simulation-controls',action='store_true');args,ros=parser.parse_known_args()
 rclpy.init(args=ros);node=GroundStation(args.simulation_controls)
 class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        data=node.position if self.path=='/pose' else dict(position=node.position,path=node.path,obstacles=node.observed,fc=node.fc,mission=node.mission,simulation_controls=node.simulation_controls)
        is_json=self.path in ('/pose','/state')
        body=json.dumps(data).encode() if is_json else HTML.encode()
        self.send_response(200);self.send_header('Content-Type','application/json' if is_json else 'text/html; charset=utf-8');self.end_headers();self.wfile.write(body)
    def do_POST(self):
        if self.path not in ('/mission','/reset'):self.send_error(404);return
        if self.headers.get('Origin','http://127.0.0.1:8086') not in ('http://127.0.0.1:8086','http://localhost:8086') or self.headers.get('Content-Type','').split(';')[0]!='application/json':self.send_error(403);return
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=16384:raise ValueError('Invalid request size')
            data=json.loads(self.rfile.read(size))
            if self.path=='/reset':
                if not node.simulation_controls:raise ValueError('Reset is only available in simulation')
                request=urllib.request.Request('http://127.0.0.1:8087/reset',data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})
                try:
                    with urllib.request.urlopen(request,timeout=3) as response:result=json.load(response)
                except urllib.error.HTTPError as error:result=json.loads(error.read())
            else:result=node.request(data)
            if args.log:
                with Path(args.log).open('a',encoding='utf8') as log:log.write(json.dumps(dict(wall_time_s=time.time(),role='ground PC endpoint map',request=data,response=result))+'\n')
        except (ValueError,KeyError,TypeError,IndexError) as e:result={'accepted':False,'reason':str(e)}
        self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(result).encode())
 server=ThreadingHTTPServer(('127.0.0.1',8086),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
 print('PC map available at http://127.0.0.1:8086; endpoints travel over ROS2 DDS.',flush=True)
 try:rclpy.spin(node)
 finally:server.shutdown();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
