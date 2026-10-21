"""Local PC map selector -> real ROS2 mission/request service.
Run in the ROS environment with namespace /drone; open http://127.0.0.1:8086.
"""
import json,threading,math,argparse,time,zlib
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from qcopter_autonomy.srv import RequestMission
from qcopter_autonomy.msg import MetricObstacles
from std_msgs.msg import String,UInt8MultiArray

HTML='''<!doctype html><meta charset="utf-8"><title>IntelQCopter mission map</title>
<style>body{font:17px system-ui;background:#eef3f7;color:#17324d;max-width:1040px;margin:30px auto}canvas{background:white;border-radius:12px;max-width:100%}button,input{padding:10px;font:inherit}#status{white-space:pre-wrap}label{margin-right:18px}</style>
<h1>PC → drone ROS2 mission</h1><p>First click: start. Second click: finish. Coordinates are metres in the drone's local <b>map</b> frame. Select navigation on Xbox with Down+B for 5 seconds before sending.</p>
<canvas id="map" width="1000" height="500"></canvas><p id="points">Click two points.</p><p>Teal: drone planned path · Orange: observed panel estimates · Purple: live pose. Unobserved space is unknown.</p>
<label>Ground z <input id="z" type="number" value="0.082" step="0.01" style="width:90px"></label><label>Cruise height <input id="h" type="number" value="1.25" step="0.05" style="width:90px"></label>
<button onclick="points=[pose.slice()];draw()">Use live pose as start</button> <button onclick="points=[];draw()">Reset points</button> <button onclick="send()">Send endpoints over ROS2</button><p id="telemetry"></p><p id="status"></p>
<script>let points=[],pose=[0,0],planned=[],obstacles=[];const c=document.getElementById('map'),ctx=c.getContext('2d');function xy(p){return[(p[0]+2)*80,250-p[1]*60]}function draw(){ctx.clearRect(0,0,1000,500);ctx.strokeStyle='#d6e1e7';ctx.font='13px system-ui';ctx.fillStyle='#567';for(let x=-2;x<=10;x++){let a=xy([x,0]);ctx.beginPath();ctx.moveTo(a[0],0);ctx.lineTo(a[0],500);ctx.stroke();ctx.fillText(x+'m',a[0]+3,493)}for(let y=-4;y<=4;y++){let a=xy([0,y]);ctx.beginPath();ctx.moveTo(0,a[1]);ctx.lineTo(1000,a[1]);ctx.stroke()}ctx.strokeStyle='#00a1b3';ctx.lineWidth=2;ctx.beginPath();planned.forEach((p,i)=>{let a=xy(p);i?ctx.lineTo(...a):ctx.moveTo(...a)});ctx.stroke();ctx.lineWidth=1;for(const o of obstacles){let a=xy(o);ctx.fillStyle='#de873555';ctx.beginPath();ctx.ellipse(...a,o[2]*80,o[2]*60,0,0,7);ctx.fill()}for(let i=0;i<points.length;i++){let a=xy(points[i]);ctx.fillStyle=i?'#dc7437':'#007f87';ctx.beginPath();ctx.arc(...a,8,0,7);ctx.fill();ctx.fillText(i?'Finish':'Start',a[0]+10,a[1])}let a=xy(pose);ctx.fillStyle='#773fb4';ctx.fillRect(a[0]-5,a[1]-5,10,10);document.getElementById('points').textContent=JSON.stringify(points)}c.onclick=e=>{if(points.length==2)points=[];const r=c.getBoundingClientRect(),x=(e.clientX-r.left)*1000/r.width,y=(e.clientY-r.top)*500/r.height;points.push([Math.round((x/80-2)*10)/10,Math.round(((250-y)/60)*10)/10]);draw()};async function send(){if(points.length!=2)return;let d={start:[...points[0],+document.getElementById('z').value],finish:[...points[1],+document.getElementById('z').value],cruise:+document.getElementById('h').value};let r=await fetch('/mission',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});document.getElementById('status').textContent=await r.text()}setInterval(async()=>{try{let r=await(await fetch('/state')).json();pose=r.position.slice(0,2);planned=r.path;obstacles=r.obstacles;document.getElementById('telemetry').textContent='FC: '+JSON.stringify(r.fc);draw()}catch{}},1000);draw();</script>'''
class GroundStation(Node):
 def __init__(self):
    super().__init__('pc_map_mission_client');self.position=[0,0,.082];self.path=[];self.observed=[];self.fc={'mode':'unknown','pilot_fresh':False,'disarmed':False};self.client=self.create_client(RequestMission,'mission/request')
    self.create_subscription(Odometry,'odometry',lambda m:setattr(self,'position',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),10)
    self.create_subscription(String,'mission/path',self.planned_path,10)
    self.create_subscription(MetricObstacles,'perception/metric_obstacles',self.observations,10)
    self.create_subscription(UInt8MultiArray,'fc/status',self.fc_status,10)
 def fc_status(self,m):
    p=bytes(m.data)
    if len(p)==16 and p[:3]==b'QS\x01' and zlib.crc32(p[:12])==int.from_bytes(p[12:],'little'):
        self.fc=dict(mode='navigation' if p[3]==1 else 'stable',pilot_fresh=bool(p[5]),disarmed=bool(p[4]))
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
    if not self.client.service_is_ready():return {'accepted':False,'reason':'Drone mission service is unavailable'}
    req=RequestMission.Request();req.start.x,req.start.y,req.start.z=map(float,data['start']);req.finish.x,req.finish.y,req.finish.z=map(float,data['finish']);req.cruise_height_m=float(data['cruise'])
    event=threading.Event();future=self.client.call_async(req);future.add_done_callback(lambda f:event.set())
    if not event.wait(5):return {'accepted':False,'reason':'ROS2 service response timed out'}
    reply=future.result();return {'accepted':reply.accepted,'reason':reply.reason}
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--log');args,ros=parser.parse_known_args()
 rclpy.init(args=ros);node=GroundStation()
 class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        data=node.position if self.path=='/pose' else dict(position=node.position,path=node.path,obstacles=node.observed,fc=node.fc)
        is_json=self.path in ('/pose','/state')
        body=json.dumps(data).encode() if is_json else HTML.encode()
        self.send_response(200);self.send_header('Content-Type','application/json' if is_json else 'text/html; charset=utf-8');self.end_headers();self.wfile.write(body)
    def do_POST(self):
        if self.path!='/mission':self.send_error(404);return
        if self.headers.get('Origin','http://127.0.0.1:8086') not in ('http://127.0.0.1:8086','http://localhost:8086') or self.headers.get('Content-Type','').split(';')[0]!='application/json':self.send_error(403);return
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=2048:raise ValueError('Invalid request size')
            data=json.loads(self.rfile.read(size));result=node.request(data)
            if args.log:
                with Path(args.log).open('a',encoding='utf8') as log:log.write(json.dumps(dict(wall_time_s=time.time(),role='ground PC endpoint map',request=data,response=result))+'\n')
        except (ValueError,KeyError,TypeError) as e:result={'accepted':False,'reason':str(e)}
        self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(result).encode())
 server=ThreadingHTTPServer(('127.0.0.1',8086),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
 print('PC map available at http://127.0.0.1:8086; endpoints travel over ROS2 DDS.',flush=True)
 try:rclpy.spin(node)
 finally:server.shutdown();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
