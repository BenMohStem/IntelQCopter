"""Local PC map selector -> real ROS2 mission/request service.
Run in the ROS environment with namespace /drone; open http://127.0.0.1:8086.
"""
import json,threading,math,argparse,time,zlib
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from qcopter_autonomy.srv import RequestMission,RequestRoute
from geometry_msgs.msg import Point
from qcopter_autonomy.msg import MetricObstacles
from std_msgs.msg import String,UInt8MultiArray

HTML='<!doctype html><meta charset="utf-8"><title>IntelQCopter route planner</title>\n<style>body{font:17px system-ui;background:#edf3f7;color:#17324d;max-width:1080px;margin:25px auto;padding:0 15px}canvas{background:white;width:100%;border-radius:12px}button,input,select{padding:10px;font:inherit}pre{white-space:pre-wrap}a{color:#007c91}</style>\n<h1>PC → drone ROS2 route</h1><p><a href="http://127.0.0.1:8087" target="_blank">Live flight cameras</a> · Select navigation on Xbox: Down+B held 5 seconds.</p>\n<label>Route <select id="count" onchange="points=[];draw()"><option value="2">Start → finish</option><option value="3">Start → middle → finish</option></select></label>\n<button onclick="points=[pose.slice()];draw()">Use live pose as start</button><button onclick="points=[];draw()">Clear</button>\n<canvas id="map" width="1000" height="500"></canvas><p id="points"></p>\n<p>Teal: planned path · Orange: observed panel estimates · Purple: current pose. Unobserved space is unknown. The middle node is flown at cruise height; landing occurs only at the finish.</p>\n<label>Ground z <input id="z" type="number" value="0.082" step="0.01" style="width:90px"></label>\n<label>Cruise z <input id="h" type="number" value="1.25" step="0.05" style="width:90px"></label>\n<button onclick="send()">Send route over ROS2</button><pre id="telemetry"></pre><pre id="status"></pre>\n<script>\nlet points=[],pose=[0,0],planned=[],obstacles=[];\nconst c=document.getElementById(\'map\'),ctx=c.getContext(\'2d\'),count=()=>+document.getElementById(\'count\').value;\nfunction xy(p){return[(p[0]+2)*80,250-p[1]*60]}\nfunction draw(){ctx.clearRect(0,0,1000,500);ctx.strokeStyle=\'#d6e1e7\';ctx.font=\'13px system-ui\';ctx.fillStyle=\'#567\';\nfor(let x=-2;x<=10;x++){let a=xy([x,0]);ctx.beginPath();ctx.moveTo(a[0],0);ctx.lineTo(a[0],500);ctx.stroke();ctx.fillText(x+\'m\',a[0]+3,493)}\nfor(let y=-4;y<=4;y++){let a=xy([0,y]);ctx.beginPath();ctx.moveTo(0,a[1]);ctx.lineTo(1000,a[1]);ctx.stroke()}\nctx.strokeStyle=\'#00a1b3\';ctx.lineWidth=2;ctx.beginPath();planned.forEach((p,i)=>{let a=xy(p);i?ctx.lineTo(...a):ctx.moveTo(...a)});ctx.stroke();ctx.lineWidth=1;\nfor(let o of obstacles){let a=xy(o);ctx.fillStyle=\'#de873555\';ctx.beginPath();ctx.ellipse(...a,o[2]*80,o[2]*60,0,0,7);ctx.fill()}\npoints.forEach((p,i)=>{let a=xy(p);ctx.fillStyle=i===0?\'#007f87\':i===count()-1?\'#dc7437\':\'#b08d00\';ctx.beginPath();ctx.arc(...a,8,0,7);ctx.fill();ctx.fillText(i===0?\'Start\':i===count()-1?\'Finish\':\'Middle\',a[0]+10,a[1])});\nlet a=xy(pose);ctx.fillStyle=\'#773fb4\';ctx.fillRect(a[0]-5,a[1]-5,10,10);document.getElementById(\'points\').textContent=JSON.stringify(points)+\' · \'+points.length+\'/\'+count()+\' selected\'}\nc.onclick=e=>{if(points.length===count())points=[];let r=c.getBoundingClientRect(),x=(e.clientX-r.left)*1000/r.width,y=(e.clientY-r.top)*500/r.height;points.push([Math.round((x/80-2)*10)/10,Math.round(((250-y)/60)*10)/10]);draw()};\nasync function send(){if(points.length!==count()){document.getElementById(\'status\').textContent=\'Select all route points first\';return}let z=+document.getElementById(\'z\').value,ps=points.map(p=>[...p,z]),d={points:ps,start:ps[0],finish:ps.at(-1),cruise:+document.getElementById(\'h\').value};try{let r=await fetch(\'/mission\',{method:\'POST\',headers:{\'Content-Type\':\'application/json\'},body:JSON.stringify(d)});document.getElementById(\'status\').textContent=await r.text()}catch(e){document.getElementById(\'status\').textContent=String(e)}}\nsetInterval(async()=>{try{let r=await(await fetch(\'/state\')).json();pose=r.position.slice(0,2);planned=r.path;obstacles=r.obstacles;document.getElementById(\'telemetry\').textContent=\'FC: \'+JSON.stringify(r.fc);draw()}catch{}},500);draw();\n</script>'

class GroundStation(Node):
 def __init__(self):
    super().__init__('pc_map_mission_client');self.position=[0,0,.082];self.path=[];self.observed=[];self.fc={'mode':'unknown','pilot_fresh':False,'disarmed':False};self.client=self.create_client(RequestMission,'mission/request');self.route_client=self.create_client(RequestRoute,'mission/route')
    self.create_subscription(Odometry,'odometry',lambda m:setattr(self,'position',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),10)
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
        if len(points) not in (2,3):return {'accepted':False,'reason':'Choose start/end or start/middle/end'}
        client=self.route_client;req=RequestRoute.Request()
        req.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points]
    else:
        client=self.client;req=RequestMission.Request();req.start.x,req.start.y,req.start.z=map(float,data['start']);req.finish.x,req.finish.y,req.finish.z=map(float,data['finish'])
    if not client.service_is_ready():return {'accepted':False,'reason':'Drone mission service is unavailable'}
    req.cruise_height_m=float(data['cruise'])
    event=threading.Event();future=client.call_async(req);future.add_done_callback(lambda f:event.set())
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
        except (ValueError,KeyError,TypeError,IndexError) as e:result={'accepted':False,'reason':str(e)}
        self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(result).encode())
 server=ThreadingHTTPServer(('127.0.0.1',8086),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
 print('PC map available at http://127.0.0.1:8086; endpoints travel over ROS2 DDS.',flush=True)
 try:rclpy.spin(node)
 finally:server.shutdown();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
