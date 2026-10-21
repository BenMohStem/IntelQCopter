"""Local PC map selector -> real ROS2 mission/request service.
Run in the ROS environment with namespace /drone; open http://127.0.0.1:8086.
"""
import json,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from qcopter_autonomy.srv import RequestMission

HTML='''<!doctype html><meta charset="utf-8"><title>IntelQCopter mission map</title>
<style>body{font:17px system-ui;background:#eef3f7;color:#17324d;max-width:1040px;margin:30px auto}canvas{background:white;border-radius:12px;max-width:100%}button,input{padding:10px;font:inherit}#status{white-space:pre-wrap}label{margin-right:18px}</style>
<h1>PC → drone ROS2 mission</h1><p>First click: start. Second click: finish. Coordinates are metres in the drone's local <b>map</b> frame. Select navigation on Xbox with Down+B for 5 seconds before sending.</p>
<canvas id="map" width="1000" height="500"></canvas><p id="points">Click two points.</p>
<label>Ground z <input id="z" type="number" value="0.082" step="0.01" style="width:90px"></label><label>Cruise height <input id="h" type="number" value="1.25" step="0.05" style="width:90px"></label>
<button onclick="points=[];draw()">Reset points</button> <button onclick="send()">Send endpoints over ROS2</button><p id="status"></p>
<script>let points=[],pose=[0,0];const c=document.getElementById('map'),ctx=c.getContext('2d');function xy(p){return[(p[0]+2)*80,250-p[1]*60]}function draw(){ctx.clearRect(0,0,1000,500);ctx.strokeStyle='#d6e1e7';ctx.font='13px system-ui';ctx.fillStyle='#567';for(let x=-2;x<=10;x++){let a=xy([x,0]);ctx.beginPath();ctx.moveTo(a[0],0);ctx.lineTo(a[0],500);ctx.stroke();ctx.fillText(x+'m',a[0]+3,493)}for(let y=-4;y<=4;y++){let a=xy([0,y]);ctx.beginPath();ctx.moveTo(0,a[1]);ctx.lineTo(1000,a[1]);ctx.stroke()}for(let i=0;i<points.length;i++){let a=xy(points[i]);ctx.fillStyle=i?'#dc7437':'#007f87';ctx.beginPath();ctx.arc(...a,8,0,7);ctx.fill();ctx.fillText(i?'Finish':'Start',a[0]+10,a[1])}let a=xy(pose);ctx.fillStyle='#773fb4';ctx.fillRect(a[0]-5,a[1]-5,10,10);document.getElementById('points').textContent=JSON.stringify(points)}c.onclick=e=>{if(points.length==2)points=[];const r=c.getBoundingClientRect(),x=(e.clientX-r.left)*1000/r.width,y=(e.clientY-r.top)*500/r.height;points.push([Math.round((x/80-2)*10)/10,Math.round(((250-y)/60)*10)/10]);draw()};async function send(){if(points.length!=2)return;let d={start:[...points[0],+document.getElementById('z').value],finish:[...points[1],+document.getElementById('z').value],cruise:+document.getElementById('h').value};let r=await fetch('/mission',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});document.getElementById('status').textContent=await r.text()}setInterval(async()=>{try{let r=await(await fetch('/pose')).json();pose=r.slice(0,2);draw()}catch{}},1000);draw();</script>'''
class GroundStation(Node):
 def __init__(self):
    super().__init__('pc_map_mission_client');self.position=[0,0,.082];self.client=self.create_client(RequestMission,'mission/request')
    self.create_subscription(Odometry,'odometry',lambda m:setattr(self,'position',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),10)
 def request(self,data):
    if not self.client.service_is_ready():return {'accepted':False,'reason':'Drone mission service is unavailable'}
    req=RequestMission.Request();req.start.x,req.start.y,req.start.z=map(float,data['start']);req.finish.x,req.finish.y,req.finish.z=map(float,data['finish']);req.cruise_height_m=float(data['cruise'])
    event=threading.Event();future=self.client.call_async(req);future.add_done_callback(lambda f:event.set())
    if not event.wait(5):return {'accepted':False,'reason':'ROS2 service response timed out'}
    reply=future.result();return {'accepted':reply.accepted,'reason':reply.reason}
def main():
 rclpy.init();node=GroundStation()
 class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body=json.dumps(node.position).encode() if self.path=='/pose' else HTML.encode()
        self.send_response(200);self.send_header('Content-Type','application/json' if self.path=='/pose' else 'text/html; charset=utf-8');self.end_headers();self.wfile.write(body)
    def do_POST(self):
        if self.path!='/mission':self.send_error(404);return
        if self.headers.get('Origin','http://127.0.0.1:8086') not in ('http://127.0.0.1:8086','http://localhost:8086') or self.headers.get('Content-Type','').split(';')[0]!='application/json':self.send_error(403);return
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=2048:raise ValueError('Invalid request size')
            result=node.request(json.loads(self.rfile.read(size)))
        except (ValueError,KeyError,TypeError) as e:result={'accepted':False,'reason':str(e)}
        self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(result).encode())
 server=ThreadingHTTPServer(('127.0.0.1',8086),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
 print('PC map available at http://127.0.0.1:8086; endpoints travel over ROS2 DDS.',flush=True)
 try:rclpy.spin(node)
 finally:server.shutdown();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
