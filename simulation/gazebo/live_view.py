"""Local flight station: native camera frames, readiness and explicit sim actions."""
import json,math,queue,threading,time,urllib.request,urllib.error
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse,parse_qs
import cv2

HTML=Path(__file__).with_name('live_dashboard.html').read_text(encoding='utf8')
class Viewer:
    def __init__(self,port=8087,log_path=None):
        self.stopped=threading.Event();self.lock=threading.Lock();self.state={};self.frames={};self.actions=queue.Queue(1);self.log_path=log_path;self.sequence=0;self.profile='gentle';self.restart_spawn=None;owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def reply(self,body,kind='application/json',code=200):
                self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.end_headers()
                try:self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):pass
            def proxy(self,method,data=None):
                path=self.path.removeprefix('/planner');path=path or '/'
                if path not in ('/','/state','/pose','/mission','/reset'):self.reply(b'Unknown route','text/plain',404);return
                try:
                    req=urllib.request.Request('http://127.0.0.1:8086'+path,data=data,method=method,headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8086'})
                    with urllib.request.urlopen(req,timeout=25) as r:self.reply(r.read(),r.headers.get('Content-Type','text/html'))
                except (OSError,urllib.error.URLError) as e:self.reply(json.dumps(dict(accepted=False,reason='ROS2 route editor unavailable: '+str(e))).encode(),code=503)
            def do_GET(self):
                if self.path.startswith('/planner'):self.proxy('GET');return
                parsed=urlparse(self.path)
                if parsed.path=='/live_graphs.js':self.reply(Path(__file__).with_name('live_graphs.js').read_bytes(),'text/javascript; charset=utf-8');return
                with owner.lock:
                    if parsed.path=='/frame.jpg':
                        view=parse_qs(parsed.query).get('view',['overview'])[0];body=owner.frames.get(view,b'');kind='image/jpeg'
                    elif parsed.path=='/state':body=json.dumps(owner.state).encode();kind='application/json'
                    else:body=HTML.encode();kind='text/html; charset=utf-8'
                self.reply(body,kind,200 if body else 503)
            def do_POST(self):
                origin=self.headers.get('Origin',f'http://127.0.0.1:{port}')
                if origin not in (f'http://127.0.0.1:{port}',f'http://localhost:{port}'):self.reply(b'{"reason":"Foreign origin rejected"}',code=403);return
                if self.path=='/stop':owner.stopped.set();self.reply(b'{"message":"Saving video and original logs"}');return
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=16384:raise ValueError('Invalid request size')
                    data=self.rfile.read(size)
                    if self.path in ('/planner/mission','/planner/reset'):self.proxy('POST',data);return
                    if self.path=='/profile':
                        profile=json.loads(data)['profile']
                        if profile not in ('gentle','normal'):raise ValueError('Unknown stick profile')
                        owner.profile=profile;self.reply(b'{"accepted":true}');return
                    if self.path=='/reset':
                        spawn=json.loads(data).get('start',[0,0])
                        if len(spawn)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in spawn) or not (-1<=spawn[0]<=9 and abs(spawn[1])<=3):raise ValueError('Reset location outside x=-1..9, y=-3..3 m')
                        env=owner.state.get('environment','env01')
                        worlds=json.loads(Path(__file__).with_name('environments.json').read_text())
                        spec=next(s for s in worlds if s['id']==env)
                        for x,y,yaw in spec['panels']:
                            dx,dy=spawn[0]-x,spawn[1]-y;c,s=math.cos(yaw),math.sin(yaw)
                            if math.hypot(max(abs(c*dx+s*dy)-.04,0),max(abs(-s*dx+c*dy)-.6,0))<.45:raise ValueError('Reset point overlaps a panel. Move the first node.')
                        owner.restart_spawn=spawn;owner.stopped.set();self.reply(b'{"accepted":true,"message":"Saving current trial and restarting scene at selected start"}');return
                    if self.path!='/action':self.reply(b'{}',code=404);return
                    action=json.loads(data)['action']
                    if action not in ('manual','navigation','disarm'):raise ValueError('Unknown simulator action')
                    with owner.lock:
                        state=dict(owner.state);info=state.get('controller') or json.loads(state.get('pilot') or '{}')
                    if not info.get('connected') or (info.get('wall_time_s') and time.time()-info['wall_time_s']>.5):raise ValueError('Xbox disconnected. Connect it before selecting flight mode.')
                    if state.get('disarmed'):raise ValueError('Disarm is latched. End this session and restart.')
                    if action=='navigation' and state.get('standalone'):raise ValueError('Standalone FC has no navigation computer. Launch play-ros.')
                    if state.get('gesture',{}).get('active'):raise ValueError('A mode gesture is already running. Wait for its progress bar.')
                    owner.actions.put_nowait(action)
                    if owner.log_path:
                        with Path(owner.log_path).open('a',encoding='utf8') as f:f.write(json.dumps(dict(wall_time_s=time.time(),source='explicit PC simulator action',action=action))+'\n')
                    self.reply(json.dumps(dict(accepted=True,message='Simulator '+action+' gesture queued; physical outputs remain disabled')).encode())
                except (ValueError,KeyError,TypeError,queue.Full) as e:self.reply(json.dumps(dict(accepted=False,reason=str(e) or 'An action is already queued')).encode(),code=409)
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler);threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.telemetry_file=(Path(log_path).parent/'telemetry.jsonl').open('a',encoding='utf8') if log_path else None
        self.last_snapshot=0
        print(f'LIVE VIEW READY http://127.0.0.1:{port}',flush=True)
    def next_action(self):
        try:return self.actions.get_nowait()
        except queue.Empty:return None
    def update(self,rgb,state,follow=None,overview=None):
        frames={}
        for name,frame in [('composite',rgb),('overview',rgb[:360,:640] if overview is None else overview),('onboard',rgb[:240,640:960])]:
            ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(frame,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,88])
            if ok:frames[name]=encoded.tobytes()
        if follow is not None:
            ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(follow,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,88])
            if ok:frames['follow']=encoded.tobytes()
        with self.lock:
            self.sequence+=1;self.frames=frames;self.state=dict(state,profile=self.profile,frame_sequence=self.sequence,camera_wall_time_s=time.time())
        if self.telemetry_file:
            self.telemetry_file.write(json.dumps(self.state)+'\n');self.telemetry_file.flush()
            if time.monotonic()-self.last_snapshot>2:
                path=Path(self.log_path).parent/'latest-state.json';temp=path.with_suffix('.tmp');temp.write_text(json.dumps(self.state),encoding='utf8');temp.replace(path);self.last_snapshot=time.monotonic()
    def close(self):
        if self.telemetry_file:self.telemetry_file.close()
        self.server.shutdown();self.server.server_close()
