"""Local native-camera viewer. It has no FC setpoint or motor command endpoint."""
import json,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import cv2

HTML='''<!doctype html><meta charset="utf-8"><title>IntelQCopter live Gazebo</title>
<style>body{background:#101d29;color:#dcebf4;font:17px system-ui;max-width:1200px;margin:25px auto;padding:0 15px}img{width:100%;background:#172c3c;border-radius:14px}pre{white-space:pre-wrap;background:#1b3244;padding:18px;border-radius:12px}a{color:#66d8e1}button{font:inherit;padding:12px;border-radius:8px}aside{color:#e7c991}</style>
<h1>Live Xbox → ESP32 C flight core → Gazebo</h1><p>Native Gazebo cameras. Physical ESC output stays disabled. This is a host build of the C core, not ESP32 CPU emulation.</p>
<img id="camera" alt="Waiting for native Gazebo frames"><pre id="state">Connecting…</pre>
<p><a href="http://127.0.0.1:8086" target="_blank">Open ROS2 mission map</a> · <button onclick="fetch('/stop',{method:'POST'})">End and save this session</button></p>
<aside>Down+A 5 s: stable. Down+B 5 s: navigation (companion build only). Left stick: yaw/vertical speed; right stick: roll/pitch. RT boost, LT reduce. Down+LT 2 s: latched disarm. After selecting stable, move left stick upward to begin simulated thrust. Restart the session after a disarm. No controller → no generated reports.</aside>
<script>setInterval(async()=>{try{document.getElementById('camera').src='/frame.jpg?t='+Date.now();let r=await(await fetch('/state')).json();document.getElementById('state').textContent=JSON.stringify(r,null,2)}catch(e){document.getElementById('state').textContent='Session ended or viewer unavailable'}},200)</script>'''

class Viewer:
    def __init__(self,port=8087):
        self.stopped=threading.Event();self.lock=threading.Lock();self.state={};self.jpeg=b'';owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                with owner.lock:
                    if self.path.startswith('/frame.jpg'):body=owner.jpeg;kind='image/jpeg'
                    elif self.path=='/state':body=json.dumps(owner.state).encode();kind='application/json'
                    else:body=HTML.encode();kind='text/html; charset=utf-8'
                self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.end_headers()
                try:self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_POST(self):
                origin=self.headers.get('Origin',f'http://127.0.0.1:{port}')
                if self.path!='/stop' or origin not in (f'http://127.0.0.1:{port}',f'http://localhost:{port}'):self.send_error(403);return
                owner.stopped.set();self.send_response(200);self.end_headers();self.wfile.write(b'Saving original logs and video.')
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler);threading.Thread(target=self.server.serve_forever,daemon=True).start()
        print(f'LIVE VIEW READY http://127.0.0.1:{port}',flush=True)
    def update(self,rgb,state):
        ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,85])
        with self.lock:
            self.state=state
            if ok:self.jpeg=encoded.tobytes()
    def close(self):self.server.shutdown();self.server.server_close()
