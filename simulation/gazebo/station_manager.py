"""Persistent local launcher, experiment editor and original recording library."""
import argparse,json,os,re,subprocess,threading,time,urllib.request,urllib.error,webbrowser,mimetypes
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse,unquote,parse_qs
from experiment_settings import defaults,validate,fields
from runtime_paths import ROOT,CONFIG

def main():
 p=argparse.ArgumentParser();p.add_argument('--stop',action='store_true');a=p.parse_args()
 if a.stop:
  req=urllib.request.Request('http://127.0.0.1:8088/stop',data=b'{}',headers={'Content-Type':'application/json'});print(urllib.request.urlopen(req,timeout=5).read().decode());return
 local=ROOT/'.local';local.mkdir(exist_ok=True);profile=local/'station-settings.json';sessions=ROOT/'artifacts/gazebo-suite';proc=None;log=None;lock=threading.Lock();active=''
 def live():
  try:
   with urllib.request.urlopen('http://127.0.0.1:8087/state',timeout=.5) as r:return json.load(r)
  except (OSError,ValueError):return None
 class Handler(BaseHTTPRequestHandler):
  def log_message(self,*args):pass
  def reply(self,data,code=200,kind='application/json'):
   body=data if isinstance(data,bytes) else json.dumps(data).encode();self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)))
   origin=self.headers.get('Origin','')
   if urlparse(self.path).path in ('/recording-status','/status') and origin=='http://127.0.0.1:8087':self.send_header('Access-Control-Allow-Origin',origin)
   self.end_headers()
   try:self.wfile.write(body)
   except (BrokenPipeError,ConnectionResetError):pass
  def do_GET(self):
   path=unquote(urlparse(self.path).path)
   if path=='/':self.reply(Path(__file__).with_name('station_manager.html').read_bytes(),kind='text/html; charset=utf-8');return
   if path=='/launching':self.reply(Path(__file__).with_name('launching.html').read_bytes(),kind='text/html; charset=utf-8');return
   if path=='/recording-status':
    run=parse_qs(urlparse(self.path).query).get('run',[''])[0]
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',run):self.reply(dict(error='Invalid run'),400);return
    marker=sessions/run/'recording-complete.json'
    self.reply(json.loads(marker.read_text(encoding='utf8')) if marker.exists() else dict(complete=False,run=run));return
   if path=='/settings':
    try:settings=validate(json.loads(profile.read_text())) if profile.exists() else defaults()
    except ValueError:settings=defaults()
    self.reply(dict(settings=settings,defaults=defaults(),fields=fields()));return
   if path=='/status':
    state=live();self.reply(dict(running=proc is not None and proc.poll() is None,live=state is not None,run=active,returncode=proc.poll() if proc else None,state=state));return
   if path=='/recordings':
    runs=[]
    for d in sorted(sessions.glob('*'),key=lambda p:p.stat().st_mtime,reverse=True):
     if not d.is_dir() or not (d/'gazebo.mp4').is_file():continue
     files=[f.name for f in d.iterdir() if f.is_file() and f.suffix in ('.mp4','.csv','.json','.jsonl','.log')]
     runs.append(dict(name=d.name,complete=any((d/n).exists() for n in ('transport.json','live-summary.json','engine-summary.json')),files=sorted(files)))
    self.reply(runs);return
   if path.startswith('/recordings/'):
    parts=path.split('/');allowed={'.mp4','.csv','.json','.jsonl','.log'}
    if len(parts)!=4 or any(x in ('.','..','') for x in parts[2:]):self.reply({},404);return
    target=(sessions/parts[2]/parts[3]).resolve()
    if target.parent.parent!=sessions.resolve() or target.suffix not in allowed or not target.is_file():self.reply({},404);return
    # Bounded streaming with HTTP Range for the browser video player.
    size=target.stat().st_size;start=0;end=size-1;partial=False
    try:
     if self.headers.get('Range'):
      spec=self.headers['Range'];assert spec.startswith('bytes=') and ',' not in spec
      x,y=spec[6:].split('-');start=int(x);end=min(size-1,int(y)) if y else size-1;assert 0<=start<=end<size;partial=True
    except (ValueError,AssertionError):self.reply({},416);return
    self.send_response(206 if partial else 200);self.send_header('Content-Type',mimetypes.guess_type(target.name)[0] or 'application/octet-stream');self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
    if partial:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
    self.end_headers()
    try:
     with target.open('rb') as f:
      f.seek(start);remaining=end-start+1
      while remaining:
       b=f.read(min(1024*1024,remaining))
       if not b:break
       self.wfile.write(b);remaining-=len(b)
    except (BrokenPipeError,ConnectionResetError):pass
    return
   if path in ('/manual-proof/','/manual-proof/summary.json','/manual-proof/control-graphs.png'):
    file='index.html' if path=='/manual-proof/' else path.rsplit('/',1)[1];body=(ROOT/'artifacts/reports/manual-control-20261021'/file).read_bytes()
    if file=='index.html':body=body.replace(b'../../gazebo-suite/',b'/recordings/')
    self.reply(body,kind=mimetypes.guess_type(file)[0] or 'application/octet-stream');return
   if path in ('/proof/','/proof/summary.json','/proof/trajectory-and-control.png'):
    file='index.html' if path=='/proof/' else path.rsplit('/',1)[1];target=ROOT/'artifacts/reports/authority-20261021'/file;body=target.read_bytes()
    if file=='index.html':body=body.replace(b'../../gazebo-suite/',b'/recordings/')
    self.reply(body,kind=mimetypes.guess_type(file)[0] or 'application/octet-stream');return
   if path in ('/guide','/control-workbench.md','/current-model-and-fc-integration.html','/current-model-and-fc-integration.md'):
    file='control-workbench.html' if path=='/guide' else path[1:];self.reply((ROOT/'docs'/file).read_bytes(),kind='text/html; charset=utf-8' if file.endswith('.html') else 'text/plain; charset=utf-8');return
   self.reply({},404)
  def do_POST(self):
   nonlocal proc,log,active
   if self.headers.get('Origin','http://127.0.0.1:8088') not in ('http://127.0.0.1:8088','http://localhost:8088'):self.reply(dict(error='Foreign origin'),403);return
   try:
    n=int(self.headers.get('Content-Length','0'))
    if n<0 or n>32768:raise ValueError('Request too large')
    data=json.loads(self.rfile.read(n) or b'{}')
    with lock:
     if self.path=='/settings':
      settings=validate(data);temp=profile.with_suffix('.tmp');temp.write_text(json.dumps(settings,indent=2)+'\n');temp.replace(profile);self.reply(dict(message='Saved for next start. Current flight unchanged.'));return
     if self.path=='/start':
      if (proc and proc.poll() is None) or live() is not None:raise ValueError('A session already runs. End and save it first.')
      env=data.get('environment','env01');mode=data.get('mode','manual')
      if env not in [f'env{i:02}' for i in range(1,11)] or mode not in ('manual','navigation'):raise ValueError('Invalid environment/mode')
      settings=validate(data['settings']);active='managed-'+time.strftime('%Y%m%d-%H%M%S')+'-'+env
      # An immutable per-launch input; live_session copies it into the original run.
      snap=local/(active+'-settings.json');snap.write_text(json.dumps(settings,indent=2)+'\n')
      command=[CONFIG.get('python') or __import__('sys').executable,'-u',str(ROOT/'simulation/gazebo/live_session.py'),'--env',env,'--run',active,'--settings',str(snap)]
      if mode=='navigation':command.append('--with-companion')
      if log:log.close()
      log=(local/(active+'-launcher.log')).open('w',encoding='utf8');proc=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));self.reply(dict(message='Starting; cameras and ROS2 may take up to 90 seconds.',run=active));return
     if self.path=='/stop':
      if live() is None:raise ValueError('No ready flight to stop. Wait for startup or inspect launcher log.')
      req=urllib.request.Request('http://127.0.0.1:8087/stop',data=b'{}',headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8087'})
      with urllib.request.urlopen(req,timeout=5) as r:result=json.load(r)
      self.reply(dict(message='Saving original MP4, telemetry and logs; wait until stopped before starting again.'));return
     self.reply({},404)
   except (ValueError,KeyError,TypeError,OSError) as e:self.reply(dict(error=str(e)),409)
 try:server=ThreadingHTTPServer(('127.0.0.1',8088),Handler)
 except OSError:
  with urllib.request.urlopen('http://127.0.0.1:8088/settings',timeout=3) as r:assert 'fields' in json.load(r)
  print('Station already available at http://127.0.0.1:8088');return
 print('STATION MANAGER READY http://127.0.0.1:8088',flush=True)
 try:server.serve_forever()
 except KeyboardInterrupt:pass
 finally:server.server_close()
if __name__=='__main__':main()
