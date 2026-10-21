"""Replay recorded human HID, unchanged, through the actual standalone C FC/plant.
No pose feedback enters the replay policy. Pose is used only for recording.
"""
import argparse,json,subprocess,time,hashlib
from pathlib import Path
from multiprocessing.connection import Client
import numpy as np
import cv2
from run_suite import environment
from runtime_paths import ROOT,PIXI,GZ_MANIFEST,GZ_PYTHON,ffmpeg

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',default='station-ready-15fps-20261021');p.add_argument('--run',required=True);p.add_argument('--seconds',type=float,default=45);a=p.parse_args()
 import re
 if not all(re.fullmatch(r'[A-Za-z0-9_-]{1,80}',s) for s in [a.source,a.run]):raise ValueError('Plain run names required')
 src=ROOT/'artifacts/gazebo-suite'/a.source;out=ROOT/'artifacts/gazebo-suite'/a.run;out.mkdir(parents=True,exist_ok=False)
 records=[json.loads(line) for line in (src/'wire-inputs.jsonl').read_text(encoding='utf8').splitlines()];records=[r for r in records if r['time_s']-records[0]['time_s']<a.seconds]
 if any(r['companion'] for r in records):raise ValueError('Manual proof requires a source interval with no companion commands')
 env=environment([PIXI,'run','--manifest-path',GZ_MANIFEST]);log=(out/'engine.log').open('w',encoding='utf8');command=[GZ_PYTHON,'-u','simulation/gazebo/engine.py','--env','env01','--run',a.run,'--standalone','--interactive'];engine=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));rec=None;client=None
 try:
  end=time.monotonic()+100
  while 'READY GAZEBO' not in (out/'engine.log').read_text(encoding='utf8',errors='replace'):
   if engine.poll() is not None or time.monotonic()>end:raise RuntimeError('Engine startup failed; see log')
   time.sleep(.2)
  client=Client(('127.0.0.1',9251),authkey=b'intelqcopter-esp32');client.send({'op':'state'});state=client.recv();next_video=0;start=state['sim_time_s'];trace=(out/'replayed-wire.jsonl').open('w',encoding='utf8');err=(out/'ffmpeg.log').open('w')
  rec=subprocess.Popen([ffmpeg(),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','15','-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','27','-pix_fmt','yuv420p','-g','30','-movflags','+frag_keyframe+empty_moov+default_base_moof',str(out/'gazebo.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=err)
  for r in records:
   # The record index is the only replay clock/policy input. No target, pose,
   # velocity, motor command, mission or collision state changes these reports.
   hid=[bytes.fromhex(h) for h in r['xbox']];client.send(dict(op='step',hid=hid,companion=[]));state=client.recv();trace.write(json.dumps(dict(original_time_s=r['time_s'],replay_time_s=state['sim_time_s'],source=r['input_source'],xbox=r['xbox'],companion=[]))+'\n')
   t=state['sim_time_s']-start
   if t>=next_video:
    canvas=np.full((540,960,3),[20,28,40],np.uint8)
    for topic,x,y in [('/overview/rgb',0,0),('/drone/rgb',640,0)]:
     client.send(dict(op='camera',topic=topic));w,h,data=client.recv();canvas[y:y+h,x:x+w]=np.frombuffer(data,np.uint8).reshape(h,w,3)
    for i,text in enumerate(['Recorded REAL human HID replay -> standalone C FC -> native Gazebo',f'No ROS2, no companion frames, no feedback pilot | t={t:.2f}s',f'Original mapped HID is unchanged; physical output disabled | FC calls={state["telemetry"]["fc_calls"]:.0f}']):cv2.putText(canvas,text,(15,390+i*35),cv2.FONT_HERSHEY_SIMPLEX,.48,(225,240,245),1,cv2.LINE_AA)
    rec.stdin.write(canvas.tobytes());next_video+=1/15
  trace.close();rec.stdin.close();rec.wait(timeout=30);rec=None;err.close();client.send(dict(op='close'));client.recv();client.close();client=None;engine.wait(timeout=30)
  result=dict(engine='Native Gazebo 10.5 / OGRE2 + standalone C host DLL',recording='Replay of original human XInput/mapped HID; not a newly live human flight',ROS2_processes=0,companion_frames=0,raw_reports_replayed=sum(len(r['xbox']) for r in records),reports_unchanged=True,feedback_policy=False,video_fps=15,source=a.source,simulated_seconds=state['sim_time_s']-start,final_state={k:v for k,v in state.items() if k!='fc'},source_sha256={n:hashlib.sha256((src/n).read_bytes()).hexdigest() for n in ['wire-inputs.jsonl','real-xbox.csv','operator-actions.jsonl']},command=command,limitations='Synthetic sensors/calibration, assumed vehicle and C host DLL; not physical BLE/ESC or Xtensa timing. PC mode gesture remains explicitly labelled in original/replayed wire.')
  (out/'manual-replay.json').write_text(json.dumps(result,indent=2)+'\n');print('MANUAL REPLAY SAVED',out,flush=True)
 finally:
  if rec:
   rec.stdin.close();rec.wait(timeout=30)
  if client:
   try:client.send(dict(op='close'));client.recv();client.close()
   except (OSError,EOFError):pass
  if engine.poll() is None:engine.terminate();engine.wait(timeout=10)
  log.close()
if __name__=='__main__':main()
