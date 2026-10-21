"""Predetermined raw-HID step commands; no state-feedback pilot or ROS2.
Only the C FC uses the explicitly labelled simulated localization sensor input.
"""
import argparse,json,subprocess,time,csv,math,hashlib
from pathlib import Path
from multiprocessing.connection import Client
import numpy as np
import cv2
from run_suite import environment
from runtime_paths import ROOT,PIXI,GZ_MANIFEST,GZ_PYTHON,ffmpeg
import struct
def raw_axis(value):
 value=max(-1,min(1,value));v=0 if value==0 else math.copysign(.12+.88*abs(value),value)
 return max(0,min(65535,round(32768+32768*v)))
def packet(roll=0,pitch=0,yaw=0,vertical=0,lt=0,rt=0,down=False,a=False,b=False):
 return struct.pack('<6H4B',raw_axis(yaw),raw_axis(-vertical),raw_axis(roll),raw_axis(-pitch),round(1023*lt),round(1023*rt),5 if down else 0,(1 if a else 0)|(2 if b else 0),0,0)


PHASES=[('select-stable',5.4,dict(down=True,a=True)),('takeoff',3,dict(vertical=.25)),('hover',2,{}),('pitch-angle',1.2,dict(pitch=5/35)),('pitch-release-brake-hold',8,{}),('roll-angle',1.2,dict(roll=5/35)),('roll-release-brake-hold',8,{}),('yaw-left',1,dict(yaw=-.25)),('yaw-left-release',2,{}),('yaw-right',1,dict(yaw=.25)),('yaw-right-release',2,{}),('descend',4,dict(vertical=-.25)),('disarm',2.4,dict(down=True,lt=1))]
def main():
 p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--no-aiding',action='store_true');p.add_argument('--long-angle',action='store_true');p.add_argument('--open-space',action='store_true');a=p.parse_args();
 global PHASES
 if a.long_angle:PHASES=[(n,4. if n in ('pitch-angle','roll-angle') else 14. if 'release-brake' in n else d,v) for n,d,v in PHASES]
 out=ROOT/'artifacts/gazebo-suite'/a.run;out.mkdir(parents=True,exist_ok=False)
 env=environment([PIXI,'run','--manifest-path',GZ_MANIFEST]);log=(out/'engine.log').open('w',encoding='utf8');command=[GZ_PYTHON,'-u','simulation/gazebo/engine.py','--env','env01','--run',a.run,'--standalone','--interactive']+(['--no-horizontal-aiding'] if a.no_aiding else [])+(['--test-open-space'] if a.open_space else []);engine=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));client=rec=None
 try:
  end=time.monotonic()+100
  while 'READY GAZEBO' not in (out/'engine.log').read_text(encoding='utf8',errors='replace'):
   if engine.poll() is not None or time.monotonic()>end:raise RuntimeError('Engine startup failed; log preserved')
   time.sleep(.2)
  client=Client(('127.0.0.1',9251),authkey=b'intelqcopter-esp32');client.send(dict(op='state'));state=client.recv();next_video=0;clock=0;rows=[];wire=(out/'predetermined-inputs.jsonl').open('w',encoding='utf8');err=(out/'ffmpeg.log').open('w')
  rec=subprocess.Popen([ffmpeg(),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','15','-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','27','-pix_fmt','yuv420p','-g','30','-movflags','+frag_keyframe+empty_moov+default_base_moof',str(out/'gazebo.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=err)
  for name,duration,values in PHASES:
   # All commands and durations are predetermined; NEVER derived from state.
   raw=packet(**values)
   for i in range(round(duration/.02)):
    client.send(dict(op='step',hid=[raw],companion=[]));state=client.recv();clock+=.02;wire.write(json.dumps(dict(time_s=clock,phase=name,HID=raw.hex(),companion=[]))+'\n')
    rows.append(dict(time_s=clock,phase=name,phase_time_s=i*.02,position=state['position'],velocity=state['velocity'],telemetry=state['telemetry'],quaternion=state['quaternion']))
    if clock+1e-8>=next_video:
     canvas=np.full((540,960,3),[20,28,40],np.uint8)
     for topic,x,y in [('/overview/rgb',0,0),('/drone/rgb',640,0)]:
      client.send(dict(op='camera',topic=topic));w,h,data=client.recv();canvas[y:y+h,x:x+w]=np.frombuffer(data,np.uint8).reshape(h,w,3)
     texts=[f'Native Gazebo / C FC only | {name} | t={clock:.2f}s',f'Predetermined HID; no feedback pilot; aiding={not a.no_aiding}',f'XY hold={state["telemetry"]["xy_hold_active"]} | C commands roll={state["telemetry"]["roll_command"]:.2f} pitch={state["telemetry"]["pitch_command"]:.2f}']
     for j,text in enumerate(texts):cv2.putText(canvas,text,(15,390+j*35),cv2.FONT_HERSHEY_SIMPLEX,.5,(225,240,245),1,cv2.LINE_AA)
     rec.stdin.write(canvas.tobytes());next_video+=1/15
   print('PHASE',name,'position',state['position'],'velocity',state['velocity'],flush=True)
  wire.close();rec.stdin.close();rec.wait(timeout=30);rec=None;err.close();client.send(dict(op='close'));client.recv();client.close();client=None;engine.wait(timeout=30)
  (out/'controller-trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf8')
  def phase(n,last=None):
   rs=[r for r in rows if r['phase']==n];return rs[-round(last/.02):] if last else rs
  def pitch(r):q=r['quaternion'];return math.degrees(math.asin(max(-1,min(1,2*(q[3]*q[1]-q[2]*q[0])))))
  def roll(r):q=r['quaternion'];return math.degrees(math.atan2(2*(q[3]*q[0]+q[1]*q[2]),1-2*(q[0]**2+q[1]**2)))
  metrics={};checks={}
  for axis,angle in [('pitch',pitch),('roll',roll)]:
   rs=phase(axis+'-release-brake-hold');last=rs[-1];target=[last['telemetry']['xy_target_x'],last['telemetry']['xy_target_y']] if not a.no_aiding else rows[rows.index(rs[0])-1]['position'][:2];speed=math.hypot(*last['velocity'][:2]);error=math.dist(last['position'][:2],target);metric=dict(release_speed_mps=math.hypot(*rs[0]['velocity'][:2]),final_speed_mps=speed,release_point_error_m=error,final_angle_deg=angle(last),held_angle_mean_deg=float(np.mean([angle(r) for r in phase(axis+'-angle',.3)])),max_braking_angle_deg=max(abs(angle(r)) for r in rs))
   metrics[axis]=metric;checks[axis+'_angle_tracks_5deg']=abs(metric['held_angle_mean_deg']-5)<2;checks[axis+'_brakes_and_holds']=speed<.12 and error<.15 and abs(metric['final_angle_deg'])<1.5
  # Body yaw sign is checked from unwrapped true quaternion heading, not HUD text.
  def yaw(r):q=r['quaternion'];return math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
  for name,sign in [('yaw-left',1),('yaw-right',-1)]:
   rs=phase(name);delta=math.degrees(np.unwrap([yaw(r) for r in rs])[-1]-np.unwrap([yaw(r) for r in rs])[0]);metrics[name]=dict(delta_deg=delta);checks[name+'_correct_sign']=sign*delta>10
  checks['C_only_no_ROS2_or_companion']=state['mapped_input_packets']==0
  checks['final_disarm']=state['disarm'];checks['all_hold_sensor_labels_explicit']=a.no_aiding or 'Perfect Gazebo' in state['localization_source']
  engine_summary=json.loads((out/'engine-summary.json').read_text(encoding='utf8'));checks['zero_native_obstacle_contacts']=engine_summary['obstacle_contact_steps']==0
  result=dict(open_space=a.open_space,long_angle=a.long_angle,native_obstacle_contacts=engine_summary['obstacle_contact_steps'],passed=all(checks.values()),expected_negative_control=a.no_aiding,checks=checks,metrics=metrics,engine='Native Gazebo 10.5 / OGRE2 and actual C host DLL',localization=state['localization_source'],policy='Fixed predetermined HID sequence, no feedback pilot and no ROS2. Only C FC computes braking commands from synthetic localization.',source_sha256=hashlib.sha256((ROOT/'firmware/src/flight_controller.c').read_bytes()).hexdigest(),phases=PHASES)
  (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 finally:
  if rec:rec.stdin.close();rec.wait(timeout=30)
  if client:
   try:client.send(dict(op='close'));client.recv();client.close()
   except (OSError,EOFError):pass
  if engine.poll() is None:engine.terminate();engine.wait(timeout=10)
  log.close()
 if not a.no_aiding and not result['passed']:raise SystemExit(2)
 if a.no_aiding and result['checks']['pitch_brakes_and_holds'] and result['checks']['roll_brakes_and_holds']:raise SystemExit('Ablation unexpectedly held both positions')
if __name__=='__main__':main()
