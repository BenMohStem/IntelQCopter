"""Isolated native Gazebo + production ROS2 + production C FC proof."""
import argparse,json,subprocess,time,hashlib,re
from pathlib import Path
from datetime import datetime,timezone
from run_suite import environment
from runtime_paths import ROOT,PIXI,ROS_MANIFEST,ROS_PYTHON,GZ_MANIFEST,GZ_PYTHON,PANEL_MODEL

def main():
 p=argparse.ArgumentParser();p.add_argument('--drop-companion-after',type=float,default=0);p.add_argument('--run',required=True);p.add_argument('--route',required=True);p.add_argument('--timeout',default=400,type=float);a=p.parse_args()

 if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',a.run):raise SystemExit('Plain unique run name required')
 out=ROOT/'artifacts/gazebo-suite'/a.run;out.mkdir(parents=True,exist_ok=False)
 route=json.loads(Path(a.route).read_text());(out/'approved-route.json').write_text(json.dumps(route,indent=2)+'\n')
 ros=environment([PIXI,'run','--manifest-path',ROS_MANIFEST,'cmd','/d','/c','tools\\ros2\\ros2_windows.cmd']);ros['ROS_DOMAIN_ID']='73'
 gz=environment([PIXI,'run','--manifest-path',GZ_MANIFEST]);procs=[];logs=[];commands=[]
 def launch(label,command,env,marker=None):
  f=(out/(label+'.log')).open('w',encoding='utf8');logs.append(f);commands.append(dict(role=label,argv=list(map(str,command))))
  pr=subprocess.Popen(command,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));procs.append(pr)
  if marker:
   end=time.monotonic()+100
   while marker not in (out/(label+'.log')).read_text(encoding='utf8',errors='replace'):
    if pr.poll() is not None or time.monotonic()>end:raise RuntimeError(label+' startup failed; preserved log')
    time.sleep(.2)
  return pr
 try:
  engine=launch('engine',[GZ_PYTHON,'-u','simulation/gazebo/engine.py','--env','env01','--run',a.run],gz,'READY GAZEBO')
  args=['--ros-args','-r','__ns:=/drone','-p','use_sim_time:=true'];exe=ROOT/'build/ros2-install/lib/qcopter_autonomy'
  for label,target,extra,marker in [('drone-mission','pilot_mission_node',[],'READY:'),('drone-yolo','yolo_node',['-p',f'onnx_model:={PANEL_MODEL}','-p','input_size:=320'],'READY_RGB:'),('drone-pnp','panel_pose_node',['-p','known_fixture_model_confirmed:=true'],'READY_PNP:')]:launch(label,[str(exe/(target+'.exe')),*args,*extra],ros,marker)
  launch('proof-pc',[ROS_PYTHON,'-u','simulation/gazebo/authority_pc.py','--out',str(out),'--route',str(out/'approved-route.json'),*args],ros,'PC READY')
  sources=['firmware/src/flight_controller.cpp','firmware/include/fc_api.h','simulation/gazebo/engine.py','simulation/gazebo/ros_bridge.py','simulation/gazebo/authority_pc.py','companion/ros2_ws/src/qcopter_autonomy/src/pilot_mission_node.cpp','companion/ros2_ws/src/qcopter_autonomy/src/navigation.cpp','build/sitl/esp32_flight_core.dll','build/ros2-install/lib/qcopter_autonomy/pilot_mission_node.exe']
  (out/'authority-manifest.json').write_text(json.dumps(dict(engine='Native Gazebo 10.5 / OGRE2',fc='Production C host DLL; not Xtensa emulation',localization='Perfect Gazebo odometry; synthetic IMU/barometer and calibration',perception='Actual YOLO + known-panel PnP; no true obstacle coordinates supplied to navigator',domain=73,commands=commands,sha256={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}),indent=2))
  boundary=launch('ros-boundary',[ROS_PYTHON,'-u','simulation/gazebo/ros_bridge.py','--run',a.run,'--timeout',str(a.timeout),'--authority-proof','--drop-companion-after',str(a.drop_companion_after),*args],ros)
  manifest=json.loads((out/'authority-manifest.json').read_text());manifest.update(commands=commands,deliberate_UART_drop_after_s=a.drop_companion_after);(out/'authority-manifest.json').write_text(json.dumps(manifest,indent=2))
  end=time.monotonic()+a.timeout*4+120
  while boundary.poll() is None:
   if time.monotonic()>end:raise TimeoutError('Proof wall timeout; retained original trial')
   time.sleep(.5)
  engine.wait(timeout=30);print('PROOF boundary exit',boundary.returncode,'data',out,flush=True)
 finally:
  for pr in reversed(procs):
   if pr.poll() is None:pr.terminate()
  for pr in procs:
   try:pr.wait(timeout=5)
   except subprocess.TimeoutExpired:pr.kill()
  for f in logs:f.close()
if __name__=='__main__':main()
