"""Launch a live scene with real Xbox input, with or without a ROS2 companion."""
import argparse,json,os,subprocess,time
from datetime import datetime,timezone
from pathlib import Path
from runtime_paths import ROOT,PIXI,GZ_MANIFEST,GZ_PYTHON,ROS_MANIFEST,ROS_PYTHON,PANEL_MODEL,CONFIG
from run_suite import environment

def main():
    p=argparse.ArgumentParser();p.add_argument('--with-companion',action='store_true');p.add_argument('--env',default='env01');p.add_argument('--run');p.add_argument('--controller',type=int,choices=range(4));p.add_argument('--duration',type=float,default=0,help='Optional wall-clock duration for a startup check');p.add_argument('--start',type=float,nargs=2,metavar=('X','Y'),default=[0,0],help='Initial ground location in map metres, selected before physics starts');args=p.parse_args()
    worlds=json.loads((ROOT/'simulation/gazebo/environments.json').read_text());assert args.env in [s['id'] for s in worlds]
    name=args.run or ('live-'+datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')+'-'+args.env)
    if not name.replace('-','').replace('_','').isalnum():raise SystemExit('Plain unique run name required')
    out=ROOT/'artifacts/gazebo-suite'/name;out.mkdir(parents=True,exist_ok=False);processes=[];logs=[]
    def launch(label,command,env=None):
        log=(out/(label+'.log')).open('w',encoding='utf8');logs.append(log)
        proc=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));processes.append(proc);return proc
    def ready(proc,label,marker):
        end=time.monotonic()+90
        while time.monotonic()<end:
            if marker in (out/(label+'.log')).read_text(encoding='utf8',errors='replace'):return
            if proc.poll() is not None:raise RuntimeError(label+' exited; inspect original log')
            time.sleep(.2)
        raise TimeoutError(label+' startup timed out')
    try:
        gzenv=environment([PIXI,'run','--manifest-path',GZ_MANIFEST]);engineargs=[GZ_PYTHON,'-u','simulation/gazebo/engine.py','--env',args.env,'--run',name,'--interactive','--spawn',*map(str,args.start)]
        if not args.with_companion:engineargs+=['--standalone']
        engine=launch('engine',engineargs,gzenv);ready(engine,'engine','READY GAZEBO')
        index=['--controller',str(args.controller)] if args.controller is not None else []
        if args.with_companion:
            rosenv=environment([PIXI,'run','--manifest-path',ROS_MANIFEST,'cmd','/d','/c','tools\\ros2\\ros2_windows.cmd'])
            rosargs=['--ros-args','-r','__ns:=/drone','-p','use_sim_time:=true'];exe=ROOT/'build/ros2-install/lib/qcopter_autonomy'
            for label,target,extra,marker in [('drone-mission','pilot_mission_node',[],'READY:'),('drone-yolo','yolo_node',['-p',f'onnx_model:={PANEL_MODEL}','-p','input_size:=320'],'READY_RGB:'),('drone-pnp','panel_pose_node',['-p','known_fixture_model_confirmed:=true'],'READY_PNP:')]:
                proc=launch(label,[str(exe/(target+'.exe')),*rosargs,*extra],rosenv);ready(proc,label,marker)
            xbox=launch('real-xbox',[ROS_PYTHON,'-u','simulation/gazebo/live_xbox_node.py','--log',str(out/'real-xbox.csv'),*index,*rosargs],rosenv);ready(xbox,'real-xbox','LIVE XBOX READY')
            pc=launch('pc-map',[ROS_PYTHON,'-u','companion/pc_mission_map.py','--log',str(out/'pc-map-missions.jsonl'),'--simulation-controls',*rosargs],rosenv);ready(pc,'pc-map','PC map available')
            boundary=launch('live-boundary',[ROS_PYTHON,'-u','simulation/gazebo/ros_bridge.py','--run',name,'--timeout','86400','--live','--duration',str(args.duration),*rosargs],rosenv)
        else:
            vision=CONFIG.get('python') or os.environ.get('QCOPTER_PYTHON') or __import__('sys').executable
            boundary=launch('live-boundary',[vision,'-u','simulation/gazebo/live_standalone.py','--run',name,'--duration',str(args.duration),*index])
        ready(boundary,'live-boundary','LIVE VIEW READY')
        print('Play at http://127.0.0.1:8087 | missions at :8086 when companion enabled',flush=True)
        print('Down+A 5s stable; then left-stick up starts simulation thrust. Down+B 5s navigation. Physical motor outputs remain disabled.',flush=True)
        while boundary.poll() is None:
            dead=[p.returncode for p in processes if p not in (boundary,engine) and p.poll() is not None]
            if engine.poll() not in (None,0):dead.append(engine.returncode)
            if dead:raise RuntimeError('A required session process stopped; see original logs')
            time.sleep(.2)
        engine.wait(timeout=20)
        if boundary.returncode:raise RuntimeError('Live boundary failed; inspect live-boundary.log')
        print('Session saved:',out,flush=True)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:proc.terminate()
        for proc in processes:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        for log in logs:log.close()
    if (out/'reset-request.json').exists():
        import sys
        spawn=json.loads((out/'reset-request.json').read_text())['start']
        next_name='live-reset-'+datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')+'-'+args.env
        command=[sys.executable,'-u',str(Path(__file__).resolve()),'--env',args.env,'--run',next_name,'--start',*[format(float(v),'.6f') for v in spawn]]
        if args.with_companion:command+=['--with-companion']
        if args.controller is not None:command+=['--controller',str(args.controller)]
        if args.duration:command+=['--duration',str(args.duration)]
        print('Resetting scene; previous original trial retained at',out,flush=True)
        os.execv(sys.executable,command)
if __name__=='__main__':main()
