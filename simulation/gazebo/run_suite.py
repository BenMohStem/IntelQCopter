"""Launch distinct PC/drone/Gazebo roles and retain every original result.
Run with the workstation Python. ROS and Gazebo environments are resolved once.
"""
import argparse,json,os,re,subprocess,time,math
from pathlib import Path
from runtime_paths import PIXI,ROS_MANIFEST,ROS_PYTHON,GZ_MANIFEST,GZ_PYTHON,PANEL_MODEL

ROOT=Path(__file__).resolve().parents[2]
def environment(command):
    # Capture activation privately; never print/store environment values or credentials.
    probe="import os,json; print('QC_ENV='+json.dumps(dict(os.environ)))"
    result=subprocess.run(command+['python','-c',probe],cwd=ROOT,capture_output=True,text=True)
    line=next((x[7:] for x in result.stdout.splitlines() if x.startswith('QC_ENV=')),None)
    if result.returncode or line is None:raise RuntimeError('Environment activation failed; inspect installation without dumping environment variables')
    return json.loads(line)
def main():
 p=argparse.ArgumentParser();p.add_argument('--prefix',required=True);p.add_argument('--only',choices=['manual','navigation','all'],default='all');p.add_argument('--env',action='append',default=None);p.add_argument('--operator',action='store_true',help='Wait for endpoints from the PC browser map instead of an automated request');args=p.parse_args()
 if not re.fullmatch(r'[A-Za-z0-9_-]+',args.prefix):raise SystemExit('Use a plain unique prefix')
 worlds=json.loads((ROOT/'simulation/gazebo/environments.json').read_text())
 rosenv=environment([PIXI,'run','--manifest-path',ROS_MANIFEST,'cmd','/d','/c','tools\\ros2\\ros2_windows.cmd'])
 gzenv=environment([PIXI,'run','--manifest-path',GZ_MANIFEST])
 rospy=ROS_PYTHON;gzpy=GZ_PYTHON
 tasks=[]
 if args.only in ('manual','all'):tasks += [(worlds[i],i) for i in range(5)]
 if args.only in ('navigation','all'):tasks += [(s,None) for s in worlds]
 if args.env:tasks=[t for t in tasks if t[0]['id'] in args.env]
 if args.operator and (args.only!='navigation' or len(tasks)!=1):raise SystemExit('Operator mode requires --only navigation and exactly one --env')
 results=[]
 for spec,manual in tasks:
    name=f'{args.prefix}-{spec["id"]}-'+('manual' if manual is not None else 'nav')
    folder=ROOT/'artifacts/gazebo-suite'/name
    folder.mkdir(parents=True,exist_ok=False)
    procs=[];logs=[]
    def launch(label,command,env):
        log=(folder/f'{label}.log').open('w',encoding='utf8');logs.append(log)
        proc=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));procs.append(proc);return proc
    def ready(proc,path,marker,timeout=90):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if marker in path.read_text(encoding='utf8',errors='replace'):return
            if proc.poll() is not None:raise RuntimeError(f'{path.name} exited before readiness')
            time.sleep(.25)
        raise TimeoutError('Readiness timeout: '+path.name)
    print('START',name,flush=True)
    try:
        engine=launch('engine',[gzpy,'-u','simulation/gazebo/engine.py','--env',spec['id'],'--run',name],gzenv)
        ready(engine,folder/'engine.log','READY GAZEBO')
        rosargs=['--ros-args','-r','__ns:=/drone','-p','use_sim_time:=true']
        if manual is None:
            exe=ROOT/'build/ros2-install/lib/qcopter_autonomy'
            mission=launch('drone-mission',[str(exe/'pilot_mission_node.exe'),*rosargs],rosenv)
            ready(mission,folder/'drone-mission.log','READY:')
            yolo=launch('drone-yolo',[str(exe/'yolo_node.exe'),*rosargs,'-p',f'onnx_model:={PANEL_MODEL}','-p','input_size:=320'],rosenv)
            ready(yolo,folder/'drone-yolo.log','READY_RGB:')
            pnp=launch('drone-pnp',[str(exe/'panel_pose_node.exe'),*rosargs,'-p','known_fixture_model_confirmed:=true'],rosenv)
            ready(pnp,folder/'drone-pnp.log','READY_PNP:')
        pcargs=[rospy,'-u','simulation/gazebo/pc_station.py','--env',spec['id'],'--run',name]
        if manual is not None:pcargs+=['--manual',str(manual)]
        if args.operator:pcargs+=['--external-mission']
        pc=launch('pc-station',pcargs+rosargs,rosenv);ready(pc,folder/'pc-station.log','PC READY')
        if args.operator:
            ground=launch('pc-map',[rospy,'-u','companion/pc_mission_map.py','--log',str(folder/'pc-map-missions.jsonl'),*rosargs],rosenv)
            ready(ground,folder/'pc-map.log','PC map available')
            print('OPERATOR: open http://127.0.0.1:8086 and send start/finish after the mode hold.',flush=True)
        bridge=launch('ros-boundary',[rospy,'-u','simulation/gazebo/ros_bridge.py','--run',name,'--timeout','600' if args.operator else '95',*rosargs],rosenv)
        started=time.monotonic();last=0
        while bridge.poll() is None:
            if time.monotonic()-started>(1800 if args.operator else 300):raise TimeoutError('Run exceeded its wall-clock deadline')
            if time.monotonic()-last>25:
                lines=(folder/'ros-boundary.log').read_text(encoding='utf8',errors='replace').splitlines()
                if lines:print(name,lines[-1][:220],flush=True)
                last=time.monotonic()
            time.sleep(.5)
        engine.wait(timeout=20)
        transport=json.loads((folder/'transport.json').read_text());summary=json.loads((folder/'engine-summary.json').read_text()) if (folder/'engine-summary.json').exists() else {}
        success=bridge.returncode==0 and summary.get('obstacle_contact_steps',1)==0
        endpoint=spec['finish'] if manual is None else ([0,0,.082] if manual%2==0 else [1.1,-.45,.082])
        if args.operator:
            requests=[json.loads(line) for line in (folder/'pc-map-missions.jsonl').read_text(encoding='utf8').splitlines()]
            accepted=[r for r in requests if r['response']['accepted']]
            if not accepted:raise RuntimeError('No accepted ground-PC map request')
            endpoint=accepted[-1]['request']['finish']
        arrival_error=math.dist(transport['final_state']['position'][:2],endpoint[:2])
        success=success and arrival_error<.2
        if manual is None:success=success and transport['pnp_frames']>0 and transport['companion_packets']>0
        else:success=success and transport['companion_packets']==0
        result=dict(name=name,environment=spec['id'],kind='manual' if manual is not None else 'navigation',operator=args.operator,passed=success,requested_landing_point=endpoint,arrival_error_m=arrival_error,transport=transport,engine=summary)
        print('RESULT',name,'PASS' if success else 'FAIL',flush=True)
    except Exception as e:
        boundary=folder/'ros-boundary.log'
        result=dict(name=name,environment=spec['id'],kind='manual' if manual is not None else 'navigation',passed=False,error=str(e),original_boundary_tail=boundary.read_text(encoding='utf8',errors='replace').splitlines()[-12:] if boundary.exists() else [])
        print('FAIL',name,str(e),flush=True)
    finally:
        for proc in reversed(procs):
            if proc.poll() is None:proc.terminate()
        for proc in procs:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill()
        for log in logs:log.close()
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');results.append(result)
    (ROOT/'artifacts/gazebo-suite'/f'{args.prefix}-suite.json').write_text(json.dumps(results,indent=2)+'\n')
 print('SUITE',len(results),'runs',sum(r['passed'] for r in results),'passed',flush=True)
 raise SystemExit(0 if results and all(r['passed'] for r in results) else 2)
if __name__=='__main__':main()
