"""Regression driver: simulated Xbox selects mode; PC HTTP only supplies route."""
import argparse,csv,json,math,subprocess,sys,time,urllib.request
from datetime import datetime,timezone
from pathlib import Path
R=Path(__file__).resolve().parents[2]
def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--points',type=int,choices=[2,3],required=True);parser.add_argument('--name');args=parser.parse_args();count=args.points;prefix=args.name or (f'route{count}-'+datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S'));out=R/'artifacts/gazebo-suite'/f'{prefix}-env01-nav'
 log=(R/'build'/f'{prefix}.log').open('w',encoding='utf8')
 proc=subprocess.Popen([sys.executable,'simulation/gazebo/run_suite.py','--prefix',prefix,'--only','navigation','--env','env01','--operator'],cwd=R,stdout=log,stderr=subprocess.STDOUT)
 points=[[0,0,.082]]+([[4,-2.4,.082]] if count==3 else [])+[[8.9,.3,.082]]
 request=dict(points=points,start=points[0],finish=points[-1],cruise=1.25);accepted=False;end=time.monotonic()+150
 try:
  while time.monotonic()<end and proc.poll() is None:
   try:
    state=json.load(urllib.request.urlopen('http://127.0.0.1:8086/state',timeout=2))
    if state['fc']['mode']=='navigation' and state['fc']['pilot_fresh']:
     body=json.dumps(request).encode();req=urllib.request.Request('http://127.0.0.1:8086/mission',data=body,headers={'Content-Type':'application/json'})
     reply=json.load(urllib.request.urlopen(req,timeout=8));print('PC route response',reply,flush=True)
     if reply['accepted']:accepted=True;break
   except (OSError,ValueError,KeyError):pass
   time.sleep(.5)
  if not accepted:raise RuntimeError('No route accepted')
  code=proc.wait(timeout=400)
  result=json.loads((out/'result.json').read_text());rows=list(csv.DictReader((out/'flight.csv').open()))
  check=dict(scope='Automated regression with simulated Xbox HID, actual ROS2 Cyclone DDS / HTTP route service / native Gazebo / active C host core. Not a real controller flight.',points=points,accepted=accepted,suite_exit=code,mission_passed=result['passed'])
  if count==3:
   airborne=[r for r in rows if float(r['z'])>.8];near=min(airborne,key=lambda r:math.hypot(float(r['x'])-4,float(r['y'])+2.4))
   check['middle_error_m']=math.hypot(float(near['x'])-4,float(near['y'])+2.4);check['middle_height_m']=float(near['z']);check['waypoint_acknowledged']='ROUTE_WAYPOINT_REACHED index=1' in (out/'drone-mission.log').read_text()
   check['passed']=result['passed'] and check['middle_error_m']<.22 and abs(check['middle_height_m']-1.25)<.15 and check['waypoint_acknowledged']
  else:check['passed']=result['passed']
  (out/'route-check.json').write_text(json.dumps(check,indent=2)+'\n',encoding='utf8');print(json.dumps(check,indent=2),flush=True)
  return 0 if check['passed'] else 2
 finally:
  if proc.poll() is None:proc.terminate()
  log.close()
if __name__=='__main__':raise SystemExit(main())
