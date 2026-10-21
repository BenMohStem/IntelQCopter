"""Check explicit simulator gestures against the active C core host DLL."""
import argparse,json,queue,struct,time
from pathlib import Path
import numpy as np
from fc_binding import FlightCore
from session_controls import SessionControls,shape_report
from runtime_paths import ROOT

class FakeViewer:
    def __init__(self):self.actions=queue.Queue()
    def next_action(self):
        try:return self.actions.get_nowait()
        except queue.Empty:return None

def main():
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);a=p.parse_args()
    if not a.name.replace('-','').replace('_','').isalnum():raise SystemExit('Plain unique name required')
    out=ROOT/'artifacts/validation'/a.name;out.mkdir(parents=True,exist_ok=False);checks=[]
    def check(name,ok):checks.append(dict(name=name,passed=bool(ok)));print(('PASS ' if ok else 'FAIL ')+name,flush=True)
    viewer=FakeViewer();controls=SessionControls(viewer);core=FlightCore(3200);info=dict(connected=True);neutral=struct.pack('<6H4B',32768,32768,32768,32768,0,0,0,0,0,0);now=0
    def gesture(name):
        nonlocal now
        viewer.actions.put(name);trace=[]
        for i in range(270 if name!='disarm' else 120):
            hid,g,injected=controls.step([neutral],now,info)
            for raw in hid:core.report(raw,False,now)
            trace.append(dict(time_s=now,source='explicit PC simulator gesture' if injected else 'test neutral pilot',hid=[r.hex() for r in hid]));now+=.02
        (out/(name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in trace),encoding='utf8')
        core.step(dict(sim_time_s=now,velocity=[0,0,0],omega=[0,0,0],position=[0,0,.082]),np.eye(3))
    gesture('manual');check('Manual button completes real C five-second mode gesture',core.status.mode_events==1 and core.status.flight_mode==0)
    gesture('navigation');check('Navigation button completes real C five-second mode gesture',core.status.mode_events==2 and core.status.flight_mode==1)
    gesture('disarm');check('Disarm button latches real C disarm',core.status.user_disarm)
    viewer=FakeViewer();controls=SessionControls(viewer);viewer.actions.put('navigation');hid,g,pc=controls.step([neutral],0,dict(connected=False));check('Disconnected controller cannot generate PC gesture',not hid and not pc and controls.active is None and viewer.actions.empty())
    viewer.actions.put('manual');hid,g,pc=controls.step([neutral],0,dict(connected=True,wall_time_s=time.time()-2));check('Stale ROS Xbox state cannot generate PC gesture',not hid and not pc)
    viewer.actions.put('navigation');controls.step([neutral],0,info);disarm=struct.pack('<6H4B',32768,32768,32768,32768,1023,0,5,0,0,0);hid,g,pc=controls.step([disarm],.02,info);check('Actual Xbox disarm has priority over PC mode gesture',hid==[disarm] and not pc and controls.active is None)
    full=struct.pack('<6H4B',65535,65535,65535,65535,1023,1023,0,0,0,0);v=struct.unpack('<6H4B',shape_report(full,'gentle'))
    decode=lambda u:max(0,(abs((u-32768)/32768)-.12)/.88)
    check('Gentle profile caps roll and pitch at five degrees',abs(decode(v[2])*35-5)<.01 and abs(decode(v[3])*35-5)<.01)
    check('Gentle profile caps vertical at half a metre per second',abs(decode(v[1])*2-.5)<.001)
    check('Gentle profile preserves full Down-LT disarm',struct.unpack('<6H4B',shape_report(disarm,'gentle'))[4]==1023)
    half=list(struct.unpack('<6H4B',full));half[2]=round(32768+32768*(.12+.88*.5));soft=struct.unpack('<6H4B',shape_report(struct.pack('<6H4B',*half),'gentle'));check('Half stick requests about 1.25 degrees, not 2.5',abs(decode(soft[2])*35-1.25)<.01)
    check('Normal profile preserves original HID bytes',shape_report(full,'normal')==full)
    result=dict(scope=__doc__+' Synthetic host inputs; no Bluetooth, physical ESC or real-time proof.',checks=checks,passed=all(c['passed'] for c in checks));(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8');return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
