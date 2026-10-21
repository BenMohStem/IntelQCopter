"""Fixed synthetic sensors test raw RT/LT input through the active C host build.
Not a flight simulation: the returned motor demands do not move a plant.
"""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/gazebo'))
from fc_binding import FlightCore
from struct import pack

def evaluate(lt,rt):
    core=FlightCore(3200);state=dict(sim_time_s=0.,position=[0,0,.082],velocity=[0,0,0],omega=[0,0,0])
    hid=pack('<6H4B',32768,32768,32768,32768,round(1023*lt),round(1023*rt),0,0,0,0)
    for n in range(500):
        state['sim_time_s']=n*.002
        if n%10==0:assert core.report(hid,False,state['sim_time_s'])
        values=core.step(state,np.eye(3))
    return float(np.mean(values))

neutral=evaluate(0,0);boost=evaluate(0,1);reduce=evaluate(1,0);cancel=evaluate(1,1)
checks={'RT_increases_collective':boost>neutral+.01,'LT_reduces_collective':reduce<neutral-.01,'simultaneous_triggers_cancel':abs(cancel-neutral)<1e-5}
report=dict(passed=all(checks.values()),checks=checks,mean_demands=dict(neutral=neutral,RT=boost,LT=reduce,both=cancel),scope='Active C host DLL, raw Xbox packets, fixed synthetic sensor bench; not flight physics or BLE.')
out=ROOT/'artifacts/validation/2026-10-21';out.mkdir(parents=True,exist_ok=True)
(out/'host-triggers.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
