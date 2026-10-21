"""Tests of simulation-only boot tuning through the compiled C API."""
import copy,ctypes as c,json,sys
from pathlib import Path
from experiment_settings import defaults,validate,fc_values
from fc_binding import FlightCore,Sample,Status

def check(name,passed):
 print(('PASS ' if passed else 'FAIL ')+name,flush=True)
 assert passed,name
def main():
 d=defaults();check('default profile valid',validate(d)==d)
 for label,change in [('NaN',lambda p:p['fc'].update(hover_collective=float('nan'))),('inertia triangle',lambda p:p['model'].update(inertia_kg_m2=[.001,.001,.1])),('unknown field',lambda p:p.update(secret=1))]:
  p=copy.deepcopy(d);change(p)
  try:validate(p);accepted=True
  except ValueError:accepted=False
  check('reject '+label,not accepted)
 core=FlightCore(3200,settings=d);dll=core.dll;dll.fc_sim_configure.argtypes=[c.POINTER(c.c_float),c.c_uint];dll.fc_sim_configure.restype=c.c_bool
 def config(p):
  v=fc_values(p['fc']);return dll.fc_sim_configure((c.c_float*30)(*v),30)
 check('host boot profile accepted',config(d))
 initial=core.observe();bad=copy.deepcopy(d);bad['fc']['pids']['roll_rate'][0]=float('nan');check('invalid C values rejected',not config(bad));check('invalid config leaves estimator unchanged',core.observe()==initial)
 raw=bytes.fromhex('00800080008000800000000000000000');check('neutral HID accepted',core.report(raw,False,0))
 check('tuning rejected after pilot reports',not config(d))
 altered=copy.deepcopy(d);altered['fc']['angle_kf'][2]=1
 core2=FlightCore(3200,settings=altered);check('changed profile loads before pilot',config(altered))
 # A measurement correction must actually use changed R, not merely save JSON.
 sample=Sample();sample.accel_g[2]=1;sample.imu_valid=sample.pressure_valid=True;sample.pressure_pa=101325;sample.pressure_sequence=999;st=Status()
 dll.fc_control_step(c.byref(sample),30_000_000,.002,c.byref(st));gain_changed=core2.observe()['roll_K']
 core3=FlightCore(3200,settings=d);dll.fc_control_step(c.byref(sample),30_000_000,.002,c.byref(st));gain_default=core3.observe()['roll_K']
 check('edited measurement variance changes actual C gain',gain_changed>gain_default)
 print(json.dumps(dict(default_gain=gain_default,edited_gain=gain_changed,scope='C host DLL; no physical output'),indent=2))
if __name__=='__main__':main()
