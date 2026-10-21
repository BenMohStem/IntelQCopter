"""Synthetic C host fault/authority/allocator stress; not physical flight timing."""
import argparse,ctypes as c,json,struct,time,random,math,statistics,hashlib,csv
from pathlib import Path
import numpy as np
from fc_binding import FlightCore,Sample,Status,Debug,ROOT
from session_controls import shape_report

class Trace(c.Structure):
 _fields_=[('now_us',c.c_uint64)]+[(n,c.c_float) for n in 'roll pitch height collective scale'.split()]+[('motor',c.c_float*4),('flags',c.c_uint32)]
def main():
 p=argparse.ArgumentParser();p.add_argument('--name',required=True);a=p.parse_args()
 out=ROOT/'artifacts/validation'/a.name;out.mkdir(parents=True,exist_ok=False)
 core=FlightCore(3200);lib=core.dll;checks=[];rng=random.Random(3200);raw=Sample();raw.imu_valid=raw.pressure_valid=True;raw.accel_g[2]=1;raw.pressure_pa=101325
 lib.fc_debug_snapshot.argtypes=[c.POINTER(Debug)];lib.fc_debug_trace.argtypes=[c.POINTER(Trace),c.c_uint];lib.fc_debug_trace.restype=c.c_uint
 lib.fc_profile_record.argtypes=[c.c_uint32]
 def debug():
  d=Debug();lib.fc_debug_snapshot(c.byref(d));return d
 def check(n,v):checks.append(dict(name=n,passed=bool(v)));print(('PASS ' if v else 'FAIL ')+n,flush=True)
 def hid(x=32768,y=32768,roll=32768,pitch=32768,lt=0,rt=0,hat=0,buttons=0):return struct.pack('<6H4B',x,y,roll,pitch,lt,rt,hat,buttons,0,0)
 now=core.time_offset_us
 def feed(pkt):
  nonlocal now
  now+=20000;buf=(c.c_uint8*len(pkt)).from_buffer_copy(pkt);return lib.fc_xbox_report(buf,len(pkt),now)
 def step(dt=.002):
  raw.pressure_sequence+=1;lib.fc_control_step(c.byref(raw),now,dt,c.byref(core.status))
 check('C debug level invalid rejected',not lib.fc_debug_configure(4))
 check('Physical gentle selectable before reports',lib.fc_configure_gentle(True))
 feed(hid(roll=65535,pitch=0,y=0));step();t=core.observe()
 check('C Gentle default-equivalent limits 5deg and 0.5mps',4.99<t['roll_command']<=5.001 and 4.99<t['pitch_command']<=5.001 and .499<t['vz_command']<=.501)
 check('Profile cannot change during flight',not lib.fc_configure_gentle(False) and not lib.fc_debug_configure(0))
 old=(c.c_uint8*16).from_buffer_copy(hid());check('Out-of-order HID rejected',not lib.fc_xbox_report(old,16,now-1))
 # Same gentle definition in hardware C and simulator preprocessor, quantization tolerance.
 for name,axis in [('half',round(32768+32768*(.12+.88*.5))),('full',65535)]:
  feed(hid(roll=axis));step();cmd=core.observe()['roll_command'];u=struct.unpack('<6H4B',shape_report(hid(roll=axis),'gentle'))[2]
  expected=max(0,(u-32768)/32768-.12)/.88*35
  check('C/UI gentle shaping agree '+name,abs(cmd-expected)<.003)
 check('Gentle passes full RT/LT travel',struct.unpack('<6H4B',shape_report(hid(lt=1023,rt=1023),'gentle'))[4:6]==(1023,1023))
 # 10s RT+maximum climb into a stationary synthetic plant forces saturation.
 ceiling_ok=finite_ok=True;stress=[]
 for i in range(5000):
  if i%10==0:feed(hid(y=0,rt=1023))
  step();d=debug();ceiling_ok &= d.collective_allocated<=.800001 and d.collective_ceiling<=.800001
  finite_ok &= all(math.isfinite(x) and 0<=x<=1 for x in core.status.motor_demand)
  if i%10==0:stress.append([i*.002,'RT',d.collective_requested,d.collective_allocated,d.collective_ceiling,*core.status.motor_demand])
 check('RT collective never exceeds 1800us equivalent',ceiling_ok and d.collective_ceiling>.799)
 check('RT saturated stress motor demands bounded finite',finite_ok)
 check('Saturation telemetry detects clamped collective',d.saturation_cycles>0)
 for i in range(250):
  if i%10==0:feed(hid(lt=1023))
  step()
  if i%10==0:
   d=debug();stress.append([10+i*.002,'LT',d.collective_requested,d.collective_allocated,d.collective_ceiling,*core.status.motor_demand])
 check('LT lowers ceiling to 1500us equivalent',abs(debug().collective_ceiling-.5)<1e-5 and core.observe()['vz_command']<0)
 mixed=True;max_spread=0
 for i in range(5000):
  if i%10==0:feed(hid(roll=rng.choice([0,65535]),pitch=rng.choice([0,65535]),x=rng.choice([0,65535]),y=0,rt=1023))
  step();d=debug();mot=list(core.status.motor_demand)
  mixed &= all(math.isfinite(x) and 0<=x<=1 for x in mot) and sum(mot)/4<=.80001 and d.torque_scale>0
  max_spread=max(max_spread,max(mot)-min(mot))
 check('Mixed-axis RT stress retains differential PID authority',mixed and max_spread>.02)
 with (out/'collective-stress.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['time_s','trigger','requested','allocated','ceiling','FL','FR','RR','RL']);w.writerows(stress)
 # Fuzz cannot bypass length/range/CRC validation.
 rejected=0
 for i in range(10000):
  pkt=rng.randbytes(rng.randrange(0,40));buf=(c.c_uint8*len(pkt)).from_buffer_copy(pkt);rejected+=not lib.fc_xbox_report(buf,len(pkt),now+i)
 check('Malformed HID fuzz rejected over 99 percent',rejected>9900)
 uart_rejected=0
 for i in range(10000):
  pkt=rng.randbytes(28);buf=(c.c_uint8*28).from_buffer_copy(pkt);uart_rejected+=not lib.fc_companion_report(buf,28,now)
 check('Random UART fuzz all rejected',uart_rejected==10000)
 # Remove companion: stable mode must keep functioning without a single UART frame.
 lib.fc_configure_companion(False);feed(hid());step()
 check('Same firmware runs stable with companion absent',core.status.flight_mode==0 and not core.status.companion_fresh and not core.status.sensor_fault)
 before=debug().invalid_calls;lib.fc_control_step(None,now,.002,c.byref(core.status))
 check('NULL sensor pointer produces fault and zero demands',core.status.sensor_fault and not any(core.status.motor_demand) and debug().invalid_calls==before+1)
 for label,dt in [('zero',0),('negative',-.002),('NaN',float('nan')),('overlong',.1)]:
  feed(hid());step(dt);check('Invalid control dt '+label,core.status.sensor_fault and not any(core.status.motor_demand))
 raw.accel_g[0]=float('nan');feed(hid());step();check('NaN IMU blocks demands',core.status.sensor_fault and not any(core.status.motor_demand));raw.accel_g[0]=0
 raw.imu_valid=False;step();check('Lost IMU blocks demands',not any(core.status.motor_demand));raw.imu_valid=True
 raw.pressure_valid=False;step();check('Lost barometer blocks demands',not any(core.status.motor_demand));raw.pressure_valid=True
 lib.fc_configure_companion(True);feed(hid());step()
 check('Optional companion re-enabled without breaking stable mode',core.status.flight_mode==0 and not core.status.sensor_fault)
 # Explicit adapter timing histogram boundary test; these injected times are NOT measurements.
 before=debug().deadline_misses
 for us in [0,50,51,100,101,250,251,500,501,1000,1001,1500,1501,2000,2001]:lib.fc_profile_record(us)
 check('Injected deadline histogram detects >2000us',debug().deadline_misses==before+1 and debug().max_execution_us==2001)
 # Repeat a healthy stationary state; collect actual host DLL API duration separately.
 times=[];trace_ok=True
 for i in range(30000):
  if i%10==0:feed(hid())
  start=time.perf_counter_ns();step();times.append((time.perf_counter_ns()-start)/1000)
  trace_ok &= not core.status.sensor_fault and all(math.isfinite(x) and 0<=x<=1 for x in core.status.motor_demand)
 check('30000-step finite/covariance soak',trace_ok and debug().min_covariance>=0)
 trace=(Trace*64)();n=lib.fc_debug_trace(trace,64)
 check('Trace ring bounded chronological with wrap',n==64 and all(trace[i].now_us<=trace[i+1].now_us for i in range(n-1)))
 with (out/'host-api-times-us.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['sample','host_ctypes_api_us']);w.writerows(enumerate(times))
 (out/'trace.json').write_text(json.dumps([dict(time_us=t.now_us,roll=t.roll,pitch=t.pitch,height=t.height,collective=t.collective,scale=t.scale,motor=list(t.motor),flags=t.flags) for t in trace[:n]],indent=2),encoding='utf8')
 d=debug();result=dict(passed=all(x['passed'] for x in checks),checks=checks,host_api_us=dict(mean=statistics.mean(times),p50=float(np.percentile(times,50)),p95=float(np.percentile(times,95)),p99=float(np.percentile(times,99)),max=max(times)),diagnostics={name:list(getattr(d,name)) if name=='execution_bins' else getattr(d,name) for name,_ in Debug._fields_},scope=__doc__+' Histogram boundaries are injected; host ctypes timings include Python overhead and do not establish ESP32 500Hz deadlines.',sha256=hashlib.sha256((ROOT/'firmware/src/flight_controller.c').read_bytes()).hexdigest())
 (out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf8');print(json.dumps(result['host_api_us']));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
