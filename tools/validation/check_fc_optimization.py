"""Isolated old/new C replay, host API profiling and direct-call fault checks; no aircraft physics."""
import argparse,ctypes as c,csv,hashlib,json,math,statistics,struct,sys,time
from pathlib import Path
R=Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'simulation/gazebo'))
from fc_binding import FlightCore,Sample,Status
class Pilot(c.Structure):
    _fields_=[(n,c.c_float) for n in 'roll pitch yaw vertical boost reduce'.split()]+[('received_us',c.c_uint64),('connected',c.c_bool)]
class Profile(c.Structure):
    _fields_=[('samples',c.c_uint64),('total_us',c.c_uint64)]+[(n,c.c_uint32) for n in 'minimum_us maximum_us last_us over_budget'.split()]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True);p.add_argument('--name',required=True);a=p.parse_args()
    out=R/'artifacts/validation'/a.name;out.mkdir(parents=True,exist_ok=False)
    cores=[FlightCore(3200,dll_path=a.before),FlightCore(3200,dll_path=a.after)]
    checks=[]
    def check(name,passed):
        checks.append(dict(name=name,passed=bool(passed)));print(('PASS ' if passed else 'FAIL ')+name,flush=True)
    raw=Sample();raw.imu_valid=raw.pressure_valid=True;raw.pressure_pa=101325
    times=[[],[]];trace=[];max_delta=0
    for i in range(20000):
        now=20_000_000+i*2000;t=i*.002
        phi=.05*math.sin(.4*t);theta=.04*math.cos(.6*t)
        raw.accel_g[:]=[-math.sin(theta),math.sin(phi)*math.cos(theta),math.cos(phi)*math.cos(theta)]
        raw.gyro_dps[:]=[.02*math.cos(.4*t)*180/math.pi,-.024*math.sin(.6*t)*180/math.pi,.1*math.sin(t)]
        if i%10==0:
            raw.pressure_sequence+=1;raw.pressure_pa=101325*(1-(.4+.1*math.sin(.2*t))/44330)**(1/.190294957)
            report=struct.pack('<6H4B',int(32768+6000*math.sin(t)),int(32768-9000*math.sin(.5*t)),int(32768+7000*math.sin(.7*t)),int(32768+6000*math.cos(.9*t)),0,1023 if i%4000<1000 else 0,0,0,0,0)
            buf=(c.c_uint8*16).from_buffer_copy(report)
            for core in cores:assert core.dll.fc_xbox_report(buf,16,now)
        # Alternate ordering to reduce consistent cache/scheduling bias.
        for j in ([0,1] if i%2 else [1,0]):
            core=cores[j];start=time.perf_counter_ns();core.dll.fc_control_step(c.byref(raw),now,.002,c.byref(core.status));times[j].append((time.perf_counter_ns()-start)/1000)
        values=[[s.roll_deg,s.pitch_deg,s.height_m,s.vertical_mps,*s.motor_demand,*s.gyro_sigma_dps] for s in [core.status for core in cores]]
        delta=max(abs(x-y) for x,y in zip(*values));max_delta=max(max_delta,delta)
        if i%50==0:trace.append([t,delta,*values[0],*values[1]])
    check('20000-step healthy old/new replay equivalent within 1e-5',max_delta<1e-5)
    core=cores[1];lib=core.dll
    lib.fc_step.argtypes=[c.POINTER(Sample),c.POINTER(Pilot),c.c_uint64,c.c_float,c.POINTER(Status)]
    pilot=Pilot(received_us=now,connected=True);status=Status()
    for name,sample,pil,dt in [('NULL raw',None,c.byref(pilot),.002),('NULL pilot',c.byref(raw),None,.002),('NaN dt',c.byref(raw),c.byref(pilot),float('nan')),('zero dt',c.byref(raw),c.byref(pilot),0),('overlong dt',c.byref(raw),c.byref(pilot),.1)]:
        status.motor_demand[:]=[.7]*4
        lib.fc_step(sample,pil,now,dt,c.byref(status))
        check('Direct public fc_step fails closed: '+name,status.sensor_fault and not any(status.motor_demand))
    lib.fc_profile_record.argtypes=[c.c_uint32];lib.fc_profile_snapshot.argtypes=[c.POINTER(Profile)]
    lib.fc_init(0x3200);injected=[0,100,2000,2001,2**32-1]
    for us in injected:lib.fc_profile_record(us)
    stats=Profile();lib.fc_profile_snapshot(c.byref(stats))
    check('Integer timing statistics include 64-bit total and exact 2ms boundary',stats.samples==5 and stats.total_us==sum(injected) and stats.minimum_us==0 and stats.maximum_us==2**32-1 and stats.over_budget==2)
    lib.fc_init(0x3200);lib.fc_debug_configure(0);lib.fc_profile_record(999);lib.fc_profile_snapshot(c.byref(stats))
    check('Boot reset and debug-off disable timing accumulation',stats.samples==0 and stats.total_us==0)
    with (out/'paired-host-api-times-us.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['sample','baseline_host_api_us','optimized_host_api_us']);w.writerows((i,*v) for i,v in enumerate(zip(*times)))
    with (out/'replay-comparison.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time_s','max_delta']+[prefix+n for prefix in ['before_','after_'] for n in ['roll','pitch','height','vz','FL','FR','RR','RL','sigma_x','sigma_y','sigma_z']]);w.writerows(trace)
    def percentile(v,p):return sorted(v)[round((len(v)-1)*p)]
    result=dict(scope=__doc__+' Host timing includes ctypes/Windows scheduling; synthetic replay is not Gazebo flight, MCU deadline or physical interface evidence. Injected timing values only check accounting.',passed=all(x['passed'] for x in checks),checks=checks,max_healthy_state_delta=max_delta,host_api_us={name:dict(mean=statistics.mean(v),median=statistics.median(v),p95=percentile(v,.95),p99=percentile(v,.99),max=max(v)) for name,v in zip(['baseline','optimized'],times)},source_sha256=hashlib.sha256((R/'firmware/src/flight_controller.c').read_bytes()).hexdigest(),binary_sha256={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in [('baseline',a.before),('optimized',a.after)]})
    (out/'summary.json').write_bytes((json.dumps(result,indent=2)+'\n').encode());print(json.dumps(result['host_api_us'],indent=2));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
