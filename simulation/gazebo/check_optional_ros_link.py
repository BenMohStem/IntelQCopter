"""Actual host FC + actual ROS2 C++ idle link; synthetic sensors, no flight physics or hardware."""
import argparse,ctypes as c,hashlib,json,os,struct,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from fc_binding import FlightCore,Sample,ROOT
class Link(c.Structure):
    _fields_=[('navigation_denied',c.c_uint32),('stable_fallbacks',c.c_uint32),('available',c.c_bool)]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dll',type=Path,required=True);p.add_argument('--name',required=True);a=p.parse_args()
    out=ROOT/'artifacts/validation'/a.name;out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']='96';rclpy.init();node=Node('optional_companion_link_proof')
    core=FlightCore(3200,dll_path=a.dll);raw=Sample();raw.imu_valid=raw.pressure_valid=True;raw.accel_g[2]=1;raw.pressure_pa=101325
    core.dll.fc_link_snapshot.argtypes=[c.POINTER(Link)]
    pub=node.create_publisher(UInt8MultiArray,'/linkproof/fc/status',10)
    start=time.monotonic();wire=[];checks=[];process=None;logs=[];accepted=0
    def now():return time.monotonic()-start
    def receive(m):
        nonlocal accepted
        data=bytes(m.data);ok=core.report(data,True,now());accepted+=ok
        wire.append(dict(wall_s=now(),kind='UART_from_actual_ROS2_cpp',frame=data.hex(),accepted=ok))
    node.create_subscription(UInt8MultiArray,'/linkproof/fc/companion_tx',receive,10)
    def check(name,value):checks.append(dict(name=name,passed=bool(value)));print(('PASS ' if value else 'FAIL ')+name,flush=True)
    def link():
        s=Link();core.dll.fc_link_snapshot(c.byref(s));return s
    def pump(seconds,nav=False):
        end=time.monotonic()+seconds;last_report=-10;last_status=-10
        while time.monotonic()<end:
            t=now()
            if t-last_report>=.01:
                packet=struct.pack('<6H4B',32768,32768,38000,32768,0,0,5 if nav else 0,2 if nav else 0,0,0)
                assert core.report(packet,False,t);last_report=t
            raw.pressure_sequence+=1
            core.dll.fc_control_step(c.byref(raw),int(t*1e6)+core.time_offset_us,.002,c.byref(core.status))
            if t-last_status>=.02:
                pub.publish(UInt8MultiArray(data=list(core.frame())));last_status=t
                s=link();wire.append(dict(wall_s=t,kind='actual_FC_status',mode=int(core.status.flight_mode),available=bool(s.available),fallbacks=int(s.stable_fallbacks),pilot_fresh=bool(core.status.pilot_fresh)))
            rclpy.spin_once(node,timeout_sec=.001)
    def launch(number):
        nonlocal process
        f=(out/f'ros2-original-{number}.log').open('w',encoding='utf8');logs.append(f)
        process=subprocess.Popen([str(ROOT/'build/ros2-install/lib/qcopter_autonomy/pilot_mission_node.exe'),'--ros-args','-r','__ns:=/linkproof'],stdout=f,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    def stop():
        nonlocal process
        if process is not None:process.terminate();process.wait(timeout=10);process=None
    try:
        pump(.4);check('No ROS2 process: stable Xbox mode and healthy FC',core.status.flight_mode==0 and core.status.pilot_fresh and not core.status.sensor_fault and not link().available)
        launch(1);pump(2.5);check('Actual ROS2 idle heartbeat discovered through CRC UART API',accepted>5 and link().available)
        check('Companion arrival does not override stable Xbox mode',core.status.flight_mode==0 and abs(core.observe()['roll_command'])>.1)
        pump(5.3,nav=True);check('Five-second raw Xbox gesture selects navigation with real companion',core.status.flight_mode==1)
        stop();pump(.6);check('Companion process loss selects stable Xbox mode',core.status.flight_mode==0 and not link().available and link().stable_fallbacks==1)
        check('Fallback restores actual Xbox angle commands',abs(core.observe()['roll_command'])>.1 and not core.status.sensor_fault)
        launch(2);pump(2.2);check('Restarted actual companion is rediscovered',link().available)
        check('Reconnection cannot seize navigation authority',core.status.flight_mode==0)
        core.dll.fc_configure_companion(False);pump(.25);check('Explicit standalone rejects optional link while Xbox works',core.status.flight_mode==0 and not link().available and core.status.pilot_fresh and not core.status.sensor_fault)
    finally:
        stop()
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()
    (out/'wire-journal.json').write_bytes((json.dumps(wire,indent=2)+'\n').encode())
    result=dict(scope=__doc__+' Nominal dt=2ms is supplied by a Python replay; this is not a measured 500Hz deadline. HID is simulated; no real BLE, UART wiring, ESCs, cameras or localization are validated.',passed=all(x['passed'] for x in checks),checks=checks,accepted_actual_ros_frames=accepted,source_sha256=hashlib.sha256((ROOT/'firmware/src/flight_controller.cpp').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(a.dll.read_bytes()).hexdigest())
    (out/'summary.json').write_bytes((json.dumps(result,indent=2)+'\n').encode());return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
