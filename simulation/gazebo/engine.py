"""Native Gazebo physics and cameras; 500 Hz active ESP32 C control core.

Only input opcodes are raw Xbox HID and companion UART frames, plus sensor clocks.
There is no test-script motor-force or flight-setpoint opcode.
"""
import argparse,csv,json,math,time,hashlib,shutil
from pathlib import Path
from multiprocessing.connection import Listener
import numpy as np
import gz.math as gm
import gz.sim as sim
from gz.transport import Node
from gz.msgs.image_pb2 import Image
from gz.msgs.contacts_pb2 import Contacts
from build_world import build,ROOT
from fc_binding import FlightCore

parser=argparse.ArgumentParser();parser.add_argument('--env',required=True);parser.add_argument('--run',required=True);parser.add_argument('--port',type=int,default=9251)
parser.add_argument('--test-open-space',action='store_true',help='Explicit controller-test world without panels or garden structures');parser.add_argument('--no-horizontal-aiding',action='store_true');parser.add_argument('--standalone',action='store_true');parser.add_argument('--interactive',action='store_true')
parser.add_argument('--spawn',type=float,nargs=2,default=[0,0]);parser.add_argument('--telemetry-hz',type=int,choices=[50,500],default=500);parser.add_argument('--settings');args=parser.parse_args()
spec=next(s for s in json.loads((ROOT/'simulation/gazebo/environments.json').read_text()) if s['id']==args.env)
if not all(math.isfinite(v) for v in args.spawn) or not (-1<=args.spawn[0]<=9 and abs(args.spawn[1])<=3):raise SystemExit('Spawn outside map bounds')
for x,y,yaw in spec['panels']:
    dx,dy=args.spawn[0]-x,args.spawn[1]-y;c,s=math.cos(yaw),math.sin(yaw)
    if math.hypot(max(abs(c*dx+s*dy)-.04,0),max(abs(-s*dx+c*dy)-.6,0))<.45:raise SystemExit('Spawn overlaps panel clearance')
if args.test_open_space:spec['panels']=[];spec['name']='open_space_controller_test'
spec['spawn']=args.spawn
from experiment_settings import defaults,load
settings=load(args.settings) if args.settings else defaults()
parameters=settings['model']
out=ROOT/'artifacts/gazebo-suite'/args.run;out.mkdir(parents=True,exist_ok=True)
world_path=build(ROOT/'build/gazebo'/f'{args.run}.sdf',spec=spec,model=parameters)
(out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n',encoding='utf8')
shutil.copy2(world_path,out/'world.sdf')
frames={};contact_names=[];contact_time=-99.;names={};node=Node();callbacks=[]
for topic in ['/drone/rgb','/overview/rgb','/follow/rgb']:
    def receive(message,topic=topic):frames[topic]=(message.width,message.height,bytes(message.data))
    callbacks.append(receive);assert node.subscribe(Image,topic,receive)
def on_contact(message):
    global contact_names,contact_time
    contact_time=message.header.stamp.sec+message.header.stamp.nsec*1e-9
    contact_names=[(names.get(c.collision1.id,c.collision1.name),names.get(c.collision2.id,c.collision2.name)) for c in message.contact]
callbacks.append(on_contact);assert node.subscribe(Contacts,'/drone/contact',on_contact)
interactive_started=False;manual_lift_request=False
fixture=sim.TestFixture(str(world_path));body=None;state=None;core=None;actual=np.zeros(4);count=0;hits=0;minimum_clearance=99.;xbox_packets=companion_packets=0
csvfile=(out/'flight.csv').open('w',newline='');writer=csv.writer(csvfile)
writer.writerow(['time_s','x','y','z','vx','vy','vz','roll_deg','pitch_deg','yaw_deg','estimated_z','estimated_vz','FL','FR','RR','RL','mode','user_disarm','pilot_fresh','companion_fresh','mode_events','xbox_reports','ground_contact','obstacle_contact','min_clearance'])
def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def pre(info,ecm):
    global body,actual,interactive_started
    if body is None:
        world=sim.World(sim.world_entity(ecm));model=sim.Model(world.model_by_name(ecm,'drone'));entity=model.link_by_name(ecm,'body')
        if entity:
            body=sim.Link(entity);body.enable_velocity_checks(ecm,True)
            for name in ['ground','drone']+[f'panel{i}' for i in range(len(spec['panels']))]:
                other=sim.Model(world.model_by_name(ecm,name));link=sim.Link(other.link_by_name(ecm,'body'));names[link.collision_by_name(ecm,'collision')]=name
    if body is not None and core is not None and state is not None:
        r=rotation(state['quaternion']);desired=core.step(state,r)
        enabled=core.status.mode_events>0 and core.status.pilot_fresh and not core.status.user_disarm and not core.status.sensor_fault
        if args.interactive:
            if core.status.mode_events and ((core.status.flight_mode==1 and core.status.companion_fresh) or (core.status.flight_mode==0 and manual_lift_request)):interactive_started=True
            enabled=enabled and interactive_started
        if not enabled:desired[:]=0
        actual+=(desired-actual)*(1-math.exp(-.002/parameters['thrust_time_constant_s']))
        thrust=actual*parameters['maximum_thrust_per_motor_n'];a=parameters['motor_diagonal_m']/(2*math.sqrt(2))
        torque=[a*(thrust[0]-thrust[1]-thrust[2]+thrust[3]),a*(-thrust[0]-thrust[1]+thrust[2]+thrust[3]),parameters['reaction_torque_per_thrust_m']*(-thrust[0]+thrust[1]-thrust[2]+thrust[3])]
        pose=body.world_pose(ecm)
        body.add_world_wrench(ecm,pose.rot().rotate_vector(gm.Vector3d(0,0,float(sum(thrust))))-gm.Vector3d(*(parameters['linear_drag_ns_per_m']*np.array(state['velocity']))),pose.rot().rotate_vector(gm.Vector3d(*torque)))
def post(info,ecm):
    global state,count,hits,minimum_clearance
    if body is None:return
    pose=body.world_pose(ecm);v=body.world_linear_velocity(ecm);w=body.world_angular_velocity(ecm)
    if not pose or v is None or w is None:return
    p=pose.pos();q=pose.rot();now=info.sim_time.total_seconds();active_contacts=contact_names if 0<=now-contact_time<.03 else []
    ground=any('ground' in pair for pair in active_contacts);collision=any(any(n.startswith('panel') for n in pair) for pair in active_contacts)
    hits+=int(collision)
    state=dict(sim_time_s=now,position=[p.x(),p.y(),p.z()],quaternion=[q.x(),q.y(),q.z(),q.w()],velocity=[v.x(),v.y(),v.z()],omega=[w.x(),w.y(),w.z()],ground_contact=ground,obstacle_contact=collision)
    if core is None:return
    for x,y,yaw in spec['panels']:
        dx,dy=p.x()-x,p.y()-y;c,s=math.cos(yaw),math.sin(yaw)
        local=np.array([c*dx+s*dy,-s*dx+c*dy]);horizontal=np.linalg.norm(np.maximum(np.abs(local)-[.04,.6],0))-.34
        clearance=max(float(horizontal),p.z()-.08-1.8,-p.z()-.08)
        minimum_clearance=min(minimum_clearance,clearance)
    count+=1
    if count%(500//args.telemetry_hz)==0:
        st=core.status;writer.writerow([now,*state['position'],*state['velocity'],pose.rot().roll()*180/math.pi,pose.rot().pitch()*180/math.pi,pose.rot().yaw()*180/math.pi,st.height_m,st.vertical_mps,*actual,st.flight_mode,st.user_disarm,st.pilot_fresh,st.companion_fresh,st.mode_events,st.xbox_reports,ground,collision,minimum_clearance])
fixture.on_pre_update(pre);fixture.on_post_update(post);fixture.finalize();server=fixture.server()
for _ in range(100):
    server.run(True,10,False)
    if state and len(frames)==3:break
    time.sleep(.02)
assert state and len(frames)==3,'Gazebo physics/cameras unavailable'
core=FlightCore(spec['seed'],companion=not args.standalone,settings=settings)
core.disable_horizontal_aiding=args.no_horizontal_aiding
print(f"READY GAZEBO {args.env} {spec['name']} port={args.port}; actual physics + OGRE2; input=HID/UART only",flush=True)
with Listener(('127.0.0.1',args.port),authkey=b'intelqcopter-esp32') as listener:
 with listener.accept() as client:
  while True:
    message=client.recv();op=message['op']
    if op=='camera':client.send(frames[message['topic']]);continue
    if op=='close':client.send({'steps':count});break
    if op=='step':
        now=state['sim_time_s']
        for data in message.get('hid',[]):
            if core.report(data,False,now):
                xbox_packets+=1
                manual_lift_request=int.from_bytes(data[2:4],'little')<26000 or int.from_bytes(data[10:12],'little')>100
        for data in message.get('companion',[]):companion_packets+=int(core.report(data,True,now))
        assert server.run(True,10,False)
    elif op!='state':raise ValueError('Only sensor/clock, raw HID and UART operations are allowed')
    reply=dict(state);reply['telemetry']=core.observe();reply['telemetry'].update(roll_true=math.degrees(math.atan2(2*(state['quaternion'][3]*state['quaternion'][0]+state['quaternion'][1]*state['quaternion'][2]),1-2*(state['quaternion'][0]**2+state['quaternion'][1]**2))),pitch_true=math.degrees(math.asin(max(-1,min(1,2*(state['quaternion'][3]*state['quaternion'][1]-state['quaternion'][2]*state['quaternion'][0]))))),yaw_true=math.degrees(math.atan2(2*(state['quaternion'][3]*state['quaternion'][2]+state['quaternion'][0]*state['quaternion'][1]),1-2*(state['quaternion'][1]**2+state['quaternion'][2]**2))),yaw_rate_true=float((rotation(state['quaternion']).T@np.array(state['omega']))[2]*180/math.pi));reply['motor_actual']=actual.tolist();reply['localization_source']='none' if args.no_horizontal_aiding else 'Perfect Gazebo x/y/vx/vy/yaw observation to C FC; synthetic external localization';reply['environment']=args.env;reply['health']={k:(list(getattr(core.status,k)) if k=='motor_demand' else bool(getattr(core.status,k)) if k!='mode_events' else int(core.status.mode_events)) for k in ['motor_demand','sensor_fault','pilot_fresh','gyro_verified','accel_calibrated','barometer_ready','companion_fresh','mode_events']};reply.update(simulation_actuation=bool(interactive_started if args.interactive else core.status.mode_events),fc=core.frame(),mode=int(core.status.flight_mode),disarm=bool(core.status.user_disarm),mapped_input_packets=companion_packets,xbox_packets=xbox_packets)
    client.send(reply);csvfile.flush()
csvfile.close()
summary=dict(engine='Native Gazebo 10.5 + OGRE2 RGB; ESP32 C core host build',environment=spec,assumed_parameters=parameters,sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['firmware/src/flight_controller.cpp','firmware/include/fc_api.h','build/sitl/esp32_flight_core.dll']},steps=count,obstacle_contact_steps=hits,min_evaluated_clearance_m=minimum_clearance,xbox_packets=xbox_packets,companion_packets=companion_packets,final_state=state,
 standalone=args.standalone,interactive=args.interactive,telemetry_hz=args.telemetry_hz,calibration='Synthetic six-face sensor bench; no real coefficients',horizontal_aiding='disabled' if args.no_horizontal_aiding else 'Perfect Gazebo x/y/vx/vy/yaw delivered to the C FC as synthetic localization; unavailable on real hardware until a driver exists',hardware_motor_arming=False,simulation_motor_gate=('Completed mode gesture, fresh pilot/calibrated sensors, plus upward stick/RT in stable or fresh companion in navigation; latched for this interactive session' if args.interactive else 'First completed mode gesture and fresh pilot/calibrated sensors')+'; physical outputs remain disabled')
(out/'engine-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('ENGINE COMPLETE',json.dumps({k:v for k,v in summary.items() if k not in ['environment','final_state']}),flush=True)
