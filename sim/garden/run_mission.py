"""Ground -> flight -> ground, real C loops + C++ mission/nav + RGB YOLO/PnP.

PyBullet physics/TinyRenderer; single RGB, downward ray lidar and simulated local
odometry. No renderer depth or segmentation is passed to navigation. Real VIO,
real radio and physical arming are not implemented. Scene is USTHB-inspired only.
"""
import os,sys,json,ctypes as ct,time,hashlib,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
os.environ['YOLO_CONFIG_DIR']=str(ROOT/'build/yolo-settings')
sys.path.insert(0,str(ROOT/'sim/sitl'));sys.path.insert(0,str(ROOT/'companion/perception'))
from run import libraries,gain_table,F64
from panel_pose import panel_observations
from world import Garden,K
import numpy as np,cv2,pybullet as p
from ultralytics import YOLO
import imageio_ffmpeg

OUT=ROOT/'artifacts/simulation/2026-10-20-missions'
PHASES=['idle','spool','takeoff','hover','navigate','approach','land','landed']

def video_frame(world,rgb,annotated,rows,phase,observed,t):
    rgb3,_=world.camera(None,None,overview=True)
    canvas=np.full((720,960,3),[22,28,40],np.uint8)
    canvas[:360,:640]=rgb3
    canvas[:240,640:]=cv2.cvtColor(annotated,cv2.COLOR_BGR2RGB)
    def text(message,x,y,size=.5,color=(225,235,245)):
        cv2.putText(canvas,message,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)
    text('IntelQcopter | PyBullet 3D, NOT Isaac | illustrative campus garden',15,385,.65)
    text(f't={t:.1f}s | {PHASES[phase]} | RGB YOLO + known-size PnP | downward 1D lidar',15,413,.57)
    text('No true obstacle positions or depth map sent to planner. Simulated odometry.',15,438,.52)
    text('Hardware motor arming DISABLED',650,275,.47,(255,170,90))
    text(f'Observed panels: {len(observed)}',650,300)
    if rows:
        a=np.array(rows);dur=max(10,t);xs=30+(a[:,0]/dur*590).astype(int)
        for value in [0,1,2]:
            y=660-value*95;cv2.line(canvas,(30,y),(620,y),(55,65,78),1);text(str(value),8,y+4,.4)
        for value in [0,dur/2,dur]:
            x=int(30+value/dur*590);text(f'{value:.0f}s',x-8,681,.38)
        for col,color,label in [(3,(80,220,255),'height'),(20,(255,180,90),'estimated height')]:
            ys=660-(a[:,col]*95).astype(int);pts=np.column_stack([xs,np.clip(ys,470,670)])
            cv2.polylines(canvas,[pts],False,color,2);text(label,35 if col==3 else 150,490,.5,color)
        text('Height [m] vs simulated time',35,700,.5)
        # Observed map + flight path only; no ground-truth obstacle map overlay.
        for cx,cy,rad in observed:
            cv2.circle(canvas,(int(670+cx*27),int(565-cy*27)),int(rad*27),(250,135,75),1)
        pts=np.column_stack([670+a[:,1]*27,565-a[:,2]*27]).astype(np.int32)
        cv2.polylines(canvas,[pts],False,(80,230,160),2);text('Built map + flight path',650,700,.48)
    return np.ascontiguousarray(canvas)

def run_case(name,layout,seed,model,weights,video=False,dropout=False,boost=0,ros=False,selection=False,gain_scale=1.3):
    source_files=['flight_controller/flight_controller.c','companion/ros2_ws/src/qcopter_autonomy/src/mission.cpp',
      'companion/ros2_ws/src/qcopter_autonomy/src/navigation.cpp','companion/perception/panel_pose.py',
      'sim/garden/world.py','sim/garden/run_mission.py','sim/ros2_garden_bridge.py']
    provenance={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source_files}
    model=dict(model)
    model.update(start_m=[0,0,.082],goal_m=[8,0,.082],
      perception_mode='single RGB YOLO + known-size panel PnP, downward 1D lidar and simulated local odometry',
      barometer_sigma_m=.03,accel_sigma_g=.003,downward_lidar_sigma_m=.01)
    model.pop('obstacles_x_y_radius_m',None) # old suite geometry is not an input to this planner
    OUT.mkdir(parents=True,exist_ok=True);rng=np.random.default_rng(seed)
    fc,nav=libraries();fc.qc_sitl_height_observation.argtypes=[ct.c_float,ct.c_float]
    if ros:
        from ros_transport import RosFlight
        fc=RosFlight()
    nav.qc_mission_start.argtypes=[F64,F64,F64,ct.c_double];nav.qc_mission_start.restype=ct.c_int
    nav.qc_mission_step.argtypes=[F64,F64,ct.c_double,ct.c_uint,F64]
    start=np.array([0,0,.082],np.float64);finish=np.array([8,0,.082],np.float64);cruise=1.25
    if selection:
        if not ros:raise ValueError('ROS selection needs --ros')
        points=fc.selected_mission();start=points[:3].astype(np.float64);finish=points[3:6].astype(np.float64);cruise=float(points[6])
        model.update(start_m=start.tolist(),goal_m=finish.tolist())
    world=Garden(model,seed,layout);drone=world.create_drone(start)
    detector=YOLO(str(weights));detector.predict(np.zeros((240,320,3),np.uint8),imgsz=320,device=0,verbose=False)
    fc.qc_sitl_reset(model['mass_kg']*model['gravity_mps2']/24,start[2],gain_table(model,gain_scale).ravel())
    state=np.r_[start,[0,0,0],0].astype(np.float64);assert nav.qc_mission_start(state,start,finish,cruise)
    command=np.array([0,0,0,.5,boost],np.float32);navout=np.zeros(6,np.float64);mission=np.zeros(9,np.float64)
    motors=np.zeros(4,np.float32);diag=np.zeros(16,np.float32);actual=np.zeros(4)
    observed=[];observations=[];events=[];rows=[];last_velocity=np.zeros(3);acc=np.zeros(3)
    last_image=-99.;last_plan=-99.;last_phase=-1;rgb=np.zeros((240,320,3),np.uint8);annotated=rgb.copy()
    contact=False;collision=False;min_clearance=99.;range_count=0;replans=0;yolo_frames=0;done_hold_steps=0;startwall=time.perf_counter()
    recorder=None
    if video:
        recorder=imageio_ffmpeg.write_frames(str(OUT/f'{name}.mp4'),(960,720),fps=10,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
        recorder.send(None)
    for step in range(45000):
        t=step*.002;pos,q=p.getBasePositionAndOrientation(drone);pos=np.array(pos);r=np.array(p.getMatrixFromQuaternion(q)).reshape(3,3)
        vel,omega=p.getBaseVelocity(drone);vel=np.array(vel);omega_body=r.T@np.array(omega)
        roll,pitch,yaw=p.getEulerFromQuaternion(q)
        if step%100==0 and not(dropout and 15<t<16):
            rgb,_unused_ids=world.camera(pos,r) # IDs deliberately discarded; depth never requested by navigation.
            bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR);prediction=detector.predict(bgr,imgsz=320,device=0,conf=.35,verbose=False)[0]
            detections=[(box.xyxy[0].cpu().numpy(),float(box.conf.item())) for box in prediction.boxes]
            measured=panel_observations(bgr,detections,K);annotated=bgr.copy();yolo_frames+=1;last_image=t
            for bbox,score in detections:
                x1,y1,x2,y2=map(int,bbox);cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,190,250),1)
                cv2.putText(annotated,f'YOLO panel {score:.2f}',(max(1,x1),max(12,y1)),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,50,250),1)
            pose_noise=rng.normal(0,.005,3) # simulated odometry; no claim of a working VIO implementation
            camera_origin=pos+pose_noise+r@np.array([.18,0,.02]);r_wc=np.column_stack([-r[:,1],-r[:,2],r[:,0]])
            for measurement in measured:
                point=camera_origin+r_wc@measurement['position_optical_m'];radius=.61+min(.3,2*measurement['range_sigma_m'])
                if not(-1<point[0]<10 and abs(point[1])<3.5):continue
                found=next((i for i,o in enumerate(observed) if np.linalg.norm(point[:2]-o[:2])<.65),None)
                entry=np.array([point[0],point[1],radius])
                if found is None:observed.append(entry);events.append({'time_s':t,'event':'new RGB-derived obstacle','position':point.tolist()})
                else:observed[found]=.8*observed[found]+.2*entry
                obs={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in measurement.items()}
                obs.update(time_s=t,estimated_world=point.tolist());observations.append(obs)
                cv2.polylines(annotated,[measurement['corners'].astype(np.int32)],True,(0,240,0),2)
                corner=measurement['corners'][0].astype(int)
                cv2.putText(annotated,f"panel {np.linalg.norm(measurement['position_optical_m']):.2f}m",tuple(corner),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,255),1)
            if step%1000==0:cv2.imwrite(str(OUT/f'{name}-camera.jpg'),annotated)
            if step in [3000,6000,10000]:cv2.imwrite(str(OUT/f'{name}-camera-{int(t)}s.png'),annotated)
        if step%25==0:
            state=np.r_[pos+rng.normal(0,.003,3),vel+rng.normal(0,.005,3),yaw].astype(np.float64)
            fresh=t-last_image<.55
            if t-last_plan>=.4:
                obs_array=np.array(observed,np.float64).ravel() if observed else np.empty(0,np.float64)
                nav.qc_nav_plan(state,np.array([finish[0],finish[1],cruise],np.float64),obs_array,len(observed));replans+=1;last_plan=t
            nav.qc_nav_step(state,int(fresh),navout)
            contact=any(c[2]==world.ground for c in p.getContactPoints(bodyA=drone))
            nav.qc_mission_step(state,navout,.05,1|(2 if fresh else 0)|(4 if contact else 0),mission)
            command[:4]=mission[:4]
            command[4]=boost if int(mission[5]) in [2,3,4] else 0 # boost inhibited during approach/landing
            if int(mission[5])!=last_phase:
                last_phase=int(mission[5]);events.append({'time_s':t,'phase':PHASES[last_phase]})
                print(name,round(t,2),PHASES[last_phase],flush=True)
        baro=pos[2]+rng.normal(0,.03) if step%10==0 else baro
        specific=r.T@(acc+np.array([0,0,model['gravity_mps2']]))/model['gravity_mps2']+rng.normal(0,.003,3)
        phi,theta=np.deg2rad(diag[:2]);vertical=(-np.sin(theta)*specific[0]+np.sin(phi)*np.cos(theta)*specific[1]+np.cos(phi)*np.cos(theta)*specific[2])*model['gravity_mps2']-model['gravity_mps2']
        sensor=np.r_[specific,np.rad2deg(omega_body)+rng.normal(0,.2,3),vertical,baro,float(step%10==0)].astype(np.float32)
        fc.qc_sitl_step(sensor,command,.002,15,motors,diag)
        if step%10==0:
            ray=world.downward_range(pos,r)
            if ray is not None and ray[0]>=0:
                distance=ray[2]*8+rng.normal(0,.01)
                if .03<distance<6:
                    measured_h=(distance+.085)*np.cos(phi)*np.cos(theta)
                    fc.qc_sitl_height_observation(measured_h,.0004);range_count+=1
        applied=motors*mission[6] if mission[7] else np.zeros(4)
        actual+=(1-np.exp(-.002/.04))*(applied-actual);thrust=actual*model['maximum_thrust_per_motor_n'];a=.45/(2*np.sqrt(2))
        torque=np.array([a*(thrust[0]-thrust[1]-thrust[2]+thrust[3]),a*(-thrust[0]-thrust[1]+thrust[2]+thrust[3]),.015*(-thrust[0]+thrust[1]-thrust[2]+thrust[3])])
        p.applyExternalForce(drone,-1,[0,0,float(thrust.sum())],[0,0,0],p.LINK_FRAME)
        p.applyExternalTorque(drone,-1,torque,p.LINK_FRAME)
        p.applyExternalForce(drone,-1,-.12*vel,pos,p.WORLD_FRAME)
        p.stepSimulation();new_vel=np.array(p.getBaseVelocity(drone)[0]);acc=(new_vel-last_velocity)/.002;last_velocity=new_vel
        for panel in world.panels:
            points=p.getClosestPoints(drone,panel,2.)
            if points:min_clearance=min(min_clearance,min(c[8] for c in points))
            if p.getContactPoints(drone,panel):collision=True
        if step%10==0:
            rows.append([t,*pos,*vel,np.degrees(roll),np.degrees(pitch),np.degrees(yaw),*command[:4],*motors,*diag,
                         int(mission[5]),len(observed),int(contact),int(t-last_image<.55),mission[7],mission[6],*actual])
        if recorder and step%50==0:recorder.send(video_frame(world,rgb,annotated,rows,int(mission[5]),observed,t))
        done_hold_steps=done_hold_steps+1 if mission[8] else 0
        if done_hold_steps>=250 or collision or pos[2]>4 or abs(roll)>.8 or abs(pitch)>.8:break
    if recorder:recorder.close()
    data=np.array(rows);headers=['time_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','roll_deg','pitch_deg','yaw_deg',
      'roll_command','pitch_command','yaw_rate_command','throttle','motor_fl','motor_fr','motor_rr','motor_rl',
      'estimated_roll','estimated_pitch','estimated_height','estimated_vz','K_roll','K_bias','K_height','K_velocity',
      'sigma_roll','sigma_height','innovation_roll','innovation_height','height_reference','hardware_armed','sensor_fault','S_angle',
      'phase','observed_count','contact','camera_fresh','simulation_actuator_gate','spool_fraction',
      'actual_motor_fl','actual_motor_fr','actual_motor_rr','actual_motor_rl']
    np.savetxt(OUT/f'{name}.csv',data,delimiter=',',header=','.join(headers),comments='')
    (OUT/f'{name}-observations.json').write_text(json.dumps(observations,indent=2),encoding='utf-8')
    (OUT/f'{name}-events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
    actual_contact=any(c[2]==world.ground for c in p.getContactPoints(bodyA=drone))
    final_error=float(np.linalg.norm(pos[:2]-finish[:2]));success=bool(mission[8] and actual_contact and not collision and final_error<.4 and max(actual)<.001)
    result={'name':name,'seed':seed,'layout':layout,'success':success,'duration_s':t,'final_xy_error_m':final_error,
       'maximum_height_m':float(data[:,3].max()),'minimum_panel_clearance_m':min_clearance,'collision':collision,
       'final_phase':PHASES[int(mission[5])],'rendered_yolo_frames':yolo_frames,'valid_range_observations':len(observations),
       'downward_lidar_updates':range_count,'replans':replans,'observed_panels':len(observed),'wall_s':time.perf_counter()-startwall,
       'method':'PyBullet + production C + C++ mission/nav + actual trained YOLO RGB inference + known-size planar PnP; downward ray lidar; simulated odometry',
       'no_depth_or_true_obstacle_map_to_planner':True,'model':model,'hardware_armed':False,
       'weights_sha256':hashlib.sha256(weights.read_bytes()).hexdigest(),'simulated_image_dropout':dropout,'RT_boost':boost,'actual_ROS2_DDS_control':ros,
       'mission_points_selected_via_ROS2':selection,'cruise_height_m':cruise,'gain_scale':gain_scale,
       'source_sha256_at_start':provenance,'virtual_ground_contact_for_landing':True,
       'final_ground_contact':actual_contact,'final_actual_motor_max':float(max(actual)),
       'scope':'controlled known-size panel course; simulated odometry/contact; not arbitrary unknown-object autonomy'}
    (OUT/f'{name}-summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
    world.close()
    if ros:fc.close()
    return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--name',default='garden-nominal');ap.add_argument('--layout',type=int,default=0)
    ap.add_argument('--seed',type=int,default=2451);ap.add_argument('--video',action='store_true');ap.add_argument('--dropout',action='store_true');ap.add_argument('--boost',type=float,default=0);ap.add_argument('--ros',action='store_true');ap.add_argument('--ros-selection',action='store_true');ap.add_argument('--gain-scale',type=float,default=1.3)
    args=ap.parse_args();model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    weights=ROOT/'build/panel-training/rgb-panel/weights/best.pt'
    result=run_case(args.name,args.layout,args.seed,model,weights,args.video,args.dropout,args.boost,args.ros,args.ros_selection,args.gain_scale)
    if not result['success']:raise SystemExit('Mission did not pass. Preserve trace and diagnose; do not label it successful.')

if __name__=='__main__':main()
