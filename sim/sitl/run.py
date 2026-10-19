"""Actual closed-loop 6-DOF CPU model calling the production C controller.

This is NOT Isaac rendering, ROS transport, hardware, or YOLO-driven navigation.
The same C++ navigation core is used by the ROS package and this baseline.
"""
from __future__ import annotations
import argparse, csv, ctypes as ct, json, math, time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
F32=np.ctypeslib.ndpointer(dtype=np.float32,flags='C_CONTIGUOUS')
F64=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')

def libraries():
    fc=ct.CDLL(str(ROOT/'build/sitl/flight_core.dll'))
    fc.qc_sitl_reset.argtypes=[ct.c_float,ct.c_float,F32]
    fc.qc_sitl_step.argtypes=[F32,F32,ct.c_float,ct.c_uint,F32,F32]
    nav=ct.CDLL(str(ROOT/'build/sitl/navigation.dll'))
    nav.qc_nav_plan.argtypes=[F64,F64,F64,ct.c_int];nav.qc_nav_plan.restype=ct.c_int
    nav.qc_nav_step.argtypes=[F64,ct.c_int,F64]
    nav.qc_nav_path.argtypes=[F64,ct.c_int];nav.qc_nav_path.restype=ct.c_int
    return fc,nav

def gain_table(model,scale=1.0):
    m=model['mass_kg'];jx,jy,jz=model['inertia_kg_m2'];thrust=model['maximum_thrust_per_motor_n']
    arm=model['motor_diagonal_m']/(2*math.sqrt(2));k=model['reaction_torque_per_thrust_m']
    bx=4*arm*thrust/jx*180/math.pi;by=4*arm*thrust/jy*180/math.pi;bz=4*k*thrust/jz*180/math.pi
    wn=12*scale;zeta=.9;wyaw=4*scale;wh=1.5*scale;pole=.8*scale;az=4*thrust/m
    return np.array([[3,0,0],[2*zeta*wn/bx,wn**2/bx,0],
                     [3,0,0],[2*zeta*wn/by,wn**2/by,0],
                     [2*zeta*wyaw/bz,wyaw**2/bz,0],
                     [(wh**2+2*zeta*wh*pole)/az,pole*wh**2/az,(2*zeta*wh+pole)/az]],np.float32)

def rotation(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def quat_multiply(a,b):
    w,x,y,z=a;v,i,j,k=b
    return np.array([w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v])

def run_case(fc,nav,nominal,out,name,scale=1.0,mass_scale=1.0,thrust_scale=1.0,seed=2350,duration=40,substeps=1):
    rng=np.random.default_rng(seed);p=nominal.copy();p['mass_kg']*=mass_scale;p['maximum_thrust_per_motor_n']*=thrust_scale
    dt=p['physics_dt_s'];g=p['gravity_mps2'];mass=p['mass_kg'];j=np.array(p['inertia_kg_m2'])
    arm=p['motor_diagonal_m']/(2*np.sqrt(2));max_thrust=p['maximum_thrust_per_motor_n']
    hover=nominal['mass_kg']*g/(4*nominal['maximum_thrust_per_motor_n'])
    gains=gain_table(nominal,scale);fc.qc_sitl_reset(hover,p['start_m'][2],gains.ravel())
    pos=np.array(p['start_m'],float);vel=np.zeros(3);q=np.array([1.,0,0,0]);omega=np.zeros(3)
    motor=np.full(4,mass*g/(4*max_thrust));acc=np.zeros(3)
    obs=np.array(p['obstacles_x_y_radius_m'],np.float64).ravel();goal=np.array(p['goal_m'],np.float64)
    state=np.r_[pos,vel,0.].astype(np.float64)
    if not nav.qc_nav_plan(state,goal,obs,len(obs)//3):raise RuntimeError('No navigation path')
    path=np.zeros(4096,np.float64);npath=nav.qc_nav_path(path,2048)
    np.savetxt(out/f'{name}-path.csv',path[:npath*2].reshape(-1,2),delimiter=',',header='x_m,y_m',comments='')
    diag=np.zeros(16,np.float32);motors=np.zeros(4,np.float32);navout=np.zeros(6,np.float64)
    command=np.array([0,0,0,.5],np.float32);baro=pos[2];rows=[];reached_dwell=0;failed=False
    minimum_clearance=float('inf');saturation=0;start=time.perf_counter()
    for step in range(int(duration/dt)):
        t=step*dt;r=rotation(q)
        roll=math.atan2(r[2,1],r[2,2]);pitch=math.asin(np.clip(-r[2,0],-1,1));yaw=math.atan2(r[1,0],r[0,0])
        if step%25==0:
            state=np.r_[pos,vel,yaw].astype(np.float64);nav.qc_nav_step(state,1,navout)
            command[:]=navout[:4]
        baro_new=step%10==0
        if baro_new:baro=pos[2]+rng.normal(0,p['barometer_sigma_m'])
        specific=r.T@(acc+np.array([0.,0,g]))/g+rng.normal(0,p['accel_sigma_g'],3)
        # Gravity compensation uses the previous C attitude estimate, not truth.
        phi,theta=np.deg2rad(diag[:2])
        vertical=(-np.sin(theta)*specific[0]+np.sin(phi)*np.cos(theta)*specific[1]+np.cos(phi)*np.cos(theta)*specific[2])*g-g
        sensor=np.r_[specific,np.rad2deg(omega)+rng.normal(0,p['gyro_sigma_dps'],3),vertical,baro,float(baro_new)].astype(np.float32)
        fc.qc_sitl_step(sensor,command,dt,7,motors,diag)
        saturation+=int(np.any((motors<.001)|(motors>.999)))
        h=dt/substeps
        for substep in range(substeps):
            motor+=(1-np.exp(-h/p['thrust_time_constant_s']))*(motors-motor)
            thrust=max_thrust*motor
            torque=np.array([arm*(thrust[0]-thrust[1]-thrust[2]+thrust[3]),
                             arm*(-thrust[0]-thrust[1]+thrust[2]+thrust[3]),
                             p['reaction_torque_per_thrust_m']*(-thrust[0]+thrust[1]-thrust[2]+thrust[3])])
            wind=np.array([.4,-.3,0.]) if 7<t<8 else np.zeros(3)
            acc=rotation(q)@np.array([0.,0,thrust.sum()])/mass-np.array([0.,0,g])-p['linear_drag_ns_per_m']*vel/mass+wind/mass
            pos+=vel*h+.5*acc*h*h;vel+=acc*h
            omega+=(torque-np.cross(omega,j*omega))/j*h
            angle=np.linalg.norm(omega)*h
            dq=np.r_[math.cos(angle/2),omega*(math.sin(angle/2)/(np.linalg.norm(omega)+1e-30))]
            q=quat_multiply(q,dq);q/=np.linalg.norm(q)
        clearance=min(np.hypot(pos[0]-o[0],pos[1]-o[1])-o[2]-p['robot_collision_radius_m'] for o in p['obstacles_x_y_radius_m'])
        minimum_clearance=min(minimum_clearance,float(clearance))
        if step%5==0:
            rows.append([t,*pos,*vel,math.degrees(roll),math.degrees(pitch),math.degrees(yaw),
                         *command,*motors,*diag,clearance])
        reached_dwell=reached_dwell+1 if navout[5] else 0
        if reached_dwell>int(1/dt):break
        if not np.all(np.isfinite(pos)) or pos[2]<.15 or np.linalg.norm(pos)>50 or abs(roll)>1 or abs(pitch)>1:
            failed=True;break
    columns=['time_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','roll_deg','pitch_deg','yaw_deg',
             'roll_command_deg','pitch_command_deg','yaw_command_dps','throttle',
             'motor_fl','motor_fr','motor_rr','motor_rl','estimated_roll_deg','estimated_pitch_deg',
             'estimated_height_m','estimated_vz_mps','K_roll_angle','K_roll_bias','K_height','K_velocity',
             'sigma_roll_deg','sigma_height_m','roll_innovation_deg','height_innovation_m',
             'height_reference_m','hardware_armed','sensor_fault','angle_innovation_variance','clearance_m']
    data=np.array(rows);np.savetxt(out/f'{name}.csv',data,delimiter=',',header=','.join(columns),comments='')
    final=float(np.linalg.norm(pos-goal));rmse=float(np.sqrt(np.mean((data[:,3]-goal[2])**2)))
    result={'name':name,'seed':seed,'gain_scale':scale,'mass_scale':mass_scale,'thrust_scale':thrust_scale,
            'success':bool(not failed and minimum_clearance>0 and final<.3 and reached_dwell>0),
            'duration_s':float(t),'final_position_error_m':final,'height_rmse_m':rmse,
            'minimum_surface_clearance_m':minimum_clearance,'saturation_fraction':saturation/(step+1),
            'cpu_wall_s':time.perf_counter()-start,'physics_substeps':substeps,'gains':gains.tolist(),'hardware_outputs_enabled':False,
            'perception_mode':p['perception_mode']}
    print(json.dumps(result),flush=True);return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=ROOT/'artifacts/simulation/2026-10-20')
    parser.add_argument('--quick',action='store_true');parser.add_argument('--duration',type=float,default=40)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    fc,nav=libraries();results=[]
    for scale in ([1.0] if args.quick else [.7,1.0,1.3]):
        results.append(run_case(fc,nav,model,args.out,f'nominal-{scale:.1f}',scale,duration=args.duration))
    successes=[r for r in results if r['success']]
    def score(r):return r['duration_s']+20*r['height_rmse_m']+10*r['final_position_error_m']+2/(r['minimum_surface_clearance_m']+.01)
    best=min(successes,key=score) if successes else None
    if best and not args.quick:
        for name,mass,thrust,seed in [('heavy',1.15,1,2351),('weak-thrust',1,.85,2352),('light',.85,1.1,2353)]:
            results.append(run_case(fc,nav,model,args.out,f'heldout-{name}',best['gain_scale'],mass,thrust,seed,duration=args.duration))
    summary={'model':model,'selection_rule':'successful, collision-free nominal cases ranked by time +20*heightRMSE +10*finalError +2/(clearance+.01)',
             'best_nominal_case':best['name'] if best else None,'best_nominal_gains':best['gains'] if best else None,
             'results':results,'engine':'CPU 6DOF + production C controller + common C++ navigation; not Isaac/ROS/YOLO'}
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    if not successes:raise SystemExit('No successful case: preserve failed traces for diagnosis')

if __name__=='__main__':main()
