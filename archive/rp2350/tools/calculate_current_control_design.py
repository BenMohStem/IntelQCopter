"""Worked model calculations, not implementation tests or automatic firmware tuning.

Calculate gains, continuous local poles and conditional periodic Kalman gains.
The default matches the documented 1.2 kg flight baseline; a second calculation
uses the separate provisional component model. No hardware parameters are inferred.
"""
from pathlib import Path
import json, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/model-design/current'
OUT.mkdir(parents=True,exist_ok=True)
baseline=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
components=json.loads((ROOT/'companion/ros2_ws/src/qcopter_description/urdf/mass-properties.json').read_text())

def roots(coefficients):
    return [{'real_per_s':float(z.real),'imag_rad_per_s':float(z.imag)}
            for z in np.roots(coefficients)]

def quartic(coefficients):
    a4,a3,a2,a1,a0=coefficients
    d2=a3*a2-a4*a1
    d3=a3*a2*a1-a4*a1*a1-a3*a3*a0
    return {'coefficients_descending':list(map(float,coefficients)),
            'routh_delta2':float(d2),'routh_delta3':float(d3),
            'continuous_local_hurwitz':bool(min(coefficients)>0 and d2>0 and d3>0),
            'poles':roots(coefficients)}

def design(model):
    m=model['mass_kg'];jx,jy,jz=model['inertia_kg_m2']
    tmax=model['maximum_thrust_per_motor_n'];tau=model['thrust_time_constant_s']
    a=model['motor_diagonal_m']/(2*math.sqrt(2));kappa=model['reaction_torque_per_thrust_m']
    b=[4*a*tmax/jx*180/math.pi,4*a*tmax/jy*180/math.pi,4*kappa*tmax/jz*180/math.pi]
    bz=4*tmax/m;lam=model['linear_drag_ns_per_m']/m
    scale=.7;zeta=.9;wn=12*scale;wy=4*scale;wh=1.5*scale;p=.8*scale;ka=3.
    gains=[[ka,0,0],[2*zeta*wn/b[0],wn**2/b[0],0],
           [ka,0,0],[2*zeta*wn/b[1],wn**2/b[1],0],
           [2*zeta*wy/b[2],wy**2/b[2],0],
           [(wh**2+2*zeta*wh*p)/bz,p*wh**2/bz,(2*zeta*wh+p)/bz]]
    result={'status':model['status'],'mass_kg':m,'inertia_kg_m2':[jx,jy,jz],
            'assumed_thrust_max_each_n':tmax,'assumed_motor_time_constant_s':tau,
            'arm_coordinate_m':a,'weight_n':m*model['gravity_mps2'],
            'hover_thrust_each_n':m*model['gravity_mps2']/4,
            'linear_thrust_model_hover_collective':m*model['gravity_mps2']/(4*tmax),
            'b_roll_pitch_yaw_deg_s2_per_normalized_command':b,
            'b_altitude_m_s2_per_normalized_collective':bz,'vertical_drag_rate_per_s':lam,
            'gain_order':['roll_angle','roll_rate','pitch_angle','pitch_rate','yaw_rate','altitude'],
            'current_scale_0_7_gains_kp_ki_kd':gains,'local_continuous_dynamics':{}}
    for axis,index,loop in [('roll',0,1),('pitch',1,3),('yaw',2,4)]:
        kp,ki,_=gains[loop];beta=b[index]
        cubic=[tau,1,beta*kp,beta*ki]
        record={'rate_characteristic':cubic,'rate_poles':roots(cubic),
                'rate_routh_kp_minus_tau_ki':kp-tau*ki}
        if axis!='yaw':record['full_angle_cascade']=quartic([tau,1,beta*kp,beta*(ki+ka*kp),ka*beta*ki])
        result['local_continuous_dynamics'][axis]=record
    kp,ki,kd=gains[5]
    result['local_continuous_dynamics']['altitude']=quartic([tau,1+tau*lam,lam+bz*kd,bz*kp,bz*ki])
    # Alternative analytic candidate: include the known assumed lag/vertical drag
    # in coefficient matching. This is calculated, not installed or flight-tested.
    rho=1/tau+lam-p-2*zeta*wh
    altitude_candidate=[tau*(p*wh**2+rho*(wh**2+2*zeta*wh*p))/bz,
                        tau*rho*p*wh**2/bz,
                        (tau*(wh**2+2*zeta*wh*p+rho*(p+2*zeta*wh))-lam)/bz]
    ck,ci,cd=altitude_candidate
    result['uninstalled_lag_and_drag_matched_altitude_candidate']={
        'kp_ki_kd':altitude_candidate,'additional_pole_rate_per_s':rho,
        'target':{'omega_n_rad_s':wh,'zeta':zeta,'real_pole_rate_per_s':p},
        'local_model':quartic([tau,1+tau*lam,lam+bz*cd,bz*ck,bz*ci]),
        'status':'Analytic candidate only; no nonlinear/sampled/hardware validation or automatic replacement'}
    return result

def observe(P,R):
    gain=P[:,0]/(P[0,0]+R)
    A=np.eye(2)-np.outer(gain,[1.,0.])
    return A@P@A.T+np.outer(gain,gain)*R,gain

def angle_riccati():
    dt=.002;qb=.0001;gyro_variance=.04;R=4.
    F=np.array([[1.,-dt],[0.,1.]])
    Q=np.array([[gyro_variance*dt**2+qb*dt**3/3,-qb*dt**2/2],[-qb*dt**2/2,qb*dt]])
    P=np.diag([1.,.01]);change=math.inf
    for step in range(300000):
        old=P.copy();P,gain=observe(F@P@F.T+Q,R);change=float(np.max(np.abs(P-old)))
        if step>1000 and change<1e-13:break
    return {'state':['angle_deg','residual_bias_dps'],'dt_s':dt,'F':F.tolist(),'Q':Q.tolist(),'R_deg2':R,
            'limiting_gain':gain.tolist(),'posterior_covariance':P.tolist(),
            'sigma_angle_deg':float(np.sqrt(P[0,0])),'sigma_bias_dps':float(np.sqrt(P[1,1])),
            'iterations':step+1,'last_covariance_change':change,
            'scope':'Conditional Riccati limit, valid only for this linear stochastic model; not measured sensor noise'}

def altitude_riccati(lidar=False,Rbaro=.0025):
    dt=.002;F=np.array([[1.,dt],[0.,1.]])
    G=np.array([dt**2/2,dt]);Q=np.outer(G,G)*1.5**2
    P=np.diag([.04,.1]);change=math.inf;kl=None
    for cycle in range(100000):
        old=P.copy()
        for _ in range(10):P=F@P@F.T+Q
        P,kb=observe(P,Rbaro)
        if lidar:P,kl=observe(P,.0004)
        change=float(np.max(np.abs(P-old)))
        if cycle>100 and change<1e-13:break
    return {'state':['height_m','vertical_velocity_m_s'],'prediction_dt_s':dt,
            'barometer_period_s':.020,'F':F.tolist(),'Q_per_prediction':Q.tolist(),'R_baro_m2':Rbaro,
            'R_lidar_m2':.0004 if lidar else None,'barometer_limiting_gain_at_correction':kb.tolist(),
            'lidar_limiting_gain_after_baro':None if kl is None else kl.tolist(),
            'posterior_covariance_after_last_correction':P.tolist(),'cycles':cycle+1,
            'last_cycle_covariance_change':change,
            'scope':'Periodic covariance calculation: independent observations every 10 predictions; actual startup/dropout gains vary'}

component_model=dict(baseline,mass_kg=components['mass_kg'],
                     inertia_kg_m2=np.diag(components['inertia_about_com_kg_m2']).tolist(),
                     status='Separate provisional 740 g component estimate; inherits assumed thrust/lag; not the flight baseline')
report={'baseline':design(baseline),'component_estimate':design(component_model),
        'kalman':{'angle_current_model':angle_riccati(),
                  'altitude_baro_only_current_tuning':altitude_riccati(),
                  'altitude_baro_and_lidar_current_tuning':altitude_riccati(True),
                  'altitude_baro_sigma_0_03_illustration':altitude_riccati(Rbaro=.03**2)},
        'no_firmware_or_simulation_parameters_changed':True,
        'scope':'Mathematical calculations for declared assumptions, not implementation tests or physical identification'}
(OUT/'calculations.json').write_text(json.dumps(report,indent=2))
fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
roll=report['baseline']['local_continuous_dynamics']['roll']['full_angle_cascade']['poles']
alt=report['baseline']['local_continuous_dynamics']['altitude']['poles']
candidate=report['baseline']['uninstalled_lag_and_drag_matched_altitude_candidate']['local_model']['poles']
for ax,groups,title in [(axes[0],[(roll,'Current roll/pitch cascade','x')],'Roll/pitch: ideal-feedback local model'),
                        (axes[1],[(alt,'Current altitude gains','x'),(candidate,'Uninstalled analytic candidate','o')],'Altitude: assumed lag + drag')]:
    for values,label,marker in groups:
        ax.scatter([v['real_per_s'] for v in values],[v['imag_rad_per_s'] for v in values],label=label,marker=marker,s=70)
    ax.axvline(0,color='red',lw=1);ax.axhline(0,color='grey',lw=.6)
    ax.grid(alpha=.25);ax.set_xlabel('Real part (1/s)');ax.set_ylabel('Imaginary part (rad/s)')
    ax.set_title(title,fontsize=10);ax.legend(fontsize=8)
fig.suptitle('Calculated continuous poles — assumed 1.2 kg plant; not sampled/full-flight validation',fontsize=11)
fig.savefig(OUT/'local-model-poles.png',dpi=180);plt.close(fig)
for name in ['baseline','component_estimate']:
    print(name,'mass_kg',report[name]['mass_kg'],'gains',report[name]['current_scale_0_7_gains_kp_ki_kd'])
print('angle limiting gains',report['kalman']['angle_current_model']['limiting_gain'])
print('altitude gains',report['kalman']['altitude_baro_only_current_tuning']['barometer_limiting_gain_at_correction'])
print('Report:',OUT/'calculations.json')
