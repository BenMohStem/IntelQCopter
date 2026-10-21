"""Reproducible hover-model calculations; assumed parameters, not measured gains."""
from pathlib import Path
import json
import numpy as np
from scipy.linalg import expm
from scipy.signal import StateSpace, lsim
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/assets/lti'
OUT.mkdir(parents=True,exist_ok=True)
D=.45; arm=D/(2*np.sqrt(2)); mass=1.2; g=9.80665; kt=6.; tau=.04; kappa=.015; dt=.002
Jx=.012; Jy=.012; Jz=.022; drag=.12
Kr=4*arm*kt/Jx*180/np.pi; Ky=4*kappa*kt/Jz*180/np.pi; Kz=4*kt/mass
M=np.array([[1,1,1,1],[arm,-arm,-arm,arm],[-arm,-arm,arm,arm],[-kappa,kappa,-kappa,kappa]])

def zoh(A,B):
    n=A.shape[0]; m=B.shape[1]
    e=expm(np.block([[A,B],[np.zeros((m,n+m))]])*dt)
    return e[:n,:n],e[:n,n:]

def feedback(plantA,plantB,c,error_row,kp,ki,derivative_row):
    # I[k+1]=I[k]+dt*e[k]; current step uses candidate I[k+1], matching C.
    e=np.asarray(error_row); d=np.asarray(derivative_row)
    Ad,Bd=zoh(plantA,plantB)
    w=(kp+ki*dt)*e+d
    F=np.block([[Ad-Bd@w[None,:],Bd*ki],[-dt*e[None,:],np.ones((1,1))]])
    G=np.r_[Bd*(kp+ki*dt)*c,[[dt*c]]]
    # Continuous controller comparison, before discretization.
    Ac=np.block([[plantA-plantB@(kp*e+d)[None,:],plantB*ki],[-e[None,:],np.zeros((1,1))]])
    Bc=np.r_[plantB*kp*c,[[c]]]
    return Ac,Bc,F,G

rollA=np.array([[0,1,0],[0,0,1],[0,0,-1/tau]])
rollB=np.array([[0],[0],[Kr/tau]])
altA=np.array([[0,1,0],[0,-drag/mass,1],[0,0,-1/tau]])
altB=np.array([[0],[0],[Kz/tau]])
yawA=np.array([[0,1],[0,-1/tau]])
yawB=np.array([[0],[Ky/tau]])
systems={
 'roll_angle':feedback(rollA,rollB,3,[3,1,0],.000829338,.003870245,[0,0,0]),
 'pitch_angle':feedback(rollA,rollB,3,[3,1,0],.000829338,.003870245,[0,0,0]),
 'yaw_rate':feedback(yawA,yawB,1,[1,0],.005375614,.008362066,[0,0]),
 'height_hold':feedback(altA,altB,1,[1,0,0],.108045,.03087,[0,.1225,0])}
report={'scope':'Assumed local hover LTI model, ideal state feedback, no saturation, noise or transport delay; not a hardware or Gazebo run.',
 'parameters':dict(D_m=D,lever_a_m=float(arm),mass_kg=mass,Jx_kg_m2=Jx,Jy_kg_m2=Jy,Jz_kg_m2=Jz,kT_N_per_command=kt,tau_motor_s=tau,kappa_m=kappa,drag_Ns_per_m=drag,dt_s=dt),
 'hover':dict(total_N=mass*g,each_N=mass*g/4,command_if_T_equals_6u=mass*g/(4*kt),firmware_placeholder=.5),
 'gains':dict(roll_accel_deg_s2_per_command=float(Kr),yaw_accel_deg_s2_per_command=float(Ky),vertical_accel_m_s2_per_collective=Kz),
 'allocation':M.tolist(),'allocation_inverse':np.linalg.inv(M).tolist(),'loops':{}}
plt.rcParams.update({'font.family':'DejaVu Sans','axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'figure.facecolor':'#f4f7fb','axes.facecolor':'white','font.size':10})
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for (name,(A,B,F,G)),ax in zip(systems.items(),axes.flat):
    poles=np.linalg.eigvals(A); zpoles=np.linalg.eigvals(F)
    duration=10 if name!='height_hold' else 18
    t=np.arange(0,duration+dt/2,dt); amplitude=1.0
    C=np.zeros((1,len(A)));C[0,0]=1
    _,continuous,_=lsim(StateSpace(A,B,C,np.zeros((1,1))),np.ones(len(t))*amplitude,t)
    state=np.zeros(len(A)); discrete=[]
    for _ in t:
        discrete.append(state[0]);state=F@state+G[:,0]*amplitude
    report['loops'][name]={'continuous_poles':[[float(v.real),float(v.imag)] for v in poles],
        'discrete_poles':[[float(v.real),float(v.imag)] for v in zpoles],
        'max_discrete_modulus':float(max(abs(zpoles))),
        'stable_continuous':bool(np.all(poles.real<0)), 'stable_discrete':bool(np.all(abs(zpoles)<1)),
        'A_closed':A.tolist(),'F_closed':F.tolist(),
        'max_continuous_discrete_output_difference':float(np.max(np.abs(continuous-np.array(discrete))))}
    ax.plot(t,continuous,label='Continuous LTI',color='#007f86',lw=2)
    ax.plot(t,discrete,label='2 ms sampled controller',color='#d37625',ls='--',lw=1.3)
    ax.axhline(1,color='#526779',lw=.8,ls=':')
    ax.set(title=name.replace('_',' ').title(),xlabel='Time (s)',ylabel='Response / unit command')
    ax.legend(fontsize=8)
fig.suptitle('Illustrative hover response · assumed F450 parameters',fontsize=16)
fig.savefig(OUT/'responses.svg');fig.savefig(OUT/'responses.png',dpi=150);plt.close(fig)

# Show exact sampled-state matrices for one rotor + one rotational axis.
Ad,Bd=zoh(rollA,rollB)
report['roll_plant_discrete']={'state':'[angle_deg, rate_deg_s, angular_acceleration_deg_s2]','Ad':Ad.tolist(),'Bd':Bd.tolist()}
# Full 16-state hover model: position, velocity, attitude, rates, four motor thrusts.
A=np.zeros((16,16));B=np.zeros((16,4))
A[0:3,3:6]=np.eye(3);A[3,7]=g;A[4,6]=-g
A[3:6,3:6]=-drag/mass*np.eye(3);A[5,12:16]=1/mass
A[6:9,9:12]=np.eye(3)
A[9:12,12:16]=np.diag([1/Jx,1/Jy,1/Jz])@M[1:,:]
A[12:16,12:16]=-np.eye(4)/tau;B[12:16,:]=kt*np.eye(4)/tau
Ad16,Bd16=zoh(A,B)
report['mimo16']={'state_order':['x','y','z','vx','vy','vz','phi','theta','psi','p','q','r','dT_FL','dT_FR','dT_RR','dT_RL'],
    'units':'m,m/s,rad,rad/s,N; inputs are normalized motor command perturbations',
    'A':A.tolist(),'B':B.tolist(),'Ad':Ad16.tolist(),'Bd':Bd16.tolist()}
(OUT/'calculations.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k in ['scope','parameters','hover','gains']},indent=2))
for name,values in report['loops'].items():
    print(name,'continuous poles=',values['continuous_poles'],'sampled |pole|max=',values['max_discrete_modulus'])
print('Saved',OUT)
