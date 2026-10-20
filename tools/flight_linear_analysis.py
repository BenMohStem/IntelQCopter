"""Numerical LTI analysis matching the paper; no generic algebra tutorial."""
import json,sys,itertools
from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'sim/sitl'))
from run import gain_table
OUT=ROOT/'artifacts/simulation/2026-10-20-missions'

def routh(a):
    if len(a)==4:
        a3,a2,a1,a0=a;return [a3,a2,(a2*a1-a3*a0)/a2,a0]
    a4,a3,a2,a1,a0=a;d2=a3*a2-a4*a1;d3=a1*d2-a3*a3*a0
    return [a4,a3,d2/a3,d3/d2,a0]

def sampled(kind,b,kp,ki,kd,dt=.002,tau=.04,drag=.1,ka=3,delay=False):
    if kind in ['yaw','roll_rate']:
        a=np.array([[0,b],[0,-1/tau]]);u=np.array([[0],[1/tau]]);ce=np.array([-1,0.]);fb=(kp+ki*dt)*ce
    else:
        a=np.array([[0,1,0],[0,-drag if kind=='height' else 0,b],[0,0,-1/tau]])
        u=np.array([[0],[0],[1/tau]])
        ce=np.array([-1,0,0.]) if kind=='height' else np.array([-ka,-1,0.])
        fb=np.array([-kp-ki*dt,-kd,0.]) if kind=='height' else (kp+ki*dt)*ce
    ad,bd,_,_,_=signal.cont2discrete((a,u,np.eye(len(a)),np.zeros((len(a),1))),dt)
    if not delay:
        closed=np.block([[ad+bd@fb[None,:],bd*ki],[dt*ce[None,:],np.ones((1,1))]])
    else:
        n=len(a);closed=np.zeros((n+2,n+2));closed[:n,:n]=ad;closed[:n,-1:]=bd
        closed[n,:n]=dt*ce;closed[n,n]=1;closed[-1,:n]=fb;closed[-1,n]=ki
    return np.linalg.eigvals(closed)

def metrics(num,den):
    t=np.linspace(0,20,20001);_,y=signal.step((num,den),T=t);dc=np.polyval(num,0)/np.polyval(den,0)
    bad=np.flatnonzero(abs(y-dc)>.02*abs(dc));settle=float(t[bad[-1]+1]) if len(bad) and bad[-1]<len(t)-1 else (0 if not len(bad) else None)
    return t,y,{'dc_gain':float(dc),'overshoot_percent':float(max(0,(max(y)-dc)/abs(dc))*100),
      'settling_2percent_s':settle,'peak_s':float(t[np.argmax(y)])}

def margin(num,den):
    w=np.geomspace(.001,1e4,100000);v=np.polyval(num,1j*w)/np.polyval(den,1j*w)
    db=20*np.log10(abs(v));ph=np.unwrap(np.angle(v))*180/np.pi
    crossings=np.flatnonzero(np.diff(np.sign(db)))
    if not len(crossings):return {'phase_margin_deg':None}
    i=int(crossings[0]);f=-db[i]/(db[i+1]-db[i]);wc=np.exp(np.log(w[i])+f*np.log(w[i+1]/w[i]));phase=ph[i]+f*(ph[i+1]-ph[i])
    pm=180+phase
    return {'gain_crossover_rad_s':float(wc),'phase_margin_deg':float(pm),
            'rough_extra_delay_s':float(pm*np.pi/180/wc) if pm>0 else 0}

def horizontal_sampled(b,kp,ki,kx,kv,dt=.002,tau=.04,drag=.1):
    # State theta_deg,p_deg_s,actuator,x_m,v_m_s,integral. Actual FC update at500Hz.
    a=np.zeros((5,5));a[0,1]=1;a[1,2]=b;a[2,2]=-1/tau;a[3,4]=1
    a[4,0]=9.80665*np.pi/180;a[4,4]=-drag
    u=np.array([[0],[0],[1/tau],[0],[0]])
    ad,bd,_,_,_=signal.cont2discrete((a,u,np.eye(5),np.zeros((5,1))),dt)
    ce=np.array([-3,-1,0,0,0.]);k=kp+ki*dt
    f=np.block([[ad+bd@(k*ce[None,:]),bd*ki],[dt*ce[None,:],np.ones((1,1))]])
    g=np.r_[bd[:,0]*k*3,dt*3][:,None]
    count=25;power=np.eye(6);summed=np.zeros((6,1))
    for j in range(count):summed+=power@g;power=power@f
    guide=np.array([0,0,0,-kx,-kv,0.])[None,:]*180/(np.pi*9.80665)
    lifted=power+summed@guide
    delayed=np.block([[power,summed],[guide,np.zeros((1,1))]])
    return float(max(abs(np.linalg.eigvals(lifted)))),float(max(abs(np.linalg.eigvals(delayed))))

def main():
    OUT.mkdir(parents=True,exist_ok=True);m=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    a=m['motor_diagonal_m']/(2*np.sqrt(2));bx=4*a*6/.012*180/np.pi;byaw=4*.015*6/.022*180/np.pi;bh=20;tau=.04;drag=.1
    result={'parameters':{'b_roll_deg':bx,'b_yaw_deg':byaw,'b_height':bh,'tau_s':tau,'drag_per_s':drag},'models':{},'finite_grid':{},'horizontal_guidance':{}}
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for scale in [.7,1.,1.3]:
        gains=gain_table(m,scale);rp,ri,_=gains[1];yp,yi,_=gains[4];hp,hi,hd=gains[5]
        models={'angle':([bx*3*rp,bx*3*ri],[tau,1,bx*rp,bx*(ri+3*rp),bx*3*ri]),
          'roll_rate':([bx*rp,bx*ri],[tau,1,bx*rp,bx*ri]),
          'yaw':([byaw*yp,byaw*yi],[tau,1,byaw*yp,byaw*yi]),
          'height':([bh*hd,bh*hp,bh*hi],[tau,1+drag*tau,drag+bh*hd,bh*hp,bh*hi])}
        for ax,(name,(num,den)) in zip(axes.flat,models.items()):
            poles=np.roots(den);assert max(poles.real)<0
            t,y,step=metrics(num,den);ax.plot(t,y,label=f'scale {scale}');ax.set(xlim=(0,6),xlabel='time [s]',ylabel='normalized response',title=name);ax.grid(alpha=.3);ax.legend()
            kindex=4 if name=='yaw' else 5 if name=='height' else 1
            kp,ki,kd=gains[kindex];kind=name
            ds=sampled(kind,byaw if name=='yaw' else bh if name=='height' else bx,kp,ki,kd)
            delayed=sampled(kind,byaw if name=='yaw' else bh if name=='height' else bx,kp,ki,kd,delay=True)
            record={'numerator':list(map(float,num)),'denominator':list(map(float,den)),
              'poles':[[float(z.real),float(z.imag)] for z in poles],'routh_first_column':list(map(float,routh(den))),
              'step':step,'sampled_eigenvalues':[[float(z.real),float(z.imag)] for z in ds],
              'sampled_spectral_radius':float(max(abs(ds))),'one_sample_delay_spectral_radius':float(max(abs(delayed)))}
            if name in ['roll_rate','yaw']:
                b=byaw if name=='yaw' else bx;record['rate_loop_margin']=margin([b*kp,b*ki],[tau,1,0,0])
            result['models'][f'{name}-{scale}']=record
        num,den=models['angle']
        for label,kx in [('navigate',1.8),('approach',1.2)]:
            denominator=np.polyadd(np.polymul([1,.1,0],den),np.polymul([1.8,kx],num))
            numerator=np.array(num)*kx;poles=np.roots(denominator);assert max(poles.real)<0
            degree=len(denominator)-1;hurwitz=np.zeros((degree,degree))
            for i in range(degree):
                for j in range(degree):
                    index=2*j-i+1
                    if 0<=index<len(denominator):hurwitz[i,j]=denominator[index]
            determinants=[float(np.linalg.det(hurwitz[:i,:i])) for i in range(1,degree+1)]
            assert min(determinants)>0
            rho,delayed=horizontal_sampled(bx,rp,ri,kx,1.8)
            assert rho<1 and delayed<1
            result['horizontal_guidance'][f'{label}-{scale}']={'order':degree,'numerator':numerator.tolist(),
              'denominator':denominator.tolist(),'poles':[[float(z.real),float(z.imag)] for z in poles],
              'Hurwitz_leading_minors':determinants,'500Hz_FC_20Hz_guidance_spectral_radius':rho,
              'one_guidance_period_delay_spectral_radius':delayed}
        stable=0;total=0;worst= -float('inf')
        for gain_factor,lag in itertools.product(np.linspace(.65,1.35,15),np.linspace(.02,.09,15)):
            for kind in ['angle','yaw','height']:
                if kind=='angle':b=bx*gain_factor;den=[lag,1,b*rp,b*(ri+3*rp),b*3*ri]
                elif kind=='yaw':b=byaw*gain_factor;den=[lag,1,b*yp,b*yi]
                else:b=bh*gain_factor;den=[lag,1+drag*lag,drag+b*hd,b*hp,b*hi]
                real=float(max(np.roots(den).real));stable+=real<0;total+=1;worst=max(worst,real)
        result['finite_grid'][str(scale)]={'stable':int(stable),'total':total,'largest_real_pole':worst,
          'grid':'plant gain .65..1.35 in15 points; actuator lag .02..09s in15 points; angle/yaw/height; no proof between points'}
    fig.suptitle('Local unsaturated transfer functions, actuator lag retained; NOT nonlinear flight certification')
    fig.savefig(OUT/'linear-step-responses.png',dpi=160);plt.close(fig)
    kp,ki,_=gain_table(m,.7)[1];hp,hi,hd=gain_table(m,.7)[5];yp,yi,_=gain_table(m,.7)[4]
    tmax=.09;bmin=.65*bx;u=ki+3*kp;amin=kp-tmax*u
    angle_b=bmin*u*amin-3*ki
    d2_height_lb=.08+(.65*bh)*(hd-tmax*hp)
    height_b=hp*d2_height_lb-hi*(1+.13*tmax)**2
    bounds={'gain_scale':.7,'b_factor_interval':[.65,1.35],'lag_interval_s':[.02,.09],
      'height_drag_interval_s_inverse':[.08,.13],'angle_A_lower':float(amin),
      'angle_Delta3_div_b_lower':float(angle_b),'yaw_Kp_minus_tauKi_lower':float(yp-tmax*yi),
      'height_Delta2_lower':float(d2_height_lb),'height_Delta3_div_b_lower':float(height_b),
      'scope':'Analytic conservative Routh bounds for local unsaturated decoupled models; no estimator/delay/nonlinear proof'}
    assert min(bounds[k] for k in ['angle_A_lower','angle_Delta3_div_b_lower','yaw_Kp_minus_tauKi_lower','height_Delta2_lower','height_Delta3_div_b_lower'])>0
    result['analytic_interval_bounds']=bounds
    (OUT/'linear-analysis.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)

if __name__=='__main__':main()
