"""Additional diagrams/plots use one explicit illustrative PID model."""
from pathlib import Path
import json
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from control_design_example import PARAMETERS, loop_polynomials, response

GREEN='#198548';DARK='#18382c';MINT='#eaf4e8';RED='#bb4038';BLUE='#466fa2';GRAY='#718178';GOLD='#c48b23'


def build_advanced(root, edition_dir, selected):
    root=Path(root);out=root/edition_dir/'assets/visual';records={}
    data=json.loads((root/'analysis/control-design-example.json').read_text())
    assert data['source_sha256']==hashlib.sha256((root/'tools/control_design_example.py').read_bytes()).hexdigest(), 'Stale illustrative model: run tools/control_design_example.py first'
    def save(fig,name,caption,kind='ILLUSTRATIVE PID MODEL'):
        fig.savefig(out/(name+'.png'),dpi=190,bbox_inches='tight',pad_inches=.14);plt.close(fig)
        records[name]={'file':'assets/visual/'+name+'.png','kind':kind,'source':'analysis/control-design-example.json and tools/control_design_example.py; no hardware/Gazebo run','caption':caption}
    def box(ax,x,y,w,h,text,c=GREEN):
        ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.012,rounding_size=.01',fc=MINT,ec=c,lw=1.2));ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=9,color=DARK)
    def arrow(ax,a,b,c=GREEN):
        ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=10,lw=1.2,color=c))
    def blank(height):
        fig,ax=plt.subplots(figsize=(8.6,height),layout='constrained');ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off');return fig,ax
    if 'pid-parallel' in selected:
        fig,ax=blank(2.5);ax.text(.025,.53,'Error e',fontsize=10)
        for y,text in [(.76,'P: Kp e'),(.43,'I: integral + anti-windup'),(.10,'D: filtered derivative')]:
            box(ax,.27,y,.28,.19,text);ax.plot([.15,.15,.26],[.53,y+.095,y+.095],c=GREEN,lw=1.1);ax.plot([.56,.64,.64],[y+.095,y+.095,.53],c=GREEN,lw=1.1)
        arrow(ax,(.10,.53),(.17,.53));ax.add_patch(Circle((.68,.53),.032,fc='white',ec=GREEN));ax.text(.68,.53,'+',ha='center',va='center');arrow(ax,(.64,.53),(.647,.53));box(ax,.77,.43,.19,.20,'Limit / mixer');arrow(ax,(.712,.53),(.77,.53))
        ax.plot([.91,.91,.60,.60],[.42,.04,.04,.49],c=GOLD,lw=1.1);arrow(ax,(.60,.49),(.56,.49),GOLD);ax.text(.72,.075,'Applied minus raw output',fontsize=8,color=GOLD,ha='center')
        save(fig,'pid-parallel','Three interpretable terms combine before saturation. Back-calculation returns saturation error to the integral branch.','PROPOSED PID IMPLEMENTATION')
    if 'yaw-height-pid' in selected:
        fig,ax=blank(2.7)
        for y,first,middle,last,feedback in [(.73,'Yaw-rate reference','Yaw rate PID','Yaw actuation / plant','Measured yaw rate'),(.24,'Height / climb reference law','Vertical-speed PID','Collective / plant','Estimated vertical speed')]:
            box(ax,.015,y-.09,.24,.18,first);ax.add_patch(Circle((.32,y),.025,fc='white',ec=GREEN));ax.text(.32,y,'+',ha='center',va='center',fontsize=9);arrow(ax,(.26,y),(.295,y));box(ax,.40,y-.09,.21,.18,middle);arrow(ax,(.345,y),(.40,y));box(ax,.72,y-.09,.25,.18,last);arrow(ax,(.615,y),(.72,y));ax.plot([.86,.86,.32],[y-.10,y-.21,y-.21],c=BLUE,lw=1.1);arrow(ax,(.32,y-.21),(.32,y-.025),BLUE);ax.text(.31,y-.10,'−',color=BLUE);ax.text(.59,y-.20,feedback,fontsize=8,ha='center',color=BLUE)
        save(fig,'yaw-height-pid','Yaw is rate controlled. A proposed height/climb reference law feeds the vertical-speed PID; its neutral/transition behaviour still needs validation.','PROPOSED CONTROL PATHS')
    if 'nyquist-example' in selected:
        fig,ax=plt.subplots(figsize=(8.6,3.1),layout='constrained');w=np.geomspace(.005,1e5,14000)
        for mu,c,label in [(1,GREEN,'Baseline: no unstable roots'),(8,RED,'8x gains: two unstable roots')]:
            z=response(1j*w,mu);ax.plot(z.real,z.imag,c=c,lw=1.8,label=label);ax.plot(z.real,-z.imag,c=c,lw=1.1,ls='--',alpha=.65)
            for f in [15,45,100]:
                a=response(1j*f,mu);b=response(1j*(f*1.04),mu)
                if -3<a.real<2 and -2.5<a.imag<2.5:arrow(ax,(a.real,a.imag),(b.real,b.imag),c)
        ax.scatter([-1],[0],c=DARK,marker='x',s=75,zorder=4,label='Critical point -1 + j0');ax.axhline(0,c=GRAY,lw=.7);ax.axvline(0,c=GRAY,lw=.7)
        ax.set(xlim=(-3,2),ylim=(-2.5,2.5),aspect='equal',xlabel='Real loop gain Re L(jω)',ylabel='Imaginary loop gain Im L(jω)',title='Nyquist frequency locus: approaching -1 matters');ax.grid(alpha=.5);ax.legend(fontsize=8,loc='upper right')
        save(fig,'nyquist-example','Positive-frequency locus and conjugate mirror, clipped near -1. Full origin indentation and outer closure are computed for the winding check; this is not measured aircraft stability.')
    if 'poles-3d' in selected:
        fig=plt.figure(figsize=(8.6,3.4),layout='constrained');ax=fig.add_subplot(111,projection='3d');multipliers=np.linspace(.5,8,110)
        for sign in [-1,1]:
            points=[]
            for mu in multipliers:
                n,d=loop_polynomials(float(mu));poles=np.roots(n+d);pair=poles[np.abs(poles.imag)>1e-6];z=pair[np.argmax(sign*pair.imag)];points.append((z.real,z.imag,mu))
            points=np.array(points);ax.plot(points[:,0],points[:,1],points[:,2],c=BLUE if sign<0 else GREEN,lw=1.8);ax.scatter(points[:,0],points[:,1],points[:,2],c=np.where(points[:,0]<0,GREEN,RED),s=8)
        yy,zz=np.meshgrid(np.linspace(-70,70,2),np.linspace(.5,8,2));ax.plot_surface(np.zeros_like(yy),yy,zz,color=RED,alpha=.08)
        ax.set(xlabel='Pole real part σ (1/s)',ylabel='Pole imaginary part ω (rad/s)',zlabel='PID gain multiplier μ',title='Oscillatory pair moves into the unstable half-plane');ax.view_init(25,-24);ax.set_box_aspect((1.4,2.0,1.1))
        save(fig,'poles-3d','Only the complex-conjugate pair is drawn; three real poles remain in the complete polynomial/root check. Green means negative real part; red means positive. All parameters are illustrative.')
    if 'bode-example' in selected:
        fig,(ax,bx)=plt.subplots(2,1,figsize=(8.6,3.8),sharex=True,layout='constrained');w=np.geomspace(.01,1e4,5000)
        for mu,c in [(1,GREEN),(8,RED)]:
            z=response(1j*w,mu);ax.semilogx(w,20*np.log10(abs(z)),c=c,label=f'μ={mu:g}');bx.semilogx(w,np.unwrap(np.angle(z))*180/np.pi,c=c)
        ax.axhline(0,c=GRAY,ls=':');bx.axhline(-180,c=GRAY,ls=':');ax.set(ylabel='Magnitude (dB)',ylim=(-70,65),title='Bode: gain and phase of the same open loop');bx.set(xlabel='Angular frequency ω (rad/s)',ylabel='Phase (degrees)',ylim=(-370,-50))
        for case,c in zip(data['cases'],[GREEN,RED]):
            for wgc,pm in zip(case['gain_crossovers_rad_s'],case['phase_margins_deg']):
                bx.plot([wgc,wgc],[-180,-180+pm],c=c,lw=2);bx.scatter([wgc],[-180+pm],c=c,s=18)
        for a in [ax,bx]:a.grid(which='both',alpha=.35)
        ax.legend(fontsize=9);save(fig,'bode-example','The curves use the same rational model and gains as Nyquist. Markers show phase margin at unity-gain crossover; margins are computed by bracketed root refinement.')
    if 'nichols-example' in selected:
        fig,ax=plt.subplots(figsize=(8.6,3.5),layout='constrained');phase=np.linspace(-359,-1,500);db=np.linspace(-40,55,500);pp,mm=np.meshgrid(phase,db);ll=10**(mm/20)*np.exp(1j*np.deg2rad(pp));closed=20*np.log10(abs(ll/(1+ll)))
        cs=ax.contour(pp,mm,closed,levels=[-12,-6,0,3,6,12],colors='#b6cbbd',linewidths=.7);ax.clabel(cs,fmt='%g dB',fontsize=7)
        w=np.geomspace(.001,1e5,7000)
        for mu,c in [(1,GREEN),(8,RED)]:
            z=response(1j*w,mu);ph=np.unwrap(np.angle(z))*180/np.pi;mg=20*np.log10(abs(z));ax.plot(ph,mg,c=c,lw=2,label=f'μ={mu:g}')
        ax.scatter([-180],[0],marker='x',c=DARK,s=65,label='Critical (-180°, 0 dB)');ax.axhline(0,c=GRAY,ls=':',lw=.7);ax.axvline(-180,c=GRAY,ls=':',lw=.7);ax.set(xlim=(-360,0),ylim=(-40,55),xlabel='Open-loop phase (degrees)',ylabel='Open-loop magnitude (dB)',title='Black–Nichols: phase and gain on one chart');ax.legend(fontsize=8);ax.grid(alpha=.3)
        save(fig,'nichols-example','Grid lines show closed-loop |L/(1+L)| levels for unity negative feedback. Crossing a generic grid line alone does not establish stability; root/winding checks remain separate.')
    if 'nyquist-contour' in selected:
        fig,(ax,bx)=plt.subplots(1,2,figsize=(8.6,2.8),layout='constrained')
        # Topological sketch: no misleading claim of actual numerical scale.
        ax.plot([0,0],[-3,-.5],c=BLUE);th=np.linspace(-np.pi/2,np.pi/2,150);ax.plot(.5*np.cos(th),.5*np.sin(th),c=GOLD);ax.plot([0,0],[.5,3],c=BLUE);th=np.linspace(np.pi/2,-np.pi/2,200);ax.plot(3*np.cos(th),3*np.sin(th),c=BLUE);ax.scatter([0],[0],marker='x',c=RED);ax.text(.65,.05,'Origin excluded\nby right indentation',fontsize=8);ax.set(aspect='equal',xlabel='Re s',ylabel='Im s',title='Clockwise RHP contour (schematic)');ax.grid(alpha=.3)
        w=np.geomspace(.02,1e5,5000)
        for mu,c in [(1,GREEN),(8,RED)]:
            z=response(1j*w,mu);bx.plot(z.real,z.imag,c=c,label=f'μ={mu:g}');bx.plot(z.real,-z.imag,c=c,ls='--',alpha=.6)
        bx.scatter([-1],[0],c=DARK,marker='x');bx.set(xlim=(-3,1),ylim=(-2,2),aspect='equal',xlabel='Re L',ylabel='Im L',title='Local view; winding uses full contour');bx.grid(alpha=.3);bx.legend(fontsize=8)
        save(fig,'nyquist-contour','Numerical winding uses ε=10^-4 and outer radius10^6, then repeats at10^-3/10^5. The finite-frequency display is not substituted for contour closure.')
    if 'stability-surface' in selected:
        mu=np.linspace(.5,8,31);tm=np.linspace(.015,.080,27);mm,tt=np.meshgrid(mu,tm);z=np.empty_like(mm)
        for i in range(mm.shape[0]):
            for j in range(mm.shape[1]):
                n,d=loop_polynomials(float(mm[i,j]),float(tt[i,j]));z[i,j]=np.max(np.roots(n+d).real)
        fig=plt.figure(figsize=(8.6,3.5),layout='constrained');ax=fig.add_subplot(111,projection='3d');ax.plot_surface(mm,tt*1000,z,cmap='RdYlGn_r',linewidth=0,alpha=.88);ax.contour(mm,tt*1000,z,levels=[0],colors=[DARK],linewidths=1.6);ax.set(xlabel='PID gain multiplier μ',ylabel='Assumed motor lag (ms)',zlabel='Rightmost pole real part (1/s)',title='Stability depends on actuator dynamics, not gains alone');ax.view_init(24,-60)
        save(fig,'stability-surface','Each surface point recomputes all five closed-loop poles. Zero is the stability boundary of this assumed rational model, not a measured operating limit.')

    if 'stats-pipeline' in selected:
        fig,ax=blank(1.35)
        labels=['Calibrated samples\nx1, ..., xN','Mean + spread\nx-bar, variance, std','Observation model\nR in correct units','Kalman weighting\nmodelled Q and R']
        for i,label in enumerate(labels):
            x=.015+i*.25;box(ax,x,.40,.205,.34,label)
            if i:arrow(ax,(x-.04,.57),(x-.01,.57))
        save(fig,'stats-pipeline','Measured spread informs an observation-error model only after reference, stationarity, bias and correlation checks. This diagram does not assert independent Gaussian sensor noise.','STATISTICAL METHOD')
    if 'poles-plane' in selected:
        fig,(ax,bx)=plt.subplots(1,2,figsize=(8.6,3.05),gridspec_kw={'width_ratios':[1.7,1]},layout='constrained')
        ax.axvspan(0,9,color=RED,alpha=.055);ax.axvline(0,c=RED,ls=':',lw=1);ax.axhline(0,c=GRAY,lw=.7)
        for sign in [-1,1]:
            points=[]
            for mu in np.linspace(.5,8,100):
                n,d=loop_polynomials(float(mu));pp=np.roots(n+d);pair=pp[np.abs(pp.imag)>1e-6];z=pair[np.argmax(sign*pair.imag)];points.append([z.real,z.imag])
            points=np.array(points);ax.plot(points[:,0],points[:,1],c=BLUE,lw=1.1)
        for mu,c,mark in [(1,GREEN,'o'),(8,RED,'x')]:
            n,d=loop_polynomials(mu);pp=np.roots(n+d);pair=pp[np.abs(pp.imag)>1e-6]
            ax.scatter(pair.real,pair.imag,c=c,marker=mark,s=36,label=f'μ={mu:g}')
            bx.scatter(pp.real,pp.imag,c=c,marker=mark,s=28,label=f'μ={mu:g}')
        ax.set(xlim=(-16,9),ylim=(-80,80),xlabel='Real part σ (1/s)',ylabel='Imaginary part ω (rad/s)',title='Complex pair: growth/decay and oscillation')
        bx.axvline(0,c=RED,ls=':',lw=1);bx.axhline(0,c=GRAY,lw=.7);bx.set(xlim=(-560,20),ylim=(-80,80),xlabel='Real part σ (1/s)',ylabel='Imaginary part ω (rad/s)',title='All five poles per case')
        for a in [ax,bx]:a.grid(alpha=.3);a.legend(fontsize=8)
        save(fig,'poles-plane','Left: the complex pair as gains increase. Right: all five roots for both cases, including the fast real modes. A 2D complex plane displays these quantities directly; no 3D projection is needed.')

    return records
