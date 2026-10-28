"""Deterministic educational figures; measured summaries and examples kept separate."""
from pathlib import Path
import json
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle, Ellipse, Wedge

GREEN='#198548'; DARK='#18382c'; MINT='#eaf4e8'; GRAY='#718178'; RED='#bb4038'; GOLD='#c48b23'; BLUE='#466fa2'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':11,'axes.labelsize':10,'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#abc0af','axes.labelcolor':DARK,'text.color':DARK,'xtick.color':DARK,'ytick.color':DARK,'grid.color':'#dbe7dd','figure.facecolor':'white','savefig.facecolor':'white'})

def build_figures(root):
    root=Path(root);out=root/'docs/university-report/assets/visual';out.mkdir(parents=True,exist_ok=True)
    records={}
    def save(fig,name,kind,source,caption):
        fig.savefig(out/(name+'.png'),dpi=190,bbox_inches='tight',pad_inches=.12)
        plt.close(fig)
        records[name]={'kind':kind,'source':source,'caption':caption,'file':'assets/visual/'+name+'.png'}
    def axes(size=(8.6,2.75),n=1):
        fig,ax=plt.subplots(1,n,figsize=size,layout='constrained')
        return fig,ax
    def box(ax,x,y,w,h,text,color=GREEN):
        ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.012,rounding_size=0.015',fc=MINT,ec=color,lw=1.2))
        ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=9,color=DARK)
    def arrow(ax,a,b,color=GREEN,rad=0):
        ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=11,color=color,lw=1.2,connectionstyle=f'arc3,rad={rad}'))
    def flow(name,rows,caption,kind='PROPOSED ARCHITECTURE',source='Report design'):
        height=1.15 if len(rows)==1 else (2.4 if len(rows)==2 else 1.0+1.05*len(rows))
        fig,ax=axes((8.6,height));ax.set(xlim=(0,1),ylim=(.60,.98) if len(rows)==1 else (0,1));ax.axis('off')
        for j,row in enumerate(rows):
            y=(.64-j*.5) if len(rows)==2 else .68-j*(.64/max(1,len(rows)-1));w=.82/len(row);gap=.16/max(1,len(row)-1)
            for k,t in enumerate(row):
                x=.025+k*(w+gap)*.96
                box(ax,x,y,w,.23 if len(rows)==1 else .19,t)
                if k:arrow(ax,(x-gap+.012,y+.095),(x-.012,y+.095))
        if name=='yolo-architecture':
            arrow(ax,(.86,.68),(.5,.55),GRAY)
            arrow(ax,(.86,.36),(.5,.23),GRAY)
        save(fig,name,kind,source,caption)

    flow('system-paths',[
        ['Xbox controller\nBLE input','Standalone FC\nestimate + control','Mixer / ESCs\nphysical plant'],
        ['Ground station\napproved nodes','ROS 2 companion\nplan + avoid','FC setpoint interface\nsame inner loops']],
        'Two command paths; motor commands belong only to the FC. The companion is optional.')
    flow('control-cascade',[
        ['Roll / pitch\nangle setpoint','Angle PID x2\nrate setpoint','Rate PID x2\ntorque request'],
        ['Yaw stick\nrate setpoint','Yaw rate PID x1','Mixer\nreserved authority'],
        ['Vertical stick\nvz setpoint','Height law +\nvz PID x1','Collective thrust\nrequest']],
        'Six full PID blocks are requested. The outer height-to-speed law remains a design choice.')

    fig,(ax,bx)=axes((8.6,3.1),2)
    a=.225/np.sqrt(2)
    for x,y in [(a,a),(a,-a),(-a,a),(-a,-a)]:
        ax.plot([0,x],[0,y],lw=9,color=RED if x>0 else '#b4c0b7',solid_capstyle='round')
        ax.add_patch(Circle((x,y),.047,fc='white',ec=DARK,lw=1.2))
        ax.text(x,y,f'({x/a:+.0f}a,{y/a:+.0f}a)',ha='center',va='center',fontsize=8)
    ax.scatter(0,0,s=150,color=GREEN);ax.set(xlim=(-.24,.24),ylim=(-.24,.24),aspect='equal',xlabel='Body x (m), forward',ylabel='Body y (m), left',title='F450 X lever arms')
    ax.grid(alpha=.5)
    bx.set(xlim=(-1,1),ylim=(-.4,1.2),aspect='equal');bx.axis('off')
    bx.add_patch(Rectangle((-.55,.25),1.1,.12,angle=12,fc=MINT,ec=DARK))
    arrow(bx,(0,.4),(.18,1.0));bx.text(.23,.95,'Thrust RT e3')
    arrow(bx,(0,.3),(0,-.25),RED);bx.text(.08,-.23,'Weight mg')
    bx.text(-.9,1.08,'World force balance',weight='bold')
    bx.text(-.9,-.38,'Hover: T = mg; tilt redirects thrust',fontsize=9)
    save(fig,'geometry-forces','ANALYTIC GEOMETRY','Supplied nominal 450 mm diagonal','Frame convention is explicit; this drawing does not assign motor numbers or spin signs.')

    fig,(ax,bx)=axes(n=2)
    ang=np.linspace(0,40,160);ax.plot(ang,1/np.cos(np.deg2rad(ang)),color=GREEN,lw=2)
    ax.set(xlabel='Tilt angle (degrees)',ylabel='T / (mg)',title='Vertical support needs more thrust');ax.grid(alpha=.5)
    mass=np.array([.9,1.038,1.2]);bx.bar(['0.900','1.038','1.200'],mass*9.80665/4,color=[GRAY,GREEN,GRAY]);bx.set(xlabel='Mass (kg), example sweep',ylabel='Hover thrust / rotor (N)',title='Mass changes the hover operating point');bx.grid(axis='y',alpha=.5)
    save(fig,'hover-sensitivity','ANALYTIC EXAMPLE','T cos(alpha)=mg; mass sweep illustrative','At the supplied mass, hover requires about 2.545 N per rotor; actual capability is unmeasured.')

    fig,(ax,bx)=axes(n=2);t=np.linspace(0,3,400);wn=4
    for z,c in [(.35,RED),(.7,GREEN),(1.2,BLUE)]:
        if z<1:
            wd=wn*np.sqrt(1-z*z);y=1-np.exp(-z*wn*t)*(np.cos(wd*t)+z/np.sqrt(1-z*z)*np.sin(wd*t))
        else:
            p1=-wn*(z-np.sqrt(z*z-1));p2=-wn*(z+np.sqrt(z*z-1));y=1+(p2*np.exp(p1*t)-p1*np.exp(p2*t))/(p1-p2)
        ax.plot(t,y,c=c,label=f'zeta={z:g}');p=np.roots([1,2*z*wn,wn*wn]);bx.scatter(p.real,p.imag,c=c,s=45)
    ax.axhline(1,color=GRAY,ls=':');ax.set(xlabel='Time (s)',ylabel='Normalized output',title='Damping versus overshoot');ax.legend(fontsize=8);ax.grid(alpha=.5)
    bx.axvline(0,color=RED,ls=':');bx.axhline(0,color=GRAY,lw=.8);bx.set(xlabel='Real pole part (1/s)',ylabel='Imaginary part (1/s)',title='Stable continuous poles lie left');bx.grid(alpha=.5)
    save(fig,'damping-poles','ILLUSTRATIVE LTI MODEL','wn=4 rad/s; zeta=0.35,0.7,1.2','Educational second-order responses, not tuned drone gains or simulated aircraft trials.')

    flow('sample-update',[
        ['New IMU sample\ntime + validity','Predict state\nactual dt','Correct estimate\nvalid observations','Control / mix\nbounded output']],
        'Sample, estimate, control and ESC update are distinct stages; slow observations are asynchronous.')
    fig,(ax,bx)=axes(n=2)
    poles=np.array([-2+3j,-2-3j,-5+0j,1+0j]);cs=[GREEN,GREEN,BLUE,RED]
    ax.axvline(0,c=RED,ls=':');ax.axhline(0,c=GRAY,lw=.7);ax.scatter(poles.real,poles.imag,c=cs,s=45);ax.set(xlabel='Re(s)',ylabel='Im(s)',title='Continuous stability: Re(s) < 0');ax.grid(alpha=.5)
    tt=np.linspace(0,2*np.pi,300);bx.plot(np.cos(tt),np.sin(tt),c=GRAY,ls=':');zp=np.exp(.1*poles);bx.scatter(zp.real,zp.imag,c=cs,s=45);bx.set(xlabel='Re(z)',ylabel='Im(z)',title='Discrete stability: |z| < 1',aspect='equal',xlim=(-1.2,1.3),ylim=(-1.1,1.1));bx.grid(alpha=.5)
    save(fig,'discrete-poles','ANALYTIC SAMPLING EXAMPLE','z=exp(s*Ts); Ts=0.1 s for plot visibility','This example uses 100 ms to show the mapping clearly; it is not the 1 ms FC implementation or its poles.')

    flow('calibration-protocol',[
        ['Rigid mounting\nfixed body axes','Six poses\n+/-X, +/-Y, +/-Z','3 repetitions\n20 s each'],
        ['Fit repeats 1-2\n12 pose means','Freeze b and C','Evaluate repeat 3\nthen tilted poses']],
        'Existing 18-pose design separates fitting and evaluation. Fixture uncertainty remains.',kind='HISTORICAL TEST DESIGN',source='P1 calibration report')

    cal=json.loads((root/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text())
    C=np.array(cal['correction_sensor_to_body']);bias=np.array(cal['bias_sensor_m_s2'])
    fig,(ax,bx)=axes((8.6,3.0),2)
    im=ax.imshow(C,cmap='PiYG',vmin=-1,vmax=1)
    for i in range(3):
        for j in range(3):ax.text(j,i,f'{C[i,j]:+.4f}',ha='center',va='center',color='white' if abs(C[i,j])>.65 else DARK,fontsize=9)
    ax.set(xticks=range(3),yticks=range(3),xticklabels=['raw x','raw y','raw z'],yticklabels=['body x','body y','body z'],title='Full matrix C, not 3 offsets only')
    bx.bar(['sensor x','sensor y','sensor z'],bias,color=[GREEN,GRAY,RED]);bx.axhline(0,color=GRAY,lw=.8);bx.set(ylabel='Bias (m/s²)',title='Offline fitted bias b');bx.grid(axis='y',alpha=.5)
    save(fig,'calibration-matrix','HISTORICAL OFFLINE FIT','P1 analysis.json','Coefficients shown are the historical fit candidate, not a new calibration or an instruction to overwrite trims.')

    fig,(ax,bx)=axes(n=2)
    labels=['+Z','-Z','+X','-X','+Y','-Y'];err=[.3682,.3877,.0668,.9660,.2406,1.1093];verr=[.06325,.06636,.05101,.17320,.05897,.19338]
    ax.bar(labels,err,color=GREEN);ax.set(ylabel='Direction error (degrees)',title='Reserved repetition 3');ax.grid(axis='y',alpha=.5)
    bx.bar(labels,verr,color=BLUE);bx.set(ylabel='Vector error (m/s²)',title='Residual includes placement uncertainty');bx.grid(axis='y',alpha=.5)
    save(fig,'calibration-residuals','REAL LOG REPLAY SUMMARY','P1 report.md reserved-pose table','These residuals include reference/placement error; they are not certified sensor accuracy.')

    est=json.loads((root/'measurements/mpu6050/2026-10-26-offline-attitude-preparation/analysis.json').read_text())
    cov=np.array(est['within_record_body_gyro_covariance_rad2_s2']);corr=cov/np.sqrt(np.outer(np.diag(cov),np.diag(cov)))
    fig,(ax,bx)=axes(n=2);xy=np.array([[-.8,-.4],[-.4,-.2],[0,0],[.4,.2],[.8,.4]])
    for n in [1,2,3]:ax.add_patch(Ellipse((0,0),2*n,.6*n,angle=30,fill=False,ec=GREEN,alpha=1/n))
    ax.set(xlim=(-3,3),ylim=(-2,2),aspect='equal',xlabel='Noise channel 1',ylabel='Noise channel 2',title='Covariance geometry (schematic)');ax.grid(alpha=.5)
    bx.imshow(corr,cmap='PiYG',vmin=-1,vmax=1)
    for i in range(3):
        for j in range(3):bx.text(j,i,f'{corr[i,j]:+.2f}',ha='center',va='center',fontsize=10,color='white' if abs(corr[i,j])>.65 else DARK)
    bx.set(xticks=range(3),yticks=range(3),xticklabels=['p','q','r'],yticklabels=['p','q','r'],title='Training gyro correlation [P4]')
    save(fig,'covariance-geometry','SCHEMATIC + MEASURED SUMMARY','P4 training covariance; ellipse illustrative','Off-diagonal terms show shared variation. Zero covariance does not generally imply independence.')

    fig,(ax,bx)=axes(n=2);n=np.arange(1,2001)
    ax.loglog(n,1/np.sqrt(n),c=GREEN,label='Independent samples')
    rho=.5;f=n+2*np.array([sum((k-np.arange(1,k))*rho**np.arange(1,k)) for k in n]);ax.loglog(n,np.sqrt(f)/n,c=RED,label='AR(1), rho=0.5 example');ax.set(xlabel='Samples N',ylabel='SEM / sample sigma',title='Correlation slows information gain');ax.legend(fontsize=8);ax.grid(alpha=.5)
    x=np.linspace(-.5,.5,400)
    for sig,c,lab in [(.177569,GRAY,'Accel roll'),(.007,GREEN,'Filtered roll')]:bx.plot(x,np.exp(-.5*(x/sig)**2)/(sig*np.sqrt(2*np.pi)),c=c,label=lab)
    bx.set(xlabel='Centered angle (degrees)',ylabel='Gaussian reference density',title='Widths matched to observed std');bx.legend(fontsize=8)
    save(fig,'uncertainty-sampling','ANALYTIC EXAMPLE + OBSERVED WIDTHS','P4 sigma values; Gaussian curves and AR(1) illustrative','Density shapes are explanatory references, not a claim that the actual noise is Gaussian or unbiased.')

    flow('boot-gyro',[
        ['Factory reset\nbaseline','Restore user trims\nreadback','Residual + body\nrotation correction','Known +/- rotations\nrequired validation']],
        'Hardware bias correction and software transform have separate roles; gyro scale/physical axes remain pending.')
    fig,(ax,bx)=axes(n=2);t=np.linspace(0,60,300)
    ax.plot(t,.1*t,color=RED,label='0.1 deg/s residual bias');ax.plot(t,0*t,color=GREEN,label='Ideal zero residual');ax.set(xlabel='Time (s)',ylabel='Integrated angle error (degrees)',title='Bias accumulates over time');ax.legend(fontsize=8);ax.grid(alpha=.5)
    sig=np.sqrt(np.diag(cov))*180/np.pi;bx.bar(['p','q','r'],sig,color=GREEN);bx.set(ylabel='Standard deviation (degrees/s)',title='Training gyro noise summary [P4]');bx.grid(axis='y',alpha=.5)
    save(fig,'gyro-bias-noise','ANALYTIC EXAMPLE + MEASURED SUMMARY','P4 covariance; bias ramp illustrative','Averaging reduces random fluctuation but does not remove an unmodelled residual bias.')

    flow('kalman-loop',[
        ['Gyro prediction\nangle + dt * rate','Innovation\naccel angle - prediction','Gain K\ntrust from P and R','Correct angle\nupdate P']],
        'Scalar roll/pitch estimator candidate. Translational acceleration can corrupt the gravity observation.')
    fig,(ax,bx)=axes(n=2);ratio=np.logspace(-3,3,250);ax.semilogx(ratio,ratio/(1+ratio),color=GREEN);ax.set(xlabel='Predicted variance / observation variance',ylabel='Kalman gain K',title='More observation noise means less trust');ax.grid(alpha=.5)
    names=['Roll','Pitch'];bx.bar(np.arange(2)-.16,[.177569,.213074],.32,color=GRAY,label='Accel angle');bx.bar(np.arange(2)+.16,[.007,.005692],.32,color=GREEN,label='Filter output');bx.set(xticks=range(2),xticklabels=names,ylabel='Standard deviation (degrees)',title='Reserved stationary replay [P4]');bx.legend(fontsize=8);bx.grid(axis='y',alpha=.5)
    save(fig,'kalman-trust','ANALYTIC LAW + REAL REPLAY SUMMARY','K=P/(P+R); P4 reserved statistics','Smoother output is not an independent accuracy result; mean errors and correlation remain.')

    fig,(ax,bx)=axes(n=2)
    peaks=[.012507,5.191244,.172872,5.354520];labs=['none','accel','bias','both'];ax.bar(labs,peaks,color=[GREEN,RED,GOLD,RED]);ax.set(ylabel='Peak angle error (degrees)',title='Four-case disturbance experiment');ax.grid(axis='y',alpha=.5)
    q=np.linspace(0,.2,200);bx.plot(q,np.rad2deg(np.arctan(q)),color=RED,label='Apparent tilt');bx.scatter([.1],[np.rad2deg(np.arctan(.1))],color=DARK);bx.set(xlabel='Lateral acceleration / g',ylabel='Apparent tilt (degrees)',title='Acceleration can masquerade as gravity');bx.grid(alpha=.5)
    save(fig,'accel-failure','EXISTING SYNTHETIC MODEL + ANALYTIC LAW','P4 four-case results; arctan(a/g)','Python model results, not Gazebo or flight. The candidate is not accepted for dynamic operation.')

    flow('vertical-filter',[
        ['IMU + attitude\nworld vertical az','Predict [z, vz]\nA and B, actual dt','Barometer / down lidar\nvalid, aged observations'],
        ['Height reference','Outer z -> vz law','Vertical speed PID\ncollective request']],
        'Two-state estimator and vertical control architecture proposed; barometer and lidar integration remain pending.')
    fig,(ax,bx)=axes(n=2);q=np.linspace(0,45,200);ax.plot(q,2*np.cos(np.deg2rad(q)),color=GREEN);ax.axhline(2,color=GRAY,ls=':');ax.set(xlabel='Beam tilt from vertical (degrees)',ylabel='Height for 2 m slant range (m)',title='Flat-ground projection z = d cos(alpha)');ax.grid(alpha=.5)
    t=np.linspace(0,4,200);bx.plot(t,1+.5*t,color=GREEN,label='z prediction');bx.plot(t,.5+0*t,color=BLUE,label='vz prediction');bx.set(xlabel='Time (s), ideal example',ylabel='z (m) / vz (m/s)',title='Constant-speed prediction without updates');bx.legend(fontsize=8);bx.grid(alpha=.5)
    save(fig,'vertical-observation','ANALYTIC EXAMPLE','Flat ground, aligned geometry; ideal prediction','A forward lidar cannot supply this height observation. Terrain and extrinsics must be checked.')

    fig,(ax,bx)=axes(n=2);e=np.linspace(-1,1,200);ax.plot(e,e,color=GRAY,ls=':',label='Linear');ax.plot(e,.5*e+.5*e**3,color=GREEN,label='Gentle example');ax.set(xlabel='Normalized stick',ylabel='Normalized angle request',title='Soft centre, full end-point authority');ax.legend(fontsize=8);ax.grid(alpha=.5)
    motors=np.array([1550,1650,1750,1700]);bx.bar(range(1,5),motors-1000,bottom=1000,color=GREEN);bx.axhline(1800,color=GOLD,ls='--',label='Example collective ceiling');bx.axhline(2000,color=RED,ls=':',label='Illustrative output endpoint');bx.set(ylim=(950,2050),xticks=range(1,5),xlabel='Motor channel',ylabel='Command (microseconds)',title='Reserve correction headroom');bx.legend(fontsize=7)
    save(fig,'input-headroom','ILLUSTRATIVE CONTROL DESIGN','Example curves and commands; not firmware gains','Actual collective/vertical law and ESC limits are TBD. A boost request is not four identical maximum outputs.')

    fig,(ax,bx)=axes((8.6,3.1),2)
    vals=[1381,1249,814,819,812,822];ax.bar(['1','3','4','5','6','7'],vals,color=[RED,RED,GREEN,GREEN,GREEN,GREEN]);ax.axhline(1000,color=RED,ls='--',label='1 ms reference');ax.axhline(800,color=GOLD,ls=':',label='New work budget (P)');ax.set(xlabel='Historical trial (trial 2 malformed)',ylabel='Maximum handler (microseconds)',title='Failed trials are retained [P3]');ax.legend(fontsize=7);ax.grid(axis='y',alpha=.5)
    bx.barh(['Proposed critical work','Proposed reserve'],[800,200],left=[0,800],color=[GREEN,GOLD]);bx.axvline(1000,color=RED,ls='--');bx.set(xlim=(0,1100),xlabel='Within 1 ms period (microseconds)',title='Budget proposal, not measured schedule');bx.grid(axis='x',alpha=.5)
    save(fig,'timing-budget','REAL LOG SUMMARY + PROPOSED BUDGET','P3 timing table; 800/200 us proposed','Historical samples were not a complete PID controller; source hashes and inter-service gaps still matter.')

    flow('yolo-architecture',[
        ['RGB camera\ntime + calibration','YOLO inference\nboxes/class/score','Tracks / bearing\nuncertainty + age'],
        ['Lidar ray\nrange + status','Association / local\nobstacle hypothesis','Local map\nunknown != free'],
        ['Localization + mission\ncoordinate transforms','Planner + trajectory\nclearance constraints','FC setpoints\nTTL / CRC / bounds']],
        'Proposed perception-to-control pipeline. Detection is not depth, localization or a collision-free corridor.',source='Design + Ultralytics detector output contract')

    fig,(ax,bx)=axes((8.6,3.0),2)
    ax.set(xlim=(0,8),ylim=(-3,3),aspect='equal',title='One ray covers a narrow direction',xlabel='Forward distance (m)',ylabel='Lateral distance (m)')
    ax.add_patch(Wedge((0,0),7,-24,24,fc=MINT,ec=GREEN));ax.plot([0,7],[0,0],color=RED,lw=2,label='1D lidar ray');ax.add_patch(Rectangle((4,1),.25,.8,fc=RED));ax.add_patch(Rectangle((5,-.25),.25,.5,fc=GOLD));ax.text(3.2,2.1,'Off-ray obstacle\nmay have no range',fontsize=8);ax.legend(fontsize=8,loc='lower left')
    bx.set(xlim=(0,8),ylim=(-2,2),aspect='equal',title='Same image bearing, different depth',xlabel='Distance (arbitrary scale)');bx.plot([0,7],[0,1.4],c=GREEN);bx.scatter([2,6],[.4,1.2],s=[60,240],c=[BLUE,RED]);bx.text(.5,-1.4,'Pixel location gives a ray.\nMetric depth needs extra information.',fontsize=9);bx.set_yticks([])
    save(fig,'perception-limits','GEOMETRIC SCHEMATIC','Pinhole bearing and single-ray sensing','Objects, field of view and distances are illustrative, not camera specifications or YOLO detections.')

    fig,(ax,bx)=axes(n=2)
    tau=.25;a_dec=1.0;v=np.linspace(0,4,160)
    ax.plot(v,v*tau+v*v/(2*a_dec),color=GREEN,lw=2);ax.set(xlabel='Speed (m/s)',ylabel='Minimum stopping distance (m)',title='Example: latency 0.25 s, decel 1 m/s²');ax.grid(alpha=.5)
    bx.set(xlim=(0,1),ylim=(0,1));bx.axis('off')
    for x,lab in [(.03,'map'),(.36,'odom'),(.69,'base_link')]:box(bx,x,.6,.25,.2,lab)
    arrow(bx,(.29,.7),(.35,.7));arrow(bx,(.62,.7),(.68,.7));box(bx,.69,.15,.25,.2,'camera / lidar');arrow(bx,(.815,.59),(.815,.37))
    bx.text(.03,.36,'Localization must make\nmission coordinates observable.',fontsize=10)
    save(fig,'stopping-frames','ANALYTIC EXAMPLE + PROPOSED FRAMES','Stopping distance v*tau+v²/(2a); ROS frame concept','Real braking, latency and localization are unmeasured; speed limits cannot be inferred from this example.')

    flow('mission-state',[
        ['Receive n nodes\nn >= 2','Validate + approve\nframe + localization','Execute node i\nupdate from sensors','Arrival confirmed\ni = i + 1'],
        ['No valid localization\nreject mission','Stale companion\nexpire command','Last node reached\nfinal policy separate']],
        'Mission progression must not stop at node 2. Loss responses and final landing policy require approval.')
    fig,(ax,bx)=axes(n=2)
    nodes=np.array([[0,0],[2,1],[4,2],[6,1]])
    ax.plot(nodes[:,0],nodes[:,1],'o--',c=GREEN);[ax.text(x,y+.2,f'{i+1}',ha='center') for i,(x,y) in enumerate(nodes)];ax.add_patch(Rectangle((2.8,.5),.6,.8,fc=RED,alpha=.6));ax.set(xlim=(-.5,6.5),ylim=(-.5,3),xlabel='Map x (m)',ylabel='Map y (m)',title='Mission sketch, not a successful route');ax.grid(alpha=.5)
    bx.step([0,1,2,3,4],[1,1,2,3,4],where='post',color=GREEN,lw=2);bx.set(xlabel='Schematic time (arbitrary units)',ylabel='Current node index',yticks=[1,2,3,4],title='Advance beyond the second node');bx.grid(alpha=.5)
    save(fig,'mission-nodes','PLANNED TEST SCHEMATIC','Mission definition, no planner execution','The dashed node sequence is input geometry; it does not claim obstacle avoidance or a collision-free path.')

    flow('simulation-proof',[
        ['Gamepad / replay\ninput messages','Actual FC algorithm\nsource hash','Motor requests\nno pose override','Gazebo physics\nsensor models'],
        ['Ground station\nmission only','Companion planner\nsensed obstacles','FC serial contract\nbounded setpoints','Same FC + plant\nend-to-end logs']],
        'A test harness may stimulate inputs and record outputs; it may not secretly move the aircraft to pass.')
    fig,(ax,bx)=axes(n=2)
    ax.imshow(np.ones((10,5)),cmap='Greens',vmin=0,vmax=2);ax.set(xticks=range(5),xticklabels=range(1,6),yticks=range(10),yticklabels=range(1,11),xlabel='Noise seed',ylabel='Environment',title='Planned autonomous campaign: 10 x 5')
    n=np.arange(1,101);bx.plot(n,100*(1-.05**(1/n)),color=GREEN);bx.scatter([50],[100*(1-.05**(1/50))],color=RED);bx.set(xlabel='Independent trials with zero failures',ylabel='95% upper failure probability (%)',title='Zero observed failure is not zero risk');bx.grid(alpha=.5)
    save(fig,'campaign-confidence','PLANNED CAMPAIGN + STATISTICAL LAW','Binomial zero-failure one-sided bound','No cells represent completed missions. Independence and comparable trials are assumptions, not guarantees.')

    fig,(ax,bx)=axes(n=2)
    t=np.linspace(0,12,250);soc=100*(1-10*t/60/4.2);ax.plot(t,soc,c=GREEN);ax.axhline(35,c=GOLD,ls='--',label='Warning request');ax.axhline(20,c=RED,ls=':',label='Historical cut request');ax.set(xlabel='Time (min), example 10 A',ylabel='Charge remaining (%)',title='Coulomb count needs usable capacity');ax.legend(fontsize=8);ax.grid(alpha=.5)
    vb=np.linspace(0,12.6,100);bx.plot(vb,vb/5.7,c=GREEN);bx.scatter([12.6],[12.6/5.7],c=RED);bx.set(xlabel='3S pack voltage (V)',ylabel='Ideal divider output (V)',title='Voltage measurement is not SOC');bx.grid(alpha=.5)
    save(fig,'battery-model','ANALYTIC EXAMPLE','4200 mAh; ideal constant 10 A; divider ratio 5.7','This is not measured endurance. Whole-pack monitoring cannot protect each cell; a motor cut in flight causes a fall.')

    flow('gates',[
        ['Power / identity\nno actuation','Calibration / frames\nphysical reference','Estimator + timing\nindependent dynamics','PID / mixer\nmodel + limits'],
        ['Stable input path\nno companion','Localization\nperception + frames','Navigation campaign\nno hidden oracle','Motor-test review\nseparate authorization']],
        'Offline preparation is allowed; failed earlier gates invalidate affected downstream acceptance.')

    # Readiness chart records acceptance category, not an invented percent complete.
    fig,ax=axes((8.6,2.6));labels=['Inertial evidence','New I2C topology','Dynamic estimator','PID / mixer','Autonomous flight'];states=[2,1,1,0,0];ax.barh(labels,states,color=[GREEN,GOLD,GOLD,GRAY,GRAY]);ax.set(xticks=[0,1,2],xticklabels=['Planned / pending','Offline or planned','Historical evidence'],xlim=(0,2.8),title='Readiness is evidence-specific, not a progress percentage');ax.invert_yaxis();ax.grid(axis='x',alpha=.5)
    save(fig,'readiness','PROJECT STATUS SUMMARY','Current plan and evidence manifest','Historical evidence does not imply flight acceptance. New sensor wiring and physical frame checks remain pending.')

    for rec in records.values():rec['sha256']=hashlib.sha256((root/'docs/university-report'/rec['file']).read_bytes()).hexdigest()
    (out/'figure-manifest.json').write_text(json.dumps(records,indent=2)+'\n',encoding='utf-8')
    return records

if __name__=='__main__':
    import sys
    print(json.dumps({'figures':len(build_figures(sys.argv[1]))}))
