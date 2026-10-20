"""Independent geometry, failure, linear algebra and refinement checks."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run import ROOT,libraries,run_case,gain_table,rotation,quat_multiply
sys.path.insert(0,str(ROOT/'companion/perception'))
from depth_geometry import obstacle_points,box_depth

OUT=ROOT/'artifacts/simulation/2026-10-20'

def main():
    fc,nav=libraries();model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    s=np.array([0,0,1.5,0,0,0,0.],np.float64);goal=np.array(model['goal_m'],np.float64)
    obstacles=np.array(model['obstacles_x_y_radius_m'],np.float64).ravel();out=np.zeros(6,np.float64)
    assert nav.qc_nav_plan(s,goal,obstacles,2)
    nav.qc_nav_step(s,0,out);assert out[4]==0 and out[3]==.5
    bad=s.copy();bad[0]=np.nan;nav.qc_nav_step(bad,1,out);assert out[4]==0
    invalid=goal.copy();invalid[0]=1e100;assert not nav.qc_nav_plan(s,invalid,obstacles,2)
    occupied=goal.copy();occupied[:2]=[3,0];assert not nav.qc_nav_plan(s,occupied,obstacles,2)
    blocked=np.array([4,0,100],np.float64);assert not nav.qc_nav_plan(s,goal,blocked,1)
    # Metric camera points: center pixel of a 4m depth plane maps to (0,0,4).
    depth=np.full((24,32),4.);pts=obstacle_points(depth,(100,100,16,12),stride=1)
    assert np.any(np.linalg.norm(pts-[0,0,4],axis=1)<1e-12)
    assert box_depth(depth,[0,0,32,24])==4
    depth[:]=np.nan;assert len(obstacle_points(depth,(100,100,16,12)))==0
    assert box_depth(depth,[0,0,32,24]) is None
    # Rotation group checks independent of plant dynamics.
    q=np.array([.8,.2,-.3,.4]);q/=np.linalg.norm(q);r=rotation(q)
    assert np.max(abs(r.T@r-np.eye(3)))<1e-14 and abs(np.linalg.det(r)-1)<1e-14
    # Baseline and selected gains including motor lag: all linear roots must be LHP.
    roots={}
    for scale in [1,1.3]:
        gains=gain_table(model,scale);arm=.45/(2*np.sqrt(2));bx=4*arm*6/.012*180/np.pi
        byaw=4*.015*6/.022*180/np.pi
        for label,b,index in [('roll',bx,1),('yaw',byaw,4)]:
            kp,ki,kd=gains[index];v=np.roots([.04,1,b*kp,b*ki]);assert max(v.real)<0
            roots[f'{label}-{scale}']=[[float(x.real),float(x.imag)] for x in v]
        kp,ki,kd=gains[5];v=np.roots([.04,1,20*kd,20*kp,20*ki]);assert max(v.real)<0
        roots[f'altitude-{scale}']=[[float(x.real),float(x.imag)] for x in v]
    # Matrix exponential for nilpotent A and noncommuting counterexample.
    a=np.array([[0.,1],[0,0]]);assert np.array_equal(a@a,np.zeros((2,2)))
    b=a.T;assert not np.array_equal(a@b,b@a)
    summary=json.loads((OUT/'summary.json').read_text());best=next(x for x in summary['results'] if x['name']==summary['best_nominal_case'])
    refinements=[];base=np.genfromtxt(OUT/f"{best['name']}.csv",names=True,delimiter=',')
    for substeps in [2,4]:
        res=run_case(fc,nav,model,OUT,f'refine-{substeps}',scale=best['gain_scale'],substeps=substeps)
        d=np.genfromtxt(OUT/f'refine-{substeps}.csv',names=True,delimiter=',');n=min(len(d),len(base))
        # Aligned logged times, not final points at potentially different stopping times.
        difference=np.sqrt(sum((d[k][:n]-base[k][:n])**2 for k in ['x_m','y_m','z_m']))
        res['max_position_difference_from_2ms_m']=float(difference.max());refinements.append(res)
        assert res['success']
    report={'failure_checks':'stale, NaN, outside map, occupied goal, blocked map: passed',
      'depth_geometry':'center projection, metric plane, invalid-depth rejection: passed',
      'rotation_algebra':'orthogonality, determinant, nilpotent power, noncommutativity: passed',
      'linear_roots_with_motor_lag':roots,'refinements':refinements,
      'refinement_note':'Control and noise sampling remain at 500Hz; only physical integration is subdivided.'}
    (OUT/'analysis-tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
    for ax,file,unit in zip(axes,['angle-mc.csv','altitude-mc.csv'],['deg','m']):
        d=np.genfromtxt(OUT/file,names=True,delimiter=',');names=d.dtype.names
        empirical='empirical_error_std_deg' if file.startswith('angle') else 'empirical_std_m'
        predicted='kalman_std_deg' if file.startswith('angle') else 'kalman_std_m'
        ax.plot(d['time_s'],d[empirical],label='empirical error std');ax.plot(d['time_s'],d[predicted],label='C predicted std')
        ax.set(xlabel='time [s]',ylabel=f'standard deviation [{unit}]',title=f'{file}: 500 trials');ax.grid(alpha=.3);ax.legend()
    fig.suptitle('Matched-model Gaussian Monte Carlo, separate from nonlinear route')
    fig.savefig(OUT/'kalman-monte-carlo.png',dpi=160);plt.close(fig)
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
