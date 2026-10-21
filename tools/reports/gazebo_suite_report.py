"""Derived plots/video index from preserved native Gazebo trials. No simulation."""
import argparse,csv,html,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=ROOT/'artifacts/gazebo-suite'
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.2,'figure.facecolor':'#f7fafc','axes.facecolor':'white'})

def load_rows(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))
def values(rows,key):return np.array([float(r[key]) for r in rows])
def detail(result,target):
    folder=EVIDENCE/result['name'];rows=load_rows(folder/'flight.csv');t=values(rows,'time_s');z=values(rows,'z')
    fig,axes=plt.subplots(2,2,figsize=(13,7.2),layout='constrained')
    ax=axes[0,0];ax.plot(t,z-.082,label='Gazebo height above initial COM');ax.plot(t,values(rows,'estimated_z'),label='C Kalman height',alpha=.8)
    ax.set(xlabel='Simulated time (s)',ylabel='Relative height (m)');ax.legend(fontsize=8)
    ax=axes[0,1]
    for key in ['roll_deg','pitch_deg','yaw_deg']:ax.plot(t,values(rows,key),label=key.replace('_deg',''))
    ax.set(xlabel='Simulated time (s)',ylabel='Native attitude (degrees)');ax.legend(fontsize=8)
    ax=axes[1,0]
    for key in ['FL','FR','RR','RL']:ax.plot(t,values(rows,key),label=key,alpha=.8,lw=.8)
    ax.set(xlabel='Simulated time (s)',ylabel='Motor response (normalised)',ylim=(-.02,1.02));ax.legend(ncol=4,fontsize=8)
    ax=axes[1,1];pilot=load_rows(folder/'xbox-reports.csv');pt=values(pilot,'time_s')
    for key in ['roll_stick','pitch_stick','vertical_stick','LT','RT']:ax.plot(pt,values(pilot,key),label=key.replace('_stick',''),lw=.9)
    ax.set(xlabel='Simulated time (s)',ylabel='Actual simulated Xbox report values');ax.legend(fontsize=8,ncol=3)
    fig.suptitle(result['name']+'\nNative Gazebo + active C host core; synthetic sensors; unmeasured aircraft parameters',fontsize=12)
    fig.savefig(target,dpi=135);plt.close(fig)
    position=np.column_stack([values(rows,k) for k in ['x','y','z']])
    result['derived_metrics']={'trajectory_distance_m':float(np.sum(np.linalg.norm(np.diff(position,axis=0),axis=1))),
       'max_speed_mps':float(np.linalg.norm(np.column_stack([values(rows,k) for k in ['vx','vy','vz']]),axis=1).max()),
       'max_abs_roll_pitch_deg':float(max(abs(values(rows,'roll_deg')).max(),abs(values(rows,'pitch_deg')).max())),
       'max_height_m':float(z.max()),'height_estimate_rmse_m':float(np.sqrt(np.mean((z-.082-values(rows,'estimated_z'))**2)))}

def trajectories(results,target,kind):
    selected=[r for r in results if r['kind']==kind]
    if not selected:return
    cols=5;rows=math.ceil(len(selected)/cols)
    fig,axes=plt.subplots(rows,cols,figsize=(19,4*rows),squeeze=False,layout='constrained')
    for ax in axes.flat:ax.set_visible(False)
    for ax,r in zip(axes.flat,selected):
        ax.set_visible(True);data=load_rows(EVIDENCE/r['name']/'flight.csv');x,y=values(data,'x'),values(data,'y')
        ax.plot(x,y,color='#008c95' if kind=='manual' else '#ca601f',lw=1.8)
        ax.scatter(x[0],y[0],c='#168344',s=35,label='start');goal=r['requested_landing_point'];ax.scatter(goal[0],goal[1],marker='x',c='#b82039',s=50,label='requested landing')
        if kind=='navigation':
            for px,py,yaw in r['engine']['environment']['panels']:
                c,s=math.cos(yaw),math.sin(yaw);rot=np.array([[c,-s],[s,c]])
                corners=np.array([[-.04,-.6],[.04,-.6],[.04,.6],[-.04,.6]])@rot.T+[px,py]
                ax.add_patch(Polygon(corners,color='#344a81',alpha=.8))
            ax.set(xlim=(-.6,9.6),ylim=(-3.1,3.1))
        else:ax.set(xlim=(-.3,2.1),ylim=(-.85,.85))
        name=r['engine']['environment']['name'].replace('_',' ')
        ax.set(title=f"{r['environment']} · {name}\n{'PASS' if r['passed'] else 'FAIL'} | landing error {r['arrival_error_m']*100:.1f} cm",xlabel='World x (m)',ylabel='World y (m)');ax.set_aspect('equal',adjustable='box')
    fig.suptitle('Native Gazebo trajectories — '+kind+'\nPanel geometry is shown for evaluation only; navigation receives RGB observations.',fontsize=15)
    fig.savefig(target,dpi=130);plt.close(fig)

def main():
    p=argparse.ArgumentParser();p.add_argument('--prefix',action='append',required=True);p.add_argument('--name',required=True);args=p.parse_args()
    if not args.name.replace('-','').replace('_','').isalnum():raise SystemExit('Plain report name required')
    selected={}
    for prefix in args.prefix:
        for result in json.loads((EVIDENCE/f'{prefix}-suite.json').read_text(encoding='utf8')):selected[(result['environment'],result['kind'])]=result
    results=sorted(selected.values(),key=lambda r:(r['kind'],r['environment']))
    out=ROOT/'artifacts/reports'/args.name;out.mkdir(parents=True,exist_ok=True)
    usable=[r for r in results if 'transport' in r and 'engine' in r]
    for r in usable:detail(r,out/(r['name']+'.png'))
    trajectories(usable,out/'manual-trajectories.png','manual');trajectories(usable,out/'navigation-trajectories.png','navigation')
    passed=[r for r in usable if r['passed']];nav=[r for r in passed if r['kind']=='navigation']
    best=min(nav,key=lambda r:r['arrival_error_m']) if nav else None
    summary={'scope':'Native Gazebo 10.5/OGRE2 + C flight core host DLL + ROS2 DDS; synthetic sensors and simulated localization; no physical BLE/ESC/hardware validation.',
        'prefixes':args.prefix,'selected_runs':len(results),'passed':sum(r['passed'] for r in results),'manual_passed':sum(r['passed'] and r['kind']=='manual' for r in results),'navigation_passed':len(nav),
        'best_landing_error_run':best['name'] if best else None,'runs':results}
    cohorts={}
    for kind in ['manual','navigation']:
        errors=np.array([r['arrival_error_m'] for r in passed if r['kind']==kind])
        if len(errors):cohorts[kind]=dict(n=len(errors),mean_landing_error_m=float(errors.mean()),sample_std_landing_error_m=float(errors.std(ddof=1)) if len(errors)>1 else None,max_landing_error_m=float(errors.max()))
    summary['observed_cohort_statistics']=cohorts
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
    cards=[];table=[]
    for r in results:
        name=r['name'];rel='../../gazebo-suite/'+name;esc=html.escape(name)
        table.append(f"<tr><td>{esc}</td><td>{'PASS' if r['passed'] else 'FAIL'}</td><td>{r.get('arrival_error_m',float('nan'))*100:.1f}</td><td>{r.get('transport',{}).get('simulated_seconds',0):.1f}</td></tr>")
        if 'transport' not in r:cards.append(f'<section><h2>{esc} — failed</h2><p>{html.escape(r.get("error",""))}</p><a href="{rel}/ros-boundary.log">Original log</a></section>');continue
        tr=r['transport'];cards.append(f'''<section><h2>{esc} — {'PASS' if r['passed'] else 'FAIL'}</h2>
<video controls preload="none" src="{rel}/gazebo.mp4"></video><img loading="lazy" src="{name}.png" alt="Plots calculated from original flight.csv">
<p>HID reports: {tr['hid_packets']} · companion frames: {tr['companion_packets']} · YOLO/PnP frames: {tr['yolo_frames']}/{tr['pnp_frames']}</p>
<p><a href="{rel}/flight.csv">Flight CSV</a> · <a href="{rel}/xbox-reports.csv">Xbox reports</a> · <a href="{rel}/engine.log">Gazebo original log</a> · <a href="{rel}/ros-boundary.log">ROS transport log</a> · <a href="{rel}/engine-summary.json">Assumptions and result</a> · <a href="{rel}/pc-mission.json">PC request</a> · <a href="{rel}/world.sdf">Exact world</a></p></section>''')
    bestlink=f'<p>Lowest measured landing error in this set: <a href="../../gazebo-suite/{best["name"]}/gazebo.mp4">{best["name"]}</a>. This selection is not a real-world reliability ranking.</p>' if best else ''
    history=[];operators=[]
    for path in sorted(EVIDENCE.glob('*/result.json')):
        r=json.loads(path.read_text(encoding='utf8'))
        if r.get('operator') and r.get('passed'):
            relative='../../gazebo-suite/'+path.parent.name
            operators.append(f'<section><h2>Additional PC map workflow check</h2><p>Outside the 15-case suite: the ground-PC HTTP map tool submitted custom endpoints through ROS2; the virtual Xbox supplied no automatic mission request.</p><video controls preload="none" src="{relative}/gazebo.mp4"></video><p><a href="{relative}/pc-map-missions.jsonl">Original PC requests/replies</a> · <a href="{relative}/pc-mission.json">Virtual Xbox role</a> · <a href="{relative}/result.json">Flight result</a></p></section>')
        if not r.get('passed'):history.append(f'<li><a href="../../gazebo-suite/{path.parent.name}/result.json">{html.escape(path.parent.name)}</a></li>')
    statistics=' · '.join(f"{kind}: mean {c['mean_landing_error_m']*100:.1f} cm, sample SD {(c['sample_std_landing_error_m'] or 0)*100:.1f} cm (n={c['n']})" for kind,c in cohorts.items())
    doc=f'''<!doctype html><meta charset="utf-8"><title>IntelQCopter · native Gazebo evidence</title>
<style>body{{font:16px system-ui;background:#edf3f7;color:#183348;max-width:1440px;margin:32px auto;padding:0 22px;line-height:1.6}}h1{{font-size:38px}}section{{background:white;padding:25px;border-radius:15px;margin:25px 0}}video{{width:70%;max-width:960px;background:#14202a}}img{{width:100%;height:auto}}a{{color:#007c91}}table{{border-collapse:collapse;width:100%;background:white}}td,th{{text-align:left;border-bottom:1px solid #d8e4ea;padding:9px}}.scope{{border-left:5px solid #d78c31;padding:15px;background:#fff9ed}}</style>
<h1>IntelQCopter · native Gazebo flight evidence</h1><p>{summary['passed']} / {len(results)} selected runs passed: {summary['manual_passed']} manual + {summary['navigation_passed']} navigation.</p>
<p class="scope">{html.escape(summary['scope'])}<br>Model: 1.2 kg, J=(0.012,0.012,0.022) kg·m², 450 mm diagonal, 6 N/motor, 40 ms lag; all unmeasured. RGB ranging uses known-size blue fixtures, not arbitrary obstacle depth. Manual pilot is a feedback algorithm emitting raw Xbox reports, not a real human.</p>
{bestlink}<p>{statistics}. These describe the selected simulated cases; they are not sensor uncertainty estimates or hardware confidence bounds.</p><p><a href="summary.json">Machine-readable results and model assumptions</a></p><h2>Trajectories from original CSV</h2><img src="manual-trajectories.png"><img src="navigation-trajectories.png">
<table><tr><th>Run</th><th>Result</th><th>Landing error (cm)</th><th>Simulation (s)</th></tr>{''.join(table)}</table>
{''.join(cards)}{''.join(operators)}<section><h2>Retained unsuccessful attempts</h2><p>These failures are excluded from the final selected set but remain original project evidence.</p><ul>{''.join(history)}</ul></section>'''
    (out/'index.html').write_text(doc,encoding='utf8');print(json.dumps({k:v for k,v in summary.items() if k!='runs'},indent=2))
if __name__=='__main__':main()
