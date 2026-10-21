"""Plot retained two/three-point Gazebo trials; does not run a simulation."""
import argparse,csv,hashlib,html,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='append',required=True);p.add_argument('--name',required=True);a=p.parse_args()
    if not a.name.replace('-','').replace('_','').isalnum():raise SystemExit('Plain report name required')
    out=ROOT/'artifacts/reports'/a.name;out.mkdir(parents=True,exist_ok=False);cards=[];results=[]
    for name in a.run:
        if not name.replace('-','').replace('_','').isalnum():raise SystemExit('Plain run name required')
        folder=ROOT/'artifacts/gazebo-suite'/name;result=json.loads((folder/'result.json').read_text());check=json.loads((folder/'route-check.json').read_text())
        with (folder/'flight.csv').open(newline='') as f:rows=list(csv.DictReader(f))
        v=lambda key:np.array([float(r[key]) for r in rows]);fig,axs=plt.subplots(1,3,figsize=(16,4.8),layout='constrained');ax=axs[0]
        ax.plot(v('x'),v('y'),label='Native Gazebo trajectory',color='#007e87');points=np.array(check['points']);ax.scatter(points[:,0],points[:,1],c=['#147f60']+(['#c9a01f'] if len(points)==3 else [])+['#d47738'],zorder=4)
        for i,(x,y,z) in enumerate(points):ax.annotate(['Start','Finish'][i] if len(points)==2 else ['Start','Middle','Finish'][i],(x,y),xytext=(4,8),textcoords='offset points')
        for x,y,yaw in result['engine']['environment']['panels']:
            c,s=np.cos(yaw),np.sin(yaw);rot=np.array([[c,-s],[s,c]]);corners=np.array([[-.04,-.6],[.04,-.6],[.04,.6],[-.04,.6]])@rot.T+[x,y];ax.add_patch(Polygon(corners,color='#334885'))
        ax.set(xlabel='Map x (m)',ylabel='Map y (m)',title='Route and physical panels',xlim=(-.6,9.5),ylim=(-3,3));ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=8)
        t=v('time_s');axs[1].plot(t,v('z'),label='Native COM height');axs[1].plot(t,v('estimated_z')+.082,label='C Kalman estimate');axs[1].axhline(1.25,color='#999',ls=':',label='Cruise target');axs[1].set(xlabel='Simulation time (s)',ylabel='Map z (m)',title='Takeoff, waypoint, landing');axs[1].legend(fontsize=8)
        for k in ['roll_deg','pitch_deg','yaw_deg']:axs[2].plot(t,v(k),label=k.removesuffix('_deg'))
        axs[2].set(xlabel='Simulation time (s)',ylabel='Degrees',title='Native attitude');axs[2].legend(fontsize=8)
        for ax in axs:ax.grid(alpha=.2)
        fig.suptitle(f'{len(points)}-point route · native Gazebo / Cyclone DDS / C host core\nSynthetic sensors and localization; unmeasured model; simulated Xbox mode selection',fontsize=12);fig.savefig(out/(name+'.png'),dpi=130);plt.close(fig)
        hashes={p:hashlib.sha256((folder/p).read_bytes()).hexdigest() for p in ['flight.csv','gazebo.mp4','world.sdf','wire-inputs.jsonl','pc-map-missions.jsonl']}
        item=dict(name=name,passed=bool(result['passed'] and check['passed']),landing_error_m=result['arrival_error_m'],obstacle_contact_steps=result['engine']['obstacle_contact_steps'],simulated_seconds=result['transport']['simulated_seconds'],route_check=check,original_sha256=hashes);results.append(item)
        rel='../../gazebo-suite/'+name;esc=html.escape(name)
        cards.append(f'<section><h2>{len(points)} points — {"PASS" if item["passed"] else "FAIL"}</h2><p>Landing error {100*item["landing_error_m"]:.2f} cm · panel contact steps {item["obstacle_contact_steps"]} · {item["simulated_seconds"]:.1f} simulated seconds</p><video controls preload="metadata" src="{rel}/gazebo.mp4"></video><img src="{esc}.png" alt="Plots from original native flight CSV"><p><a href="{rel}/route-check.json">Waypoint checks</a> · <a href="{rel}/flight.csv">Original CSV</a> · <a href="{rel}/drone-mission.log">Original ROS2 log</a> · <a href="{rel}/engine.log">Original Gazebo log</a> · <a href="{rel}/pc-map-missions.jsonl">PC request and response</a> · <a href="{rel}/engine-summary.json">Model assumptions/source hashes</a></p></section>')
    (out/'summary.json').write_text(json.dumps(dict(scope=__doc__,runs=results),indent=2)+'\n',encoding='utf8')
    body='''<!doctype html><meta charset="utf-8"><title>Live Xbox setup and route evidence</title><style>body{font:17px system-ui;line-height:1.6;background:#edf3f7;color:#183348;max-width:1380px;margin:32px auto;padding:0 20px}section{background:white;padding:24px;border-radius:14px;margin:24px 0}img{width:100%}video{width:80%;max-width:960px}a{color:#007e87}.scope{border-left:5px solid #d8953c;padding:18px;background:#fff9ed}</style><h1>Xbox flight setup · original route evidence</h1><p><a href="../../../docs/live-xbox-and-companion.md">Launch/control/UART guide</a> · <a href="http://127.0.0.1:8087">Live cameras</a> · <a href="http://127.0.0.1:8086">PC route map</a></p><p class="scope">Native Gazebo 10.5/OGRE2, active ESP32 C host DLL and independent ROS2 processes using Cyclone DDS. These route regressions use simulated Xbox gestures; they are separate from the real Xbox startup checks. Real Xbox connection/reports and YOLO processing were verified in live startup sessions, not claimed as a completed human flight. QEMU separately passed 44 checks. Physical motor outputs remain disabled.<br>Assumed model: 1.2 kg, inertia (0.012,0.012,0.022) kg·m², 450 mm diagonal, 6 N/motor, 40 ms lag; synthetic calibration, sensor noise and map localization. Known-size blue-panel ranging only, not arbitrary obstacle depth or measured hardware precision.</p><p><a href="summary.json">Results and hashes of original evidence</a> · <a href="../../gazebo-suite/live-standalone-smoke-20261021/live-summary.json">Initial live Xbox startup</a> · <a href="../../gazebo-suite/live-standalone-spawn-20261021/live-summary.json">Corrected standalone launch and custom spawn</a> · <a href="../../gazebo-suite/live-ros-smoke-20261021/transport.json">Live ROS2 startup</a> · <a href="../../emulation/live-companion-policy-20261021/uart.log">Original 44-check QEMU UART</a></p><p>The initial standalone startup succeeded internally but its launcher incorrectly reported normal shutdown as failure; its logs/video are preserved. A later launch verified the correction.</p>'''
    (out/'index.html').write_text(body+''.join(cards),encoding='utf8');print(json.dumps(results,indent=2))
if __name__=='__main__':main()
