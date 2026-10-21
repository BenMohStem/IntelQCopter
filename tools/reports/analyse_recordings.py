"""Native-frame video review + numerical analysis of preserved simulation CSVs.

No new flight, frame interpolation, or physical parameter identification is performed.
"""
import argparse,csv,hashlib,html,json,math
from pathlib import Path
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]

def analyse(name,out):
    folder=ROOT/'artifacts/gazebo-suite'/name
    with (folder/'flight.csv').open(newline='') as f:rows=list(csv.DictReader(f))
    v=lambda key:np.array([float(r[key]) for r in rows]);t=v('time_s');mode=v('mode').astype(int);events=v('mode_events').astype(int)
    indexes=np.flatnonzero(mode==1)
    if len(indexes):
        first=indexes[0];end=first+1
        while end<len(rows) and mode[end]==1 and events[end]==events[first]:end+=1
        segment=slice(first,end)
    else:segment=slice(0,len(rows))
    st=t[segment];yaw=np.unwrap(np.deg2rad(v('yaw_deg')[segment]));rate=np.rad2deg(np.gradient(yaw,st)) if len(st)>1 else np.array([0.])
    airborne=v('z')[segment]>.8
    result=dict(name=name,scope='Native Gazebo / active C host core / ROS2 Cyclone DDS. Synthetic sensors/localization and unmeasured F450 plant. First contiguous navigation-mode interval evaluated separately from later manual interaction.',telemetry_hz=float(1/np.median(np.diff(t))),segment_start_s=float(st[0]),segment_end_s=float(st[-1]),yaw_travel_deg=float(np.rad2deg(np.abs(np.diff(yaw)).sum())),yaw_rate_rms_dps=float(np.sqrt(np.mean(rate**2))),peak_yaw_rate_dps=float(np.abs(rate).max()),max_abs_roll_pitch_deg=float(max(np.abs(v('roll_deg')[segment]).max(),np.abs(v('pitch_deg')[segment]).max())),sampled_panel_contact_rows=sum(r['obstacle_contact']=='True' for r in rows[segment]),max_horizontal_speed_mps=float(np.hypot(v('vx')[segment],v('vy')[segment]).max()),height_estimate_rmse_m=float(np.sqrt(np.mean((v('z')[segment]-.082-v('estimated_z')[segment])**2))))
    if (folder/'engine-summary.json').exists():
        engine=json.loads((folder/'engine-summary.json').read_text());result['whole_session_native_contact_steps']=engine['obstacle_contact_steps'];result['assumed_parameters']=engine['assumed_parameters']
    if (folder/'result.json').exists():
        r=json.loads((folder/'result.json').read_text());result.update(route_passed=r.get('passed',False),landing_error_m=r.get('arrival_error_m'))
    if (folder/'route-check.json').exists():result['waypoints']=json.loads((folder/'route-check.json').read_text()).get('waypoints',[])
    # One-second bins retain min/max as well as means; original high-rate CSV stays intact.
    summary=[]
    for second in range(math.floor(t[0]),math.ceil(t[-1])):
        mask=(t>=second)&(t<second+1)
        if not mask.any():continue
        item=dict(time_s=second)
        for key in ['z','roll_deg','pitch_deg','yaw_deg','vx','vy','vz']:
            q=v(key)[mask];item.update({key+'_mean':float(q.mean()),key+'_min':float(q.min()),key+'_max':float(q.max())})
        summary.append(item)
    with (out/(name+'-1s-summary.csv')).open('w',newline='',encoding='utf8') as f:w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    cap=cv2.VideoCapture(str(folder/'gazebo.mp4'));fps=cap.get(cv2.CAP_PROP_FPS);count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));result['native_video_fps']=fps;result['video_frames']=count;result['video_seconds']=count/fps if fps else 0
    chosen=set(np.linspace(0,max(0,count-1),12).astype(int));thumbnails=[];previous=None;frame_rows=[];index=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        small=cv2.resize(frame[:360,:640],(160,90));delta=float(np.mean(np.abs(small.astype(float)-previous.astype(float)))) if previous is not None else 0.;previous=small
        sec=index/fps;near=int(np.clip(np.searchsorted(t,t[0]+sec),0,len(t)-1));frame_rows.append(dict(frame=index,video_time_s=sec,nearest_sim_time_s=float(t[near]),mean_image_delta=delta,z=float(v_cached(rows,near,'z')),roll_deg=float(v_cached(rows,near,'roll_deg')),pitch_deg=float(v_cached(rows,near,'pitch_deg')),yaw_deg=float(v_cached(rows,near,'yaw_deg'))))
        if index in chosen:
            thumb=cv2.resize(frame,(320,180));tile=np.full((210,320,3),[30,40,50],np.uint8);tile[:180]=thumb;cv2.putText(tile,f'Native {fps:g} fps | video {sec:.1f}s',(8,200),cv2.FONT_HERSHEY_SIMPLEX,.45,(235,240,245),1);thumbnails.append(tile)
        index+=1
    cap.release();result['analysed_video_frames']=index
    with (out/(name+'-frames.csv')).open('w',newline='',encoding='utf8') as f:w=csv.DictWriter(f,fieldnames=list(frame_rows[0]));w.writeheader();w.writerows(frame_rows)
    sheet=np.zeros((math.ceil(len(thumbnails)/3)*210,960,3),np.uint8)
    for i,img in enumerate(thumbnails):sheet[(i//3)*210:(i//3+1)*210,(i%3)*320:(i%3+1)*320]=img
    cv2.imwrite(str(out/(name+'-contact-sheet.jpg')),sheet)
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained');ax=axes[0,0];ax.plot(v('x'),v('y'),color='#aab3bd',label='Whole session');ax.plot(v('x')[segment],v('y')[segment],color='#007f87',label='First navigation interval');ax.set(xlabel='Map x (m)',ylabel='Map y (m)');ax.legend(fontsize=8)
    axes[0,1].plot(st,np.rad2deg(yaw));axes[0,1].set(xlabel='Simulation time (s)',ylabel='Unwrapped yaw (degrees)',title='Full-rate telemetry, no angle-wrap jumps')
    ax=axes[1,0]
    for key in ['roll_deg','pitch_deg']:ax.plot(st,v(key)[segment],label=key)
    ax.set(xlabel='Simulation time (s)',ylabel='Degrees');ax.legend(fontsize=8)
    ax=axes[1,1];ax.plot(t,v('z'),label='Native COM height');ax.plot(t,v('estimated_z')+.082,label='C Kalman height + ground COM offset');ax.axvspan(st[0],st[-1],color='#00a19c',alpha=.08);ax.set(xlabel='Simulation time (s)',ylabel='Map z (m)');ax.legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle(name+'\nOriginal telemetry; simulated plant and sensors, not physical measurements',fontsize=12);fig.savefig(out/(name+'-graphs.png'),dpi=125);plt.close(fig)
    result['original_sha256']={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in ['flight.csv','gazebo.mp4','engine.log','world.sdf']}
    return result

def v_cached(rows,i,k):return rows[i][k]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='append',required=True);p.add_argument('--name',required=True);a=p.parse_args()
    if any(not n.replace('-','').replace('_','').isalnum() for n in [a.name,*a.run]):raise SystemExit('Plain names required')
    out=ROOT/'artifacts/reports'/a.name;out.mkdir(parents=True,exist_ok=False);results=[analyse(n,out) for n in a.run];cards=[]
    for r in results:
        n=r['name'];rel='../../gazebo-suite/'+n
        cards.append(f'''<section><h2>{html.escape(n)}</h2><p>Original video: {r['native_video_fps']:g} fps · {r['analysed_video_frames']} native frames analysed · telemetry: {r['telemetry_hz']:.0f} Hz</p><video controls preload="metadata" data-fps="{r['native_video_fps']}" src="{rel}/gazebo.mp4"></video><div><button onclick="step(this,-1)">Previous native frame</button> <button onclick="step(this,1)">Next native frame</button> <button onclick="speed(this,.5)">Half speed</button> <button onclick="speed(this,1)">Normal speed</button></div><p>Evaluated navigation interval {r['segment_start_s']:.2f}–{r['segment_end_s']:.2f}s; yaw travel {r['yaw_travel_deg']:.1f}°, yaw-rate RMS {r['yaw_rate_rms_dps']:.2f}°/s. Whole-session native panel-contact steps: {r.get('whole_session_native_contact_steps','unknown')}. Sampled navigation contact rows: {r['sampled_panel_contact_rows']}.</p><p><a href="{n}-frames.csv">Per-native-frame analysis</a> · <a href="{n}-1s-summary.csv">One-second graph means/min/max</a> · <a href="{rel}/flight.csv">Original telemetry</a> · <a href="{rel}/drone-mission.log">Original ROS2 log</a> · <a href="{rel}/engine-summary.json">Model assumptions and native contacts</a></p><img src="{n}-graphs.png"><details><summary>Representative native frames</summary><img src="{n}-contact-sheet.jpg"></details></section>''')
    comparison=None
    if len(results)>=2:
        before,after=results[:2];comparison=dict(before=before['name'],after=after['name'],yaw_travel_change_percent=100*(after['yaw_travel_deg']/before['yaw_travel_deg']-1),yaw_rate_rms_change_percent=100*(after['yaw_rate_rms_dps']/before['yaw_rate_rms_dps']-1),scope='Single repeatable simulated case per version; not confidence bounds or measured aircraft improvement.')
    data=dict(scope=__doc__,runs=results,comparison=comparison);(out/'summary.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf8')
    title='''<!doctype html><meta charset="utf-8"><title>Flight recordings and heading analysis</title><style>body{font:16px system-ui;background:#edf3f7;color:#183348;max-width:1400px;margin:30px auto;padding:0 20px;line-height:1.6}section{background:white;padding:23px;border-radius:12px;margin:20px 0}video{width:90%;max-width:960px}img{width:100%}button{padding:10px;font:inherit}a{color:#007e87}.scope{background:#fff5df;border-left:5px solid #d69540;padding:15px}</style><h1>Native recordings · frame review · control data</h1><p class="scope">Native Gazebo 10.5/OGRE2, active ESP32 C host core, real ROS2 Cyclone DDS. Synthetic sensor/calibration/localization data; unmeasured 1.2 kg F450 model, inertia (0.012,0.012,0.022) kg·m², 6 N/motor and 40 ms lag. No physical BLE/I2C/UART/ESC or aircraft precision claim. New capture targets 15 fps; earlier 5 fps recordings remain 5 fps, with no interpolation. Full telemetry drives the numerical metrics; one-second graph summaries retain minima and maxima.</p><p>Baseline vs path-heading-hold runs use the same five-node mission and simulated Xbox reports. The browser trial uses real Xbox input plus explicitly labelled PC simulator gestures; its later manual collision is retained and must not be described as an entirely successful session.</p><p><a href="summary.json">Metrics, comparison, source hashes and assumptions</a> · <a href="../../../docs/live-xbox-and-companion.md">Flight station instructions</a></p>'''
    if comparison:title+='<p><b>Single-case comparison:</b> yaw travel change '+f"{comparison['yaw_travel_change_percent']:.1f}%"+', yaw-rate RMS change '+f"{comparison['yaw_rate_rms_change_percent']:.1f}%"+'. See original data before drawing conclusions about the real aircraft.</p>'
    script='''<script>function video(b){return b.closest('section').querySelector('video')}function step(b,d){let v=video(b);v.pause();v.currentTime=Math.max(0,Math.min(v.duration,v.currentTime+d/Number(v.dataset.fps)))}function speed(b,r){video(b).playbackRate=r}</script>'''
    (out/'index.html').write_text(title+''.join(cards)+script,encoding='utf8');print(json.dumps(data,indent=2))
if __name__=='__main__':main()
