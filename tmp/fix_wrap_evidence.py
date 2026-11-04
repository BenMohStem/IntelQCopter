from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/attitude-preview/main.cpp'
s=p.read_text(encoding='utf-8')
s=s.replace('    float roll;\n    float pitch;','    float roll;\n    float pitch;\n    float quaternion[4];')
needle='''        packet.roll = roll;'''
s=s.replace(needle,'''        for (uint8_t component = 0; component < 4; component++)
        {
            packet.quaternion[component] = attitude_q[component];
        }
'''+needle)
needle='''        if (packet.diagnostic)
        {'''
s=s.replace(needle,needle+'''
            Serial.printf("Q,%lu,%.8f,%.8f,%.8f,%.8f\\n", (unsigned long)packet.sequence,
                packet.quaternion[0], packet.quaternion[1],
                packet.quaternion[2], packet.quaternion[3]);''')
s=s.replace('FUSION_PREVIEW_V1','FUSION_PREVIEW_V2')
p.write_text(s,encoding='utf-8',newline='\n')
p=r/'tools/visualization/attitude_preview.py';s=p.read_text(encoding='utf-8')
s=s.replace('fig,axes=plt.subplots(2,1,sharex=True)','fig,axes=plt.subplots(3,1,sharex=True,figsize=(10,8))\nqhistory=[deque(maxlen=300) for _ in range(5)]')
s=s.replace("axes[1].set_xlabel('Host receive time (s)')", "axes[2].set_xlabel('Host receive time (s)')\naxes[2].set_ylabel('Quaternion components')\naxes[2].set_ylim(-1.05,1.05)\nqlines=[axes[2].plot([],[],label=name)[0] for name in ['w','x','y','z']]")
needle="        if verified and value.startswith('D,'):"
s=s.replace(needle,"""        if verified and value.startswith('Q,'):
            fields=value.split(',')
            if len(fields)==6:
                values=[float(v) for v in fields[2:]]
                for buf,v in zip(qhistory,[time.monotonic()-started]+values):buf.append(v)
"""+needle)
s=s.replace('    for i,line in enumerate(lines):line.set_data(history[0],history[i+1])', '''    for i,line in enumerate(lines):
        values=list(history[i+1])
        # Euler wrap is a coordinate boundary. Preserve originals in the log.
        if i==0:
            for j in range(len(values)-1,0,-1):
                if abs(values[j]-values[j-1])>180:values[j]=float('nan')
        line.set_data(history[0],values)
    for i,line in enumerate(qlines):line.set_data(qhistory[0],qhistory[i+1])''')
s=s.replace('    axes[0].set_ylim(-180,180)\n    return lines', '    axes[0].set_ylim(-180,180)\n    axes[2].set_ylim(-1.05,1.05)\n    return lines+qlines')
p.write_text(s,encoding='utf-8',newline='\n');compile(s,str(p),'exec')
out=r/'measurements/software/2026-10-31-wrap-review';out.mkdir(exist_ok=True)
raw=r/'artifacts/attitude-preview/20261031T000710/serial-original.log'
ls=raw.read_text(errors='replace').splitlines()
a=np.array([[float(v) for v in l.split(',')[1:]] for l in ls if l.startswith('D,') and len(l.split(','))==13])
delta=np.diff(a[:,7]);circular=(delta+180)%360-180;wrapped=abs(delta)>180
result={'engine':'Offline analysis of actual FUSION_PREVIEW_V1 log, not simulation','log_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'identity':[l for l in ls if l.startswith('HELLO,')],'rows':len(a),'roll_wrap_crossings':int(wrapped.sum()),'raw_step_max_deg':float(abs(delta).max()),'same_step_shortest_arc_deg':float(abs(circular[np.argmax(abs(delta))])),'max_shortest_arc_step_deg':float(abs(circular).max()),'near_vertical_pitch_rows':int(sum(abs(a[:,8])>80)),'limits':'Euler-only log cannot establish quaternion continuity, physical truth or full estimator accuracy. Some residual motion may be real, coordinate sensitivity or estimator error; do not conflate all with wrap.'}
(out/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
fig,axes=plt.subplots(2,1,figsize=(10,5),layout='constrained')
t=(a[:,0]-a[0,0])/1000
axes[0].plot(t,a[:,7],color='#08774c',label='Logged roll (Euler)');axes[0].plot(t,a[:,8],color='#c56d20',label='Logged pitch (Euler)');axes[0].legend();axes[0].set_ylabel('deg')
axes[1].plot(t[1:],delta,label='Ordinary difference');axes[1].plot(t[1:],circular,label='Shortest circular difference');axes[1].legend();axes[1].set(xlabel='Nominal sequence time (s)',ylabel='deg / diagnostic interval')
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('Actual fusion log: Euler wrapping vs rotation | no quaternion recorded in this version')
fig.savefig(out/'wrap-comparison.png',dpi=150);plt.close(fig)
(out/'README.md').write_text('''# Why the previous symptom remained

Actual log confirms FUSION_PREVIEW_V1, source53a7f37086916ed53eb6b6ac257aafe1ee19e964a96974d86fb41b1cc329e10e. The quaternion implementation was running. analysis.json counts wrap boundaries and reports circular differences; wrap-comparison.png uses every diagnostic record. Large raw roll changes near±180degrees are coordinate wraps. Near±90degree Euler pitch also makes roll/yaw individually poorly conditioned. These are distinct effects; neither is removed merely by using quaternion propagation. Not every residual fluctuation is therefore solved or explained.

![Actual angle and difference comparison](wrap-comparison.png)

V2 adds queued, same-sample quaternion telemetry Q,sequence,w,x,y,z. No estimator gain, calibration, axis labels or filter algorithm changed. Viewer adds quaternion traces and breaks the drawn roll line at±180degree crossings rather than drawing a false full turn. Original D and Q values remain unchanged in saved logs. No low-pass plot trick or forced angle freeze. This is a diagnostic change, not a claim of fixed physical accuracy.

Next one short physical check: upload V2, start resting, tilt/return gently, then inspect quaternion norm and sign-invariant relative rotation between consecutive samples against body rates. Any large quaternion discontinuity must be investigated as a genuine state/timing/frame issue. Current Euler-only logs cannot supply that missing evidence. Stop requesting calibration runs until a specific sensor/reference error is demonstrated.

There is no PID motor loop implemented in this preview. Future large-angle feedback must use a suitable quaternion/vector attitude error, or explicitly restrict a near-hover Euler controller's operating region; feeding raw Euler wrap jumps to derivative feedback would be wrong. Choosing a higher-order filter cannot make a globally unique Euler chart exist. Absolute yaw remains unobservable without an external heading reference.

Research sources: NASA aircraft kinematics, https://ntrs.nasa.gov/api/citations/20160002944/downloads/20160002944.pdf ; Adafruit pinned algorithm, https://github.com/adafruit/Adafruit_AHRS ; x-io acceleration rejection design, https://github.com/xioTechnologies/Fusion/blob/main/README.md . x-io rejects acceleration using angular disagreement and recovery logic; it is a possible later robustness improvement, not evidence that changing libraries fixes Euler wraps. No blind algorithm replacement here.

Commands unchanged: upload-attitude-preview --upload-port COM3, then view-attitude --port COM3. New hardware validation pending; motors disabled.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-31-wrap-review','date':'2026-10-31','domains':['operator-revision'],'summary':'User reports same jitter after fusion and requests deeper analysis/research.','outcome':'Confirmed fusion source from actual log; quantified Euler wrap jumps separately from vertical singularity. Add same-sample quaternion telemetry and diagnostic traces; no claim of physical estimator fix before new evidence.'}
(r/'knowledge/events/2026-10-31-wrap-review.json').write_text(json.dumps(event,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:f.write('\n[Actual fusion wrap diagnosis and quaternion instrumentation](../measurements/software/2026-10-31-wrap-review/README.md). No claim all residual jitter is solved.\n')
print(result)
