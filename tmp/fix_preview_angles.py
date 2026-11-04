from pathlib import Path
import json
import numpy as np
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'tools/visualization/attitude_preview.py'
s=p.read_text()
s=s.replace("fig.suptitle('Calibrated gravity tilt preview â€” provisional axes, motors disabled')", "fig.suptitle('Calibrated gravity tilt preview - motors disabled')")
s=s.replace("fig.suptitle('Calibrated gravity tilt preview — provisional axes, motors disabled')", "fig.suptitle('Calibrated gravity tilt preview - motors disabled')")
s=s.replace("axes[0].set_ylabel('Gravity tilt (deg)')", "axes[0].set_ylabel('Gravity tilt (deg)')\naxes[0].set_ylim(-180,180)\nstatus=axes[0].text(0.01,0.98,'Waiting for data',transform=axes[0].transAxes,va='top',fontsize=9)")
needle="            for buf,v in zip(history,[time.monotonic()-started,float(fields[8]),float(fields[9]),float(fields[12])*57.295779513]):buf.append(v)"
replacement="""            roll,pitch=float(fields[8]),float(fields[9])
            norm=sum(float(fields[i])**2 for i in (2,3,4))**0.5
            warning='Near vertical pitch: roll is poorly observable from gravity' if abs(pitch)>80 else 'Gravity tilt only - axes provisional'
            if abs(norm-9.80665)>1.5:warning+=' | Acceleration disturbance'
            status.set_text(warning)
            for buf,v in zip(history,[time.monotonic()-started,roll,pitch,float(fields[12])*57.295779513]):buf.append(v)"""
assert needle in s
s=s.replace(needle,replacement)
s=s.replace('    for ax in axes:ax.relim();ax.autoscale_view()', '    for ax in axes:ax.relim();ax.autoscale_view()\n    axes[0].set_ylim(-180,180)')
p.write_text(s,encoding='utf-8',newline='\n')
compile(s,str(p),'exec')
log=r/'artifacts/attitude-preview/20261030T221524/serial-original.log'
a=np.array([[float(v) for v in line.split(',')[1:]] for line in log.read_text(errors='replace').splitlines() if line.startswith('D,') and len(line.split(','))==13])
pitch=np.rad2deg(np.arctan2(-a[:,1],np.hypot(a[:,2],a[:,3])))
roll=np.rad2deg(np.arctan2(a[:,2],a[:,3]))
out=r/'measurements/software/2026-10-30-preview-angle-review';out.mkdir(exist_ok=True)
result={'engine':'Offline recomputation of actual preview acceleration/angle diagnostics','rows':len(a),'max_pitch_recomputation_error_deg':float(np.max(np.abs(pitch-a[:,8]))),'max_roll_recomputation_error_deg':float(np.max(np.abs(roll-a[:,7]))),'roll_range_deg':[float(a[:,7].min()),float(a[:,7].max())],'pitch_range_deg':[float(a[:,8].min()),float(a[:,8].max())],'near_vertical_pitch_rows':int(sum(abs(a[:,8])>80)),'limitations':'Firmware rounds angles to0.001deg and acceleration to6decimal places. No independent pose reference; cannot attribute every fluctuation to a unique cause.'}
(out/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
(out/'README.md').write_text('''# Gravity-angle preview review

Actual serial diagnostics replayed; analysis.json reports independent angle recomputation from logged acceleration. The plot's corrupted title was encoding mojibake. Title is now ASCII; angle scale is fixed at±180degrees and a near-vertical-pitch warning is shown. No angle smoothing, zeroing, calibration or firmware change was made. A large acceleration-magnitude deviation also warns of disturbance; that test cannot detect all non-gravitational acceleration.

Formula: roll=atan2(ay,az); pitch=atan2(-ax,sqrt(ay²+az²)). Under static gravity these are conventional roll/pitch for an upright reference. At pure roll90degrees with pitch0, ay is approximately g and pitch remains well-conditioned. Near pitch90degrees, ay and az approach0; roll becomes ill-conditioned. Thus a general claim that roll90 causes a pitch singularity would be incorrect. Data reaches pitch89.573degrees; substantial cross-axis movement and real acceleration can also change gravity-derived angles. Raw logs do not provide independent true attitude.

Existing display maps D fields8/9 to roll/pitch correctly. Dynamic autoscaling exaggerated tiny resting fluctuations; fixed limits now show their angular size consistently. Roll wrap at±180degrees remains a coordinate discontinuity, not a physical instantaneous rotation. Gyro/accelerometer fusion and an internal quaternion representation are later improvements; Euler display still has coordinate singularities. No claim the plot change fixes a physical estimator or validates feedback direction. Restart viewer only, no upload needed.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-30-preview-angle-review','date':'2026-10-30','domains':['operator-revision'],'summary':'User reports angle fluctuations at large tilt and corrupted plot title.','outcome':'Reviewed actual logged angle formulas; near-vertical pitch observed. Fixed title encoding and angle scale; added observability/disturbance notices. No firmware or calibration change; fusion still pending.'}
(r/'knowledge/events/2026-10-30-preview-angle-review.json').write_text(json.dumps(event,indent=2)+'\n')
print(result)
