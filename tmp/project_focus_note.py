from pathlib import Path
import json,re
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
t=Path(r'C:\Users\Mohamed\Desktop\Work\IntelQCopter-USTHB')
p=t/'learning/gravity-angle-preview/report.html'
s=p.read_text(encoding='utf-8')
s=s.replace('Understanding the attitude preview','Drone attitude estimation')
s=s.replace('One measured recording, one geometric explanation, and a readable account of the plotting fix.','Measured sensor behavior, the geometry of attitude, and the next estimator for IntelQCopter.')
s=re.sub(r'<div class="card"><strong>Result\.</strong>.*?</div>', '<div class="card"><strong>Engineering result.</strong> Calibrated acceleration provides useful tilt information near hover, but large tilts expose coordinate ambiguity and movement disturbs the gravity reference. IntelQCopter needs continuous gyro-based attitude propagation with accelerometer correction before attitude feedback is qualified. That estimator is planned; the current measurements do not validate flight control.</div>',s,count=1,flags=re.S)
start=s.index('<h2>3 · What changed in the code')
end=s.index('<p class="small">Provenance:')
s=s[:start]+'''<h2>3 · How the drone will estimate attitude</h2>
<div class="flow"><span>Gyro: angular velocity</span><span>→ propagate rotation q</span><span>← accelerometer gravity correction</span><span>→ attitude error → PID → motor mixer</span></div>
<p>Gyroscopes track rapid changes but accumulate bias error. Accelerometers constrain gravity direction when translation is small, but cannot provide absolute heading. Their fusion is required for a useful attitude estimate. No magnetometer/GPS heading correction is validated by this measurement.</p>
<h2>4 · Four numbers for one rotation</h2>
<div class="equation">q = w + xi + yj + zk = (w, v)<br>v = (x, y, z), &nbsp; w² + x² + y² + z² = 1</div>
<p>A quaternion extends complex arithmetic. The symbols i,j,k are imaginary basis units; i²=j²=k²=ijk=−1. Multiplication is order-dependent (ij=k, ji=−k). For attitude, a unit quaternion represents a 3D rotation:</p>
<div class="equation">q = (cos(α/2), u sin(α/2))</div>
<p>α is the rotation angle and u a unit vector along its axis. w is the scalar part; v is the vector part. Four components satisfy one unit-length constraint, leaving three local degrees of freedom. q and −q represent the same physical rotation.</p>
<div class="equation">Example: 90° about X → q = (√2/2, √2/2, 0, 0)</div>
<p>The components are not independent roll, pitch and yaw angles. They encode a rotation together. A quaternion state avoids the Euler-coordinate singularity in propagation, while the displayed Euler angles remain ambiguous at singular poses. This does not eliminate sensor noise, drift, or the need to validate feedback directions.</p>
<h2>5 · Next project milestone</h2><p>Implement and evaluate gyro/accelerometer attitude fusion on the ESP32 with motors disabled. Compare response during a guided tilt and return, then qualify command directions and PID/mixer behavior. Existing calibration remains the development baseline. The actual measurements above precede this proposed fusion; they are not evidence that it already works.</p>
''' + s[end:]
s=s.replace('Diagnostic rows are nominally 10 Hz; charts are not 1000 Hz raw attitude ground truth.','Diagnostic rows are nominally 10 Hz; charts are not 1000 Hz raw attitude ground truth.')
p.write_text(s,encoding='utf-8')
idx=t/'learning/index.html';idx.write_text(idx.read_text(encoding='utf-8').replace('Detailed syntax is expandable.','Implementation syntax is kept outside the teacher summary.').replace('Attitude preview: angle sensitivity, real data and plotting corrections','Drone attitude: real sensor data, singularities and quaternion fusion'),encoding='utf-8')
event={'schema':2,'id':'2026-10-30-project-focused-teacher-notes','date':'2026-10-30','domains':['presentation-preferences'],'summary':'Teacher illustrations should explain the drone and essential engineering, not development tools or syntax; detailed tools remain in personal learning material.','outcome':'Teacher note now emphasizes sensor fusion, quaternion rotation and feedback qualification; plotting syntax removed from teacher note only. Quaternion fusion clearly labelled proposed.'}
for root in [r,t]:
    (root/'knowledge/events/2026-10-30-project-focused-teacher-notes.json').write_text(json.dumps(event,indent=2)+'\n')
    p=root/'knowledge/domains/presentation-preferences.json';d=json.loads(p.read_text());d['entries'].append({'id':'preferences.project-focused-notes','status':'user_preference','summary':event['summary'],'sources':[{'kind':'user','date':'2026-10-30','reference':'User clarifies teacher illustrations must discuss the project, not its tools'}]});p.write_text(json.dumps(d,indent=2)+'\n')
print('Updated teacher project note; retained developer code explanation.')
