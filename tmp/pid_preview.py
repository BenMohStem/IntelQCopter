from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/attitude-preview/main.cpp';s=p.read_text(encoding='utf-8')
code='''// Diagnostic roll-rate PID, zero setpoint. No actuator consumes this value.
// Illustrative normalized-output gains, NOT physical flight settings.
struct PidPreview
{
    float p;
    float i;
    float d;
    float output;
    float previous_rate;
    bool initialized;
};
static PidPreview roll_pid = {};

static void update_pid_preview(PidPreview *pid, float rate)
{
    const float dt = 0.001f;
    const float limit = 0.2f;
    const float error = -rate;
    pid->p = 0.1f * error;
    float slope = pid->initialized ? (rate - pid->previous_rate) / dt : 0.0f;
    float derivative = -0.001f * slope;
    pid->d += dt / (0.02f + dt) * (derivative - pid->d);
    float candidate_i = fmaxf(-limit, fminf(limit, pid->i + 0.05f * error * dt));
    float sum = pid->p + candidate_i + pid->d;
    // Keep integration when it helps leave saturation; reject outward growth.
    if (!((sum > limit && error > 0.0f) || (sum < -limit && error < 0.0f)))
    {
        pid->i = candidate_i;
    }
    pid->output = fmaxf(-limit, fminf(limit, pid->p + pid->i + pid->d));
    pid->previous_rate = rate;
    pid->initialized = true;
}

'''
s=s.replace('// Core 1 owns sensor state.',code+'// Core 1 owns sensor state.') if '// Core 1 owns sensor state.' in s else s.replace('// Core 1 owns sensor state. Core 0 owns slow serial formatting/transmission.',code+'// Core 1 owns sensor state. Core 0 owns slow serial formatting/transmission.')
assert 'static void update_pid_preview' in s
s=s.replace('    float quaternion[4];','    float quaternion[4];\n    float pid_terms[4];')
s=s.replace('''            Serial.printf("Q,%lu''','''            Serial.printf("C,%lu,%.6f,%.6f,%.6f,%.6f\\n", (unsigned long)packet.sequence,
                packet.pid_terms[0], packet.pid_terms[1], packet.pid_terms[2], packet.pid_terms[3]);
            Serial.printf("Q,%lu''')
s=s.replace('''    latest_roll_deg = roll;''','''    update_pid_preview(&roll_pid, body_gyro[0]);
    latest_roll_deg = roll;''')
s=s.replace('''        packet.roll = roll;''','''        packet.pid_terms[0] = roll_pid.p;
        packet.pid_terms[1] = roll_pid.i;
        packet.pid_terms[2] = roll_pid.d;
        packet.pid_terms[3] = roll_pid.output;
        packet.roll = roll;''')
s=s.replace('FUSION_TARGETS_V1','PID_DEMO_V1');p.write_text(s,encoding='utf-8',newline='\n')
p=r/'tools/visualization/attitude_preview.py';s=p.read_text(encoding='utf-8')
s=s.replace('plt.subplots(3,1,sharex=True,figsize=(10,8))','plt.subplots(4,1,sharex=True,figsize=(11,10))\npid_history=[deque(maxlen=300) for _ in range(5)]')
s=s.replace("axes[2].set_xlabel('Host receive time (s)')", "axes[3].set_xlabel('Host receive time (s)')\naxes[3].set_ylabel('Roll-rate PID demo')\npid_lines=[axes[3].plot([],[],label=name)[0] for name in ['P','I','D','Sum (bounded)']]")
s=s.replace("        if verified and value.startswith('T,'):", """        if verified and value.startswith('C,'):
            fields=value.split(',')
            if len(fields)==6:
                for buf,v in zip(pid_history,[time.monotonic()-started]+[float(v) for v in fields[2:]]):buf.append(v)
        if verified and value.startswith('T,'):""")
s=s.replace('    for i,line in enumerate(target_lines):line.set_data(target_history[0],target_history[i+1])','    for i,line in enumerate(target_lines):line.set_data(target_history[0],target_history[i+1])\n    for i,line in enumerate(pid_lines):line.set_data(pid_history[0],pid_history[i+1])')
s=s.replace('return lines+qlines+target_lines','return lines+qlines+target_lines+pid_lines')
p.write_text(s,encoding='utf-8',newline='\n');compile(s,str(p),'exec')
out=r/'measurements/software/2026-10-31-pid-preview';out.mkdir(exist_ok=True)
(out/'README.md').write_text('''# First PID calculation, zero roll-rate target

This small next step demonstrates one inner roll-rate PID only. The zero target means stopping rotation, not returning to a level angle. Xbox targets remain visible but do NOT drive this demonstration. No angle cascade, mixer, PWM or ESC adapter is connected. No motor output or tuning acceptance.

At every nominal1ms FIFO sample, e=0−ωx (rad/s). P=0.1e. I candidate=clip(I+0.05eΔt,±0.2). D raw=−0.001(ωx−previousωx)/Δt; a first-order filter withτ=0.02s smooths D. Its coefficient is Δt/(τ+Δt). First sample has zero derivative. Sum=P+I+D, output clipped±0.2. Outward integral growth during saturation is rejected, while integration that reduces saturation remains allowed. These illustrative coefficients produce dimensionless demo commands; they are not gains calculated from the aircraft dynamics and cannot drive flight motors. No Ki/Kd tuning claim.

C records contain sequence,P,I,D,bounded_sum. The graph shows them. Hold still, gently twist about drone X, then stop: P should oppose rate and D respond to rate change. I remembers accumulated rate error; a stopped tilted drone may retain I. No need to chase it back to zero or recalibrate. This demo is purely for understanding PID terms and safeguards. It is not a roll-angle controller or physical acceptance of feedback signs.

Commands: upload-attitude-preview --upload-port COM3, then view-attitude --port COM3. New combined firmware requires its own hardware timing/input check. Previous logs verify only previous source; newest Xbox-target log has953Trecords and both validity states with full configured endpoints, not an independently attested disconnect/axis-sign pass.

Teacher's previous quaternion illustration was already pushed as7e77c87. Do not add claims that this new PID has been flight-tested to that report.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-31-pid-preview','date':'2026-10-31','domains':['operator-revision'],'summary':'User requests next step; prior illustration already published to teacher branch.','outcome':'Prepare one roll-rate PID numerical demonstration with zero target and illustrative gains, saturation/conditional integration/filtered measurement derivative. Graphs only, no angle cascade/mixer/output or hardware acceptance.'}
(r/'knowledge/events/2026-10-31-pid-preview.json').write_text(json.dumps(event,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:f.write('\n[First roll-rate PID numerical demonstration](../measurements/software/2026-10-31-pid-preview/README.md). Graphs only; no actuator path.\n')
