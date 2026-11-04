from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/attitude-preview/main.cpp';s=p.read_text(encoding='utf-8')
old='''    bool recent = xbox.isConnected() && !xbox.isWaitingForFirstNotification()
        && (uint32_t)(now - xbox.getReceiveNotificationAt()) <= 300;'''
new='''    // A held stick may produce no new report. Keep its DISPLAY target connected.
    bool input_ready = xbox.isConnected() && !xbox.isWaitingForFirstNotification();'''
assert old in s;s=s.replace(old,new).replace('    if (recent)\n','    if (input_ready)\n').replace('    bool valid = recent && neutral_seen;','    bool valid = input_ready && neutral_seen;').replace('pitch = stick_command(xbox.xboxNotif.joyRVert);','pitch = -stick_command(xbox.xboxNotif.joyRVert);').replace('PID_DEMO_V1','PID_DEMO_V2')
p.write_text(s,encoding='utf-8',newline='\n')
out=r/'measurements/software/2026-10-31-held-stick-fix';out.mkdir(exist_ok=True)
test=(r/'measurements/software/2026-10-31-xbox-targets/test_targets.py').read_text()
test=test.replace('tick(1000);assert(!observed.valid);','''xbox.xboxNotif.joyRVert=65535;xbox.received=700;tick(700);
 assert(observed.valid && observed.pitch==-20);
 tick(2000);assert(observed.valid && observed.pitch==-20);
 xbox.xboxNotif.joyRVert=0;xbox.received=2100;tick(2100);
 assert(observed.valid && observed.pitch==20);
 xbox.connected=false;tick(2200);assert(!observed.valid && observed.pitch==0);
 xbox.connected=true;xbox.received=2300;tick(2300);assert(!observed.valid);
 xbox.xboxNotif.joyRVert=32768;xbox.received=2400;tick(2400);assert(observed.valid);''')
test=test.replace('startup, disconnect, recenter, stale guard','startup, held stick without reports, reversed pitch, disconnect and recenter')
(out/'test_targets.py').write_text(test,encoding='utf-8')
(out/'README.md').write_text('''# Pitch direction and held-stick fix

Two functional changes only: negate the right vertical stick mapping; stop clearing display readiness merely because an unchanged stick has not produced a report for300ms. Before: that timeout set neutral_seen=false, so the command went to0 and required recentering. After: retain the last decoded display target while connected and initial input received. Disconnect still zeros targets and requires neutral after reconnect. The independent300ms Xbox-worker-to-telemetry watchdog remains, so a stopped worker cannot leave its packet valid indefinitely.

This preview has no actuator path. Holding the last BLE report while connected is not an accepted flight-link failsafe; a measured liveness policy must be resolved before connecting targets to motors. No new task/queue/optimization or calibration change. FIFO nominal1kHz configuration retained.

Operator requests correctness/readability first, optimization later. Two small code corrections, no extra hardware test campaign. Check pitch forward/back and hold one stick still for2seconds, then release. Host regression covers both pitch endpoints, held values without reports, disconnect/reconnect and neutral recovery, plus bounded monotonic mapping.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-31-held-stick-fix','date':'2026-10-31','domains':['operator-revision'],'summary':'User reports reversed pitch and held-stick commands dropping to zero until recentered; asks simple correct code at nominal1kHz, optimization later.','outcome':'Reverse pitch sign and remove input-report-age gate from display readiness. Preserve disconnect-neutral gate and independent worker watchdog; no actuator or flight failsafe acceptance.'}
(r/'knowledge/events/2026-10-31-held-stick-fix.json').write_text(json.dumps(event,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:f.write('\n[Simple pitch/held-stick preview correction](../measurements/software/2026-10-31-held-stick-fix/README.md).\n')
