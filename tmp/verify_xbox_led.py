from pathlib import Path
import serial
import time
import hashlib
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
d=serial.Serial()
d.port='COM3'
d.baudrate=921600
d.timeout=.5
d.dtr=False
d.rts=False
lines=[]
out=r/'measurements/software/2026-10-30-xbox-led'
try:
    d.open()
    d.reset_input_buffer()
    d.write(b'?')
    until=time.monotonic()+8
    with (r/'artifacts/acquisition-comparison/xbox-menu/20261030T125921Z/led-boot-original.log').open('xb') as f:
        while time.monotonic()<until:
            line=d.readline()
            f.write(line)
            if line:
                lines.append(line.decode('ascii',errors='replace').strip())
finally:
    d.close()
sha=hashlib.sha256((r/'lessons/xbox-test-menu/main.cpp').read_bytes()).hexdigest()
assert any(line.startswith('ACQ_MENU_V1,'+sha) for line in lines), lines
selected=[line for line in lines if line.startswith(('ACQ_MENU_V1,','CORE,','MENU,','XBOX,'))]
(out/'boot.json').write_text(json.dumps({'source_sha256':sha,'observed':selected,'engine':'Real ESP32-S3 serial query','LED_visual_validation':'pending operator observation','button_validation':'pending operator actions'},indent=2)+'\n')
print('\n'.join(selected))
guide='''# Xbox connection indicator and simpler test selection

Current installed motor-disabled test-menu firmware, 30 October 2026.

- Green fast blink: searching, 80 ms on / 80 ms off (about 6 flashes/s).
- Green slow blink: connected, waiting for the first input report.
- Green steady: connected and at least one report received.
- DevKitC-1 RGB GPIO48 is assumed; the separate fixed power LED cannot be controlled. Some board revisions use another RGB GPIO: override XBOX_LED_GPIO only after checking the actual board.
- The slow-core library connection call may temporarily pause LED updates during a connection attempt. This is not a hardware heartbeat or flight-readiness signal.

RECENT_INPUT means a report arrived within 300 ms. It can become zero while the controller is idle; it no longer resets the menu or changes the connected LED. Connected alone is not a future flight failsafe: command freshness/loss policy still requires separate validation before motors.

Discovery now requests active scan responses using the existing library. No claim that pairing is fixed until observed on this controller. SCANNING is printed for diagnosis. Reconnection remains managed by the Xbox library; no bonds are erased by our added code.

## Run a test

Close Serial Monitor, then:

```powershell
cd C:\\Users\\Mohamed\\Desktop\\Work\\QCopter-Stepwise
.\\qcopter.ps1 record-xbox-menu --port COM3
```

Pair the Xbox to the ESP32, not the PC. D-pad LEFT/RIGHT selects the test shown in the terminal. Initial selection is DIRECT_NORMAL. Press A to start it. Keep the drone stationary for four seconds and wait for Saved test. B cancels; Y shows selection/information. Menu also advances selection. Release buttons between presses. Records save automatically; Ctrl+C exits.

Choices: DIRECT_NORMAL, FIFO_NORMAL, DIRECT_PAUSES, FIFO_PAUSES. The last two deliberately inject acquisition pauses; they are not normal control modes. Selection does not start a test until A is pressed. A held button does not repeat.

## Evidence and limits

See build.log and boot.json for the exact new source and actual checks. Two initial compile attempts failed (API/header mismatch and private API usage); originals retained and corrections made using this pinned Arduino core's neopixelWrite and public NimBLE scan API. Prior four capture results belong to the earlier source and do not automatically validate this changed source. Physical LED color/blink, button use, and connected-load timing require new observation. No PID, PWM, motor output or calibration change. Production firmware/main.cpp user edit remains untouched and stale in its old evidence snapshot.
'''
(out/'README.md').write_text(guide,encoding='utf-8')
p=r/'important_sources/README.md'
with p.open('a',encoding='utf-8') as f:
    f.write('\nLatest Xbox menu/LED instructions: [connection indicator and simpler test selection](../measurements/software/2026-10-30-xbox-led/README.md). Supersedes earlier Menu-only interaction instructions.\n')
p=r/'knowledge/state.json'
state=json.loads(p.read_text())
state['current_stage']='Xbox menu revised: RGB search/connected indicator, idle report age separated from connection, D-pad selection and A-run. Built/uploaded; see new boot evidence. Physical LED/buttons and timing for this changed source pending. Previous captures remain historical.'
state['operator_revision']['last_port_inventory']='COM3; motor-disabled two-core Xbox menu with LED status'
p.write_text(json.dumps(state,indent=2)+'\n')
event={'schema':2,'id':'2026-10-30-xbox-led-menu','date':'2026-10-30','domains':['acquisition-performance'],'summary':'User reports connected Xbox with intermittent freshness and unclear tests; requests fast green search indicator and simpler selection.','outcome':state['current_stage']}
(r/'knowledge/events/2026-10-30-xbox-led-menu.json').write_text(json.dumps(event,indent=2)+'\n')
