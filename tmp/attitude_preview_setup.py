from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
d=r/'lessons/attitude-preview';d.mkdir(exist_ok=True)
s=(r/'firmware/main.cpp').read_text().replace('0.2808364175f','0.288364175f')
(d/'main.cpp').write_text(s,encoding='utf-8',newline='\n')
(d/'source_stamp.py').write_bytes((r/'lessons/xbox-test-menu/source_stamp.py').read_bytes())
(d/'platformio.ini').write_text('[platformio]\ndefault_envs = preview\nsrc_dir = .\n[env:preview]\nplatform = espressif32@7.1.3\nboard = esp32-s3-devkitc-1\nframework = arduino\nbuild_src_filter = +<main.cpp>\nbuild_flags = -DARDUINO_RUNNING_CORE=1\nextra_scripts = pre:source_stamp.py\nmonitor_speed = 921600\nlib_deps = adafruit/Adafruit MPU6050@2.2.9\n')
p=r/'tools/visualization/attitude_preview.py';p.parent.mkdir(exist_ok=True)
p.write_text('''"""Live calibrated gravity tilt and yaw-rate preview; no motor control."""
import argparse
import hashlib
import time
from pathlib import Path
from collections import deque
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port',default='COM3')
args=parser.parse_args()
folder=root/'artifacts/attitude-preview'/time.strftime('%Y%m%dT%H%M%S')
folder.mkdir(parents=True,exist_ok=False)
device=serial.Serial()
device.port=args.port
device.baudrate=921600
device.timeout=0
device.dtr=False
device.rts=False
device.open()
log=(folder/'serial-original.log').open('wb')
expected=hashlib.sha256((root/'lessons/attitude-preview/main.cpp').read_bytes()).hexdigest()
device.write(b'?')
verified=False
pending=b''
history=[deque(maxlen=300) for _ in range(4)]
fig,axes=plt.subplots(2,1,sharex=True)
fig.suptitle('Calibrated gravity tilt preview — provisional axes, motors disabled')
lines=[axes[0].plot([],[],label=name)[0] for name in ['Roll','Pitch']]
lines.append(axes[1].plot([],[],label='Yaw rate')[0])
axes[0].set_ylabel('Gravity tilt (deg)')
axes[1].set_ylabel('Yaw rate (deg/s)')
axes[1].set_xlabel('Host receive time (s)')
for ax in axes:ax.legend();ax.grid(alpha=.2)
started=time.monotonic()
def update(_):
    global pending,verified
    raw=device.read(min(device.in_waiting,32768))
    log.write(raw)
    pending+=raw
    while b'\\n' in pending:
        line,pending=pending.split(b'\\n',1)
        value=line.decode('ascii',errors='replace').strip()
        if value.startswith('ERROR,'):
            print(value);plt.close(fig);return lines
        if value.startswith('HELLO,'):
            if expected not in value:raise ValueError('Wrong firmware source; upload preview first')
            verified=True
        if verified and value.startswith('D,'):
            fields=value.split(',')
            if len(fields)!=13:continue
            for buf,v in zip(history,[time.monotonic()-started,float(fields[8]),float(fields[9]),float(fields[12])*57.295779513]):buf.append(v)
    for i,line in enumerate(lines):line.set_data(history[0],history[i+1])
    for ax in axes:ax.relim();ax.autoscale_view()
    return lines
try:
    print('Recording automatically to',folder)
    print('Tilt gently, then return to the starting position. Close plot to finish.')
    animation=FuncAnimation(fig,update,interval=50,cache_frame_data=False)
    plt.show()
finally:
    device.close();log.close()
    print('Original serial saved:',folder)
''',encoding='utf-8')
(d/'README.md').write_text('''# Live attitude preview, first visible step

This first preview displays calibrated accelerometer gravity tilt and corrected body yaw rate. It is NOT yet the fused Kalman attitude estimate: translational acceleration corrupts gravity tilt. It reuses the calibrated runtime in an isolated single main.cpp; the user's production coefficient edit is preserved. This snapshot uses the historical proper-rotation coefficient0.288364175, consistent with config/gyro-frame.json. The altered0.2808364175 value is not an accepted rotation update.

Build: .\\qcopter.ps1 build-attitude-preview
Upload when ready: .\\qcopter.ps1 upload-attitude-preview --upload-port COM3
View: .\\qcopter.ps1 view-attitude --port COM3

Close other serial users. Gently tilt left/right and forward/back, then return. Stop once the curves respond and return near their starting values. One short session, no calibration recording required. Logs save automatically; closing the plot ends recording. Physical axis labels remain provisional. No Xbox integration, Kalman fusion, PID or motor outputs in this preview. Next small change is gyro/accelerometer fusion after this display works.
''')
path=r/'project.json';j=json.loads(path.read_text())
for name,args in [('build-attitude-preview',['run','-d','lessons/attitude-preview']),('upload-attitude-preview',['run','-d','lessons/attitude-preview','-t','upload'])]:j['commands'][name]={'runtime':'pio','path':'lessons/attitude-preview/platformio.ini','args':args,'description':name.replace('-',' ')}
j['commands']['view-attitude']={'runtime':'python','path':'tools/visualization/attitude_preview.py','args':[],'description':'Live gravity tilt/yaw-rate plot with automatic original logging'}
path.write_text(json.dumps(j,indent=2)+'\n')
event={'schema':2,'id':'2026-10-30-attitude-preview-preparation','date':'2026-10-30','domains':['operator-revision'],'summary':'Start visible attitude milestone.','outcome':'Prepared isolated calibrated gravity-tilt/yaw-rate preview and auto-logging plot; fusion remains next small change. User firmware edit preserved; no physical acceptance inferred.'}
(r/'knowledge/events/2026-10-30-attitude-preview-preparation.json').write_text(json.dumps(event,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:f.write('\n[First live gravity-tilt and yaw-rate preview](../lessons/attitude-preview/README.md). Fusion follows; no motor outputs.\n')
