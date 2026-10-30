"""Live calibrated gravity tilt and yaw-rate preview; no motor control."""
import argparse
import hashlib
import time
from pathlib import Path
from collections import deque
import serial
import os
import matplotlib
try:
    import PyQt5
except ImportError as error:
    raise SystemExit('Interactive plotting needs PyQt5: install PyQt5==5.15.11 in the Python environment used by qcopter.ps1') from error
# Use this interpreter's Qt plugins, even when started from a ROS terminal.
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=str(Path(PyQt5.__file__).parent/'Qt5/plugins/platforms')
os.environ['QT_API']='pyqt5'
matplotlib.use('QtAgg',force=True)
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
fig,axes=plt.subplots(3,1,sharex=True,figsize=(10,8))
qhistory=[deque(maxlen=300) for _ in range(5)]
fig.suptitle('Quaternion fusion preview - motors disabled')
lines=[axes[0].plot([],[],label=name)[0] for name in ['Roll','Pitch']]
lines.append(axes[1].plot([],[],label='Yaw rate')[0])
axes[0].set_ylabel('Fused Euler angles (deg)')
axes[0].set_ylim(-180,180)
status=axes[0].text(0.01,0.98,'Waiting for data',transform=axes[0].transAxes,va='top',fontsize=9)
axes[1].set_ylabel('Yaw rate (deg/s)')
axes[2].set_xlabel('Host receive time (s)')
axes[2].set_ylabel('Quaternion components')
axes[2].set_ylim(-1.05,1.05)
qlines=[axes[2].plot([],[],label=name)[0] for name in ['w','x','y','z']]
for ax in axes:ax.legend();ax.grid(alpha=.2)
started=time.monotonic()
def update(_):
    global pending,verified
    raw=device.read(min(device.in_waiting,32768))
    log.write(raw)
    pending+=raw
    while b'\n' in pending:
        line,pending=pending.split(b'\n',1)
        value=line.decode('ascii',errors='replace').strip()
        if value.startswith('ERROR,'):
            print(value);plt.close(fig);return lines
        if value.startswith('HELLO,'):
            if expected not in value:raise ValueError('Wrong firmware source; upload preview first')
            verified=True
        if verified and value.startswith('Q,'):
            fields=value.split(',')
            if len(fields)==6:
                values=[float(v) for v in fields[2:]]
                for buf,v in zip(qhistory,[time.monotonic()-started]+values):buf.append(v)
        if verified and value.startswith('D,'):
            fields=value.split(',')
            if len(fields)!=13:continue
            roll,pitch=float(fields[8]),float(fields[9])
            norm=sum(float(fields[i])**2 for i in (2,3,4))**0.5
            warning='Near vertical pitch: Euler roll is ambiguous' if abs(pitch)>80 else 'Mahony fusion - axes provisional'
            if abs(norm-9.80665)>1.5:warning+=' | Acceleration disturbance'
            status.set_text(warning)
            for buf,v in zip(history,[time.monotonic()-started,roll,pitch,float(fields[12])*57.295779513]):buf.append(v)
    for i,line in enumerate(lines):
        values=list(history[i+1])
        # Euler wrap is a coordinate boundary. Preserve originals in the log.
        if i==0:
            for j in range(len(values)-1,0,-1):
                if abs(values[j]-values[j-1])>180:values[j]=float('nan')
        line.set_data(history[0],values)
    for i,line in enumerate(qlines):line.set_data(qhistory[0],qhistory[i+1])
    for ax in axes:ax.relim();ax.autoscale_view()
    axes[0].set_ylim(-180,180)
    axes[2].set_ylim(-1.05,1.05)
    return lines+qlines
try:
    print('Recording automatically to',folder)
    print('Tilt gently, then return to the starting position. Close plot to finish.')
    animation=FuncAnimation(fig,update,interval=50,cache_frame_data=False)
    plt.show()
finally:
    device.close();log.close()
    print('Original serial saved:',folder)
