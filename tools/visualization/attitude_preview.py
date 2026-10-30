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
fig,axes=plt.subplots(2,1,sharex=True)
fig.suptitle('Calibrated gravity tilt preview â€” provisional axes, motors disabled')
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
    while b'\n' in pending:
        line,pending=pending.split(b'\n',1)
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
