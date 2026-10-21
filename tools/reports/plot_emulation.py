"""Plot original emulator UART-derived CSV, without inventing a flight path."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('run',nargs='?',default='verified-replay');a=p.parse_args()
folder=(ROOT/'artifacts/emulation'/a.run).resolve()
if not folder.is_relative_to((ROOT/'artifacts/emulation').resolve()): raise SystemExit('Run must stay inside artifacts/emulation')
with (folder/'telemetry.csv').open() as f: rows=list(csv.DictReader(f))
t=[float(r['time_us'])/1e6 for r in rows]
plt.rcParams.update({'font.family':'DejaVu Sans','figure.facecolor':'#f4f7fb','axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18})
fig,axes=plt.subplots(3,1,figsize=(12,8),layout='constrained')
for motor in ('FL','FR','RR','RL'):
    axes[0].scatter(t,[float(r[motor]) for r in rows],label=motor,s=14)
axes[0].set(ylabel='Normalized motor demand',ylim=(-.03,.7))
axes[0].legend(ncol=4,loc='lower center')
for field in ('roll_deg','pitch_deg'):
    axes[1].plot(t,[float(r[field]) for r in rows],label=field)
axes[1].set(ylabel='Estimated angle (deg)',ylim=(-.1,.1));axes[1].legend()
for field,offset in [('fault',0),('warn',1.4),('disarm',2.8)]:
    axes[2].scatter(t,[float(r[field])+offset for r in rows],label=field,s=12)
axes[2].set(ylabel='Flags (separated vertically)',xlabel='Synthetic input time (s)')
axes[2].legend(ncol=3)
for ax in axes: ax.set_xlim(7.75,10.05)
for sec,label in [(7.804,'roll command'),(8.904,'climb command'),(9.654,'stale'),(9.756,'IMU'),(9.858,'baro'),(9.962,'20%')]:
    axes[0].axvline(sec,lw=.5,color='#68808e',alpha=.7)
    axes[0].text(sec,.665,label,rotation=90,fontsize=8,va='top',ha='right')
fig.suptitle('ESP32-S3 QEMU · recorded replay samples\nSynthetic stationary sensors · no interpolation across unrecorded intervals',fontsize=15)
fig.savefig(folder/'telemetry.png',dpi=150)
fig.savefig(folder/'telemetry.svg')
print('Plotted',folder/'telemetry.png')
