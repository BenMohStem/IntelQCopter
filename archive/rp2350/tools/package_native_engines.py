"""Package the two completed native runs and make a time-aligned video."""
from pathlib import Path
import hashlib,json,subprocess
import imageio_ffmpeg
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/simulation/2026-10-20-engines'
NAMES={'isaac':'isaac-garden-raytraced','gazebo':'gazebo-garden-ros'}
records={}
for engine,name in NAMES.items():
    summary=json.loads((OUT/f'{name}-summary.json').read_text());metrics=json.loads((OUT/f'{name}-metrics.json').read_text())
    validation=json.loads((OUT/f'{name}-validation.json').read_text())
    assert summary['success'] and all(validation['source_hash_matches'].values())
    assert validation['phase_sequence_complete'] and validation['native_ground_contact'] and validation['virtual_motors_stopped']
    assert validation['physical_arming_all_zero'] and validation['video_decode_exit']==0
    assert summary['actual_ROS2_DDS_control'] and summary['native_RGB_cameras']
    records[engine]={'run':name,'summary':summary,'metrics':metrics,'transport':json.loads((OUT/f'{name}-transport.json').read_text())}
ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
duration=max(r['summary']['duration_s'] for r in records.values())+.1
command=[ffmpeg,'-y','-v','error','-i',str(OUT/f'{NAMES["isaac"]}.mp4'),'-i',str(OUT/f'{NAMES["gazebo"]}.mp4'),
 '-filter_complex','[0:v]tpad=stop_mode=clone:stop_duration=120[a];[1:v]tpad=stop_mode=clone:stop_duration=120[b];[a][b]hstack=inputs=2:shortest=1[v]',
 '-map','[v]','-t',str(duration),'-c:v','libx264','-preset','fast','-crf','21','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'isaac-vs-gazebo.mp4')]
result=subprocess.run(command,capture_output=True,text=True)
assert result.returncode==0,result.stderr
decode=subprocess.run([ffmpeg,'-v','error','-i',str(OUT/'isaac-vs-gazebo.mp4'),'-f','null','-'],capture_output=True,text=True)
assert decode.returncode==0 and not decode.stderr
fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
for engine,record in records.items():
    data=np.genfromtxt(OUT/f'{record["run"]}.csv',delimiter=',',names=True);t=data['time_s']
    axes[0,0].plot(data['x_m'],data['y_m'],label=engine)
    axes[0,1].plot(t,data['z_m'],label=engine)
    axes[1,0].plot(t,data['estimated_height']-data['z_m'],label=engine)
    axes[1,1].plot(t,np.maximum(np.abs(data['roll_deg']),np.abs(data['pitch_deg'])),label=engine)
for ax in axes.flat:ax.grid(alpha=.2);ax.legend()
axes[0,0].set(title='Native trajectories',xlabel='x [m]',ylabel='y [m]');axes[0,0].axis('equal')
axes[0,1].set(title='Height',xlabel='simulated time [s]',ylabel='m')
axes[1,0].set(title='Height estimator error',xlabel='simulated time [s]',ylabel='m')
axes[1,1].set(title='Maximum absolute roll/pitch',xlabel='simulated time [s]',ylabel='degrees')
fig.suptitle('Isaac PhysX and Gazebo | same assumed airframe / C loops / C++ mission / DDS')
fig.savefig(OUT/'engine-comparison.png',dpi=170);fig.savefig(OUT/'engine-comparison.pdf');plt.close(fig)

rows=[]
for title,key,scale,unit in [
 ('Simulated mission','duration_s',1,'s'),('Landing XY error','final_xy_error_m',100,'cm'),
 ('Evaluated panel clearance','minimum_panel_clearance_m',1,'m'),('Wall time (excluding engine startup)','wall_s',1,'s')]:
    rows.append(f'| {title} | '+ ' | '.join(f'{records[e]["summary"][key]*scale:.3f} {unit}' for e in ['isaac','gazebo'])+' |')
for title,key,scale,unit in [('Cruise height RMSE','cruise_height_rmse_m',100,'cm'),('Maximum roll','maximum_abs_roll_deg',1,'deg'),('Maximum pitch','maximum_abs_pitch_deg',1,'deg')]:
    rows.append(f'| {title} | '+' | '.join(f'{records[e]["metrics"][key]*scale:.3f} {unit}' for e in ['isaac','gazebo'])+' |')
table='| Recorded quantity | Isaac Sim | Gazebo |\n|---|---:|---:|\n'+'\n'.join(rows)
readme='''# Native Isaac Sim and Gazebo evidence — 20 October 2026

[Synchronized video](isaac-vs-gazebo.mp4) · [Viewer](index.html) · [Comparison graphs](engine-comparison.png)

These are **actual native engine runs**, not PyBullet footage relabelled. Isaac
uses CPU PhysX and DirectX 12 ray-traced camera products. Gazebo 10.5 uses native
physics and OGRE2 camera sensors. Both execute the unchanged C flight equations,
shared C++ mission/navigation and actual ROS 2 DDS adapters, with GPU YOLO and
known-size planar PnP. Both complete takeoff through native ground contact and
virtual motor wind-down. Physical motor arming remains disabled.

'''+table+'''

Videos are aligned to **simulated time**, not simultaneous wall-clock starts.
The combined video holds the final frame of the shorter run. Individual videos
include original progressing graphs. Wall-time measurements include control,
DDS, inference, rendering and encoding, but exclude engine startup/training.
These two development runs are not a controlled rendering-performance benchmark.

## What is still simulated or unfinished

- The 1.2 kg mass, inertia, 6 N/motor thrust and 40 ms actuator lag are assumed.
  Isaac's internal mass and 2 ms step were queried; this is not airframe measurement.
- Local odometry is synthetic; real VIO/GPS localization is not implemented.
- The downward range is an analytic floor-only virtual ray, not a native lidar
  plugin or physical TOFSense driver. No obstacle depth or true map enters planning.
- Blue panels have known 1.09 x 1.69 m interiors. Both final runs map two tracks
  for two panels; this is not arbitrary unknown-object avoidance or real validation.
- Reported clearance is an evaluation-only conservative cylinder/box metric.
  Collision/landing flags come from native contact sensors/forces.
- The C core is a host DLL. RP2350 UF2 emulation is not in these loops. The ROS
  C++ wrappers remain unbuilt; actual DDS uses the documented Python adapters.
- The 47-page modelling PDF describes earlier experiments; this directory is the
  newer native-engine evidence. Hardware I/O, PCB completion and failsafes remain.

## Retained diagnostic runs

The first Gazebo trials never confirmed landing because the contact topic was
authored at the wrong SDF level. Isaac trials include an occluded camera, stale
camera transforms and a real panel collision. The MinimalRendering writeback
trial completed but had a 38 degree startup roll excursion and duplicate tracks;
it is not the selected final case. Stationary preflight, stronger contact solver
settings, native USD transform writeback, matte materials and ray-traced rendering
produce the selected run above. Failure logs/data are retained, not replaced.
The initial Isaac diagnostic marked aborted was stopped for its occluded view;
its inherited partial summary does not establish a valid perception result.

The raw console contains an inherited intermediate PyBullet method label from the
reused orchestration function. The final wrapper log line and summary correctly
identify the native engine. No PyBullet physics or rendering calls are executed.

## Reproduce

See ../../../sim/gazebo/README.md and ../../../sim/isaac/README.md. Individual
commands above were executed; tools/run_native_garden.ps1 collects those commands
as a convenience orchestrator (not separately executed end to end). Use unique
run names. It runs engines separately; the recordings can then be compared in
simulated time. Existing downloaded weights/SDKs/environments stay out of Git.

Each final run has its original CSV, summary, observations/events, derived graphs,
validation JSON, exact source snapshot ZIP and DDS transport JSON. Original logs
are under ../../test-runs/2026-10-20-engines and the Isaac retry subdirectories.
The SHA256 manifest excludes itself. Source hashes match the files at packaging.
'''
(OUT/'README.md').write_text(readme,encoding='utf-8')
html='''<!doctype html><html lang="en"><meta charset="utf-8"><title>IntelQCopter · Isaac + Gazebo</title>
<style>body{font:17px system-ui;background:#101722;color:#e6edf5;max-width:1500px;margin:30px auto;padding:20px}a{color:#6edee9}video,img{width:100%;border-radius:12px}section{background:#1b2635;padding:22px;margin:20px 0;border-radius:12px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:20px}table{border-collapse:collapse;width:100%}td,th{padding:12px;border-bottom:1px solid #405064;text-align:left}h1{font-size:38px}small{color:#adbdcc}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style>
<h1>IntelQCopter · two native engines</h1><p>Isaac Sim 6.2 / PhysX / ray-traced RGB · Gazebo 10.5 / OGRE2 RGB · actual C + C++ mission + ROS DDS + YOLO/PnP</p>
<video controls preload="metadata" src="isaac-vs-gazebo.mp4" poster="isaac-garden-raytraced-preview.png"></video>
<p><strong>Isaac left · Gazebo right.</strong> Aligned by simulated time. Shorter recording holds its final frame. These are separate completed runs, not guaranteed simultaneous wall-clock starts.</p>
<section><h2>Recorded results</h2><table><tr><th>Quantity</th><th>Isaac</th><th>Gazebo</th></tr>'''
for title,key,scale,unit in [('Mission time','duration_s',1,'s'),('Landing XY error','final_xy_error_m',100,'cm'),('Panel clearance','minimum_panel_clearance_m',1,'m'),('Wall time','wall_s',1,'s')]:
    html+='<tr><td>'+title+'</td>'+''.join(f'<td>{records[e]["summary"][key]*scale:.3f} {unit}</td>' for e in ['isaac','gazebo'])+'</tr>'
html+='''</table><p>Both: full phase sequence, no panel collision, native ground contact, virtual motors stopped, physical arming disabled. Complete video decoding and final source hashes checked.</p></section><img src="engine-comparison.png" alt="Data comparison from the two recorded native engine runs"><div class="grid">'''
for engine,name in NAMES.items():
    html+=f'''<section><h2>{engine.title()} original recording</h2><video controls preload="metadata" src="{name}.mp4" poster="{name}-preview.png"></video><p><a href="{name}.csv">CSV</a> · <a href="{name}-summary.json">Summary/source hashes</a> · <a href="{name}-metrics.json">Metrics</a> · <a href="{name}-validation.json">Validation</a> · <a href="{name}-transport.json">DDS measurements</a> · <a href="{name}-source-snapshot.zip">Exact source ZIP</a></p><img src="{name}-graphs.png" alt="{engine} recorded data graphs"></section>'''
html+='''</div><section><h2>Scope and original logs</h2><p>Assumed airframe; simulated localization; analytic downward floor ray. Known-size blue panel fixtures, not general unknown-object autonomy. Native contact gates landing. Host C DLL, not an emulated RP2350. ROS C++ wrappers remain unbuilt. Windows execution does not establish a hard real-time hardware deadline.</p>
<p><a href="../../test-runs/2026-10-20-engines/isaac-mission-raytraced.log">Isaac original flight log</a> · <a href="../../test-runs/2026-10-20-engines/gazebo-mission-ros.log">Gazebo original flight log</a> · <a href="README.md">Full scope and retained failures</a> · <a href="sha256-manifest.json">SHA256 manifest</a> · <a href="https://github.com/BenMohStem/IntelQCopter">GitHub repository</a></p></section></html>'''
(OUT/'index.html').write_text(html,encoding='utf-8')
(OUT/'comparison.json').write_text(json.dumps(records,indent=2))
validation={'both_native_runs_pass':True,'both_DDS':True,'combined_video_decode_exit':decode.returncode,
 'comparison_alignment':'simulated time; shorter run holds final frame','physical_arming':False}
(OUT/'comparison-validation.json').write_text(json.dumps(validation,indent=2))
files=[p for p in OUT.iterdir() if p.is_file() and p.name!='sha256-manifest.json']
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(OUT/'sha256-manifest.json').write_text(json.dumps(manifest,indent=2))
print(table);print('Combined video decoded successfully; both final source validations passed.')
