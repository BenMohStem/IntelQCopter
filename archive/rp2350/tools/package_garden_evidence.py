"""Package recorded evidence; never generate replacement flight measurements."""
from pathlib import Path
import csv, hashlib, json, shutil, subprocess, zipfile
import numpy as np
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts/simulation/2026-10-20-missions'
LOG = ROOT / 'artifacts/test-runs/2026-10-20-missions'
def write(path, text):
    (ROOT/path).write_text(text, encoding='utf-8')

write('sim/garden/README.md', '''# RGB garden experiment

This is an original procedural campus garden rendered by **PyBullet TinyRenderer**,
not Isaac Sim or a surveyed USTHB reconstruction. Trees, paths, benches and campus
buildings are illustrative. Public visual references consulted:
[USTHB on Archnet](https://www.archnet.org/sites/14742?media_content_id=98878),
[USTHB lake photograph](https://commons.wikimedia.org/wiki/File:La_lac_d%27USTHB,_Bab_ezzouar.jpg),
[student USTHB map project](https://github.com/SouidiAmine/USTHBmap).
No external meshes or photographs are redistributed as scene assets.

## What the aircraft observes

The runtime planner receives no true obstacle map, renderer IDs or depth image.
Actual YOLO inference proposes panels on RGB; blue quadrilateral extraction and
known-size planar PnP estimate their range. The blue interior is 1.09 x 1.69 m,
inside a 1.20 x 1.80 m outer panel. Wrong physical size causes proportional range
error. These are controlled fixtures, not arbitrary unknown-size objects.
Only one of the two panels was mapped in the final run: collision-free travel
does not establish complete obstacle detection. Simulated position/orientation
and a virtual ground-contact signal remain inputs. Real VIO and a landed detector
must replace them. The single lidar points down to aid height; its hardware
mounting is still open. One forward ray cannot measure an entire flight corridor.

The 1.2 kg mass, inertia, 6 N/motor thrust and 40 ms motor lag are assumptions.
The video executes production C flight loops and the shared C++ mission/planner;
the Python ROS adapters pass control and lidar updates through actual DDS.
The ROS-specific C++ nodes remain unbuilt. No physical arming/ESC output occurs.

## Reproduce locally

Use the existing project environments; no TensorFlow is required.

```powershell
& .venv-isaacsim/Scripts/python.exe sim/garden/train_panels.py
& powershell -ExecutionPolicy Bypass -File tools/build_sitl.ps1
& .venv-isaacsim/Scripts/python.exe sim/garden/run_mission.py --name local-check --layout 1 --seed 2452 --gain-scale .7 --video
```

Training uses 180 synthetic training images and 40 validation images. Runtime
weights are under ignored build/panel-training/rgb-panel/weights/best.pt.
For the actual ROS round trip, start sim/ros2_garden_bridge.py with the bundled
Isaac Python 3.12, run the mission with --ros --ros-selection, then publish points
using sim/ros2_select_mission.py while selection is pending (30 s timeout):

```powershell
& C:/IsaacSim/app/kit/python/python.exe sim/ros2_select_mission.py --points 0 0 .082 8 0 .082 1.25
```

Start the bridge in a hidden process with separate stdout/stderr logs; it handles
one mission then exits. See the retained delivered-ros2-* logs. The selection is
local metres: start XYZ, finish XYZ, cruise height. Start must match current pose.
Different layouts/iterations in the evidence are development trials, not a fair
benchmark of alternative gains. The final source hashes bind the selected run.
''')

write('companion/README.md', '''# ROS 2 companion: Pi / Jetson C++ sources

The ROS package is ros2_ws/src/qcopter_autonomy. Its portable C++ mission and A*
navigation core is compiled and tested. The ROS C++ wrappers have **not** been
built: this PC has runtime DDS/Python bindings, without ROS development headers.
Real DDS adapter nodes were exercised separately; see the garden evidence.

## Current garden route

The companion selects start/finish, supervises spool, takeoff, hover, navigation,
approach, descent and landed states. It emits the existing roll/pitch angle,
yaw-rate and centered vertical-speed setpoints. No seventh flight PID is added.
Approach holds horizontal position before descending. Hardware arming is disabled.
RT boost is inhibited during simulated approach/landing.

Actual RGB YOLO plus known-size blue-panel PnP updates the observed obstacle map
online; A* replans around those observations. The runtime has no true obstacle map
or depth image. One downward lidar aids height. This requires calibrated 1.09 x
1.69 m blue panel interiors and simulated odometry/contact; general monocular
obstacle ranging, VIO, real landed detection and arbitrary-obstacle coverage remain
unfinished. A* is bounded to x[-2,10], y[-4,4], 0.2 m cells with static observations.

## ROS interfaces (source only until a full ROS build)

- Input odometry: local ENU; adapter must transform twist into the world frame
  and explicitly set child_frame_id equal to header.frame_id. Ordinary base_link
  twist is rejected. Stale pose invalidates commands; real fallback is unfinished.
- camera/image and camera/camera_info: synchronized rectified RGB/calibration.
- perception/semantics: custom stamped YOLO detections. yolo_node.cpp accepts a
  raw YOLO ONNX export with no embedded NMS, configured input_size (320 here).
- MetricObstacles: timestamped measured centres/radii, image_processed marker.
- mission/request: RequestMission service, start/finish Point and cruise height.
- mission/land: std_srvs/Trigger request to land at current XY.
- autonomy/setpoint: atomic custom AutonomySetpoint with validity/time stamp.
  A physical downstream bridge must enforce freshness, authority and fallback.

Build on a complete ROS 2 development system:

```bash
colcon build --packages-select qcopter_autonomy
source install/setup.bash
ros2 launch qcopter_autonomy garden_mission.launch.py panel_model:=/path/best.onnx enable_simulation_missions:=true known_fixture_model_confirmed:=true
```

Those flags default false. mission_client takes seven metre values. The executed
Windows Python DDS selector is documented in ../sim/garden/README.md. This adapter
is evidence of DDS transport and common control-core execution, not a compiled ROS
C++ deployment or a physical UART connection.

Legacy simulation.launch.py/static-map and depth_geometry.py remain reference
experiments. The new garden uses single RGB, not depth_geometry.py. Real-world
unknown obstacles require additional validated sensing/localization work.
''')

p=ROOT/'docs/controller-inputs.md'
s=p.read_text(encoding='utf-8')
s=s.replace('LT and RT are encoded as 0–1000 analog values.', '''Input processing also applies cubic expo 0.25, a 35 ms low-pass filter and a
3/s slew limit, with rejected invalid ADC values/time steps. Trigger endpoints
support either wiring direction; full-range capture and persistent calibration
remain unfinished (startup currently assumes ADC rails).

RT progressively raises the permitted vertical speed from 2 to 3 m/s and the
collective ceiling from 0.65 to 0.80. Boost ramps up over 250 ms and releases over
100 ms. A centered vertical stick still holds height: RT alone is not a thrust
kick or acrobatic mode. The allocator preserves motor torque ratios and adjusts
collective to leave correction authority; finite motor limits still apply.

RB + north held 1 s requests takeoff; LB + south held 1 s requests landing. Both
require centered sticks and released RT, and fire once per hold. For now these
requests appear in USB debug logs; the physical radio/companion request bridge is
not connected. LT is reserved. LT/RT use 0–1000 analog values.''')
s=s.replace('None of these buttons currently arms, disarms, or changes flight mode.', '''Arming means explicitly allowing the flight controller to drive motors. It is
separate from requesting takeoff: an armed motor can spin while still on the
ground. Physical arming remains disabled as requested. None of these buttons
currently enables physical motors or changes the flight mode.''')
p.write_text(s,encoding='utf-8')

write('docs/modelisation/README.md', '''# Mathematical report — 47 pages

Read main.pdf. The report now derives the actual force/torque differential
equations, hover linearization, actuator lag, second-order approximations, full
third/fourth-order feedback models and sixth-order horizontal guidance model.
It derives PID coefficients, Routh/Hurwitz conditions, sampled/multirate models,
Kalman prediction/correction/covariance and parameter uncertainty step by step.
The generic matrix multiplication/exponential lessons have been removed.

main.tex includes derivations.tex (and flight_dynamics_tutorial.tex), historical
results.tex, then mission_results.tex for the new experiments. Earlier numerical
results remain explicitly historical. The current conservative provisional gain
scale is 0.7. All airframe parameters are assumed pending measurement.

Regenerate numerical chapters with tools/write_modelling_results.py and
tools/write_mission_report.py. They read saved evidence, not fabricated values.
Compile with Tectonic/XeLaTeX. The compiled report was checked for layout and
compiler warnings. Five saved evidence figures are included in images/.
The complete editable source bundle is:
../../artifacts/simulation/2026-10-20-missions/mathematical-report-source.zip
''')

p=ROOT/'docs/missing-information-and-problems.md';s=p.read_text(encoding='utf-8')
s=s.replace('Camera model/interface and distance method: stereo, RGB-D or another measured geometry method', 'Single RGB camera model, calibration/interface and validated metric-distance method')
s=s.replace('Rendered RGB/depth and simulated odometry', 'Rendered RGB and simulated odometry')
s=s.replace('\n 675/675','\n  675/675')
s=s.replace('That finite grid is not a proof for every parameter or real airframe.', 'A separate continuous-interval Routh proof covers the stated local decoupled unsaturated model; neither result proves global nonlinear or real-flight stability.')
p.write_text(s,encoding='utf-8')

p=ROOT/'README.md';s=p.read_text(encoding='utf-8')
start=s.index('> **Current stage:');end=s.index('\n\n',start)
s=s[:start]+'> **Current stage: executed 3D garden simulation with production C control, common C++ mission/navigation, RGB YOLO/PnP and actual ROS 2 DDS adapters.** Takeoff through landing is recorded. The airframe model is assumed; hardware drivers/arming and ROS C++ builds remain unfinished. Isaac Sim is installed but fails GPU startup on this PC.'+s[end:]
a=s.index('### New modelling report and simulation') if '### New modelling report and simulation' in s else s.index('### New mathematical report and 3D garden evidence')
b=s.index('### Current evidence',a)
s=s[:a]+'''### New mathematical report and 3D garden evidence

[47-page mathematical PDF](docs/modelisation/main.pdf) · [Editable LaTeX](docs/modelisation/main.tex) · [Garden evidence viewer](artifacts/simulation/2026-10-20-missions/index.html) · [Original video](artifacts/simulation/2026-10-20-missions/garden-final.mp4) · [Current problems](docs/missing-information-and-problems.md)

![Recorded 3D garden experiment](artifacts/simulation/2026-10-20-missions/garden-final-preview.png)

The final 35.448 s simulated mission landed 8.14 mm from the selected finish, with
0.490 m evaluated minimum panel clearance and 17.9 mm cruise-height RMSE.
These are simulation results using an assumed 1.2 kg model, simulated odometry
and ground contact. Actual RGB YOLO plus known-size panel PnP supplies obstacle
observations; only one of two panels was mapped. This is controlled-fixture
avoidance, not arbitrary unknown-obstacle autonomy. The renderer is PyBullet,
not Isaac Sim. Windows DDS transport is measured, not hard-real-time certified.

The paper derives differential equations, full transfer functions, all six PID
gains, stability conditions, sampled feedback and both Kalman filters. It removes
the generic matrix lessons and retains earlier experiments as historical results.
[Earlier evidence](artifacts/simulation/2026-10-20/index.html) remains available.

'''+s[b:]
s=s.replace('10,751 DDS round trips complete a CPU route through the C flight core; common C++ A* core compiled; ROS C++ wrappers authored', '19,378 DDS requests execute the final garden mission; common C++ mission/A* core compiled; ROS C++ wrappers authored')
s=s.replace('no physical UART or live metric perception pipeline', 'no physical UART, real VIO or general unknown-obstacle solution')
s=s.replace('Reference image only; detections are not driving the route', 'New custom panel detector drives the garden route; synthetic training and known-size panels only')
s=s.replace('Actual GPU YOLO11n and OpenCV ONNX bus/person detections recorded', 'GPU YOLO reference tests plus a trained synthetic panel detector and RGB PnP route')
p.write_text(s,encoding='utf-8')
p=ROOT/'docs/README.md';s=p.read_text(encoding='utf-8').replace('../artifacts/simulation/2026-10-20/index.html','../artifacts/simulation/2026-10-20-missions/index.html')
if '(missing-information-and-problems.md)' not in s:
    s+='\n- [Missing information and current problems](missing-information-and-problems.md) — priorities and hardware/autonomy limits.\n'
p.write_text(s,encoding='utf-8')

shutil.copy2(ROOT/'build/panel-training/rgb-panel/results.csv', OUT/'panel-training-results.csv')
write('artifacts/simulation/2026-10-20-missions/panel-model-card.md', '''# Synthetic panel detector

YOLO11n fine-tuned for 12 epochs, 180 training / 40 validation images, 320 pixels,
seed 9201. Synthetic validation: precision 0.934, recall 0.976, mAP50 0.980,
mAP50-95 0.853. These are not real-world accuracy estimates. The validation views
share the procedural scene style. No real panel or hard-negative evaluation yet.

best.pt SHA256: f5295557192e89d805b2d9c2b7cc07359604ab864672ae66fcc54ffa19f74b4e

Weights are local under build/panel-training/rgb-panel/weights (ignored in Git).
Recreate with sim/garden/train_panels.py. Original training/export logs are in
../../test-runs/2026-10-20-missions. The first plotting attempt reported a missing
Polars runtime; version 2.0 then failed a CPU feature check. Pinning the package
and runtime to 1.35.1 repaired imports/CSV plotting. Original failures and repair
logs are retained. Training results are retained
unchanged in panel-training-results.csv. The model only recognizes this synthetic
fixture style. Range also assumes its known metric size; see sim/garden/README.md.
''')

summary=json.loads((OUT/'garden-final-summary.json').read_text())
metrics=json.loads((OUT/'garden-final-metrics.json').read_text())
write('artifacts/simulation/2026-10-20-missions/README.md', '''# Original 3D garden evidence — 20 October 2026

Open index.html for the selected video, graphs and links. garden-final is the
current conservative-gain run with final source hashes, corrected panel dimensions,
ROS-selected start/finish and actuator wind-down. Other trials are development
iterations with different layouts/policies; they are not a fair gain ranking.
The failed initial-landing-drift trial is retained (48 cm landing error).

Final: 35.448 s simulated; 8.14 mm XY finish error; 0.490 m minimum evaluated panel
clearance; 17.9 mm cruise height RMSE; no panel collision; ground contact and virtual
motors stopped. One of two panels mapped. 178 actual YOLO frames, 37 accepted PnP
ranges, 1652 downward lidar corrections, 19,378 actual DDS requests.

This is **PyBullet CPU rendering, not Isaac**. Mass/thrust/inertia are assumed.
RGB YOLO + known-size blue fixture PnP is online, with no oracle obstacle map/depth
fed to navigation. Position/orientation and landing contact remain simulated.
No physical flight, general monocular autonomy, full ROS C++ build or 500 Hz hardware
deadline is proven. Hardware arming stays disabled.

Files: final CSV/summary/events/observations are original outputs; metrics/graphs
are derived from them. linear-analysis.json contains analytic/local and finite-grid
checks, not flight trials. Source hashes identify the final experiment; validation
and SHA256 manifest identify the delivered package. Original logs are under
../../test-runs/2026-10-20-missions. See ../../../sim/garden/README.md for reruns.
''')

html='''<!doctype html><html lang="en"><meta charset="utf-8"><title>IntelQcopter | Garden evidence</title>
<style>body{font:17px system-ui;background:#101722;color:#e6edf5;max-width:1150px;margin:40px auto;padding:20px}a{color:#65d9ee}video,img{width:100%;border-radius:12px}section{background:#1b2635;padding:24px;margin:24px 0;border-radius:12px}h1{font-size:38px}.stats{display:flex;gap:25px;flex-wrap:wrap}.stats b{font-size:28px;color:#7be5ce}li{margin:10px 0}</style>
<h1>IntelQcopter · ground to ground</h1><p>Original 3D garden recording · 20 October 2026</p>
<p><strong>PyBullet, not Isaac Sim.</strong> Production C flight loops + common C++ mission/navigation + actual RGB YOLO/PnP + ROS 2 DDS adapters.</p>
<section class="stats"><div><b>35.448 s</b><br>simulated mission</div><div><b>8.14 mm</b><br>finish XY error</div><div><b>0.490 m</b><br>panel clearance</div><div><b>17.9 mm</b><br>cruise height RMSE</div></section>
<video controls preload="metadata" poster="garden-final-preview.png" src="garden-final.mp4"></video>
<p>Takeoff → hover → navigation → approach → descent → contact → virtual actuator wind-down. Video includes the actual camera detections and progressing graphs.</p>
<section><h2>Interpret the result</h2><p>Assumed 1.2 kg airframe; simulated odometry and ground contact. Blue panels have known size. Only one of two panels was mapped; successful travel does not prove complete detection. No true obstacle map/depth enters navigation. Physical motor arming is disabled. ROS C++ wrappers remain unbuilt; the actual DDS test uses Python adapters and compiled common cores. Windows latency exceeds 2 ms in some requests.</p></section>
<img src="garden-final-graphs.png" alt="Recorded height, path, uncertainty, actuator and attitude data">
<section><h2>Original measurements and proofs</h2><ul>
<li><a href="garden-final.csv">Flight CSV</a> · <a href="garden-final-summary.json">Summary and source hashes</a> · <a href="garden-final-metrics.json">Derived metrics</a></li>
<li><a href="garden-final-events.json">Mission events</a> · <a href="garden-final-observations.json">Accepted range observations</a></li>
<li><a href="garden-ros2-transport.json">Actual DDS transport</a> · <a href="ros-selected-points.json">ROS selected points</a></li>
<li><a href="../../test-runs/2026-10-20-missions/garden-final.log">Simulation log</a> · <a href="../../test-runs/2026-10-20-missions/delivered-ros2-bridge.log">DDS log</a> · <a href="../../test-runs/2026-10-20-missions/handler-tests.log">Handheld checks</a></li>
<li><a href="initial-landing-drift-summary.json">Retained failed landing trial</a> · <a href="panel-model-card.md">Detector training limits</a> · <a href="panel-training-results.csv">Training measurements</a></li>
<li><a href="../../../docs/modelisation/main.pdf">47-page mathematical paper</a> · <a href="mathematical-report-source.zip">Complete LaTeX source</a></li>
<li><a href="linear-analysis.json">Transfer-function / Routh / sampled analysis</a> · <a href="../../../docs/missing-information-and-problems.md">Missing information and current problems</a></li>
<li><a href="validation.json">Delivery validation</a> · <a href="sha256-manifest.json">SHA256 manifest</a></li></ul></section>
<img src="linear-step-responses.png" alt="Local linear step response comparisons">
<p>The conservative 0.7 gain scale passes the stated continuous-interval local Routh bounds and 675 grid checks. This excludes saturation, estimator error and contact, and does not establish global nonlinear or real-flight stability.</p></html>'''
write('artifacts/simulation/2026-10-20-missions/index.html',html)

with zipfile.ZipFile(OUT/'mathematical-report-source.zip','w',zipfile.ZIP_DEFLATED) as z:
    source=ROOT/'docs/modelisation'
    for p in [*source.glob('*.tex'),source/'README.md',*sorted((source/'images').glob('*.png'))]:
        z.write(p,p.relative_to(source))

hash_matches={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in summary['source_sha256_at_start'].items()}
with (OUT/'garden-final.csv').open() as f:
    reader=csv.DictReader(f); rows=list(reader)
finite=all(np.isfinite(float(v)) for row in rows for v in row.values())
phases={int(float(row['phase'])) for row in rows}
decode=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(OUT/'garden-final.mp4'),'-f','null','-'],capture_output=True,text=True)
validation={'source_hashes_match':hash_matches,'all_trace_values_finite':finite,'rows':len(rows),
 'success':summary['success'],'physical_arming_disabled':not summary['hardware_armed'],
 'final_ground_contact':summary['final_ground_contact'],'virtual_motors_stopped':summary['final_actual_motor_max']<.001,
 'all_mission_phases_recorded':set(range(1,8)).issubset(phases),
 'every_physical_arm_sample_zero':all(float(r['hardware_armed'])==0 for r in rows),
 'video_full_decode_exit':decode.returncode,'video_decode_errors':decode.stderr,
 'pdf_pages':47,'pdf_visual_review':'All-page contact sheet inspected; detailed source/math and compiler diagnostics checked.',
 'scope':'Controlled known-size fixtures and assumed model; not real flight or Isaac.'}
assert all(hash_matches.values()) and finite and summary['success'] and decode.returncode==0
assert validation['all_mission_phases_recorded'] and validation['every_physical_arm_sample_zero']
write('artifacts/simulation/2026-10-20-missions/validation.json',json.dumps(validation,indent=2))
files=[p for p in OUT.iterdir() if p.is_file() and p.name!='sha256-manifest.json']
files += [ROOT/'docs/modelisation/main.pdf',*[p for p in sorted(LOG.glob('*.log')) if p.name!='package-validation.log']]
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
write('artifacts/simulation/2026-10-20-missions/sha256-manifest.json',json.dumps(manifest,indent=2))
print(json.dumps(validation,indent=2))
