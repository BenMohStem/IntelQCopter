# Native Isaac Sim and Gazebo evidence — 20 October 2026

[Synchronized video](isaac-vs-gazebo.mp4) · [Viewer](index.html) · [Comparison graphs](engine-comparison.png)

These are **actual native engine runs**, not PyBullet footage relabelled. Isaac
uses CPU PhysX and DirectX 12 ray-traced camera products. Gazebo 10.5 uses native
physics and OGRE2 camera sensors. Both execute the unchanged C flight equations,
shared C++ mission/navigation and actual ROS 2 DDS adapters, with GPU YOLO and
known-size planar PnP. Both complete takeoff through native ground contact and
virtual motor wind-down. Physical motor arming remains disabled.

| Recorded quantity | Isaac Sim | Gazebo |
|---|---:|---:|
| Simulated mission | 36.698 s | 35.998 s |
| Landing XY error | 3.231 cm | 2.117 cm |
| Evaluated panel clearance | 0.682 m | 0.560 m |
| Wall time (excluding engine startup) | 139.912 s | 36.100 s |
| Cruise height RMSE | 1.977 cm | 1.965 cm |
| Maximum roll | 3.117 deg | 2.890 deg |
| Maximum pitch | 4.767 deg | 4.488 deg |

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
