# ESP32/Xbox + ROS2 + native Gazebo

## What actually runs

| Role | Runtime | Allowed flight inputs |
|---|---|---|
| Ground PC / virtual Xbox pilot | Python ROS2 process | Raw 16-byte Xbox reports; one start/finish ROS2 service request in navigation cases |
| Drone computer | ROS2 C++ processes | RGB images, camera calibration, simulated local odometry, FC status; no world obstacle list |
| Flight controller | Active `flight_controller.c` built as a host DLL | Raw HID and CRC-protected UART frames; synthetic IMU/pressure samples |
| Aircraft/world | Native Gazebo 10.5, OGRE2 | Motor demands from that C controller, first-order motor lag and rotor forces |

Gazebo is **not** running the Xtensa CPU. [QEMU replay](../emulation/README.md)
separately checks the same C source on an ESP32-S3 CPU model. Neither path tests
physical BLE, I²C, UART electrical operation, ESC timing or actual aircraft flight.
Physical motor outputs remain disabled. Gazebo has an explicit simulation-only
motor gate following a completed mode gesture and valid pilot/sensor input.

## Run on this workstation

Open PowerShell in the ESP32 checkout, **not the Pico checkout**:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-ESP32-Xbox
.\qcopter.ps1 build
.\qcopter.ps1 build-host
.\qcopter.ps1 build-companion
.\qcopter.ps1 gazebo-suite --prefix my-new-run --only all
```

Choose a new prefix each time. Existing original results are never overwritten.
`all` performs **five manual flights + ten navigation flights**. Individual runs:

```powershell
.\qcopter.ps1 gazebo-suite --prefix my-manual --only manual --env env01
.\qcopter.ps1 gazebo-suite --prefix my-navigation --only navigation --env env06
```

The console prints progress. Each run writes `gazebo.mp4`, `flight.csv`,
`xbox-reports.csv`, process logs, exact `world.sdf`, engine parameters/results
and the PC's accepted/rejected endpoint request. Current navigation recordings
also retain `perception.jsonl` (YOLO boxes and PnP observations) and
`wire-inputs.jsonl` (the actual HID/UART bytes entering the plant boundary).

View the MP4 while the test is complete; the original 5 fps recording contains
native overview/RGB views, mode, input counters and mission status. It is played
at simulation time. Execution may take longer than real time.

If ROS2 is temporarily unavailable while you configure it, continue independently:

```powershell
.\qcopter.ps1 build
.\qcopter.ps1 emulator-build
.\qcopter.ps1 emulator-run --name my-new-cpu-check
```

These commands do not install, repair or change the base ROS2 installation.

## Environments

`environments.json` defines ten deterministic authored scenes: campus garden,
courtyard, warehouse, hangar, orchard, parking, construction, sports court,
laboratory and woodland. Their panel positions/orientations differ. Scene decor
is outside the flight corridor. They are illustrative test fixtures, **not scans
or accurate reconstructions of USTHB**. Navigation does not receive these files.

Manual cases use feedback-based **virtual humans**, not recordings of real human
flights. The virtual pilot reads the simulator's pose/velocity HUD and emits
only Xbox-format stick/trigger reports at the simulated clock rate. It climbs,
moves/yaws through waypoints, returns or moves to a second landing point, lowers
height using LT while correcting drift with the right stick, and holds Down+LT
after ground contact. No test script calls an FC setpoint or motor API.

Navigation cases hold Down+B for five seconds, send neutral Xbox reports to keep
the pilot link fresh, then the ground PC calls `/drone/mission/request` with only
start/finish/cruise height. The C++ drone mission computer takes off, plans/replans
from camera observations, travels and lands. The simulated Xbox disarms after
the drone computer reports landing complete.

## PC endpoint selection

```powershell
.\qcopter.ps1 pc-map
```

Open **http://127.0.0.1:8086**. Click the current start position, then the finish;
set ground z/cruise height and send. This calls the same ROS2 service used by the
tests. Do not send another mission while an automated suite owns that aircraft.
The purple marker is the live pose received from the drone over DDS.

Coordinates are metres in a shared local `map` frame. This is a local grid,
not GPS/geographic navigation or automatic map alignment. The drone rejects a
start more than 25 cm from its current horizontal position and requires fresh
Xbox navigation mode. An unavailable service returns an explanation.

For separate machines, use matching `ROS_DOMAIN_ID` (this workstation uses 64)
and compatible DDS discovery/network access. The PC publishes requests; only the
drone companion produces flight setpoints. Cross-machine discovery, a real
localization source and hardware UART have **not** been tested here. Simulation
currently uses an explicit `map` pose/world-velocity contract (`map` child frame);
adapt a standard body-frame odometry source before physical integration.

## Perception and model limits

- Single RGB only: trained YOLO panel detector → visible blue-panel corners →
  known-size PnP with a known flat-floor centre-height check → observed obstacle map → inflated-grid A*. No simulated depth,
  object IDs or hidden obstacle coordinates are sent to navigation.
- The inner blue panel is known to be 1.09 × 1.69 m and its map centre must be 0.90 ±0.20 m above the flat floor. This does **not** establish
  distance to arbitrary trees, people, wires or unknown objects. A single fixed
  1D lidar does not resolve that limitation. Lidar is not part of these trials.
- Localization is simulated Gazebo odometry, not camera VIO/GPS/SLAM.
- Assumptions in `simulation/model/parameters.json`: 1.2 kg, inertia
  (0.012, 0.012, 0.022) kg·m², 450 mm motor diagonal, 6 N/motor, 40 ms motor lag.
  These are **unmeasured**. Sensors use independent synthetic noise, not measured
  vibration/temperature/latency distributions. The aircraft collision shape is
  a 0.34 m radius cylinder, not a validated flexible F450 structure.
- Success means completion, native ground contact/disarm, zero native panel
  contact, horizontal endpoint error <20 cm and the appropriate command path.
  It does not establish real-world safety or statistical reliability.
- Every original failure is retained. A Gazebo descriptor-set warning can appear
  before readiness; successful physics/RGB operation is checked independently.

## Local dependencies and another checkout

Machine locations belong in ignored `.local/config.json`: `gazebo_manifest`,
`gazebo_python`, `ros_manifest`, `ros_python`, `panel_model`, `ffmpeg`, `zig`.
This workstation reuses the installed Gazebo/vision runtimes from the sibling
Pico checkout; no Isaac engine is launched. A vision environment still has a
historical name containing `isaacsim`; its name does not identify the simulator.

`pixi.toml` specifies the Windows Gazebo dependencies. Default paths expect its
local `.pixi` environment, C:/dev ROS2, an FFmpeg executable on PATH and the panel
ONNX model at `build/models/panels.onnx`. The trained model is an external local
dependency; a fresh clone must provide it. A general COCO model is not a drop-in
replacement for this fixture detector. The original training recipe is retained
on main at `sim/garden/train_panels.py`. Windows launcher scripts target this
workstation; the C++ package builds normally with `colcon` on a configured ROS2
Linux companion. No Pi/Jetson deployment or YOLO acceleration is claimed.
