# RGB garden experiment

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
