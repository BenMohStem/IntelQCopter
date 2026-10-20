# Native Gazebo garden

Gazebo 10.5.0 is installed with Pixi in `.pixi/envs/default` (ignored in Git).
Portable Pixi is at `C:\WinLibs\pixi\pixi.exe`; the committed lock file pins the
packages. This uses conda-forge packages through Pixi, without Anaconda or changes
to the TensorFlow environment. Windows support is experimental upstream:
https://gazebosim.org/docs/latest/install_windows/

The garden SDF has native rigid-body physics, collision geometry, OGRE2 forward
and overview cameras, and a native contact sensor. The original mission's C flight
loops, C++ mission/navigation, input/noise model and RGB YOLO/known-size PnP are
reused through a narrow engine adapter. No PyBullet physics or render calls run
in this adapter; importing its orchestration module prints PyBullet's import
banner, and the inherited intermediate summary label is corrected by the wrapper.
The final `*-summary.json` and last log line identify the actual engine.

## Assumptions and limits

- Airframe mass, inertia, thrust and motor lag are assumed. Native contact gates
  simulated landing; it is not a real landed detector or motor arming mechanism.
- Localization is simulated. One downward floor ray is calculated analytically,
  not provided by a native lidar plugin. No obstacle depth/true map enters planning.
- Blue panel interiors are known to be 1.09 x 1.69 m. This is not general unknown
  object ranging. Static panel clearance is an evaluation-only cylinder/box metric.
- The contact topic must be inside `<contact><topic>`, as required by the installed
  Contact system. Contact timestamps expire after 20 ms because Gazebo publishes
  positive contacts only. Failed earlier topic trials are retained.
- Full ROS C++ wrappers remain unbuilt. `--ros` tests the actual DDS Python
  adapters around the compiled production C core, with separate transport logs.

## Run

From the repository root, in separate terminals:

```powershell
& C:/WinLibs/pixi/pixi.exe install --manifest-path sim/gazebo/pixi.toml
& C:/WinLibs/pixi/pixi.exe run --manifest-path sim/gazebo/pixi.toml python -u sim/gazebo/engine_server.py
& .venv-isaacsim/Scripts/python.exe sim/gazebo/run_mission.py --name gazebo-local --video
```

For DDS, also start `C:/IsaacSim/app/kit/python/python.exe -u
sim/engines/ros_bridge.py --engine gazebo`, then add `--ros` to the mission command.
The bridge handles one run. Do not probe engine TCP ports with a generic socket
checker: their authenticated single-client listener treats it as a connection.
Wait for the `live engine ready` log message. Use unique run names to retain trials.

`probe.py` checks real gravity/contact and `camera_probe.py` saves actual sensor
images. Camera visibility uses a small body plus red/white arms; the larger
collision envelope stays invisible. Both engines derive geometry from garden.sdf.
