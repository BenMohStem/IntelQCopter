# Windows Jazzy, Gazebo and RViz

## What changed

The latest instruction asks to repair and use the user's Jazzy build. Therefore
`C:\dev\jazzy` and Gazebo are preserved. Isaac Sim's standalone application at
`C:\IsaacSim` was removed; `artifacts/setup/jazzy-repair/isaac-uninstall.json`
records the path and absence check. Source files and earlier test recordings stay
in version control. System CUDA and the separate TensorFlow setup were not changed.

The `.venv-isaacsim` directory is a historically named Torch/OpenCV/YOLO scientific
Python environment. It is not the Isaac application or its ROS runtime. Current
ROS uses Python 3.12 from `C:\dev\.pixi\envs\default`, while inference/video uses
the existing Python 3.10 environment. Keep these Python/DLL environments separate.

## Why the user's build looked unfinished

The original log is preserved at `artifacts/setup/jazzy-repair/original-user-build.log`.
It omitted its final summary because the summary plugin itself could not load.
The colcon event log showed failures even where the last terminal lines were green.

| Problem | Verified cause | Repair |
|---|---|---|
| `colcon_core.output_style` missing | colcon-output 0.2.15 requires core >=0.19; environment pinned core 0.17.1 | Pin output 0.2.13 in C:/dev/pixi.toml |
| gz-cmake/Ogre patches fail | Source patch files retained CRLF; downloaded vendor sources had LF | Back up and normalize only `.patch` files; record before/after SHA256 |
| gz-utils cannot find gz-cmake3 | Installed config under `opt/gz_cmake_vendor/share/cmake/gz-cmake3` was not discovered | Set exact package directory in the rebuild launcher |
| RViz packages skipped | Failed vendors and missing rendering helper dependencies | Rebuild full selected dependency chain |
| rosbag benchmark compiler C1083 | Object path exceeds Windows compiler's practical length limit | Build those two packages under short `C:/dev/b`, keep original install |
| Project cv_bridge absent | Core ROS source workspace did not include vision_opencv | Fetch official vision_opencv 4.1.0, the Jazzy release; build C++ cv_bridge |
| OpenCV version detection fails | OpenCV 4.9 wrapper does not recognize VS2026 runtime | Select installed `Library/cmake/x64/vc16/lib` config explicitly |
| ROS node callbacks fail compilation | rclcpp callback traits need concrete message/service types | Use explicit typed callbacks |
| Navigation library cannot link on Windows | Shared library lacked exported symbols/import library | Make internal navigation_core explicitly static |

No entire source/build/install directory was deleted. Patch backups are local
under `build/jazzy-patch-backup`; the committed normalization manifest is under
`artifacts/setup/jazzy-repair`. The benchmark messages' long-path warnings can
still appear; the short-directory build completed successfully.

## Start a recorded garden mission

Open PowerShell in the project directory:

```powershell
powershell -ExecutionPolicy Bypass -File tools/simulation/run_native_garden.ps1
```

This starts native Gazebo physics/RGB, native Jazzy DDS adapters, production C
flight loops, shared C++ mission/navigation, RGB YOLO, known-size panel PnP, MP4
recording and CSV/graphs. Each run gets a timestamp. Original stdout/stderr are
under `artifacts/test-runs/native-local`. Video, data, graphs, source snapshots and
validation are under `artifacts/simulation/2026-10-20-engines/<run>-gazebo*`.
The dated folder is historical naming; generated run names identify new tests.

The successful migrated run is `jazzy-migration-v4-gazebo`: all seven mission
phases recorded, native ground contact, virtual motors stopped, physical arming
zero, source hashes match, complete MP4 decode passes. Its 35.998 simulated seconds
took 60.619 wall seconds while builds ran concurrently. This is not proof of
500 Hz real-time scheduling on the RP2350 or desktop OS.

The garden is original procedural geometry resembling a campus garden, not an
accurate USTHB reconstruction. Blue panel interiors are known 1.09 x 1.69 m
fixtures. One downward analytic floor ray aids altitude; it is not a native
TOFSense driver or a full obstacle-depth sensor. Position is simulated odometry.

## Build and run the ROS C++ package

From PowerShell:

```powershell
& C:/WinLibs/pixi/pixi.exe run --manifest-path C:/dev/pixi.toml cmd /c tools\build\build_companion_windows.cmd
& C:/WinLibs/pixi/pixi.exe run --manifest-path C:/dev/pixi.toml cmd /c tools\ros2\ros2_windows.cmd python tools/validation/test_ros2_companion.py
```

The successful integration test used recorded native Gazebo images plus synthetic
pose, and exercised actual compiled ROS C++ executables over DDS. It proved
disabled-by-default mission rejection, fresh-pose mission acceptance, active
setpoints, YOLO detections and PnP output. It does not establish physical distance
accuracy or a complete closed-loop flight through the ROS C++ wrappers.

To launch these nodes manually with the existing exported ONNX:

```powershell
& C:/WinLibs/pixi/pixi.exe run --manifest-path C:/dev/pixi.toml cmd /c tools\ros2\ros2_windows.cmd ros2 launch qcopter_autonomy garden_mission.launch.py panel_model:=C:/Users/Mohamed/Desktop/Work/QCopter/build/panel-training/rgb-panel/weights/best.onnx known_fixture_model_confirmed:=true
```

This defaults to inactive missions. These nodes require synchronized `camera/image`
(rectified RGB/BGR), `camera/camera_info`, `odometry` and simulation contact topics.
The recorded flight adapter does not yet publish this entire standard-topic graph;
the independent integration harness supplies it. Do not mistake a launched node
with no sensor publishers for an operational autonomous system.

## View the F450 in RViz

Double-click `tools/simulation/view_f450_windows.cmd` or run it from a terminal. It starts
robot_state_publisher and RViz using `qcopter_description/launch/view.launch.py`.
The executed startup check initialized OpenGL 4.6, kept both processes running
and received all 14 fixed component transforms. Original logs are under
`artifacts/setup/jazzy-repair/rviz`; this is not a screenshot inspection claim.
The model has four orange motor bodies, two red/two white arms, 9047 propellers
and the provisional battery. Other electronics are excluded as requested.

RViz visualizes ROS data; Gazebo supplies the flight physics. The model viewer
alone does not simulate lift or execute flight control.

## Useful recording commands

Prefix ROS commands with the same Pixi/`tools\ros2\ros2_windows.cmd` wrapper. Once the
standard sensor graph is running, a separate terminal can record:

```text
ros2 bag record -o build/bags/garden-unique-name /camera/image /camera/camera_info /odometry /perception/semantics /perception/metric_obstacles /autonomy/setpoint /tf /tf_static
```

Stop recording with Ctrl+C so metadata is finalized. The existing mission MP4/CSV
are tested; this rosbag command is a usage recipe, not a claim that a hardware bag
or full standard-topic flight recording has already been produced.

## Upstream references

- [ROS source installation](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Alternatives/Windows-Development-Setup.rst)
- [Official Jazzy package release metadata](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml)
- [Official vision_opencv 4.1.0](https://github.com/ros-perception/vision_opencv/tree/4.1.0)
- [Gazebo native Windows installation](https://gazebosim.org/docs/latest/install_windows/)

The existing native Gazebo runtime is 10.5.0. Windows support is experimental.
This test uses an explicit adapter, not the official Jazzy/Harmonic ros_gz pairing.
