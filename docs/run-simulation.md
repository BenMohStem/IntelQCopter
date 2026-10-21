# Run the IntelQCopter simulation

This guide is for the **`esp32-xbox` branch on the configured Windows workstation**. The `main` branch contains the RP2350/RP2040 project.

The simulation runs **native Gazebo + the actual C flight-control core compiled as a Windows DLL**. Navigation adds the ROS2 C++ companion and YOLO/known-panel ranging. No ESP32 board or drone hardware is needed. Physical ESC outputs remain disabled. Gazebo does not emulate the ESP32 CPU; QEMU is a separate tool.

## 1. Quick start on Mohamed's configured PC

Connect and turn on the Xbox Series X/S controller, then open **PowerShell**:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-ESP32-Xbox
.\qcopter.ps1 xbox-check
.\qcopter.ps1 station
```

Open **[http://127.0.0.1:8088/](http://127.0.0.1:8088/)** in your browser. These addresses refer to your own PC and only work while its server is running.

1. Choose **FC only · real Xbox** for manual flight, or **FC + onboard ROS2 navigation** for both modes.
2. Choose **Garden 1–10**. Each scene has a different obstacle arrangement.
3. Click **Start simulation** once. A new flight tab opens automatically when the scene is ready. Startup can take up to 90 seconds.
4. Check the native camera frame counter, Xbox connection and readiness message.
5. Follow the manual-flight or navigation instructions below.

Run **one simulation at a time**. The workshop stays open between sessions. If the browser blocks a popup, use its launch message/live-station link after startup.

### After changing source code

End the running session first so Windows releases its loaded DLL/executables. Then rebuild only the changed components:

```powershell
.\qcopter.ps1 build-host       # C FC used by Gazebo
.\qcopter.ps1 build-companion  # ROS2 C++ nodes; needed for navigation changes
.\qcopter.ps1 station
```

`build` compiles hardware firmware; it does not rebuild the Gazebo host DLL. These commands do not flash hardware.

## 2. Fly with the Xbox

1. Select **Enable manual flight** in the flight tab. Wait for the five-second mode-selection progress bar. Alternatively, hold **D-pad Down + A for five seconds**.
2. Gently push the **left stick up** to start simulated thrust and climb.
3. Centre the left stick to hold height. Use small right-stick movements to tilt and travel.
4. Descend with the left stick down or LT. After landing, use **Disarm simulation**, or hold **D-pad Down + LT ≥90% for two seconds**.

| Input | Command |
|---|---|
| Left stick horizontal | Yaw rate |
| Left stick vertical | Vertical speed; centre holds height |
| Right stick horizontal/vertical | Roll/pitch angle |
| RT | Increase climb/collective allowance |
| LT | Reduce climb/collective allowance |
| Down + A, 5 seconds | Stable manual mode |
| Down + B, 5 seconds | Navigation mode when a companion is enabled |
| Down + LT, 2 seconds | Latched disarm; reset/restart before flying again |

**Gentle is the default:** squared stick response, up to ±5° stick-commanded tilt, ±0.5 m/s base vertical speed and ±45°/s yaw. Triggers retain full travel. Full RT permits at most **0.8 collective, equivalent to 1800 µs under the provisional 1000–2000 µs mapping**, leaving differential motor range for PID corrections. LT reduces the allowance. There is no physical ESC pulse driver.

A held tilt accelerates the drone; it does not command a fixed horizontal speed. On centring, the C FC briefly counter-tilts to brake, then levels and holds the release point using **explicit ideal simulated horizontal localization**. Braking takes time and can overshoot. The real IMU/barometer alone cannot reproduce this position hold. Braking corrections may exceed the Gentle stick limit, up to 7°.

The UI mode buttons send labelled Xbox-format simulation gestures through the C input parser. They require a connected controller. They do not bypass the FC with direct motor forces.

## 3. Choose start → any intermediate nodes → finish

Start the workshop session with **FC + onboard ROS2 navigation**. FC-only mode has no route planner or companion navigation.

1. In the flight tab, find **PC route planner**.
2. Click **Use drone as start**, then click the map to add nodes. You can drag, reorder or remove them; JSON save/load is available.
3. The **first node is start**, the **last is finish**, and every other node is visited in order at cruise height. Use two nodes for start/finish, three for start/middle/finish, or up to **64 nodes**.
4. Leave cruise COM height at **1.25 m** for your first run.
5. Click **Review route** and read the summary. Keep **Select navigation automatically in the simulator** enabled, or select it yourself with Down+B held for five seconds.
6. Click **Approve and fly** once. Watch mission phase and node progress.
7. After landing, reset or end the session before another flight. Selecting Stable cancels navigation.

The first node must match the stationary drone's actual ground position. To choose a different starting point, place the first node and use **Reset scene at first node** before approval. This saves the current trial and starts a new world there; it is not an in-flight teleport. Reset locations overlapping a panel are rejected.

Map coordinates are **local metres**, not GPS: x from −1 to 9 m and y from −3 to 3 m. Adjacent nodes must be at least 0.4 m apart. All node ground heights must match; cruise COM z must be 0.6–2.5 m and at least 0.4 m above start. Rejection messages explain invalid nodes, missing inputs, stale state or an already-active mission.

The PC sends the approved route through ROS2. The onboard C++ mission/planner uses camera observations and simulated odometry, sends CRC-protected UART-format commands, and the C FC computes motor demands. The navigator receives no hidden world obstacle list. Known-panel YOLO/PnP ranging and ideal localization remain simulation assumptions; this is not general RGB-only obstacle-distance measurement.

## 4. Cameras, live graphs and automatic recording

The flight tab offers Follow drone, Garden overview, Onboard RGB and composite/HUD views. Live graphs show angles, height, velocities, Kalman diagnostics, motor demands, collective headroom and ROS2 callback timings when available.

- Recording starts **automatically**, at **15 fps in simulation time**.
- Click **End session** to stop physics, finalize the recording and close the flight tab. The home workshop remains available.
- Closing the browser tab alone does **not** stop the simulator.
- A manually opened tab may need manual closure because of browser restrictions.
- Start again from the workshop whenever needed.

Terminal alternative for ending a ready session:

```powershell
.\qcopter.ps1 station-stop
```

Each launch/reset saves a distinct directory under `artifacts/gazebo-suite/<run-name>/`. The workshop's **Original recordings and data** library lists saved runs.

| File | What it contains |
|---|---|
| `gazebo.mp4` | Original recording |
| `flight.csv` | 500 Hz logical physics/control data |
| `controller-trace.jsonl` | C estimator, setpoint and diagnostic snapshots |
| `wire-inputs.jsonl` | ROS sessions: actual incoming HID/UART frames |
| `mapped-xbox.jsonl` | FC-only sessions: shaped Xbox reports |
| `ros2-profile.jsonl` | ROS sessions: callback profiling |
| `engine.log`, `engine-summary.json` | Engine, assumptions, contacts and source/binary hashes |
| `settings.json` | Immutable settings used for that run |
| `recording-complete.json` | Encoder completion result |

Files depend on session type. CPU inference, video frame rate, logical 500 Hz control and wall-clock simulation speed are different measurements. Slow rendering/inference can reduce the real-time factor.

**Latest evidence, while the workshop is running:** [videos, graphs and fault tests](http://127.0.0.1:8088/hardening-proof/) · [debugging guide](http://127.0.0.1:8088/debug-guide).

GitHub displays Markdown and images, but does not run the workshop or render its interactive HTML reports. Clone the repository and use the local server to play the reports. Original MP4/data files are also versioned under `artifacts/`.

## 5. Direct terminal launch — optional

Instead of the workshop's Start button:

```powershell
# Manual FC only; does not start ROS2 or YOLO
.\qcopter.ps1 play --env env01

# OR, after ending that session: FC + ROS2 companion
.\qcopter.ps1 play-ros --env env07
```

Open **[http://127.0.0.1:8087/](http://127.0.0.1:8087/)** after `LIVE VIEW READY`. Direct launches do not use the workshop's automatic new-tab launch workflow. The route editor is embedded in the flight tab; its standalone address is **http://127.0.0.1:8086/** during a ROS2 session. Do not start a second session to open the route editor.

## 6. Fresh clone / another PC

The preceding quick start assumes the installed dependencies and ignored local configuration already exist. **A fresh Git clone alone is not a ready simulation installation.** The current launcher scripts target Windows and some ROS2/Visual Studio paths on Mohamed's workstation.

```powershell
git clone --branch esp32-xbox --single-branch https://github.com/BenMohStem/IntelQCopter.git
cd IntelQCopter
```

Provide these prerequisites before running `station`:

| Requirement | Manual only | ROS2 navigation |
|---|---|---|
| Python with NumPy and OpenCV | Required | Required |
| Pixi + Gazebo 10.x/OGRE2 Python runtime | Required | Required |
| Zig C compiler and compiled `build/sitl/esp32_flight_core.dll` | Required | Required |
| FFmpeg executable | Required | Required |
| Xbox detected by Windows XInput | Required for interactive flight | Required for interactive mode selection/fresh pilot |
| Configured ROS2 Jazzy + `rmw_cyclonedds_cpp`, colcon, compiler and C++ OpenCV | No | Required |
| Compiled ROS2 overlay in `build/ros2-install/` | No | Required |
| Trained known-panel YOLO ONNX model | No | Required |

`simulation/gazebo/pixi.toml` declares the Gazebo environment. From the checkout, a local installation is created with:

```powershell
pixi install --manifest-path simulation/gazebo/pixi.toml
```

Set machine paths in **ignored `.local/config.json`**. The following is a template, not a copy of the configured PC's paths; replace all example locations. Forward slashes avoid JSON backslash escaping.

```json
{
  "python": "C:/Tools/qcopter-python/Scripts/python.exe",
  "pixi": "C:/WinLibs/pixi/pixi.exe",
  "gazebo_manifest": "simulation/gazebo/pixi.toml",
  "gazebo_python": "simulation/gazebo/.pixi/envs/default/python.exe",
  "zig": "C:/Tools/zig/zig.exe",
  "ffmpeg": "C:/Tools/ffmpeg/bin/ffmpeg.exe",
  "ros_manifest": "C:/dev/pixi.toml",
  "ros_python": "C:/dev/.pixi/envs/default/python.exe",
  "panel_model": "build/models/panels.onnx"
}
```

The Python selected by `python` needs NumPy/OpenCV; Matplotlib is additionally used by report generators. `python` and `zig` are executable paths, not directories. Do not replace an existing working `.local/config.json` with this template on Mohamed's configured PC.

For navigation, also adapt workstation-specific paths in `tools/ros2/ros2_windows.cmd` and `tools/build/build_companion_windows.cmd` if your Pixi/ROS2/Visual Studio installation differs. Local configuration does not override every path inside those wrappers. The ROS2 wrapper selects **Cyclone DDS**, domain 64, and the installed overlay. You do not need to activate a separate shell for normal project launch commands.

The trained panel ONNX file and installed SDKs/environments/build outputs are **not included in Git**. Obtain or reproduce the known-fixture model, then set `panel_model` to it. An arbitrary COCO detector is not a drop-in replacement. You can use manual-only simulation while ROS2/model setup is unfinished. TensorFlow and Isaac Sim are not needed for this simulation.

Then build the host FC and, if needed, the companion with the commands in section 1. Linux/Pi/Jetson deployment is separate from this Windows launcher setup.

## 7. Troubleshooting

| Symptom | What to check |
|---|---|
| `qcopter.ps1` is not found | Run from the checkout root; use `Get-Location`, `Test-Path .\qcopter.ps1`, and check `git branch --show-current` says `esp32-xbox`. The sibling `QCopter` folder is the Pico checkout. |
| Xbox disconnected | Connect by USB or pair to Windows, turn it on, and run `.\qcopter.ps1 xbox-check`. The interactive UI does not invent pilot freshness. |
| Drone sits on ground while time passes | Complete manual mode selection, wait for its gesture, then push left stick up. The default session does not take off automatically. Check readiness/sensor/pilot messages. |
| New tab missing | Allow the workshop popup or use its live-station link after readiness; do not click Start repeatedly. |
| No route planner | Start a **FC + onboard ROS2 navigation** session rather than FC only. |
| Route not accepted | Check review/rejection text, correct start location, fresh Xbox/FC/odometry, bounds and inactive previous mission. Use **Reset scene at first node** before approving a new start. |
| Camera unavailable or startup fails | Read `.local/station-manager-error.log`, `.local/<run>-launcher.log` and the run's `engine.log` / `live-boundary.log`. Verify configured Gazebo/Python/FFmpeg/model paths. |
| DLL/executable cannot be rebuilt | End the running session and wait for recording completion, then rebuild. Windows locks loaded binaries. |
| Port already in use | Keep only one simulation; ports 8086/8087 are session services, 8088 is the persistent workshop, and 9251 is the local plant boundary. |
| Disarm latched or crashed | Preserve the recording, then reset the scene or end/start another session. |
| GitHub HTML link shows source | Use the local workshop report links after cloning and launching the server. |

Keep failed recordings and logs. These tests use assumed mass/inertia/propulsion, synthetic sensors/calibration and ideal localization. Physical BLE/I²C/UART/ESC operation and the real ESP32's 500 Hz deadlines remain unvalidated. See [diagnostics and flight-readiness limits](debugging-and-flight-readiness.md).
