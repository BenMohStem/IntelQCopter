# Flight Station: Xbox, missions and recordings

Worktree: **esp32-xbox**, `C:\Users\Mohamed\Desktop\Work\QCopter-ESP32-Xbox`. Main retains RP2350/RP2040. Physical motor outputs remain disabled.

## Launch

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-ESP32-Xbox
.\qcopter.ps1 xbox-check
.\qcopter.ps1 play-ros
```

Open **http://127.0.0.1:8087**. This single page contains the flight controls, native cameras, Xbox input monitor and route editor. It activates the existing Pixi/Jazzy overlay with **rmw_cyclonedds_cpp**, domain 64 and simulation time. No base ROS2 installation changes are needed.

For FC-only operation, end the existing session and launch:

```powershell
.\qcopter.ps1 play
```

This starts no ROS2 companion, YOLO or navigation process. The route editor is hidden and navigation selection is rejected. Run only one session at a time.

## Fly manually

1. Connect and turn on your Xbox. The input monitor shows connection, stick positions, triggers and button bits.
2. Click **Enable manual flight**. Wait for the five-second gesture progress bar to finish.
3. Gently push the **left stick up** to start simulated thrust and climb. Centre it to hold height.
4. The right stick requests roll/pitch **angles**; left horizontal requests yaw **rate**.
5. Descend with left stick down or LT. After touching down, click **Disarm simulation** or hold **D-pad Down + LT ≥90% for two seconds**.

The simulator mode buttons encode ordinary Xbox HID gestures into the C core. They require a connected controller and are logged as **explicit PC simulator actions**, separately from actual XInput reports. They do not implement physical arming. Xbox Down+A/Down+B held five seconds still selects stable/navigation directly. Actual Xbox disarm input takes priority over a PC mode gesture.

### Gentle controls

The new default **Gentle** profile applies only to simulator input:

| Command | Gentle | Normal |
|---|---:|---:|
| Maximum roll/pitch angle | 5° | 35° |
| Stick vertical-speed request | ±0.5 m/s | ±2 m/s |
| Stick yaw-rate request | ±45°/s | ±180°/s |
| Trigger scaling | 20% | 100% |
| Stick response after 12% deadband | Squared/exponential | Linear |

Half of the active roll/pitch stick travel requests approximately 1.25°, rather than 2.5°. Full Down+LT disarm remains available. Hardware firmware limits and PID gains are unchanged; the physical BLE target does not automatically inherit this Windows simulator profile.

**Why does angle control accelerate the drone?** Near a steady tilted hover, neglecting drag:

\[
T\cos\theta\approx mg,\qquad m\ddot x\approx T\sin\theta
\quad\Longrightarrow\quad \ddot x\approx g\tan\theta.
\]

A held 35° tilt corresponds to about 6.87 m/s² horizontal acceleration; 5° corresponds to about 0.86 m/s² under those assumptions. Centring the stick requests level attitude, **not zero horizontal velocity**. Use small opposite tilt to brake. Gentle reduces commanded acceleration; it is not a horizontal-speed limiter or position-hold autopilot. The FC still has angle outer loops and rate inner loops for roll/pitch, and rate control for yaw.

## Plan start → any intermediate nodes → finish

1. Click **Use drone as start**, then click the map to add nodes. Or click **Load example** for a five-node route.
2. Drag nodes, edit their x/y values, reorder with arrows, remove a node, undo, or clear. Save/load JSON is available.
3. Choose cruise COM height, initially **1.25 m**.
4. Click **Review route**. The first node is start; the last is finish. All intermediate nodes are visited at cruise height, with no intermediate landing.
5. Check the route summary. Leave **Select navigation automatically in the simulator** enabled unless you prefer the Xbox gesture yourself.
6. Click **Approve and fly**. This sends the navigation gesture, waits for FC acknowledgement, then sends the route through the real ROS2 service. It does not send motor forces or bypass the C control core.
7. Watch readable mission phases and node progress. After landing, disarm or reset before another flight.

The service supports **2–64 nodes**, not just two/three. The route is a local metric map, not GPS. Bounds are x∈[-1,9], y∈[-3,3] metres; adjacent nodes must be at least 0.4 m apart. Cruise z must be 0.6–2.5 m and at least 0.4 m above the starting ground COM. Known obstacle clearance can reject a node. A request explains the specific invalid node, missing mode/input/pose, or active mission.

The first node must match the stationary vehicle's current ground position. To choose another start, set the first node and click **Reset scene at first node**. This finalizes the current evidence, starts a new world at that location, and reconnects the station. It is an explicit world setup operation; the running aircraft is not teleported through flight commands. Reset locations overlapping a panel are rejected. The ordinary **Reset scene at origin** button recovers from a crash/disarm/out-of-bounds flight.

Teal shows the current planned path; numbered nodes and dashed lines show the requested route. Orange regions are observed panel clearance estimates. Unobserved obstacles are not supplied to the drone as a hidden map. If no path is available, the UI says so. Switching to stable cancels navigation authority.

## Camera and recording

- **Follow drone**: native attached third-person camera, useful for manual flight.
- **Garden overview**: fixed native garden camera.
- **Onboard RGB**: native forward camera.
- **Recorded composite + HUD**: follow/onboard views and transport diagnostics in live sessions.

New native cameras and MP4 recording target **15 fps in simulation time**. The browser requests live frames up to 15 fps without repeatedly aborting image loads. Older recordings remain at their native 5 fps. No duplicated/interpolated frames are presented as new evidence.

Actual wall-clock responsiveness depends on measured real-time factor. The 500 Hz control/integration step is logical simulation time; slower rendering/inference can reduce wall-clock speed. A static onboard view while stationary is normal. Frame sequence/freshness and the Xbox monitor distinguish stationary scenes from stopped transport.

Each launch/reset creates a new folder under `artifacts/gazebo-suite/`:

| File | Contents |
|---|---|
| gazebo.mp4 | Original native views/HUD; new recordings 15 fps |
| flight.csv | Native state, C estimates/demands and contacts; new runs retain 500 Hz, older runs 50 Hz |
| real-xbox.csv | Actual Windows input, including raw HID bytes |
| wire-inputs.jsonl | ROS live/suite HID and UART bytes; new live runs distinguish original/scaled/operator input |
| mapped-xbox.jsonl | Transmitted standalone reports and profile |
| operator-actions.jsonl | Explicit PC simulator actions |
| pc-map-missions.jsonl | Original PC requests/replies |
| perception.jsonl | Original YOLO/PnP observations |
| engine.log / engine-summary.json | Native engine, parameters, source hashes and whole-session contacts |
| transport.json / live-summary.json | ROS / standalone session result |
| reset-request.json | Explicit starting location for the next world |

**End and save video** finalizes the recording. A disarm is latched until reset/restart. Preserve failures and collisions; a successful mission segment does not make an entire session successful if subsequent manual flight crashes.

## Review and analyse a recording

```powershell
.\qcopter.ps1 analyse-recordings --run RUN_FOLDER_NAME --name UNIQUE_REPORT_NAME
```

Repeat `--run` to compare multiple recordings. The report:

- Scans every native video frame and exports per-frame image-change/state data.
- Provides normal/half-speed playback and previous/next native-frame controls.
- Creates representative-frame contact sheets and full-telemetry graphs.
- Exports one-second graph summaries containing means **and extrema**, keeping the original telemetry intact.
- Measures unwrapped yaw travel/rate, attitude, speed, height-estimate error and contacts, with original-file hashes.

This analysis does not infer real mass, thrust or sensor noise from an assumed simulator. It separates the first navigation interval from later manual interaction. Reports state actual sampling rates: 500 Hz for new telemetry, 50 Hz for earlier recordings. Native engine contact totals cover the whole session; sampled contact rows within a segment are a different quantity.

See [recordings and heading comparison](../artifacts/reports/station-analysis-20261021/index.html). The repeatable five-node case shows less unwanted yaw after ROS2 uses intended path velocity and retains arrival heading. It does not establish real-aircraft performance or statistically characterize all environments.

## Scope and physical communication

Live flight runs the active C core as a **host DLL**, with native Gazebo 10.5/OGRE2 physics/cameras. QEMU is separate Xtensa CPU testing. The model assumes 1.2 kg, inertia (0.012,0.012,0.022) kg·m², 450 mm diagonal, 6 N/motor and 40 ms lag. Sensors, calibration and localization are synthetic. The garden is illustrative. Ranging is specific to known-size blue panels (1.09×1.69 m inner face on a known flat floor), not arbitrary obstacle depth. The large ground plane supports recovery outside the initial garden; the validated planning corridor remains bounded.

The physical companion interface remains **115200 baud, 8N1, CRC-checked UART**, with 3.3 V logic:

| ESP32 provisional pin | Companion |
|---|---|
| GPIO17 / TX2 | RX |
| GPIO18 / RX2 | TX |
| GND | GND |

Verify exact board routing before wiring. Never connect 3S battery voltage to a GPIO. The portable ROS2 C++ `uart_bridge_node` accepts a serial `port` (`COM...` or `/dev/tty...`). FC status is not full odometry/battery telemetry. A future companion needs real camera/localization drivers and the existing documented map-velocity frame adapter. Electrical UART, physical BLE/I²C/ESC timing and separate-machine DDS remain unvalidated. See [binary protocol](xbox-ros2-protocol.md).

`.\qcopter.ps1 build-standalone` compiles `FC_ENABLE_COMPANION=0`: no Serial2 activity, no navigation selection and no companion commands. `.\qcopter.ps1 build` supports an optional companion. Physical motor arming/output remains disabled in both.
