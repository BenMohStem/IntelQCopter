# Missing information and current problems

## Information needed from the hardware

| Priority | Missing item | Why it matters |
|---|---|---|
| 1 | Complete takeoff mass, component positions/weights and centre of mass | Inertia, thrust reserve, hover feed-forward and gain validity |
| 1 | Exact motor model, ESC model/protocol, prop brand/spin direction; thrust/current measurements on 3S | 1400KV and 40A do not determine thrust or actuator dynamics |
| 1 | Single RGB camera model, calibration/interface and validated metric-distance method | A monocular bounding box has no unique metric distance |
| 1 | Local position/velocity source: VIO, stereo odometry, tracking system or future GPS | IMU/barometer alone cannot bound horizontal position drift |
| 1 | Exact LoRa chip/module, band, antenna and required command rate/range | Driver selection, packet airtime and link watchdog cannot be finalized |
| 2 | Exact RP2350 board/PCB package and voltage regulator/power distribution choices | Pin allocation, analogue grounding and PCB routing |
| 2 | Joystick/trigger electrical ranges/directions, display model and final button mapping | End-point calibration, wiring and UI |
| 2 | TOFSense-F2 Mini exact interface/version and mounting; confirm sensor authenticity/models | Bus driver, rate/latency, offsets, valid return rules |
| 2 | Battery connector, measured pack mass, voltage-divider/current sensor calibration | SOC uncertainty and low-battery behaviour |
| 2 | Photos/measurements of exact USTHB garden and panel sizes/materials | An illustrative scene cannot establish clearance in the actual location |

## Completed in the new simulation work

- A procedural 3D campus garden now runs with floor contact, takeoff, navigation,
  approach and landing. It is an illustrative environment, not surveyed USTHB.
- A synthetic panel detector is trained and used on the rendered RGB stream.
  Range is calculated from calibrated known-size panel geometry, without giving
  the planner a depth map or true obstacle positions. The 1D lidar points down.
- The shared C++ mission/nav core and production C loops run together. A bundled
  ROS2 selector can send explicit start/finish points; control and lidar updates
  pass through actual DDS adapter nodes. These adapters do not validate the
  physical serial bridge. ROS C++ wrappers now build and pass a separate recorded
  image/mission DDS integration test, but are not yet the complete flight path.
- The handheld has tested shaping/filtering/slew, signed trigger endpoints and
  one-shot takeoff/land request gestures. RT enables a climb/collective boost;
  centered vertical stick still holds height. Physical arming stays disabled.
- The paper removes the generic matrix-multiplication/exponential lessons and
  adds explicit differential-equation, transfer-function and stability derivations.

## Current engineering problems

- The RP2350 image has no connected physical sensor/radio/ESC pipeline. Motor
  arming remains disabled. A simulated motor command is not an ESC output.
- The editable PCB is a partially routed draft; power integrity, current sensing,
  analogue filtering and manufacturing checks remain unfinished.
- All reported gains use an assumed 1.2kg model, assumed thrust/inertia and motor
  lag. They are simulation candidates, not an identified airframe tune.
- Independent roll/pitch angle filters approximate gyro rates as Euler rates and
  accelerometer observations as gravity. Translation/vibration violates those
  assumptions. Covariance consistency under synthetic noise is not flight proof.
- A single fixed 1D lidar sees one ray. A downward sensor aids altitude; a forward
  sensor cannot certify the entire rotor corridor. Camera + YOLO needs a metric
  geometry/localization solution and must not ignore unrecognized obstacles.
- The existing COCO YOLO model has no generic panel class. Synthetic/custom panel
  training needs held-out evaluation and later real images, including hard negatives.
- The new navigation demonstration remains restricted to calibrated known-size,
  coloured rectangular panels and simulated local odometry. It does not solve
  metric ranging for arbitrary unknown-size obstacles or implement real VIO.
- Finite-grid linear analysis finds instability for some large motor-lag/plant-gain
  combinations: scale1.3 passes634/675, baseline1.0 passes673/675. Scale0.7 passes
  675/675 tested points and a full mission, so it becomes the conservative provisional
  default. A separate continuous-interval Routh proof covers the stated local decoupled unsaturated model; neither result proves global nonlinear or real-flight stability.
- RT boost is functional in simulation but can worsen settling/navigation timing.
  It is inhibited during simulated approach/landing; coordinated physical pilot/
  companion authority and power-limit integration remain unfinished.
- Native Jazzy and the project ROS C++ nodes now build. The Gazebo flight uses
  Python DDS adapters around production C and the shared C++ mission core; a full
  ROS C++ wrapper flight and physical companion/serial integration remain pending.
- Isaac Sim was removed at the user's request. Historical successful native
  PhysX/RGB flight evidence remains; the current workflow uses Gazebo.
- The requested battery policy is now warning at 35% estimated SOC and latched
  disarm at 20%, regardless of height, with no automatic landing. USB and simulation
  ROS flags exist; real acquisition and LoRa notification remain unfinished.
  Physical arming remains disabled. This policy update has not been exercised in a new flight.
- Calibration now has a two-slot RP2350 flash journal and stationary gyro boot
  verification/recalculation. Physical power-loss/endurance checks, acquisition,
  guided six-face capture, and automatic mapping of measured noise into all filter
  settings remain unfinished.
- GPS/sensor-loss recovery, companion authority, landed detection and dropout
  behavior still require explicit state-machine verification.
- Git origin is the public BenMohStem/IntelQCopter repository. Setup changes and
  original evidence are committed/pushed as separate steps; downloaded environments
  and model weights are excluded.

## Work sequence

1. Keep the six flight loops: roll/pitch angle cascades, yaw rate, height target
   driven by centered vertical-speed throttle. Mission logic belongs on the companion.
2. Derive and assess the complete local linear models, actuator lag, sampling,
   stability conditions and parameter sensitivity; measure hardware when available.
3. Improve handheld calibration, shaping, freshness and testable input processing.
4. Add a companion ground/takeoff/hold/navigate/land/landed supervisor with explicit
   start/finish selection; it emits the same flight-mode setpoints.
5. Test a 3D illustrative garden with panel geometry hidden from the planner.
   Rendered RGB and simulated odometry are declared assumptions; later replace
   them with real perception/localization and validate again.

The evidence directory distinguishes measured simulation output, source-only ROS
work, physical unknowns and historical Isaac execution. USTHB is the intended test
site; the current scene must not be described as a surveyed campus digital twin.
