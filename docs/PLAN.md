# Current software plan

**FC target: 1,000 Hz.** Runtime stays in one Arduino-style main.cpp, mostly C-style functions, Allman braces and Serial.printf. Motor outputs/physical arming remain disabled. BMP280 is unavailable and deferred. ROS2, Xbox and simulation remain later software stages. PCB exploration is a separate user-authorized workstream; small sensor/software gates stay in force.

| Stage | Current evidence | Next action |
|---|---|---|
| Accelerometer calibration and boot restoration | Real six-pose/tilt/XYZ/reset evidence preserved; reference uncertainty remains | Preserve coefficients |
| Gyro bias and boot restoration | Real stationary, reset and full-power-cycle recordings checked; residual bias remains | Preserve measured constants; do not refit validation records |
| Stationary repeatability | Assistant-run originals and covariance reports saved; support condition assumed | No more repeated low-rate sampling unless it resolves a specific issue |
| 1 kHz FIFO/state processing | Real 1 kHz captures: 60k samples after cross-core repair, max handlers 814/819/812 us with no reported overruns | Continue timing regression as work is added |
| Gyro body-axis mapping | Rotation-only candidate implemented and computation/timing checked | User deferred known physical rotations until later |
| Attitude filter | Offline candidate prepared and checked; no MCU integration | Address modeled acceleration sensitivity; physical frame gate remains pending |
| Rate/angle PIDs and mixer | Not implemented in this branch | Add one controlled stage at a time, then simulation/gain validation |
| Altitude/vertical control | BMP280 unavailable | Defer; do not claim hover capability |

[Current 1 kHz guide](fifo1000-stage.md) describes the exact runtime, test commands and limits. High average sample rate is not a complete 1 kHz FC or hard deadline proof. Physical tests must identify actual engine/source/configuration and retain failures.

The user delegates fixed stationary tests to the assistant and prefers concise results after the work. The user explicitly authorized flashing for this bench work. Physical arming/output stay disabled. Known physical rotations are deferred by the user. Moving/reference tests still need physical operator action. Do not fabricate level, stationarity, dynamics or successful hardware operation.

After each meaningful change, run relevant tests, commit and push without rewriting history. If a hardware stage fails, repair that stage and rebuild later work on it. Keep original evidence private; publish reviewed derived summaries. No empty commits.

[Earlier detailed plan/history](../archive/plan-history/2026-10-26-before-1khz.md) and the Git history preserve earlier accepted work. Archived references to a 500 Hz target are superseded by the user's 1,000 Hz request.

Current FIFO1000_V2 computes provisional body rates: additional 20k hardware run ~1000.69 Hz, max handler 822 us. [Frame guide](gyro-frame-stage.md). Physical gyro sign/scale checks and attitude/PID acceptance remain pending.


## Next small step: fewer physical tests, more information per recording

The user deferred physical rotations and requested efficient experimental design.
Prepare a near-level scalar Kalman angle candidate on the PC only. Freeze noise
statistics from the first three repaired FIFO captures; reserve the latest capture
for a separate stationary replay. No calibration change, new stationary capture,
firmware change, or upload is needed for this step.

Use a four-case full factorial model experiment: gyro bias absent/present crossed
with translational acceleration absent/present. Preserve every outcome and state
the assumed motion/noise. This screens two disturbances and their interaction; it
does not reproduce all possible flights or validate the physical gyro axes.
Keep distinct inexpensive protocol/failure checks. Later, three operator sessions
(positive and negative motion about X, Y, and Z) can cover physical direction;
scale accuracy additionally requires a measured angular reference.

After physical frame validation, port the small candidate update into main.cpp,
measure its actual 1 kHz execution time, and evaluate motion before PID work.

Offline preparation completed: 80k original samples reverified, 54 offline checks passed, and all four modeled disturbance combinations retained. The acceleration cases reach about 5.2–5.35 degrees of modeled angle error; the candidate is not accepted for flight. [Results](../measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md) and [equations](attitude-preparation.md). Main.cpp/calibration/firmware timing evidence are unchanged.

User requested complete module connections and chose I2C when SPI adds no practical benefit. [Connected I2C reference](../hardware/connected-i2c/README.md) supplies shared sensor bus, interrupts, lidar/companion UART, ESC signals and separately selectable BEC links. PCB is unrouted. Firmware integration and physical timing/identity checks remain pending; current main.cpp is unchanged.

## Latest autonomous checkpoint: 27 October 2026

The startup guard now covers current carrier GPIO4/1/14/21 and legacy12/2,
all input-pulldown with no motor output.54 offline tests,19 actual-source host
cases and the ESP32 build pass. This revision has NOT been uploaded: no serial
device was detected. [Results](../measurements/software/2026-10-27-input-guard/README.md).
Historical real timing is preserved under its original source SHA.

Next hardware step when the board is connected: upload this small input-only
revision and run one1kHz integrity/timing regression, then the deferred known
rotations with the user. Do not refit calibration or claim dynamic attitude/PID
acceptance from stationary or fake-peripheral tests. Fresh zero-jumper PCB
candidates must pass independent bare-copper connectivity and native DRC/parity
before being chosen; candidate pin/address changes do not silently enter firmware.

## Planned wiring change: MPU6050 auxiliary-bus bypass (27 October 2026)

User decision: use XDA/XCL to simplify the user-designed PCB and avoid wire
jumpers where useful. This supersedes the earlier requirement to leave these
pins disconnected. This is a PLAN ONLY: existing firmware, PCB exports and
hardware acceptance have not been changed or validated for this wiring.

- Wire downstream sensor SDA to MPU XDA, and SCL to MPU XCL; use 3.3V
  logic/pull-ups and common GND. Do not also connect these lines to GPIO40/41.
  The downstream sensor shares the MPU host bus (currently GPIO9 SDA/GPIO8 SCL).
  Which sensors use this route must be reflected in the final wiring manifest.
- After Adafruit MPU initialization/reset, preserve unrelated register bits:
  clear USER_CTRL (0x6A) bit5 I2C_MST_EN; set INT_PIN_CFG (0x37) bit1
  I2C_BYPASS_EN. Do not overwrite FIFO configuration when changing these bits.
- Read back both bits, then initialize/check downstream sensor identities and
  addresses on that same TwoWire object. Adafruit setI2CBypass(true), if used,
  must be checked against the installed library; explicitly verify master-off.
- Configure once after each MPU power-up/full device reset, and repeat after
  recovery that resets/reinitializes the MPU or changes these settings. Normal
  sampling does not require rewriting bypass every cycle. The setting is volatile,
  not permanent storage. An ESP reboot runs initialization again.
- Acceptance: readback and sensor identity, cold boot, MCU reboot, MPU reset/recovery,
  and 1kHz FIFO timing/integrity regression with downstream reads. Pending real tests.
  Added bus traffic must not silently invalidate the existing acquisition budget.
- Bypass is not a long-cable buffer. Keep wiring short and verify combined pull-ups.
  Physical motor outputs remain disabled. No gain/calibration changes in this step.

Reference: [TDK register map, INT_PIN_CFG and USER_CTRL](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
