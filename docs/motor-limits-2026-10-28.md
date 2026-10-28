# Motor limits and system tools - 2026-10-28

User request supersedes the prior blanket arming restriction: armed idle 10%, normal final per-motor maximum 45%, full-boost final per-motor maximum 65%. Disarm/fault/stale-input paths remain zero. These percentages describe normalized ESC command range, not measured RPM, thrust or power.

## Implemented scope

- Active RP2350 and ESP32 C control cores bound the final coupled motor allocation, including attitude/yaw corrections, to the 10%-to-current-ceiling interval. Boost interpolates the ceiling from 45% to 65%; negative/reduce boost cannot exceed the normal cap. Existing invalid-input and battery-disarm paths remain zero. These core outputs are still mathematical demands: the incomplete physical adapters were not invented.
- TMP/Drone's known MPU-only ESP32-S3 test uses SDA=9, SCL=8 and original motor GPIO order FL=12, FR=1, RR=2, RL=14. It implements deliberate A press after release, fresh controller/calibrated IMU, neutral sticks/triggers, and plausible entry attitude; X/Y stops. Startup/reconnect/fault/flash-write inhibition require a new release/press. Armed centered input produces 10% commands, normal manual input reaches at most 45%, RT boost reaches at most 65%. Every final PWM command is clamped. Invalid numerical commands and LEDC errors fail closed. NVS writes occur inhibited/disarmed. No firmware was flashed and no motors were tested.

## Remaining plan

- RP2350: obtain four verified motor GPIOs, ESC protocol/pulse endpoints and a real radio/sensor driver before enabling hardware. The user explicitly deferred these unknowns to the plan. Then add the same neutral-input arming state machine and verify stop priority, 10% idle and 45/65% caps at the physical output boundary.
- Main ESP32 scaffold: physical ESC/arming adapter still absent; preserve current calibration/barometer/companion gates and implement hardware arming only against its selected verified hardware. Its reduced limits are already applied to the mathematical mixer.
- Stepwise ESP32: active firmware is a calibrated 1kHz FIFO/sensor lesson with pending physical gyro-frame checks, not an implemented flight/PID/motor runtime. Keep source/user edits/calibration evidence untouched; carry this 10/45/65 policy into the future actuator lesson. Historical snapshots and original Drone source remain unchanged.
- First hardware validation must measure pulse endpoints, motor order/directions, stop behavior and real idle response with propellers removed. Software tests do not establish safe physical RPM or flight acceptance.

## Verification and assumptions

Actual engine for these checks: native host execution of production C/C++ compiled with shared Zig 0.16.0, plus a PlatformIO ESP32-S3 firmware compilation using Arduino-ESP32 3.3.12. No Gazebo physics, QEMU replay or physical MCU/ESC measurements were used.

RP existing host regression and ESP32 host DLL build passed. ESP32 actual-core deterministic test passes 20,000 extreme-allocation cases plus invalid command/sensor/battery stop cases. TMP source-extracted tests pass 100,000 bounded allocations, idle/endpoints, arm/re-arm rules, stop/fault/reconnect, flash inhibition, and mocked LEDC failure cases. TMP firmware compiles (36072-byte static RAM / 823435-byte flash in initial successful build). The first build exposed inconsistent pre-existing PID member spelling; it was fixed consistently. The initial direct ESP32 test harness omitted fc_init and produced NaN; the harness now uses the actual boot initializer. These failed attempts are preserved in the chat/tool history; they are not physical trials.

TMP PWM assumes the original standard 1000-2000 us / 50Hz ESC interface, with Arduino LEDC 14-bit resolution (small pulse quantization). Hardware endpoints, wiring, direction, motor/prop response, mounting, PID suitability, runtime concurrency/timing and MCU fault response remain unvalidated. Reduced ceilings can prevent hover in old simulations whose assumed hover command exceeds 45%; do not silently raise the limit to make them pass.

Shared-tool revision: prefer installed system tools, create venvs only for demonstrated compatibility requirements. Zig moved out of .venv-tests to C:/WinLibs/zig/zig.exe, and normal host scripts now resolve that shared compiler. Custom Gazebo/Pixi and ROS/RViz/URDF setups are preserved. Workspace-wide ML migration inventory remains outside this repository in SYSTEM_TOOLS_AUDIT_2026-10-28.md.

Knowledge-index limitation: the ESP32 knowledge build was run, but it reports a missing original gazebo-suite result removed at the user request in the preceding iteration. Repair of these report/dependency references was explicitly deferred; no historical evidence was fabricated to make the index pass. Important-source refresh and hash checks pass.

Final transition check: releasing RT immediately restores the 45% hard per-motor cap even while the internal boost slew decays. The ESP32 core also rejects nonfinite reduce-trigger input. Both production-core regressions pass with these cases.
