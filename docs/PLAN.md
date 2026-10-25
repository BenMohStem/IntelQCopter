# Software plan: one small, measured step at a time

## Current position

The new secured-mount session `20261025T132707615977Z` contains 18 completed poses and 360,000 samples. Original log hashes, sequences, checksums and configuration/source identities have been checked offline. The user reports completing this session after restarting because the earlier mounting detached. Exact pose angles, fixture uncertainty and continuing mounting rigidity are not independently certified by the software.

The deleted detached-mount session is excluded. Earlier calibration/level coefficients do not validate the new mounting.

**Current prepared step: full coupled XYZ calibration.** The user requested applying all three axes together, replacing the sequential X-only implementation. [Complete calibration and live recording](full-accelerometer-calibration.md). Main.cpp restores all three MPU bias words, removes only their software rounding remainders and applies all nine matrix terms to each coherent XYZ sample. No pose-dependent zeroing or norm forcing. Compile and algebra/protocol checks pass; physical response and reset/power-cycle validation remain pending. The exact raw recorder stays preserved in archive/bench-checkpoints/raw-pose1000/main.cpp.txt.

**Previous completed step: rough independent tilted-pose review.** Four new 20-second logs passed integrity checks. The operator confirmed raised-side order +X, -X, +Y, -Y and a hand-held ruler with about ±1° placement error. Frozen-candidate differences are 1.06°, 0.25°, 0.42°, 0.21°, with no refit. See the [tilt report](../measurements/mpu6050/2026-10-25-tilt-validation/report.md). This supports the next controlled bench lesson, not sub-degree accuracy or flight readiness.

**Previous completed step: offline calibration and noise analysis.** See the [derived report and graphs](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md). This uses original hardware recordings; the correction curves are mathematical replay. No simulator, upload or new hardware test was performed. The raw 20-second recorder used for those measurements is preserved as a checkpoint; current firmware applies the complete coupled XYZ calibration.

## Next steps and their acceptance evidence

| Step | One new idea | Status and evidence needed |
|---|---|---|
| C1 | Record six orientations three times | Recorded: 18 × 20,000 samples; integrity checked |
| C2 | Fit calibration on repetitions 1-2 | Calculated provisional bias and response matrix; equal pose weights |
| C3 | Reserve repetition 3 and inspect noise | Offline replay complete; direction errors about 0.07-1.11°, with reference uncertainty included; transients/correlation present |
| C4 | Test independently measured tilted orientations | Four 45° records reviewed with operator-confirmed raised sides and approximate ±1° hand-held reference; combined/fixed-reference checks remain optional refinements; no sub-degree acceptance |
| C5 | Verify calibration implementation | **Full XYZ prepared, not physically passed:** all three bias words and full matrix together, as requested. User uploads and runs record-accel in arbitrary orientations; the isolated X check is superseded |
| C6 | Restore offsets after reset/power cycle | Save fixed coefficients in firmware flash, restore volatile MPU bias registers using the same baseline convention, preserve compensation bits and verify no duplicate subtraction |
| C7 | Characterize short-term noise and drift | When characterizing noise specifically, use a separate 2-3 minute stationary recording after temperature settles; assess full covariance, autocorrelation, block means and transients |
| C8 | Learn one attitude estimate | Only after calibration checks: one roll/pitch observation, then gyro integration and a small Kalman lesson using measured noise assumptions |

C4 is deliberately separate from C5: a smaller same-pose residual is not proof of accuracy at a new angle. C7's independent stationary experiment can be done before C5 if needed to diagnose drift. A long-term Allan/bias study is a later optional measurement, not something claimed from a 20-second capture.

No blanket joint Gaussian or sample-independence assumption is accepted. Marginal distributions and selected projections are diagnostics; off-diagonal covariance remains measured. Covariance of a mean must account for time dependence and reference uncertainty.

## C4: measurements reviewed with limited reference accuracy

Keep the MPU/FC fixed in its final mounting. Use an independent reference to measure actual roll/pitch, their signs and uncertainty. For each pose, record the measured angles, temperature and 20 seconds of raw data after settling. User-selected order: +45° roll, -45° roll, +45° pitch, -45° pitch; combined tilts remain a later check. These are approximate targets; actual measured angles determine the reference vectors. Do not use MPU-estimated angles as their own calibration reference. Use [the 45-degree recorder](record-45-degree-tilts.md), which defines signs and records actual reference angles/method/uncertainty. The new `record-tilts` command reuses the existing firmware and strict capture checks; the original `record-poses` command stays for the six axis orientations. The user completed four new records. Their original roll/pitch labels were clarified after capture; original metadata remains unchanged. The assistant verified logs offline and did not open a serial port or upload firmware.

Review reserved-pose errors and fixture uncertainty together before choosing an acceptance threshold. Do not quietly refit a validation dataset and call its residual independent accuracy. If it informs a new fit, collect new validation data.

## How each lesson is implemented

For this step the user explicitly chose the full 3D matrix at once. Do not require isolated X/Y/Z lessons or prescribe table/flipped runtime modes. Reference poses are metrology inputs, not conditions under which the calibration algorithm is enabled. Apply the same vector transformation at all orientations.

Use one active Arduino-style `firmware/main.cpp`, mostly C-style functions, Adafruit/Wire where suitable, Allman braces, four spaces, one statement per line and `Serial.printf`. Aim for one new idea and 3-10 lines per firmware lesson; explain larger exceptions. PC analysis tools stay in `tools/calibration/` and do not clutter firmware. Do not re-import the archived large controller.

For each change: explain it, prepare the smallest implementation, compile/check what is relevant, describe the expected physical result, commit and push. A compiler or synthetic test is not a hardware pass. The user uploads. Keep original successful/failed evidence local and publish reviewed derived reports without raw console dumps, credentials or environments.

If step 6 fails after 7-10 were prepared, repair step 6 in a new commit/checkpoint. Rebuild later steps on the repaired step rather than leaving invalid descendants active. Preserve history; no force-push. Hardware/configuration/mounting changes invalidate affected calibrations and downstream acceptances.

## After calibration

Test BMP280, identify the actual magnetometer chip before choosing a driver, and test one LiDAR distance. Use known-height/distance references. Add their calibration/persistence in separate lessons. Temperature and deliberate recalibration need explicit procedures; runtime recalibration can use Arduino Preferences later if required.

Then add Xbox connection, one stick, one mapping and disconnect behavior in separate steps. Standalone flight stays **yaw rate, roll/pitch angle, altitude hold with vertical-speed command, no XY braking**. Attitude/rate loops, mixer, loop timing and gains come later. The flight-control target remains 500 Hz; the current 1000 Hz bench capture is a separate measurement task. Physical motor arming and output pulses remain disabled under the current instruction.

Mass/inertia and PID-gain work are deferred to the later standalone-FC simulation phase. The measured/assumed parameters are preserved in `simulation/parameters/params-2026-10-24.txt`; simulation is not a hardware validation. You build the PCB and Gazebo environment; assistance focuses on commands and small problems unless you ask for more. ROS2 companion/navigation follows the standalone fundamentals and must remain optional.

## Earlier lesson history

The initial serial/Wire/MPU lessons, old calibration implementations and original plan remain in Git history. The older large diagnostics are archived under `archive/before-small-lessons/`. [Recording instructions](record-18-poses.md), [MPU configuration](mpu6050-configuration.md), [calibration math](accelerometer-calibration.md), and [ROS2/Gazebo terminal commands](start-ros-gazebo.md) remain available; historical coefficients in those guides are not accepted for the new mounting.
