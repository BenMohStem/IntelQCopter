# IntelQCopter — a finite path to a working flight controller

Updated 30 October 2026. This is the current roadmap; it replaces the scheduling in [the archived plan](../archive/plan-history/2026-10-30-before-visible-milestones.md). Historical reports remain evidence for their exact source versions. No hardware acceptance is created by changing this document.

**Immediate goal: see the drone's estimated roll and pitch respond live, then implement PID outputs with motors disabled.** No additional calibration campaign is requested now.

## Where we actually are

| Capability | What we have | What it does NOT yet prove |
|---|---|---|
| Accelerometer calibration | Six-pose fitted coefficients and retained validation evidence | Exact physical reference alignment in every motion |
| Gyro bias | Measured frozen correction and restart evidence | Physical axis signs or precision gyro scale |
| IMU reading | Actual nominal1kHz FIFO/direct recordings, including20second gyro captures | Complete1kHz PID deadlines under full load |
| Xbox | Connection, input endpoint observations, menu with operator-specific Menu/Y correction | Every flight command sign, deadband or flight disconnect policy |
| Attitude filter | Offline preparation and limitations documented | Live MCU attitude acceptance |
| PID and mixer | Earlier project references; not implemented/accepted in this Stepwise FC | Physical flight stability |

The installed board currently runs `lessons/xbox-test-menu/main.cpp`, a motor-disabled measurement harness. `firmware/main.cpp` is the separate calibrated development source and contains a pre-existing, unreviewed gyro matrix coefficient edit. It is not the installed menu. The next integration must review that edit explicitly and preserve its original diff; do not overwrite it or call its stale evidence current.

## Milestones and stopping conditions

| # | Visible result | Work | Minimum evidence to move on |
|---|---|---|---|
| 1 — next | Live roll/pitch angles and yaw rate | Integrate the existing calibrated IMU and a small attitude estimator into the single Arduino main.cpp; label provisional axes; retain Xbox on the slow core | Build, relevant existing-data replay, then one guided motor-disabled tilt session with saved logs. Corrected rates/angles change consistently and recover when returned to the starting pose. This is a preview, not flight acceptance. |
| 2 | Xbox sticks produce understandable target values | Angle targets for roll/pitch, rate target for yaw; clear deadbands/limits, neutral startup, input timeout and disconnect behavior | One combined neutral/sweep/release/disconnect session. Each physical stick direction and neutral action matches the displayed target; invalid/lost input is rejected. No repeated timing campaign. |
| 3 | PID terms and four computed motor commands on graphs | Roll/pitch angle-to-rate cascades, yaw rate PID; mixer, integral limiting, saturation and correction headroom | Software/model tests show restoring correction signs, bounded commands, correct saturation and stop paths. Before physical output, resolve one guided bidirectional X/Y/Z direction check. Outputs still disabled. |
| 4 | Simulated response and initial gains | Use supplied mass/inertia with provenance; identify unknown motor/thrust parameters as assumptions. Help the user connect the actual FC logic to their own Gazebo model | A small set: attitude step/release, disturbance recovery, saturation and input loss. Report overshoot, settling and remaining error. Simulation results are not hardware gain acceptance. |
| 5 — later authorization | Propeller-free hardware output verification | Known board pinout/protocol, measured ESC behavior, motor order/direction and disarm | Only after an explicit change to the current motor-output restriction. Each output matches its intended motor; stop paths work. This plan does not authorize motor pulses. |
| 6 — hardware prerequisite | Altitude and vertical-speed estimates, then altitude control | Obtain/test BMP280 or an appropriate range sensor; validate vertical estimate and implement height/vertical-speed control | Known height changes agree with the reference and simulation shows bounded vertical response. MPU-only data cannot provide dependable fixed altitude. Must be completed for the requested height-hold flight mode; it does not block milestones1–4. |
| 7 — later | Controlled flight evaluation | Measured hardware parameters, reviewed gains, accepted axes/inputs/outputs/altitude, explicit operator authorization and suitable test conditions | Progressively validate actual responses and fix observed failures. No claim that the bench or simulation already proves safe flight. |

Physical sign mistakes can turn negative feedback into positive feedback; that is why the short direction check remains mandatory before motors. Precision gyro scale calibration is a separate question, triggered by a reproducible reference-angle discrepancy rather than being an endless prerequisite for a motor-disabled display.

## Calibration is now a frozen baseline, not an open-ended task

Use the existing coefficients for development, retaining their documented uncertainty. Do not refit them from the ambiguous recent motions. Unknown sign stays unknown. Original logs and failed trials remain preserved.

Reopen a specific calibration only when:

- The sensor/mounting changes, or register readback shows the intended correction was not applied.
- A correctly labelled, independently referenced check shows a reproducible error affecting the milestone.
- Clipping, corruption or drift is observed and traced to the sensor/configuration rather than hand motion or reference uncertainty.

A small residual while resting on an unverified support is not enough to reopen six-pose calibration. More samples cannot remove a wrong reference angle. Current +X/-X confusion calls for clear physical labels, not another statistical fit.

## A test must answer a named question

Before asking the operator to record, give: the question, one action, expected outcome, duration, stopping condition and next feature it unlocks. If the question can be answered from saved evidence, use it. Assistant handles builds, offline checks and routine analysis; the operator performs only physical actions/reference observations that cannot be done remotely.

Default is one informative session per milestone. If it is ambiguous, explain exactly what was missing and propose one targeted repeat. Do not silently launch a third repeat or a new calibration campaign; summarize the unresolved decision and continue independent implementation with provisional labels where appropriate. Relevant regression after a source/configuration change is still required, but need not repeat unrelated sensor characterization.

No new tests merely to produce more graphs. Do not change calibration, filter bandwidth, acquisition mode and gains together: each small implementation step should have one understandable purpose. Every report says whether evidence is hardware, replay or simulation.

## Deferred work

Advanced temperature compensation, Allan deviation, additional statistical studies and precision scale fitting are optional investigations when an observed issue justifies them. ROS2/YOLO/GPS autonomy follows a working standalone FC. Pico/RF branch, report cosmetics and PCB redesign do not compete with this ESP32 software milestone. The user builds the PCB and simulation unless they ask otherwise.

Future RGB red-at20% battery warning and safe landing/return remain plans: voltage/current sensing and appropriate navigation/altitude capabilities are absent. No battery threshold action is implemented. Small activity LED controllability remains unidentified and is not a blocker.

## What happens on the next implementation turn

Prepare milestone1: review the existing coefficient discrepancy, bring up a small motor-disabled attitude preview using preserved calibration and existing offline filter work, compile and replay relevant data, then provide a visible live display and one guided tilt check. Do not ask for another long stationary or six-pose dataset by default. Physical axes may remain labelled provisional during preview; they must be resolved before enabling feedback to motors.

Each completed change is committed and pushed when connectivity permits. Failed later acceptance invalidates dependent acceptances, not historical logs. Fix the failed step and revalidate its dependents without deleting evidence or rewriting Git history.
