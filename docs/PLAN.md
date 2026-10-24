# Software plan â€” one small change at a time

## How we will work

You design and make the PCB. This plan covers software only.

There will be **one active `firmware/main.cpp`**, written in Arduino style with C-style functions. Braces go on their own lines. One statement per line. Clear names, short functions, and comments explaining why.

The first programme will be about 20 lines. Afterwards, aim for **3â€“10 new lines per step**. If a feature needs more, split it again. Do not paste the full prepared controller back into the file.

For each step:

1. Explain the one new idea and show the small change.
2. Change only what that idea needs.
3. State exactly what you should see on the real board.
4. Commit the step separately. A build is not a hardware pass.
5. With hardware available, test before continuing. Without it, label the step prepared and untested. Prepare later lessons only when you ask; do not automatically grow the active file.

## Start here

| Step | One new idea | Expected real result |
|---|---|---|
| 01 | Print one sentence every second | Serial Monitor repeatedly shows `ESP32 is running` |
| 02 | Add `millis()` to that message | Time increases by approximately1000ms per message |
| 03 | Start Wire on SDA9/SCL8 | Programme still runs; no sensor readings yet |
| 04 | Ask the MPU6050 library whether it responds | A clear connected/missing result |
| 05 | Read only the accelerometer Z axis | Value changes when the sensor is turned |
| 06 | Read only the gyroscope Z axis | Value changes when the sensor rotates |

**Current active lesson: step05, print accelerometer Z using Serial.printf.** Prepared for your upload and test. No physical acceptance is recorded for steps01–05. Step06 remains planned; no calibration or control loop is included.

## After those small steps

Continue with separate commits: add accelerometerX; add accelerometerY; add gyroX; add gyroY; initialise BMP280; print pressure; print temperature. Each is its own lesson.

Then test the actual magnetometer identity, and later one LiDAR reading, using a suitable Arduino library for the identified device. Do not guess an HMC driver for a QMC chip. Do not add ROS2 or Xbox while learning these sensor calls.

### Calibration, one idea at a time

1. Average a small group of stationary gyroX readings.
2. Subtract that one bias and print the result.
3. Repeat for gyroY, then gyroZ, in separate steps.
4. Explain and add standard deviation for one axis.
5. Record one accelerometer orientation, then its opposite.
6. Calculate offset/scale for that one axis; test it.
7. Repeat the other axes individually.
8. Save one calibration value with Preferences; test reading it back.
9. Add reboot persistence, then a complete versioned record and corruption checks, separately.

Barometer baseline and LiDAR/reference-distance tests are later small lessons too. Magnetometer calibration follows identification and raw-data tests. Nothing here claims a sensor has already passed in reality.

### Flight software later

After understanding and testing sensors/calibration: Xbox connection; one stick; one mapping; one disconnect check. Then one angle estimate, one control loop, and one mixer calculation at a time. Introduce the Arduino libraries only when their lesson needs them.

The eventual standalone mode stays **yaw rate, pitch/roll angle, altitude hold plus vertical-speed command, with no XY braking**. ROS2 communication/navigation comes after the standalone fundamentals. No motor pulses or arming are added under the current instruction.

## If a step fails

If06 fails after07â€“10 were prepared, return to06's implementation checkpoint in a repair branch. Fix and test06. Old07â€“10 do not remain in the active build. Implement a fresh07, reusing old ideas only after reviewing them. Preserve old code, commits and failed evidence as history. Never force-push to hide attempts.

The existing gate/repair tools stay available behind the lessons. You do not have to read those tools to understand the first main.cpp. Earlier sensor diagnostics are archived in `archive/before-small-lessons/` and Git at `a47d4b5`.

## Simulation is yours too

You create the Gazebo worlds, models and simulation. I explain the terminal commands and help with one small problem at a time; I will not build a complete simulation unless you ask. Start independently with [opening ROS2, RViz and the built-in Gazebo world](start-ros-gazebo.md). This does not add simulation code to the18-line firmware lesson. Robot loading, ROS topics, bridges and FC integration are separate later lessons.
