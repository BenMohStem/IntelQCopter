# Step 05 - one accelerometer reading

## Goal

Print just the accelerometer Z value, once per second. Keep the user's Serial.printf output style. No gyro output, calibration, altitude calculation or PID yet.

## One small scope change

Move `bool mpu_started = false;` from inside setup to just below `Adafruit_MPU6050 mpu;`. This makes it available in both setup and loop. Do not leave a second declaration inside setup, because that would hide the shared variable. Setup still sets it using mpu.begin.

## The new block in loop

```cpp
    if (mpu_started)
    {
        sensors_event_t acceleration;
        mpu.getAccelerometerSensor()->getEvent(&acceleration);
        Serial.printf("Acceleration Z = %.3f m/s^2\n", acceleration.acceleration.z);
    }
```

1. `if (mpu_started)` prevents a sensor read if initialization failed. The uptime message still runs.
2. `sensors_event_t acceleration;` creates a record where the library can put one accelerometer event. The Arduino sensor library defines this record type.
3. `mpu.getAccelerometerSensor()` obtains the accelerometer interface. `->getEvent(...)` calls its read function. `&acceleration` passes the address of our record so the function can fill it.
4. `acceleration.acceleration.z` selects the Z field inside that event's acceleration fields. The repeated word comes from our variable name and the library's field name.
5. `%.3f` prints the value with three digits after the decimal point. `\n` ends the line. Extra decimal digits do not establish sensor accuracy.

The library internally reads a complete MPU data block; our programme uses and prints only acceleration Z. Its installed getEvent implementation returns true unconditionally, so it does not provide a dependable runtime disconnected-sensor check. The initialization guard is not continuous sensor health monitoring. Do not use these readings for flight yet.

## What Z means

This is the sensor's local Z axis, not necessarily the drone's vertical axis until mounting orientation is established. Its unit is m/s^2. An accelerometer at rest measures support against gravity: with +Z upward, expect about +9.81 m/s^2; inverted, about -9.81 m/s^2; with Z horizontal, about zero. Offsets, mounting error, motion and noise affect the result.

It is not altitude or vertical speed. Simply subtracting 9.81 while the sensor is tilted is not a complete gravity correction. Those calculations belong to later attitude and altitude lessons.

## Your check

Proceed only after the earlier initialization test works. With propellers removed, upload yourself, then open Monitor at 115200 baud. Press RESET/EN if necessary to see the startup messages.

The startup messages now use the user's printf wording:

```text
I2C controller started : Yes.
MPU6050 initialized : Yes.
```

Hold the sensor still with its marked +Z upward. You should get repeating lines similar to:

```text
ESP32 is running, uptime = 1500 ms
Acceleration Z = 9.810 m/s^2
```

This output is illustrative, not measured. Gently turn the unpowered-motor assembly or sensor, let it settle, then compare +Z up, sideways, and +Z down. Wait for a fresh one-second message in each pose. Do not disconnect a powered sensor. Preserve your actual output, including failures. If initialization says No, no acceleration line should appear; stop and diagnose the earlier lesson.

## Status

Prepared only. No build, flash or hardware test performed by the assistant. No physical pass is claimed. The important_sources entry points to the same active source, not a duplicate. Library API and unconditional getEvent return were inspected in the installed Adafruit MPU6050 2.2.9 source.
