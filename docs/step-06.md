# Step 06 - read gyroscope Z

## One small addition

Your accelerometer XYZ corrections and printf style are preserved. Inside the existing initialized-sensor block, add:

```cpp
        sensors_event_t rotation;
        mpu.getGyroSensor()->getEvent(&rotation);
        Serial.printf("Gyro Z = %.3f rad/s\n", rotation.gyro.z);
```

1. `rotation` is a new record for gyroscope data.
2. `getGyroSensor()` obtains the library's gyro interface. `->getEvent(&rotation)` fills the record using its address.
3. `rotation.gyro.z` selects angular speed around the sensor's Z axis. `%.3f` prints three decimal places; `\n` starts the next line.

The unit is radians per second, not degrees and not yaw angle. One full turn is 2*pi radians, so 1 rad/s is about 57.3 degrees/s. A stationary gyro should be close to zero even if the sensor is tilted. A small stationary offset is normal and will need its own later calibration. Your accelerometer offsets must not be applied to the gyro.

## Your test

Upload yourself, open Monitor at 115200 baud, and keep propellers removed. With the sensor still, look for a value near zero. Then rotate it gently and continuously around its marked Z axis for several seconds. One direction should give one sign; reversing should reverse the sign. Stop and confirm it returns near its stationary value. Motion during a single instant can be missed by the current one-second reporting interval.

Expected illustrative output:

```text
Gyro Z = 0.012 rad/s
```

This is not a required bias or measured evidence. Save your actual stationary and moving readings. The sensor-axis sign is not yet a verified drone/Xbox yaw convention. This lesson does not integrate an angle, estimate attitude, command motors or run a flight-rate loop.

The separate accelerometer and gyro calls currently obtain separate samples. That is acceptable for this slow inspection lesson, but they are not a synchronized flight-control sample. The installed library's getEvent return does not provide dependable runtime disconnect detection; no continuous health guarantee is claimed.

## Your accelerometer calibration snapshot

The user reported calibration of X/Y/Z complete. The source now subtracts these user-provided values (m/s^2):

| Axis | Subtracted offset |
|---|---:|
| X | 3.350011 |
| Y | -0.411099 |
| Z | -0.716789 |

These numbers and the original edits are preserved in a separate commit. No calibration log or method has yet been supplied for review. A stationary single-pose average contains both bias and the gravity projection. A general accelerometer bias should not cancel gravity across every orientation: at rest the corrected magnitude should remain about 9.81 m/s^2. In particular, a large X correction can represent tilt rather than sensor bias. Review how these offsets were obtained before using them in attitude estimation; do not silently treat them as verified calibration.

## Status

Prepared only. No build, upload or physical gyro test performed by the assistant. Motor outputs remain disabled. Mass/inertia parameters remain reserved for the later standalone-FC Gazebo phase; no gains are calculated here.
