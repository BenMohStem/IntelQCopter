# ESP32-S3 setup

## Hardware and scope

Target: esp32-s3-devkitc-1, following the user's Drone project.
Input: Xbox Series X/S BLE controller, not an Xbox 360 or proprietary Xbox Wireless receiver.
Pin choices are inherited assumptions, not a verified PCB wiring diagram.

| Signal | GPIO / address | Notes |
|---|---|---|
| I²C SDA / SCL | 9 / 8 | 3.3 V logic, suitable pull-ups; configured 400 kHz |
| MPU6050 | 0x68 | ±8g, ±500 degrees/s, DLPF 42 Hz, nominal 500 Hz sample rate |
| BMP280 | 0x76 or 0x77 | Factory compensation through Adafruit driver |
| FL / FR / RR / RL motor pins | 12 / 1 / 2 / 14 | Inputs with pulldowns; no ESC pulses |
| Optional status GPIO | 42 | Verify board/wiring; may not be the onboard RGB LED |

Sensor-to-body transform is identity until verified. Before future motor output,
check +x forward, +y left, +z up. Rotate gyro and accelerometer axes consistently.
HMC5883L, TOFSense and battery ADC/current sensing remain unconnected.
Companion UART software is implemented at 115200 baud on RX18/TX17; routing, wiring and electrical operation remain unvalidated.

## Build without upload

    pio pkg install
    pio run -e esp32-s3

This workstation also has PlatformIO at:

    C:\Users\Mohamed\.platformio\penv\Scripts\platformio.exe

Dependencies are pinned in platformio.ini. The downloaded Xbox library gets one
guarded raw-report callback in its local .pio cache via tools/platformio/patch_xbox.py.
The library handles pairing; our observer queues the complete 16-byte report with its receive timestamp.
The C control task parses it and owns gesture/authority state. It does not poll the library's
mutable public input fields across cores. The original Drone library cache is unchanged.

The build produces .pio/build/esp32-s3/firmware.bin. Compilation is not evidence
of pairing, axes, calibration accuracy or real-time deadlines. No upload is performed.

## Pairing and console, once hardware exists

1. Power from suitable USB/bench supply with propellers removed.
2. Open a 115200-baud serial console.
3. Put the Series X/S controller into Bluetooth pairing mode.
4. Observe pilot freshness and connection status.
5. Verify each stick direction, center and trigger before further integration.

No hardware/controller is available for these steps. Pairing depends on controller firmware.
The 250 ms report-age limit is provisional; measure idle/moving report rates and dropouts.
Stale/disconnected input invalidates the mathematical command. No automatic landing is connected.

## Calibration console

| Key | Meaning |
|---|---|
| C | Help; periodic FC lines contain status |
| R | Clear fits; queue an empty NVS record; repeat gyro capture |
| S | Queue a save; allow another batch of retries |
| 0 / 1 | Capture specific force along +X / −X |
| 2 / 3 | Capture specific force along +Y / −Y |
| 4 / 5 | Capture specific force along +Z / −Z |

Place the board stationary in the requested pose before sending its digit.
Each face captures 200 fresh accepted samples. All six must pass geometry/noise checks;
accumulated face count alone does not mean the fit passed. Repeat invalid faces.
Gyro verification needs 500 accepted stationary samples and refreshes missing/changed bias.
A stationary fixture is necessary: these heuristics cannot establish zero motion independently.

The 160-byte record uses Preferences/NVS namespace intelqcopter, key imu-v1:
magic, schema, sensor profile, eFuse identity, flags, gyro count,
seven three-axis coefficient/variance vectors, reserved bytes and CRC32.
No raw C struct is persisted. This is NVS, not the RP2350 two-sector journal.
The slow task saves no faster than every five seconds, with readback and three retries.
No actual calibration has been captured or marked valid yet.
Ground barometric pressure is measured from 100 readings each session, not persisted.

## Timing and estimator limits

A timer requests 500 updates/s. The control task runs on core 0. Core 1 has separate sensor/UART/NVS and lower-priority Bluetooth service tasks; blocking BLE reconnect calls are isolated from periodic pressure acquisition. Measured dt and pending notifications reveal delays.
I²C, BLE scheduling, flash and driver timeouts can still cause misses.
NVS writes can pause both cores. They are allowed here only while physical arming
is permanently disabled; a future arming path must enforce no writes during flight.

Gravity removal uses the previous roll/pitch estimate. Independent angle filters retain
the approximate body-rate/Euler-rate assumption from main; this is not a quaternion estimator.
BMP280 filtering correlates samples. Sensor noise, delay, temperature, mounting,
motor response and execution time need physical measurement.

## Companion and controller modes

See [Xbox and UART protocol](xbox-ros2-protocol.md). Base ROS2 packages were not installed or modified in this task. Pi/Jetson hardware has not been selected or exercised.
