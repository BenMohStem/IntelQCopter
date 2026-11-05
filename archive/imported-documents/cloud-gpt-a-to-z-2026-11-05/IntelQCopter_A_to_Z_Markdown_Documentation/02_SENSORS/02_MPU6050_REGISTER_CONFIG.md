# MPU6050 — register-level acquisition and filtering

## Known silicon and bus

MPU6050 at I²C `0x68`, with `0x69` possible when AD0 is strapped high. Direct six-axis accelerometer/gyro. The FC v2 uses primary ESP32 Wire bus and an aux bypass for BMP280 access; there is no requirement for a physical MPU INT wire.

## FC v2 normal configuration

| Property | Configured | Meaning |
|---|---|---|
| DLPF_CFG | `1` | Gyro nominal 188 Hz bandwidth/~1.9 ms delay; accelerometer nominal 184 Hz/~2.0 ms |
| Sample rate divider | `SMPLRT_DIV=1` | 1 kHz base / (1+1) = 500 Hz |
| Gyro full-scale | ±250°/s | November calibration range; saturation possible during fast tumbles |
| Accel full-scale | ±2 g | November calibration range |
| FIFO | 12-byte accel XYZ + gyro XYZ | Temperature not in each FIFO frame |
| AUX master | disabled | Allows BMP XDA/XCL bypass |
| `INT_PIN_CFG=0x02` | bypass bit | Not an external interrupt pin requirement |

## Scaling and consistency

At ±250°/s the nominal gyro conversion is 131 LSB/(°/s); at ±2g accelerometer nominal sensitivity is 16384 LSB/g. Verify actual register readback before applying conversions. If ranges are changed, the raw-to-SI conversion changes, and calibration biases must be re-expressed consistently; no silently reused counts-to-degrees factors.

## FIFO and latency

FIFO buffering protects against short host delays but introduces measurement age when backlog accumulates. The v2 source limits burst drains, uses newest complete frame, counts discarded frames, detects misalignment and overflow, and resets FIFO when needed. A FIFO backlog is not free precision: repeatedly integrating a stale frame as though it came from the current clock instant produces timing errors. Report FIFO count, `fifo_lost`, `fifo_over`, read duration and effective sample age.

## Boot hardware offsets

Gyro offset register addresses 0x13–0x18 and accelerometer offset area 0x06–0x0B are used by v2. The code must read original values, preserve reserved/factory temperature-compensation bits, write bounded corrections, read them back, then measure **residual bias**; theoretical offset LSB scaling should not substitute for before/after board measurements.

## Research reference

TDK/InvenSense MPU-6000/6050 Register Map Rev 4.2: <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf>.
