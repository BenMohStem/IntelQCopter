# Two-core Xbox test menu — 30 October 2026

Engine: actual ESP32-S3 and MPU6050, GPIO9 SDA / GPIO8 SCL, 400 kHz I²C. Separate motor-disabled measurement firmware; **no PID or actuator control**. Production `firmware/main.cpp` and calibration were not changed.

## Actual checks

PlatformIO build and upload succeeded. Source SHA256: `48678d2b730e9a9e0a3d39f49a07f62159f175f6dc98a6a827ff38f84136603b`.

Device reported `CORE,acquisition=1,slow=0,heap=150276,min_heap=149600,PID=NOT_IMPLEMENTED`.

| Test | Samples | Read rate (Hz) | Median service (µs) | Maximum read gap (µs) |
|---|---:|---:|---:|---:|
| d | 4002 | 1000.49 | 603 | 1202 |
| f | 4003 | 1000.60 | 780 | 1111 |
| D | 3925 | 981.18 | 603 | 3649 |
| F | 4003 | 1000.66 | 780 | 3600 |

All four captures passed record sequence, timestamp order, raw count and payload checksum checks. These are four-second observations, one trial per condition, not a hard real-time guarantee. `D`/`F` deliberately pause acquisition 2.5 ms every 100 ms; these measure backlog behavior, not flight behavior. No overflow occurred in these runs. Normal conditions must not be pooled with deliberately paused conditions.

**Xbox was disconnected throughout.** BLE scanning ran, but connected notification traffic, physical Menu/A/B/Y actions, reconnect behavior and cancellation still require operator testing. Serial requests used the same queue as Xbox selections; that does not prove the physical gamepad path.

Original serial log, per-run raw CSVs and installation logs remain privately preserved under `artifacts/acquisition-comparison/`. `results.json` contains original-log SHA256 and statistics. The old full-flash backup is private and must never be published. Upload log verified written flash hashes. The board now runs this menu harness, not the earlier benchmark or production FC.

## How to use

Close Serial Monitor and other COM3 users. From `QCopter-Stepwise`:

```powershell
.\qcopter.ps1 record-xbox-menu --port COM3
```

Turn on the Xbox Series X/S controller and use pairing mode if needed. Observe `CONNECTED=1,FRESH=1` before testing buttons. Menu (≡) opens the selection; each further press advances. Release between presses. A runs the selected test; B requests cancellation and closes the menu; Y prints information. The recorder saves every completed run automatically and reports each 1000 received samples. Ctrl+C stops recording and requests cancellation. This is a bench menu, never a flight mode or motor control.

Four choices: direct normal (`d`), FIFO normal (`f`), direct with injected pauses (`D`), FIFO with injected pauses (`F`). PC-only Python/host tests are not executed on the ESP32.

```powershell
# Automatically request all four tests through the slow-core queue:
.\qcopter.ps1 record-xbox-menu --port COM3 --run-all
# Build only; does not upload:
.\qcopter.ps1 build-xbox-menu
```

## Core ownership

```text
core 0: Xbox/BLE → button edges → request queue → core 1: sensor acquisition
core 0: chunked serial dump ← completion queue ← core 1: capture buffer
                     buffer semaphore prevents overlapping captures
```

Only core 1 touches sensor I²C after initialization. Only core 0 handles Xbox and normal serial output. No serial dump occurs inside timed acquisition. A bounded queue and semaphore prevent overlapping tests; cancellation uses an event bit. Capture runs for four seconds and then dumps buffered samples. This is not a continuous flight-control scheduler. Splitting tasks cannot remove shared radio interrupts, memory contention or I²C time; a future 1 kHz PID still needs end-to-end deadline measurements under connected Xbox load.

## Sensor choices explained

FIFO overflow loses **oldest data** while new data continues entering. Reads return oldest retained bytes, not automatically the freshest sample. After overflow, reject the interval and reset/reframe FIFO before using packets. Manufacturer reference: [MPU6050 register map, FIFO_R_W](https://naylampmechatronics.com/img/cms/000033/MPU-6000-Register-Map1.pdf).

±4 g is usable for gentle flight, but gravity already contributes about 1 g at rest. Vibration, impacts and maneuvers can clip an axis. ±250°/s is a rotation-rate limit, not a tilt-angle limit: 90° in 0.25 s averages 360°/s and exceeds it. Smaller ranges improve quantization, not necessarily accuracy. We retain ±8 g and ±500°/s until representative motion/vibration data justify smaller ranges.

94/184 Hz describe filter bandwidth, not measurement range or CPU speed:

| Adafruit setting | Accel bandwidth / delay | Gyro bandwidth / delay |
|---|---|---|
| 184 Hz | 184 Hz / 2.0 ms | 188 Hz / 1.9 ms |
| 94 Hz | 94 Hz / 3.0 ms | 98 Hz / 2.8 ms |

Both can output at 1 kHz with divider 0. Filtering trades noise rejection against delay; the delay is not CPU execution time. This comparison retains 260 Hz mode and divider 7, matching the previous benchmark. Changing range/filter requires reviewing scaling and Kalman noise assumptions; no new flight setting is accepted here.

Clock source controls the MPU's internal sample timing. The power-up internal 8 MHz oscillator is one option; a phase-locked loop (PLL) can instead use X/Y/Z gyro references or supported external references. Current `MPU6050_PLL_GYROX` locks timing to the X-gyro reference; it does not use measured rotation as a clock and all axes still work. It is independent of the ESP32 CPU clock. Clock-stop is not a sampling mode. Sensor timing affects `angle += rate × dt`; PLL selection does not itself guarantee an exact measured 1 ms interval. See the manufacturer register map's `PWR_MGMT_1`, `SMPLRT_DIV` and `CONFIG` sections.

Additional checks: serial cancellation produced START,d then DONE,d,USER_ABORT on the actual ESP32. This verifies the cancellation event path, not the physical Xbox B button. Eight existing synthetic analysis checks passed. Knowledge domain integrity passed; the production firmware snapshot remains STALE because of its pre-existing user edit, deliberately preserved. No acceptance fingerprint was refreshed.

