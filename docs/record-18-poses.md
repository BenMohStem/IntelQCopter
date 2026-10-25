# Record the 18 one-minute calibration runs

This is a temporary measurement version of **firmware/main.cpp**, using Arduino, Wire and Adafruit MPU6050. It has no motor driver and applies none of our previous software corrections or user offset writes. The post-reset offset registers are recorded so their actual baseline is known. Previous calibration code remains in Git at a1d3c78.

## Flash once, then run the recorder

1. Upload the `bench` environment yourself from PlatformIO, as usual. No automatic upload is performed by the recorder.
2. Close PlatformIO Serial Monitor (and any other program using COM3).
3. Open PowerShell and run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-poses --port COM3
```

Use the board's actual COM port if it changed. You do not need to edit the pose in main.cpp or reflash for each run. Keep the PC connected and awake throughout each recording.

The recorder asks you to place the drone in each position. Keep the MPU rigidly mounted. The label means **that DRONE axis points upward**, not necessarily the matching axis printed on the MPU. Use a stable support/reference rather than holding the drone in your hand. Propellers remain removed and motor outputs disabled.

After pressing Enter there are five seconds for settling, followed by 60,000 samples: nominally one minute at 1000 Hz. Five seconds is just a settling pause, not a guarantee of thermal stability; let the powered sensor temperature settle before starting the session. No automatic next pose begins until you press Enter again.

| Repetition | Pose order |
|---|---|
| 1 | Z+, Z-, X+, X-, Y+, Y- |
| 2 | Z+, Z-, X+, X-, Y+, Y- |
| 3 | Z+, Z-, X+, X-, Y+, Y- |

Reposition for each repetition. Total measurement time is about 18 minutes, plus repositioning and settling. All 18 runs contain **1,080,000 samples** in total.

## Automatic saving

Everything goes into a new local folder under `artifacts/pose-recordings/`. It is excluded from Git so original console captures are not published automatically.

Each run has a uniquely named attempt folder containing:

- `serial.log`: original individual samples, configuration, offset readbacks, firmware source SHA and final averages.
- `summary.json`: count, mean and sample standard deviation for every channel, measured average rate, checksum and capture status.

The session's `session.json` tracks completed poses. Do not move/delete individual files or alter the manifest. The terminal prints the exact resume command. After Ctrl+C or a failed run, rerun with that `--resume` folder: completed poses are skipped, the incomplete pose gets a new attempt folder, and the failed attempt remains preserved. A source change or different board/configuration/register baseline requires a separate session.

All data are sensor axes, before our calibration. Physical summaries use accel m/s², gyro rad/s and temperature degrees Celsius. Means are over all 60,000 samples. Standard deviation describes spread, not calibration accuracy or uncertainty of the mean; correlated samples are handled later during analysis. Keep the complete folder, not just the averages.

## What makes it a 1000 Hz capture

MPU configuration is ±8 g, ±500 degrees/s, DLPF mode 0 (260 Hz accel / 256 Hz gyro), divider 7. Its 8 kHz gyro base divided by 8 yields a nominal 1 kHz FIFO rate; the accelerometer updates at 1 kHz. The FIFO supplies complete samples without using a CPU delay to imitate sampling. The board reads its FIFO over 400 kHz I²C and sends compact integer readings at 921600 baud. Old 115200-baud monitoring is unsuitable for this stream.

Every `S` line is `S,index,ax,ay,az,temp,gx,gy,gz`, in signed raw counts. The index runs from 0 to 59999. Accel scaling is 4096 counts/g, gyro scaling 65.5 counts/(degree/s), temperature conversion counts/340 + 36.53. These are nominal datasheet conversions, not measured calibration coefficients. Time within the capture is inferred from the FIFO sequence; there is no independently timestamped instant for every sample.

The board stops on I²C failure, FIFO overflow/high backlog, timeout or an abort. The PC checks sequence, count, payload checksum, firmware averages and elapsed time. A rate outside 970-1030 Hz is rejected as a transport/timing problem; this 3% gate is not an accuracy specification. Identical successive integer readings are valid and are not discarded as duplicates. Successful capture means the checks passed, **not that the calibration or flight controller is validated**. Actual board/USB/cable throughput still needs your real first run.

If a run fails, preserve it and send its error; do not quietly lower the rate or treat partial data as 60,000 samples. The existing low-rate `capture`/`imu-noise` commands and historical `check-accel` are not the tools for this new protocol. Use `record-poses`.

## After the 18 runs

Send the printed session folder path. We will assess movement, temperature drift, repeatability, uncertainty and pose-reference quality before fitting. Initially reserve the third repetition to test a fit from repetitions 1-2. Later, independently measured tilted poses are still required to validate performance between the six axis orientations. No new calibration is calculated or installed automatically by this recorder.

For following main.cpp: `setup` configures the sensor; `report_configuration` identifies it; `start_recording` clears the FIFO; `read_one_sample` saves one packet and its running sums; `loop` handles the recorder's commands. The PC tool handles pose names and disk files so this logic does not clutter the microcontroller.

Technical source: [TDK MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), sample divider, FIFO enable/count/data and sensor-output registers. This is real-hardware capture preparation, not simulation. No firmware was uploaded by the assistant.

## Checks completed before your upload

PlatformIO ESP32-S3 compilation passed (19,412 bytes RAM; 307,905 bytes flash). The built binary contains the exact main.cpp SHA. Six Python synthetic-protocol tests passed, including a complete 60,000-sample transcript and rejection of loss, overflow, bad timing/checksum and wrong source/baseline. These tests use generated data, not a real sensor or an ESP32 emulator. See [verification record](../measurements/mpu6050/2026-10-25-pose-recorder-preparation/verification.json). Original compiler output remains local. Hardware collection has not begun.
