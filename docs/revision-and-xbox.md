# Resume with your own tests - 30 October 2026

Current hardware, user confirmed: old ESP32-S3 PCB with MPU6050 only. The new PCB is in progress, not substituted or accepted. Windows inventory currently lists COM3, not COM4; port identity must be checked by the operator. No serial port was opened for this lesson preparation.

## Our next small lesson: Xbox input only

One small Arduino/C-style main.cpp uses the existing asukiaaa Xbox library. This is an isolated PlatformIO project; firmware/main.cpp and its local gyro-matrix edit are preserved. This firmware executes ON THE ESP32-S3. PC/MSVC tests were optional software checks, never deployment targets.

Build without flashing:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 build-xbox
```

When you choose to upload: remove propellers and disconnect ESC signal plugs for this input-only exercise. Close any Serial Monitor. Confirm the actual ESP32 port by unplugging/replugging its USB and checking PlatformIO Devices. Uploading replaces the currently running MPU firmware; its source/calibration constants remain saved in the project.

```powershell
.\qcopter.ps1 upload-xbox --upload-port COM3
.\qcopter.ps1 monitor-xbox --port COM3
```

Replace COM3 if your confirmed port is different. No upload was performed during preparation.

1. Turn on the Xbox Series X/S controller and hold its pairing button until the logo flashes rapidly. Disconnect it from the PC/console first; it must connect directly to the ESP32 over BLE.
2. Observe CONNECTED=1 and INPUT_READY=1. Move each stick, press each button, and move LT/RT. The library prints raw values; there is no throttle or flight interpretation yet.
3. Release every control: verify sticks return approximately to their centre and triggers to rest. Record observed ranges before choosing deadbands.
4. Turn the controller off: input must become disconnected/stale/not-ready. Turn it back on and inspect reconnection. A lack of fresh notifications is not necessarily a broken radio: an unchanged control may not generate reports; move a stick to check.
5. Keep a short serial log and your observations. Pairing, axis directions, ranges and reconnect remain PENDING until this test is done on your controller.

The 300 ms freshness threshold is only a bench display rule, not an accepted flight failsafe. BLE connect/reconnect code may block. Do not insert xbox.onLoop into the 1 kHz acquisition task. Later integration needs a separate slow task, a synchronized latest-command snapshot, measured latency and source-bound timing regression. No PWM, Servo, arming, PID, mixer or motor output is included.

Library: https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32 (MIT). Upstream reports testing controller firmware5.13.3143.0; other firmware compatibility must be verified, not assumed. Dependencies are pinned in the lesson platformio.ini. No controller firmware update is required automatically.

## Revision order: do one, inspect its evidence, then continue

Historical passed results stay preserved. These rows are fresh OPERATOR REVISION gates, all pending. We are not deleting commits, refitting calibration or declaring previous tests invalid merely because you did not run them personally.

| Revision | What you do | What we learn |
|---|---|---|
| R0 - current source | Review the local gyro_to_drone coefficient difference before uploading the MPU firmware | Avoid applying historical rotation proofs to changed coefficients |
| R1 - Xbox input | Do the five checks above | Controller pairing, raw controls and link behaviour on your ESP32 |
| R2 - restore MPU bench | Upload the reviewed MPU firmware; inspect identity, ranges and offset readback | Correct sensor and boot correction, no duplicate bias subtraction |
| R3 - stationary recording | Run one20s record-1khz capture | Sample integrity, corrected values, actual timing; independently level it only if testing level |
| R4 - known rotations | Positive/negative X/Y/Z movements with an angle reference | Physical gyro signs, frame and scale; do not use these for stationary bias fitting |
| R5 - replay yourself | Run the existing offline tests and saved-record analysis with explanations | Understand54software tests and19host cases; host timing is not MCU timing |
| R6 - small attitude update | Only after the frame gate: add the estimator to MCU and test it | Motion response, acceleration sensitivity and combined1kHz timing |
| R7 - control | Add one PID capability at a time and validate in your simulation | Rate yaw, angle roll/pitch; altitude waits for a real altitude sensor |

For R3, after the existing MPU source is reviewed/restored:

```powershell
.\qcopter.ps1 record-1khz --port COM3 --condition stationary_assumed --note "old PCB; MPU only; operator revision"
```

For R5, after reviewing the local gyro coefficient:

```powershell
.\qcopter.ps1 check-all --native
```

This command builds/runs software checks on the PC, never flashes the board. A failure stops our progression; repair that stage, preserve the failed evidence, then rebuild its dependent steps. No blanket re-calibration is needed if the sensor and mounting are unchanged.

## FIFO versus direct register reads

At1kHz the MPU produces one sample each1ms. Direct reads give the register contents at read time; a late ESP32 read can skip intermediate samples. A coherent burst read avoids mixing bytes but cannot reconstruct a skipped sample.

FIFO queues successive sensor samples. Our MPU packet is14bytes (acceleration, temperature, gyro); the1024-byte FIFO holds73complete such packets, about73ms at1kHz. This is theoretical storage, not an allowed flight delay: old samples are stale and overflow must be rejected. FIFO preserves acquisition history; it does not guarantee a fresh1kHz PID output. Sample rate, sensor filter delay, FIFO backlog, CPU execution time and motor update rate are separate measurements.

The PC was used to analyse logs and inject software faults. Actual firmware deployment remains ESP32-S3/Arduino/PlatformIO.

MPU FIFO reference: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf (sections7.16-7.17).
