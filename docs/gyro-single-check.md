# Twenty-second gyro rotation check

New dedicated `r` mode retains all three RAW gyro axes at nominal 1 kHz for 20 seconds. It uses a compact 20200-row buffer shared with the old timing buffer, rather than allocating a too-large full timing buffer. No accelerometer or temperature is retained in this mode. No calibration, PID or motor changes. The old d/f/D/F timing tests stay four seconds.

Three earlier operator runs have uncertain +X/-X labels. Originals remain untouched; no physical sign validation is claimed. See uncertain-runs.json. Do not infer the intended physical label from the same gyro output being tested.

```powershell
.\qcopter.ps1 record-xbox-menu --port COM3 --single r --note "rotation about marked drone X; physical direction described after run; approx 90 deg"
```

Mark the existing drone +X direction with tape. Right thumb along +X, curled fingers give positive rotation ABOUT X (not raising the X end). If unsure, use a literal description of which side rises and include a photo; leave sign unknown instead of guessing.

Press Enter ready. At `Recording test: r`: hold still for about 3 seconds; rotate once gently about X over 3–5 seconds; hold the new position until 20 seconds ends. Do not rotate back during capture. Wait for Saved test and automatic exit before returning the drone. No Xbox button needed. One run first; sign and opposite direction can be checked after reviewing the physical description. Approximate hand angle is not precision gyro scale evidence.

Data saves afterward; every 1000 saved samples is reported during transfer. Sensor measurement times are nominal 1 ms apart; saved read_gap_us is retrieval timing, not exact measurement timing. Overflow, excessive gap, I2C errors, full buffer and cancellation reject the run. Checksum covers raw gyro words; count, sequence, duration and timestamp accumulation are also checked. No model-derived physical pass.
