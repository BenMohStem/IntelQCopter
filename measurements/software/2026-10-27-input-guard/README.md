# Small bench input-guard repair — 27 October 2026

The current carrier uses ESC signal GPIOs **4,1,14,21**. Setup previously pulled
only the legacy **12,1,2,14** set down. Two added calls keep GPIO4 and21 in
`INPUT_PULLDOWN` too. All six pins remain inputs; there is no PWM/ESC driver.
This changes startup only. No sensor/calibration, FIFO processing, protocol,
gyro frame, filter or PID logic changed. Runtime remains one `firmware/main.cpp`.

## Actual results

- **54 Python checks pass**: protocol/calibration/arithmetic/offline filter tests.
- **19 host fault cases pass**, compiling actual main.cpp with MSVC/fake Arduino,
  FIFO/I2C/serial and cooperative task scheduling. Every boot path now asserts
  the exact six unique GPIOs and INPUT_PULLDOWN modes, including init failures.
- **ESP32-S3 PlatformIO build passes** with the matching source hash embedded:
  19,504 bytes RAM; 312,901 bytes flash; inputs unchanged during build.
- **No serial device detected** by PlatformIO or Windows serial inventory.
  No upload, new sensor recording or hardware timing test was attempted.

Source SHA256: `f26d314eb637b9ccac15ee0a62376f722900d8affbbf0f8c66d20df1b2235422`.
Firmware binary SHA256: `7db4970626319e6151159ff70dc54f5f8f4b41c3c92ef9a6840bcdfb5833a838`.

[Machine-readable results](summary.json) · [Original compiler output](compiler-original.log) ·
[Original Python test output](offline-tests-original.log) · [Host case summary](native-summary.json)

Original synthetic streams remain in `artifacts/native-fifo-tests/20261026T234046007357Z`;
real historical recordings and failed trials were not altered or refitted.
The last real timing evidence belongs to the prior source SHA91947b5c…,
not this unflashed build. Synthetic host clocks cannot validate 1kHz MCU timing.
Physical gyro rotations and barometer availability still gate later estimator/PID
acceptance. Motor arming remains disabled. No telemetry queue bug was established,
so that tested code was left unchanged.
