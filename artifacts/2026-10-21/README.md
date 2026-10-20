# Original results — 2026-10-21

| Evidence | Result and scope |
|---|---|
| hardware-build.log | Actual ESP32-S3 hardware target compiles: 31,732 bytes static RAM, 588,741 program flash |
| ../emulation/verified-replay/uart.log | Espressif QEMU ESP32-S3 executes 4,982 logical steps; 19 checks, zero failures |
| ../emulation/verified-replay/summary.json | Actual emulator command, checks, source and binary hashes |
| ../emulation/verified-replay/telemetry.csv | Original numeric UART rows from synthetic input replay |
| ../emulation/attempt-1/uart.log | Retained failed startup; flash-configuration mismatch fixed in the emulator build |
| ../emulation/attempt-2/uart.log | First successful replay, before estimator reset correction |
| ../emulation/builds/ | Original compiler output for each emulator revision |
| knowledge-checks.log | 9 isolated validation/failure-containment checks pass |
| math-render.log | 322 LaTeX expressions rendered with strict parsing, including 113 displayed equations |
| lti-calculation.log | Assumed-parameter hover calculations and continuous/sampled closed-loop poles |
| manifest.json | Source/document/binary SHA-256 fingerprints for the final work |

The controller's `fc_init()` now resets angle/altitude estimator state as well as
calibration/session/controller state. The final replay checks that explicit reset.
The math implementation remains in one C file, organized into numbered sections.

QEMU inputs are synthetic. The hardware adapter (BLE, I²C, scheduling and NVS)
is compiled but is not exercised by this replay. No actual sensor calibration,
pairing, ESC actuation, real-time deadline, new Gazebo flight or physical flight
is established by these results.

The guide's response graphs are independent numerical LTI calculations with ideal
state feedback. They are not QEMU or Gazebo output. Motor/inertia/noise values
remain assumptions. The original nonlinear papers and prior simulator evidence
are retained in archive/ with their historical scope.

Diagrams and plot PNGs were inspected visually. The HTML equations were strictly
parsed and local links/anchors checked; no browser rendering check is claimed.
