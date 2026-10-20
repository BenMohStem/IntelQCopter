# Current status — ESP32/Xbox

## Confirmed

- ESP32-S3 aircraft, Xbox Series X/S input; no custom RP2040 handheld here.
- C control/estimation/calibration, minimal C++ boundary for libraries.
- Roll/pitch angle, yaw rate, centered-stick height hold and vertical-speed commands.
- RT boost. Warn at 35% estimated SOC; latched disarm at 20%; no automatic landing.
- Physical arming/output remains disabled.

## Source implementation

Xbox reports → synchronized pilot snapshot → 500 Hz target C task →
six loops/estimators → normalized motor demands → telemetry only.
MPU/BMP acquisition and versioned NVS calibration hooks are implemented.
No hardware has exercised them.

The ESP32-S3 target compiled successfully. See ../artifacts/esp32-build/ for
original build output and source/binary hashes. This is build evidence, not
a measured timing or Bluetooth result.

## Missing, in order

1. Exact-board wiring, mounted axes, I²C integrity, controller firmware and pairing verification.
2. Measured report rate, idle behavior, latency/range/dropouts and control timing.
3. Actual calibration captures and reboot/power-loss persistence checks.
4. HMC5883L, lidar and voltage/current ADC integration; usable capacity/OCV identification.
5. Complete mass/COM/inertia, thrust, motor delay and reaction torque measurements.
6. ESC protocol/wiring and an explicitly authorized future arming/output design.
7. ESP32-specific host/simulation adapter; no ESP32 flight recording exists.
8. Companion transport/authority, localization, camera distance geometry and real-scene perception.

The 1.2 kg flight baseline and 0.740 kg component URDF are distinct assumptions,
not measured aircraft mass. The battery's approximate 150 g remains unverified.

## Archive policy

Current summaries supersede scattered historical status claims.
Prior reports, source, failed trials, videos, PCB drafts and math remain in
archive/rp2350/. Original launchers assume the old root layout: use main for that workflow.
Historical evidence does not establish ESP32 pairing, deadlines or flight.
