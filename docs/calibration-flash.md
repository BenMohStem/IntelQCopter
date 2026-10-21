# Calibration: capture → validate → save → verify after boot

Implemented in the single runtime file, [flight_controller.c](../firmware/flight_controller/flight_controller.c).
The Pico2/RP2350 target compiles this storage layer; the desktop library excludes physical flash operations.
**No RP2350 has been flashed or used to verify persistence yet.** Physical sensor acquisition is unfinished.

## Boot sequence

1. Read both reserved flash sectors.
2. Reject incomplete records, wrong schema, or CRC/commit mismatches.
3. Select the newest complete generation, including generation counter wrap.
4. Check board identity, sensor profile and coefficient ranges.
5. Load valid measured fits into RAM. Missing/incompatible data require recalibration.
6. Verify saved gyro bias using at least **500 accepted fresh stationary samples** while disarmed.
7. Recalculate the gyro fit when missing or when any axis changes beyond

   $$|\hat b_{\rm new}-\hat b_{\rm saved}|>
   \max\left(0.05,\;5\sqrt{P_{\rm new}+P_{\rm saved}}\right)\quad[^\circ/s].$$

8. Apply IMU calibration only when gyro verification and both accelerometer/gyro fits exist.

The threshold is provisional. Capture rejection restarts the stationary window.
At 500 fresh samples/s, 500 samples take one second; actual driver rate determines duration.
Accelerometer scale needs six-face capture, not just a level pose.
The empty sensor stub never creates fake calibration observations.

## What persists

| Persisted | Measured again each session / kept volatile |
|---|---|
| Accelerometer offset, gain and fit variances | Ground barometric pressure / height zero |
| Gyro bias, sample noise, bias variance and count | PID integral states |
| Magnetometer offset and diagonal scale | Live SOC / starting battery charge |
| Voltage, current and lidar linear fits / residual sigma | Current verification window |

Loading magnetic or linear fits does not connect absent drivers or add magnetic fusion.
Noise estimates are retained but are **not automatically substituted into all Kalman noise settings**.

## Flash layout and interrupted writes

Two 4096-byte sectors at the end of physical flash are reserved by the linker.
For the current **4 MiB Pico2** configuration:

| Slot | Flash offset | XIP address |
|---|---:|---:|
| A | 0x003FE000 | 0x103FE000 |
| B | 0x003FF000 | 0x103FF000 |

| Offset within sector | Length | Data |
|---|---:|---|
| 0 | 32 bytes | Magic, schema, generation, profile, board UID, mask, gyro count |
| 32 | 108 bytes | Nine 3-axis binary32 vectors |
| 140 | 72 bytes | Three binary64 linear-fit triples |
| 212 | 296 bytes | Reserved, initialized to 0xFF |
| 508 | 4 bytes | CRC32 over bytes 0–507 |
| 512 | 256 bytes | Final commit page: magic, generation, CRC, inverted CRC |

Serialization explicitly uses little endian instead of compiler-dependent structs.
Erase the inactive sector, program data, then program the commit page last.
Keep the previous sector intact during that update and check readback.
This supports recovery from ordinary interrupted writes; hardware brownout/endurance behavior remains unverified.
CRC detects corruption; it is not authentication.

A newer incomplete record permits recovery from an older committed record.
A complete newer record with an incompatible profile requires recapture instead of reuse of an old configuration.
Clearing writes a newer empty record so normal reboot does not resurrect the old fit.

## Save conditions and endurance

- Only core 0 saves, while the caller is disarmed.
- Core 1 joins Pico SDK flash lockout before writes are allowed.
- A RAM callback uses flash_safe_execute.
- Linker exclusion plus a runtime overlap check protect firmware.
- Identical fits skip erasure.
- Automatic saves are spaced at least five seconds apart; after three failures they stop until manual retry.

PICO_FLASH_SIZE_BYTES must describe the actual chip. The linker separately shortens the firmware region by 8192 bytes.
The current main remains physically disarmed. Future arming code must pass actual state to these APIs.

## USB console

| Key | Effect |
|---|---|
| C | Print store status, fit mask, verification and unsaved changes |
| R | Clear RAM fits, persist an empty record, request recapture |
| S | Save measured fits; allow retry after automatic failures |

Status: 0 empty, 1 loaded, 2 recalibration required, 3 I/O error.
Mask: accelerometer 1, gyro 2, magnetic 4, voltage 8, current 16, lidar 32.
These commands do not generate observations. Guided six-face/magnetic capture UI remains unfinished.

Recalibrate after changing a sensor, mounting, units, range or digital filtering;
bump FC_CAL_PROFILE_ID for incompatible configuration changes.
Board UID does not identify a replacement sensor on the same board.
Full-chip erase loses calibration; firmware updates must preserve the reserved sectors.

Reference: [Pico SDK flash APIs](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html).
