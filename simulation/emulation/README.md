# ESP32-S3 CPU emulation

This is the official **Espressif QEMU ESP32-S3 machine**, running an Xtensa build
of the active `firmware/src/flight_controller.c`. The harness replaces the hardware
adapter with deterministic inputs. It boots through the emulated ROM and a real
Arduino/ESP-IDF bootloader. No real board is connected or flashed.

## Run it yourself on Windows

Open PowerShell in this branch's checkout:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-ESP32-Xbox
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $py simulation/emulation/setup.py
& $py simulation/emulation/build.py
& $py simulation/emulation/run.py --name my-first-s3-run
```

`setup.py` downloads a pinned 36 MB archive only if absent, verifies its SHA-256,
and extracts into ignored `build/qemu/`. This local install already exists.
`build.py` uses the installed PlatformIO environment and creates an 8 MB merged
flash image. `run.py` prints **live original UART output** and stores each run in
`artifacts/emulation/<name>/`. Pick a new name each time: existing evidence is never overwritten.

Expected final marker for the current scenario:

```text
QCOPTER_EMU_END,checks=39,failures=0,ticks=4982
```

The baseline uses 4,982 logical 2 ms control steps. Additional gesture tests use a separate synthetic report timeline with explicit five/two-second holds. QEMU may
execute that in a different amount of wall time. This does not measure a real
500 Hz deadline. The runner stops QEMU after the end marker, or reports failure
if the marker does not arrive before its timeout.

## What the checks exercise

- Stationary gyro verification, pressure reference and six-face accelerometer fitting.
- Calibration record creation/load, CRC corruption rejection and wrong-board rejection.
- Fresh boot gyro verification and full estimator-state reset.
- Roll mixer sign response to a commanded angle with fixed synthetic sensors.
- Stale pilot report, invalid IMU and invalid pressure handling.
- Warning at 35%, disarm at 20%, and a disarm latch that remains set after SOC rises.

Inputs are deliberately synthetic, including a known +0.8°/s gyro-x bias. The
calibration routine estimates and subtracts it. The scenario is **input replay**:
motor demands do not drive a physical drone plant. The plot is not a flight trajectory.

- Continuous Down+A / Down+B mode holds, report-gap reset and once-per-hold behavior.
- Down+LT two-second disarm and its latch.
- UART CRC, finite/range/length checks, duplicate sequence rejection and expiry.

## What is not emulated by this harness

The physical I²C transactions, MPU6050/BMP280 register behavior, Xbox BLE radio
and pairing, sensor mounting, analog battery monitor, ESCs and actual two-task
timing are not exercised. Calibration **serialization** is checked, but Preferences/NVS
power-loss persistence is not. Motor outputs remain disabled in the hardware build.
The first harness is intentionally limited to making the CPU/control evidence reproducible.

## Evidence and the failed first attempt

Latest: [39-check UART log](../../artifacts/emulation/uart-mode-gestures-2026-10-21/uart.log) · [summary and hashes](../../artifacts/emulation/uart-mode-gestures-2026-10-21/summary.json).
The following links retain the earlier 19-check baseline and initial failures.


- [Verified original UART log](../../artifacts/emulation/verified-replay/uart.log)
- [Checks, command and source/binary hashes](../../artifacts/emulation/verified-replay/summary.json)
- [Raw numeric data](../../artifacts/emulation/verified-replay/telemetry.csv)
- [Plot of that raw replay](../../artifacts/emulation/verified-replay/telemetry.png)
- [First startup failure](../../artifacts/emulation/attempt-1/uart.log)

Attempt 1 booted with a QIO-configured Arduino SDK but a DIO merged image and
failed the default flash initialization assertion. The emulator configuration now
selects DIO in both build and image merge. Attempt 2 then passed 18 checks. Its
retained data exposed estimator state persisting across `fc_init()`; the final
source resets those states and the verified replay passes 19 checks. No assertion
or failing check was suppressed to obtain the passing run.

Build and merge artifacts live in `build/`; original run evidence lives in `artifacts/`.
`-snapshot` keeps QEMU from modifying the merged input flash image.

## Primary references

- [Espressif ESP32-S3 QEMU instructions](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/esp32s3/README.md)
- [Official emulator guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/tools/qemu.html)
- [Pinned release](https://github.com/espressif/qemu/releases/tag/esp-develop-9.2.2-20260417)
- [Espressif tool checksums](https://github.com/espressif/esp-idf/blob/release/v5.5/tools/tools.json)

QEMU is third-party GPL software; its files/licenses stay in the downloaded local
cache and are not copied into the public project source.

## Optional companion regression (2026-10-21)

Latest replay: 44 checks passed. Five additional checks cover standalone mode selection, rejection of companion frames/navigation gesture, and QS capability bits. Original UART and source hashes: `artifacts/emulation/live-companion-policy-20261021/`. These are synthetic C-core CPU tests, not real BLE/UART/ESC or live flight.
