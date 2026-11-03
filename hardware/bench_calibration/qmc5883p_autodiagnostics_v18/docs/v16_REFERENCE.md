# IntelQCopter v16 - Guided Terminal System Tests (No GUI)

## Start here

This package is an *interactive no-GUI* test runner for the ESP32-S3,
QMC5883P, MPU6050 bypass, 3.3 V power, Wi-Fi activity, I2C bus load, and
serial connection. You do **not** need to edit `platformio.ini`.

Requirements: Windows PowerShell 5.1 or 7, your installed PlatformIO toolchain,
ESP32-S3 connected by USB, and pyserial (normally included with PlatformIO's Python
virtual environment). If the script reports pyserial missing and uses ordinary Python, run
`py -3 -m pip install -r requirements.txt`.

**With propellers removed and ESC signal wires disconnected**, open PowerShell
inside this extracted project folder and run (or double-click `START_TESTS.cmd`):

```powershell
.\tools\run_tests.ps1
```

Or use the familiar old command (now opens the exact same wizard):

```powershell
.\tools\test_mag.ps1
```

The script lists USB/serial ports, asks you to choose the ESP32 if more than
one exists, then displays an easy numbered test menu.

Choose **R** to run the recommended sequence:

`A0 (50 Hz) -> A1 (100 Hz) -> A2 (200 Hz) -> B1 (50 Hz AP) -> B2 (50 Hz scanning) -> C1 (50 Hz bus load) -> C2 (50 Hz AP + bus load)`.

Or select any single numbered test. To diagnose 200 Hz specifically, choose
**A2**, then (if it fails) **A3** (low OSR) and **A4** (bounded recovery).

## What happens for EACH test

1. The runner chooses the correct settings from a locked preset; it updates
   `platformio.ini` and keeps a one-time original backup in `results/`.
2. It prints the exact rate/Wi-Fi/I2C/recovery settings and checks for
   propellers/ESC isolation.
3. It runs PlatformIO compile and upload to your selected port.
4. It tells you to **remove every source of board power for at least 10 seconds**.
   Press Enter **while the board is still unplugged**. The runner then starts
   waiting for the COM port; reconnect power when prompted.
5. It captures the serial text automatically for 45 seconds, even if the COM
   port changes after USB re-enumeration (when identifiable).
6. It shows key boot, register, I2C, Wi-Fi and HEALTH lines; most CSV rows are
   hidden from the terminal but **ALL raw serial bytes are saved**.
7. It analyzes the measured rate and failure flags, saves a text/JSON summary
   and returns to the menu. Results also appear as `[last: ...]` markers.

### Output files

```text
results/
    platformio.ini.original.backup
    history.json
    runs/
        20261103-..._A2_COM3.log
        20261103-..._A2_COM3.summary.txt
        20261103-..._A2_COM3.summary.json
```

The raw log is the most useful artifact to send back for debugging. Send
`results/runs/*.log` or a zip of `results/runs/` along with the summary JSONs.

### Simple one-shot commands

```powershell
# Choose automatically from ports, then open test menu
.\tools\run_tests.ps1

# List COM ports
.\tools\run_tests.ps1 -ListPorts

# Run exactly one preset; parameters are applied automatically
.\tools\run_tests.ps1 -Port COM3 -Test A2

# Run one preset for 90 seconds
.\tools\run_tests.ps1 -Port COM3 -Test B1 -DurationSec 90

# Inspect settings without flashing or opening the port
.\tools\run_tests.ps1 -Test C2 -DryRun

# Analyze an already recorded serial log (no hardware)
.\tools\run_tests.ps1 -AnalyzeFile .\results\runs\YOUR_LOG.log
```

`-DurationSec` may be from 20 to 3600 seconds. For Wi-Fi tests use at least
45 seconds so both the Wi-Fi-off and Wi-Fi-on windows are visible.

### Test names and objectives

| ID | Magnetometer | Wi-Fi | Extra bus load | I2C | Purpose |
|---|---:|---|---|---:|---|
| A0 | 50 Hz | OFF | OFF | 100 kHz | Basic reference |
| A1 | 100 Hz | OFF | OFF | 100 kHz | Verify working 100 Hz |
| A2 | 200 Hz | OFF | OFF | 100 kHz | Reproduce 200 Hz issue |
| A3 | 200 Hz, OSR1=1 | OFF | OFF | 100 kHz | Compare `0xFD` with `0xCD` |
| A4 | 200 Hz, recovery=ON | OFF | OFF | 100 kHz | Investigate bounded I2C recovery |
| B1 | 50 Hz | AP after 5s | OFF | 100 kHz | Wi-Fi-on regression versus A0 |
| B2 | 50 Hz | STA scans after 5s | OFF | 100 kHz | RF/scanning stress versus A0 |
| C1 | 50 Hz | OFF | MPU+BMP raw reads | 100 kHz | I2C scheduling stress versus A0 |
| C2 | 50 Hz | AP after 5s | MPU+BMP reads | 100 kHz | Combined Wi-Fi and bus load |
| C3 | 100 Hz | AP after 5s | MPU+BMP reads | 100 kHz | Higher-rate combined test |
| C4 | 200 Hz | AP after 5s | MPU+BMP reads | 100 kHz | Only after A2 is stable |
| D1 | 50 Hz | AP after 5s | MPU+BMP reads | 400 kHz | Controlled bus-speed comparison with C2 |
| D2 | 200 Hz | AP after 5s | MPU+BMP reads | 400 kHz | Advanced bus-speed comparison with C4 |

**Keep A2 recovery OFF initially.** If 200 Hz fails with Wi-Fi off, do not
assume Wi-Fi alone is responsible. A successful 100 Hz boot does not prove the
200 Hz setup or the entire production Wi-Fi stack is correct.

## Key engineering cautions

- Motors are always kept as GPIO INPUT by the included C++ firmware; there
  are no ESC or flight outputs. Keep propellers and ESC signal wires removed.
- No software script can verify 3.3 V stability. If Wi-Fi correlates with
  magnetometer failures, measure voltage at the actual sensor pins and examine
  PCB decoupling, I2C pull-ups, bus rise times and MPU bypass.
- AP and scan tests reproduce *parts* of Wi-Fi activity, not your original
  production UDP/FreeRTOS task graph. Passing these tests is NOT final
  verification of the complete flight-control system.
- Opening a COM port may reset the ESP32 depending on USB/UART hardware.
  If the `BUILD`/`CONFIG` lines are missing, the runner reports the test as
  inconclusive rather than pretending it verified the boot.
- Serial for this diagnostic firmware stays **115200 baud**, on purpose. It
  is unrelated to your separate 500 Hz binary calibration logger at 921600.
- The output is labelled `PROVISIONAL PASS`, not `SYSTEM VALIDATED`. A physical
  cold-boot/repetition test is necessary; the Python runner cannot guarantee a
  hardware repair or distinguish power-rail faults without measurements.

## Technical information

- `tools/guided_tests.py`: test presets, PlatformIO upload, cold-boot procedure,
  serial capture, saved history and summary.
- `tools/analyze_mag_log.py`: postprocessor for v13-v16 text logs.
- `tools/run_tests.ps1`: PowerShell 5.1-safe ASCII-only command launcher.
- `tools/test_mag.ps1`: compatible alias.
- `src/main.cpp`: v15 system-aware diagnostic firmware, unchanged for v16.
- `docs/v15_manual_diagnostic_reference.md`: original detailed diagnostic notes.

Offline unit tests (not hardware tests):

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m unittest discover -s tests -v
```

References: PlatformIO CLI [device list](https://docs.platformio.org/en/latest/core/userguide/device/cmd_list.html)
and pySerial [API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html).
