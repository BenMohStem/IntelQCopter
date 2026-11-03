# IntelQCopter v18 — No-purchase automated QMC5883P communication triage

This **is not flight firmware**. Propellers removed, ESC signal and power connections physically isolated. Motor pins are only INPUT. Run from Windows PowerShell with the board attached to COM3, and close any PlatformIO monitor, VS Code serial monitor, PuTTY, Arduino Serial Monitor or other program that may hold COM3.

## Why this release exists

The v17 session dated 2026-11-03 showed that 7 firmware uploads succeeded, but **all 7 failed to obtain even one magnetometer sample**. Before start, the QMC ID `0x80` and control registers read correctly. The first reads after enabling mode `0xC5`, `0xC9`, `0xCD` or `0xFD` returned no bytes with `i2cWriteReadNonStop returned Error -1`. The remaining 14 test configurations never uploaded because Windows returned `PermissionError(13): Access is denied` on COM3. No Wi-Fi or UART baud-rate conclusion is supported by those failed tests.

v18 prioritizes read-transaction diagnostics rather than rerunning the full matrix.

## Run

1. Extract this ZIP to a **new** directory. Do not mix it with v17 files.
2. Open PowerShell in the extracted `IntelQCopter_QMC5883P_AutoDiagnostics_v18` folder.
3. Run:

   ```powershell
   .\tools\run_auto_tests.ps1 -Port COM3 -Suite triage -PowerMode warm
   ```

No manual selection, 60-second COM wait, manual reconfiguration, USB unplugging or external hub required. Every test is independently compiled, uploaded, captured, evaluated and bundled. A source-only read diagnostic `T0` parks the magnetometer in suspend before any measurement-mode write. `triage` runs: `T0 F1 H0 A0 G5 T1 A1`.

The script writes `results/SESSION_*_SEND_THIS.zip`. **Send that ZIP**, not a screenshot or terminal copy. `--connect-timeout` defaults to unlimited waiting for COM to appear. A separate maximum measurement duration remains as a bounded safety watchdog; persistent zero-byte errors finish automatically after roughly eight health windows. If COM3 stays busy, it retries three times after the initial upload and **stops the suite** with an error archive; it does not spam the rest of the matrix.

To test more configurations after we analyze the triage ZIP:

```powershell
.\tools\run_auto_tests.ps1 -Port COM3 -Suite softreset -PowerMode warm
.\tools\run_auto_tests.ps1 -Port COM3 -Suite full -PowerMode warm
```

Do not run the full suite until I²C readback is understood.

## Triage design

| Test | Purpose | Does it establish a controlled output rate? |
| --- | --- | --- |
| T0 | QMC ID, MPU bypass, control registers in suspend; **do not start measurement** | No (prestart communication test) |
| F1 | Normal 50Hz (`0xC5`), **STOP+START** register reads | Yes, if verified and measured |
| H0 | Historical `0x87`, **continuous mode**, STOP+START | No. MODE bits=11, ODR bits are not an authoritative 50Hz output rate |
| A0 | Normal 50Hz (`0xC5`), repeated-START register reads | Yes, if verified and measured |
| G5 | Normal 50Hz, explicit identity-guarded QMC software reset, STOP+START reads | Yes, if verified and measured |
| T1 | QST's `0xC3` continuous-mode example, STOP+START | No. Continuous mode is **not** a 10Hz mode |
| A1 | Normal 100Hz (`0xC9`), repeated-START | Yes, if verified and measured |

**Important:** `0xC3` and `0x87` have MODE bits `11` (continuous). They are not fixed-rate alternatives. `0xCD` is QST Rev. E's documented normal-mode 200Hz example. QST's QMC5883P is distinct from QMC5883L: never substitute L registers.

On a post-start verification failure, firmware now prints `POST_START_BUS`: it compares **both** QMC read methods, the MPU `WHO_AM_I` read using both methods, BMP ID, I²C address ACKs and GPIO8/GPIO9 levels. This helps distinguish a shared I²C problem from a QMC-specific issue. A high SDA/SCL level alone is not proof of clean power or timing.

## What v18 actually fixes

- Corrects the PowerShell splatting bug (`@RunArguments`, never empty `@Args`).
- Correctly verifies the compiled firmware nonce/STATE version and build settings.
- Keeps Python serial handles inside `try/finally` and closes them after each capture.
- Upload retries on **Windows COM port busy** only (not C++ build errors), with bounded backoff and a fatal stop if the port remains unavailable.
- Adds `T0`, H0, T1 and post-start transaction-method probes, reducing uninterpretable runs.
- Keeps existing v17.1 software-reset pair tests without adding hardware. Firmware verifies QMC chip ID before issuing a reset write.
- Caps persistent I²C error flood by reducing failure-poll rate once 25 unsuccessful reads have occurred.
- Does not claim a software restart removes sensor power.

## Requirements / verification

Windows 10/11; Python 3 with `pyserial` (`py -3 -m pip install pyserial` if needed); working PlatformIO installation and CH343 driver, as used in prior tests. Firmware target `esp32-s3-devkitc-1`, `espressif32@7.1.3`, Arduino framework; SDA=GPIO9, SCL=GPIO8, MPU=0x68, QMC=0x2C, BMP=0x76. **There is no real-hardware compilation or sensor validation in this release**; the Python test suite is simulated/offline. Run offline tests with `py -3 -m unittest discover -s tests -v`.

## References

- QST, *QMC5883P Datasheet Rev. E*, pp. 12 and 15, QST official PDF: https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf
- Arduino-ESP32, `Wire.cpp` `requestFrom` / repeated-START implementation: https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.cpp
- ESP32 repeated-START `i2cWriteReadNonStop` failure issue: https://github.com/espressif/arduino-esp32/issues/9617
