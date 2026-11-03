# QMC5883P 100 Hz setup and fault isolation

## Known configuration

The current bench target is an ESP32-S3 calibration/diagnostic setup, separate
from this repository's RP2350 Pico flight firmware:

| Item | Target / known observation |
|---|---|
| Magnetometer | QST QMC5883P, I²C address `0x2C`, expected ID `0x80` |
| Bus | SDA GPIO9, SCL GPIO8; MPU6050 at `0x68`; BMP280 `0x76` through MPU bypass |
| Magnetometer rate | 100 Hz, QMC control register `0x0A = 0xC9`; `0x0B = 0x08`; axis sign `0x29 = 0x06` |
| Full calibration serial | CH343 USB-UART, 921600 baud, 52-byte v3 CRC-16/CCITT-FALSE frames; Wi-Fi off |
| Diagnostic serial | v18 triage uses 115200 baud; do not confuse this with the full-calibration stream |
| Flight / motor state | Bench-only; flight not established. Propellers off and ESC signals/power isolated for diagnostics. |

The 100 Hz register profile is a *target*, not proof of the achieved rate. The
isolated v13 run reportedly produced about 92–95 fresh samples/s. It has not
been verified under the integrated MPU/BMP/UART load. The reported v17 integrated
tests uploaded seven configurations but received zero fresh magnetic samples;
14 further attempts were blocked by COM3 access denied. Thus the immediate fault
is post-start QMC reads, not a demonstrated UART baud problem.

## Safe, staged setup

1. Keep the ESP32 calibration firmware separate from the Pico flight firmware.
   Do not merge an ESP32 `main.cpp` into the single-file Pico runtime.
2. Remove propellers and physically isolate ESC motor signals and power. Confirm
   motor GPIOs remain inputs. Do not flash flight firmware or enable outputs as
   part of this magnetometer diagnosis.
3. Connect the CH343 board and identify its current COM port. Close every other
   serial owner (PlatformIO monitor, IDE monitor, PuTTY, Arduino monitor, GUI).
   Only one program can own the port during upload/capture.
4. Use the v18 **triage** suite first, on the board, with the documented warm
   reset mode. From an extracted v18 package in PowerShell:

   ```powershell
   .\tools\run_auto_tests.ps1 -Port COM3 -Suite triage -PowerMode warm
   ```

   Replace `COM3` with the detected port. This uploads diagnostic firmware; do
   not start it until the bench conditions above are met. The suite is designed
   to compare STOP+START and repeated-START reads, with QMC and MPU post-start
   bus probes. It includes a 100 Hz repeated-START case only after lower-risk
   checks. Do not jump to the full matrix or change wiring/configuration between
   cases.
5. Preserve the generated `results/SESSION_*_SEND_THIS.zip` and all logs,
   including failures. Record whether reset was warm or a true power removal;
   the former does not prove the sensor was power-cycled. Treat zero fresh XYZ
   samples as a failed run even if ID, register readback, ACK, or upload passes.
6. Once the diagnostic shows stable fresh XYZ, run the full calibration package
   at 921600 baud. Its upload flag is explicit (`-UploadFirmware`); without it,
   the launcher should use the already-installed firmware. First use the
   20-second binary preflight and confirm MAG valid/fresh flags and decoded XYZ.
   Only then begin calibration. Keep Wi-Fi off and LiDAR optional/skipped if it
   adds another variable.
7. Validate achieved magnetic rate from timestamps, dropped/missed samples,
   calibrated field magnitude, axis orientation, and an independent withheld
   3D run. A fitted-on-data residual is not independent validation. The historic
   capture review estimates 11,000 accepted fresh samples and 94/s during motion,
   but reports uneven directional coverage and in-sample fit statistics only.

## Diagnostic interpretation

- If STOP+START succeeds and repeated-START fails, focus on the ESP32 Arduino
  I²C transaction path and preserve the exact post-start bus probes.
- If both methods fail for QMC while MPU/BMP probes still work, focus on QMC
  configuration, sensor state/power, and the QMC-specific transaction.
- If QMC, MPU and BMP probes all fail after mode start, investigate shared I²C
  bus state, bypass configuration, wiring, power, and timing before touching
  calibration code.
- If fresh QMC data is present in v18 but absent in the full stream, compare the
  calibration firmware's initialization/read sequence and then inspect UART
  packet counts/CRC at 921600. UART throughput alone cannot explain zero QMC
  samples inside the firmware.

Do not issue QMC reset/reinitialization workarounds, change ODR, I²C clock,
transaction style and UART settings together. Let one controlled comparison
identify the failing boundary first. QMC5883P is not QMC5883L; do not substitute
register conventions.

## Source material and evidence

The original source handoff and v4 calibration package are preserved in the
repository. The newer v4 source is under
[`../hardware/bench_calibration/esp32s3_qmc5883p_100hz`](../hardware/bench_calibration/esp32s3_qmc5883p_100hz/README.md).
Its earlier full-system capture and the new preflight are retained in that
package's `captures/` folder; the older v13/v17/v18 bundle remains under
[`../artifacts/handoffs`](../artifacts/handoffs/README.md).

The v4 bench firmware was compiled and flashed to the ESP32-S3 identified on
COM3. A subsequent 20-second preflight yielded 9,023 frames at 490.2 Hz, 161
fresh BMP samples, but zero MAG-valid and MAG-fresh samples; CRC errors and
sequence loss were both zero. The STOP+START QMC read change therefore did not
fix the live sensor. No GUI capture or motor test was run. Engine identity is
the physical ESP32-S3, identified by esptool; QMC5883P and sensor wiring are
user-reported. Assumed settings remain SDA GPIO9, SCL GPIO8, QMC `0x2C` / ID
`0x80`, control `0xC9`, I²C 100 kHz, and UART 921600. Board revision, wiring,
power integrity, sensor response and full-power reset remain unverified. The
isolated v13 MAG-only probe was uploaded but never started; its log shows the
ESP32 ROM waiting in `DOWNLOAD(USB/UART0)` mode. This is invalid sensor evidence.
Rerun that probe after the user's other firmware test is finished, confirm
normal probe startup, and record fresh XYZ after a true 10-second all-power
removal.
