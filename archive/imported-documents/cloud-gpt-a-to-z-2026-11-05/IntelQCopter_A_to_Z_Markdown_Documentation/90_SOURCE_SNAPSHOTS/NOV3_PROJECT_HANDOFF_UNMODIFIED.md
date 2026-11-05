# IntelQCopter — comprehensive engineering handoff for a second AI

**Snapshot date:** Tuesday, 3 November 2026 (Algeria time).  
**Project:** IntelQCopter, a custom multirotor/flight-controller, sensor-calibration, navigation and eventual autonomy effort.  
**Immediate user goal:** get reliable full-sensor calibration and a trustworthy 500 Hz flight-control foundation before a competition planned for **5 November 2026**. The user says the aircraft has **not yet flown**.  
**Hardware proof status:** **NO VERIFIED FLIGHT, NO VERIFIED MOTOR-ACTUATING FC, NO VERIFIED INTEGRATED MAGNETOMETER RECOVERY.** Do not imply otherwise.  
**Most urgent blocker:** QMC5883P communication/data loss after activating measurement mode, even at 50 Hz on later firmware; serial COM-port exclusivity and occasional reset/enumeration issues compound automated testing.

> This is a handoff assembled from user conversations and actual project source/log archives. **Confirmed on hardware**, **configured in source**, **proposed**, and **estimated** are distinct statuses throughout. Firmware syntax checks and successful uploads do not prove hardware operation. Files inside `reference_packages` are historical snapshots, **not verified flight-capable releases**.

## 1. Guidance for the second/local AI

Your job is to cooperate with another ChatGPT agent on the same user's IntelQCopter project. Begin by reading this document, `AGENTS.md`, `NEXT_ACTIONS.md`, the actual latest sources, and the accompanying evidence. Work from concrete files and log traces, not previous assistant confidence statements. Preserve existing working code; change one relevant variable at a time and provide complete, runnable, Windows-friendly artifacts or concise patches. Avoid making the user repeatedly choose test flags, manually configure COM ports or unplug USB unless physically indispensable. No YKUSH or additional hardware purchases are planned. Be candid when a repair is unverified.

**Critical collaboration constraint:** both agents may analyze files in parallel, but should **never flash concurrently**, run two serial readers on the same CH343 COM port, or silently overwrite each other's firmware, captures or `platformio.ini`. Use versioned experiment folders, saved build-flags manifests, serial logs, Git commits, and reproducible test results. Hardware changes require user's action. Ask for the smallest missing datum only when necessary; prioritize processing already supplied files.

**Do not conflate three separate things:**
1. **Magnetometer-only diagnostic** — v13 worked at 50/100 Hz in one set of cold/warm tests, 200 Hz failed; later v17 all 7 completed variants failed.
2. **Complete sensor-calibration firmware** — 500 Hz IMU *target*, 100 Hz QMC *configured*, 10 Hz BMP *nominal*, 921600 baud binary stream, GUI; currently no confirmed valid integrated QMC data in the latest repair.
3. **Flight controller** — is still a **motor-disabled bench** for acquisition/timing/attitude. No validated hover controller, complete motor/ESC runtime or flight test. Neither a magnetometer configuration write nor a timing bench makes an airworthy FC.

## 2. Project identity and version evolution

- GitHub URL mentioned in the engineering study: **https://github.com/BenMohStem/IntelQCopter**. The study dated 1 Nov 2026 examined historical branches `main`, `esp32-xbox` and `codex/stepwise-esp32`. **Check the actual current repository and branches before assuming those commits remain current.** Historical architecture on `main`: RP2350 flight computer with RP2040 handheld/LoRa concepts, Kalman and PID scaffolding, but no proven real-sensor-to-ESC flight runtime. `esp32-xbox` explored ESP32-S3 + Xbox/ROS2/Gazebo integration. The stepwise ESP32-S3 branch focused on safe motor-disabled sensor measurements. These are separate development tracks.
- Earlier hardware chats mentioned an **RP2350** controller and a loosely described 5-inch red/white quad frame. **Current physical sensor PCB/testing track is ESP32-S3**, often described as **F450/Q450**. Treat the earlier RP2350 as historical/planned, not evidence of the MCU used in the present COM3 logs.
- Original ambition includes sensor-fusion modeling, multirotor dynamics, rate/attitude/altitude control, MATLAB/Python analysis, Gazebo/ROS2, and optionally companion YOLO vision; competition urgency has shifted priority to the simplest working, Wi-Fi-free sensor/flight stack. Only implement advanced autonomy after basic flight prerequisites are met.
- Previous iterations v5–v11 developed a full 500 Hz calibration stream, thermal/math analytics and GUI. v12/v13 isolated QMC diagnosis. v14–v16 explored 200 Hz, Wi-Fi, I²C and guided scripts. v17/v18 automate root-cause diagnostics. Independently, **full UART calibration MAG repair v3** and **FC 500 Hz bench v1** were produced. The numbering belongs to *different subprojects*: v18 diagnostics is not a replacement for the v3 full calibration GUI.

## 3. Real vehicle hardware — confirmed, described, and unknown

| Component | Current knowledge | Confidence / cautions |
|---|---|---|
| Frame | Described as F450/Q450-style X quad with 2 red and 2 white arms; 450 mm nominal diagonal motor spacing | Model and actual measured dimensions not confirmed; an earlier “5-inch” description conflicts with later F450 description |
| MCU/PCB | ESP32-S3 board/PCB, about **100 × 75 mm** in the mass-model discussion; physical motor GPIOs 12/11/14/13 listed in code | Exact PCB schematic, regulator, connectors and power distribution not independently verified |
| IMU | **MPU6050** at I²C `0x68`, 6-axis gyro/accelerometer | Recognized by firmware/scan; actual orientation relative to airframe and final calibration remain to validate |
| Barometer | **BMP280** at `0x76` (potential alternate `0x77` in code); connected on MPU XDA/XCL through bypass | Address was seen in v13/v17 scans; v16 log sometimes scanned only QMC/MPU; access is sensitive to MPU bypass |
| Magnetometer | **QMC5883P**, address `0x2C`, ID `0x80` | **Confirmed identity** in multiple logs; NOT QMC5883L/HMC5883L even if older docs or GUI labels say those |
| LiDAR | **Nooploop TOFSense F2 Mini**, mounted under middle of lower F450 PCB, downward facing | Sensor and mechanical position are user-described; actual module supply/logic and ESP32 UART pin route must be checked. Current GUI LiDAR stages use a **separate USB-UART** COM source, not forwarded through ESP32's 52-byte stream |
| Motors | Four orange **A2212 / A2122 10T**, described as **1400 KV** brushless | Precise model, measured thrust curves/rotation mapping/current and motor limits unverified |
| ESCs | **4 × 40 A** ESCs, mounted about halfway from center on arms, below arms/facing −Z | ESC control protocol, exact harness/wiring, calibration and arming/failsafe not established |
| Battery | **3S 4200 mAh LiPo**, on top of lower PCB | Real mass, C-rating, health and center of mass not measured; some previous weight models used 348 g as an assumption |
| Propellers | Prior project referenced **8045**, another record **9047**, earlier described 5-inch props | **Confirm actually mounted propellers**, direction and suitability; never assume from filenames |
| USB serial | Windows **CH343 USB-Enhanced-SERIAL**, frequently COM3, sometimes COM4; bridge VID:PID `1A86:55D3` | COM number can change; port exclusive; USB reboot and driver/enumeration issues seen |
| Companion PC | Optional **Intel N150 / Samsung mini-PC** for ROS2/YOLO, to be located close to lower PCB with centered horizontal CG | Future second configuration; not evidence that it is presently installed or electrically integrated |
| Extra sensors/radio | LiDAR, possible telemetry/LoRa, voltage-current headers, RC/Bluepad32 concepts | No validated end-to-end FC receiver/remote or actuator integration on current board |

### Electrical and geometric details

- ESP32-S3 main I²C: **SDA GPIO9, SCL GPIO8**; current standard diagnostic bus speed **100000 Hz**. A **400000 Hz** option exists in FC bench, **not yet verified** on the actual wiring. At 100 kHz, sharing MPU 500 Hz + magnetometer 100 Hz + BMP 10 Hz can consume significant I²C time; measure task deadlines and I²C latencies.
- MPU6050 `USER_CTRL` **register `0x6A=0x00`** (disable MPU auxiliary master); MPU `INT_PIN_CFG` **register `0x37=0x02`** enables **I²C bypass** to XDA/XCL (BMP280). **This register is NOT a request to physically connect the MPU INT pin.** Earlier drafts listed MPU INT GPIO7 and QMC DRDY GPIO10; the competition calibration and diagnostics are designed to **require neither physical interrupt line**.
- Magnetometer on main I²C at `0x2C`; status register `0x09` is an internal I²C-readable bitfield, **not a physical DRDY wire**. The current full calibration v3 restores internal DRDY bit checking; the FC bench v1, independently, polls changing XYZ and **does not** establish conversion freshness.
- Code's motor GPIO ordering annotation is `FL,FR,RR,RL = 12,11,14,13`; **do not treat it as a verified FC output mapping**. The current calibration and FC bench set those GPIOs to **INPUT**.
- Earlier mechanical coordinate estimate: origin at center of lower F450 board, +X forward, +Y left, +Z up. Nominal motors at `(±159.1,±159.1)` mm in an X arrangement; estimated ESCs at `(±79.55,±79.55)` mm. LiDAR approximately directly below center. These are **geometry assumptions**, not measured board coordinates.
- Do not connect 3S battery directly to a 3.3 V input; verify power-regulation and proper common grounds. Wi-Fi power transients, shared 3.3 V integrity, magnetometer brownouts and wiring parasitics are hypotheses, **not proven causes**.

## 4. Software/toolchain and preferences

- Development computer: Windows 10/11, Windows PowerShell (usually 5.1), Python 3 (`py -3`, reported Python **3.15** for recent pip installs; older test logs show Python 3.11), PlatformIO in `~/.platformio/penv/Scripts/platformio.exe`.
- PlatformIO target in calibration and diagnostic projects: `platform = espressif32@7.1.3`, `board = esp32-s3-devkitc-1`, `framework = arduino`; BMP280 libraries Adafruit BMP280 2.6.8, Unified Sensor 1.1.15, BusIO 1.17.4. Some previous logs reported an 8 MB flash S3 and embedded PSRAM, while PlatformIO described a no-PSRAM board definition. **Verify board variant, not important to immediate magnetometer bug.**
- Preferred **full calibration UART 921600 baud**, because user reports it worked for binary capture; isolated diagnostic uses **115200 baud** for human-readable text. PlatformIO/esptool may switch **upload** speed to **460800**; this is distinct from running firmware baud.
- On UART, 52-byte binary packet telemetry must be decoded by tools, **not** a plain text serial monitor. Successful serial attachment or upload is not evidence of sensor freshness. The CH343 driver/Windows COM assignment sometimes requires reconnect/reset, and only one process can hold the port.
- Wi-Fi **OFF** for competition calibration and FC bench. User observed earlier association between enabling Wi-Fi and magnetometer failures at 50 Hz, but later v17 confirms problems **also happen with Wi-Fi OFF**. Do not settle on “Wi-Fi caused it” as the only root cause.
- User requests **one-command launchers with automatic COM detection**, `-UploadFirmware` meaning **compile + flash**, omitted flag meaning **run existing firmware**, sensible defaults, automatic records/reports. Avoid GUI for diagnostics but keep **full calibration GUI**. GUI offers skipping optional LiDAR. User requests no Yupkit/YKUSH or extra purchases, no arbitrary 60-second connection timeout, and testing that can run unattended when feasible. Software reset is **not** equivalent to cutting power to the sensor.
- Beware prior script failures: Unicode em dash causing Windows PowerShell 5.1 encoding parse errors; positional PowerShell arguments passing `-SkipLidar` into numeric `GyroSamples`; empty `--upload-port` to PlatformIO; a Python-launch path opening an interactive Python shell rather than tests; COM access-denied after repeated uploads. **Test PowerShell 5.1 compatibility; use named splatting, ASCII-only script text, explicitly resolved nonblank COM ports and bounded retries.**

## 5. Magnetometer: known register map, working result and regressions

### QMC5883P (NOT 5883L) known registers

| Purpose | Register/value | Meaning or use |
|---|---|---|
| Chip ID | `0x00` → `0x80` | Confirmed in user logs |
| XYZ | `0x01`–`0x06` | Three signed 16-bit, little-endian magnetic outputs |
| Status | `0x09` | Bit 0 DRDY, bit 1 overflow; read as internal status, not GPIO |
| Control 1 | `0x0A = 0x00` | Suspend before configuring |
| Control 1: 50 Hz | `0x0A = 0xC5` | Fixed-profile normal-mode setting |
| Control 1: **100 Hz** | `0x0A = 0xC9` | **Only isolated high-rate profile demonstrated working** |
| Control 1: 200 Hz | `0x0A = 0xCD` | Documented normal-mode option; **failed** on current hardware captures |
| Control 2 | `0x0B = 0x08` | Range/set-reset configuration for the successful profile |
| Axis-sign | `0x29 = 0x06` | Special sign configuration from tested driver |
| Experimental low OSR | `0x0A = 0xFD` | Experimental 200 Hz lower oversampling, **failed in v17** |

**Observed v13 initialization order:** MPU bypass; read QMC ID; write suspend `0x0A=0x00`; wait **20 ms**; write sign `0x29=0x06`; write control2 `0x0B=0x08`; write selected control1 `0x0A=0xC5/C9/CD`; read back ID/C1/C2. Read QMC internal status then XYZ only after fresh conversion. Keep serial/no-Wi-Fi isolated to establish ground truth. V13 set `Wire.endTransmission(false)` (repeated START) before `Wire.requestFrom`.

### Hardware test evidence — the most consequential facts

| Test and date | Configuration | Observed evidence | Conclusion |
|---|---|---|---|
| **v13, 3 Nov, 50 Hz** | MAG-only, no Wi-Fi, 100k I²C, 115200 UART; `0xC5` | Initial registers verified, 2,558 valid CSV samples in ~54.76 s, ~46.7 Hz measured; 7 early status failures, 0 overflows | **Actually worked** in this capture |
| **v13, 3 Nov, 100 Hz** | MAG-only, `0xC9`, other settings same | Initial readback `ID=0x80`, C1=`0xC9`, C2=`0x08`; ~92–95 Hz, thousands of samples, 15 startup status failures and 1 reported overflow in stream; no ongoing register verification errors | **Actually worked** in this capture; recommended target configuration, **not proof full system works** |
| **v13, 3 Nov, 200 Hz** | MAG-only, `0xCD` | 0 fresh samples; initial verification read errors; ~1,890 status failures then C1 and C2 read as `0x00`; register verification not confirmed | **Did not work** |
| **v16 guided system test, 3 Nov, 50 Hz** | Wi-Fi OFF and simulated bus load OFF | QMC ID and pre-start registers readable; after enabling `0xC5`, `Wire.cpp:499 requestFrom(): i2cWriteReadNonStop returned Error -1`, no fresh rows, verify invalid | **Failed despite 50 Hz and Wi-Fi OFF** |
| **v17 automated session, 3 Nov ~18:45** | 21 planned: rates 50/100/200, low-OSR, recovery, Wi-Fi stress, bus loading, baud, STOP+START | **First 7 successfully uploaded tests A0,A1,A2,A3,A4,B1,B2 all 0 fresh samples**; pre-start ID and config read good, immediately post-start read returns zero bytes; **remaining 14 did not run** due to Windows COM3 `PermissionError(13)` | Current strongest evidence: **post-enable QMC or bus/read-method problem**, independent of Wi-Fi. Unrun tests must not be counted as MAG failures. All recorded v17 resets were **warm**. |
| **v18 triage release** | Tests `T0 F1 H0 A0 G5 T1 A1`; STOP+START vs repeated START and QMC soft reset differential probes | Source/script and offline tests exist, but **no uploaded physical v18 results are known as of this snapshot** | **Next diagnostic experiment**, not a confirmed fix |
| **Full UART calibration MAG repair v3** | 921600 serial, Wi-Fi off, MPU 500 Hz target, QMC 100 Hz, BMP 10 Hz nominal; restores v13 status check | Code packaged and syntax checks reported, but user has not provided a successful integrated fresh-MAG capture. Earlier integrated builds repeatedly failed. | **Do not claim repaired** |

### Precise unresolved root-cause questions

1. Does enabling QMC measurement mode trigger a **device reset, bus-specific failure, or Arduino-ESP32 repeated-START compatibility problem**? The repeated-START path (`Wire.endTransmission(false)` then `Wire.requestFrom`) reported `i2cWriteReadNonStop returned Error -1`, and v17 pre-start bus communication was good. The observed state could also involve supply integrity or IC behavior. **Do not assume any cause without evidence.**
2. In the failing state, can QMC be read with **STOP+START** instead? Can the MPU `WHO_AM_I` and BMP280 ID still be read? Do the QMC control registers retain their values or reset? v18's `POST_START_BUS` and `T0/F1/A0/G5` tests answer this.
3. Was there a **true electrical power cycle**? Warm MCU resets, RTS toggling and serial reconnection do **not** prove QMC VDD went to zero.
4. Does moving from MAG-only to full 500 Hz MPU + BMP polling at 100 kHz I²C cause overload/starvation, even if the QMC itself works? Capture real I²C timing and rate. 400 kHz is an experimental choice, not established on these wires.
5. After communication works, does Wi-Fi change behavior from a known-good baseline? Compare Wi-Fi on/off at the **same hardware state**, not broken baseline.
6. Avoid preemptive periodic QMC suspend/start cycles: earlier full code could repeatedly disrupt reads. Separate error counters for ID, status, XYZ, config verification, data staleness, and serial packet errors.
7. Never infer “fresh” merely from changed XYZ or from a status-byte print if the actual transaction failed. Identical consecutive XYZ values **can** be separate valid conversions if DRDY was set. Do not calibrate with stale or fabricated data.

### Recommended immediate test

Use **v18 diagnostic project** in `reference_packages` (not the full GUI) to answer the differential question without requiring a purchased YKUSH:

```powershell
.\tools\run_auto_tests.ps1 -Port COM3 -Suite triage -PowerMode warm
```

Close all other COM3 readers and keep props off. Archive `results/SESSION_*_SEND_THIS.zip`. **Warm mode cannot independently establish hardware power-cycle behavior.** If v18 shows STOP+START succeeds where repeated START fails, patch only the QMC read method in a cloned full calibration project, and verify data; if both fail, inspect post-start bus response and supply integrity, including QMC module connections. Avoid launching a 21-test matrix until a baseline is working.

## 6. Full UART-only calibration — latest snapshot v3

**Archive:** `reference_packages/IntelQCopter_100Hz_UART_MAG_REPAIR_v3.zip`. This is the full calibration with GUI and optional LiDAR, **not a flight controller**.

- `platformio.ini` env `esp32-s3-calibration-500hz`, `espressif32@7.1.3`, Arduino; flags `IQ_I2C_HZ=100000`, `IQ_WIFI_ENABLED=0`, `IQ_MAG_TEXT_DIAGNOSTIC=0`, `CORE_DEBUG_LEVEL=0`.
- `src/main.cpp`: MPU6050 500 Hz nominal (2 ms absolute-deadline loop; 14-byte burst, DLPF, gyro ±250 °/s, accel ±2 g), QMC5883P 100 Hz profile, internal status polled at 200 Hz, BMP280 nominal 10 Hz, **no motor actuation**.
- `START_USB_CALIBRATION.ps1`: auto-detect CH343 COM, optional `-UploadFirmware` to compile/flash, omit to start existing installed firmware and GUI immediately. `-Check -Seconds 20` performs UART preflight instead of GUI. `.cmd` variants exist. Avoid passing empty `--upload-port`.
- `tools/live_calibration_gui.py`: guided gyro, 6-face accel, MAG yaw/pitch/roll + 3D tumble, BMP pressure and optional LiDAR stages. Raw captures saved under `captures/`. Separate Python analysis, calibration, plotting and thermal scripts included.
- **User-requested GUI defaults:** gyro **30,000** stationary accepted samples; accelerometer **5,000 per face**, six faces `+X,-X,+Y,-Y,+Z,-Z`; magnetometer **2,000 per yaw, pitch and roll**; 3D tumble **8,000**; barometer **1,000**; settle delay **3 s** per stage; **LiDAR 300 per range**, optional `Skip ALL LiDAR ranges` button.
- **LiDAR range points**: **25, 50, 100, 200 cm**. The v3 GUI expects TOFSense F2 Mini on **separate USB-UART COM**, even if physically mounted under lower F450 PCB. If LiDAR is wired only to ESP32 UART, that source is **not yet integrated** into the binary ESP32 stream; skip LiDAR in GUI until proper routing exists. Do not assign ESP32 and LiDAR same COM port.
- **Run:**

```powershell
py -3 -m pip install -r requirements.txt
.\START_USB_CALIBRATION.ps1 -UploadFirmware -Check -Seconds 20
# If UART/IMU/MAG/BMP stream passes:
.\START_USB_CALIBRATION.ps1  # no reflash, starts the GUI
```

Important: user explicitly reported magnetometer missing in earlier full-calibration packages; v3 restoration to a v13-like status transaction was a proposed fix **not yet confirmed** by any supplied successful full-stream log. Do not silently skip MAG or accept empty ellipsoid data.

### Binary UART transport (do not break existing GUI)

**52 bytes**, little-endian payload, sync `A5 5A`, version `3`, CRC-16/CCITT-FALSE (`init 0xFFFF`, polynomial `0x1021`, CRC over first **50** bytes), sent at **921600 baud** over CH343. Exact original layout from `src/main.cpp`:

| Offset(s) | Length | Payload |
|---|---:|---|
| 0–1 | 2 | Sync bytes `A5 5A` |
| 2 | 1 | Protocol version `3` |
| 3 | 1 | Flags |
| 4–7 | 4 | Sequence `uint32` |
| 8–11 | 4 | Acquisition timestamp microseconds `uint32` |
| 12–13 | 2 | Deadline lateness `uint16` microseconds |
| 14–19 | 6 | Accelerometer x/y/z signed `int16` |
| 20–25 | 6 | Gyroscope x/y/z signed `int16` |
| 26–27 | 2 | MPU raw temperature `int16` |
| 28–33 | 6 | QMC raw x/y/z signed `int16` |
| 34–37 | 4 | BMP raw pressure `uint32` |
| 38–41 | 4 | BMP raw temperature `uint32` |
| 42–43 | 2 | BMP temperature centi-°C `int16` |
| 44–47 | 4 | BMP pressure centi-Pa `uint32` |
| 48–49 | 2 | Missed-deadline counter `uint16` (wrapped/truncated) |
| 50–51 | 2 | CRC-16 little-endian |

Flag bits from firmware: `0` MPU valid; `1` BMP valid; `2` MAG valid; `3` BMP fresh; `4` MAG fresh; `5` deadline >250 µs late; `6` QMC5883P identity; `7` older QMC5883L marker (should **not** be set). **A set valid bit is generated by firmware logic, not independent sensor-health certification.** Before changing layout add a protocol version and update host decoder/tests together.

## 7. 500 Hz FC bench (current developmental state)

**Archive:** `reference_packages/IntelQCopter_FC_500Hz_Bench_v1.zip`. Pure bench firmware with **all four actuator GPIOs INPUT** and **no ESC PWM, DShot, arming or runtime motor control**.

- Target scheduler **2 ms / 500 Hz**, MPU6050 14-byte burst, QMC 100 Hz configured, BMP ~10 Hz; Wi-Fi disabled, diagnostics text at **921600 baud**. Bench GPIO and chip addresses match full calibration.
- Startup provisional stationary gyro bias estimated from ~512 samples; roll/pitch complementary filter from accel/gyro, **yaw integrated only and drifts**. This is **not** full calibrated 3D attitude or a validated control system; body-frame axis signs and alignment are unconfirmed.
- Tracks actual `hz`, `cycles`, `imu_ok/imu_fail`, `mag_changed`, `bmp_fresh`, `missed`, `late250`, `dt_gt4ms`, `imu_max_us`, `sensor_max_us`, `work_max_us`, roll/pitch/yaw and p/q/r, bias status and MAG config/I²C failures.
- The FC bench takes neither the GUI's full calibration matrices nor validated motor drive into its estimator. It uses **changed XYZ**, not QMC internal DRDY, so MAG freshness is different from the full calibration's sensor logic.
- `RUN_FC_BENCH.ps1 -UploadFirmware` uploads & records with **100 kHz** I²C. Omit flag to re-record installed firmware. `-FastI2C` chooses **400 kHz** experimental build; do not assume it's superior until errors and timing have been compared. Results under `results/`.
- User desires “blazing fast” 500 Hz FC, but **schedule must be measured**, not inferred from the presence of `delayMicroseconds` or configured MPU sample divider.

### Flight development still missing / must be verified

1. Power-chain and current budget under load; reliable independent **hard disarm** and signal integrity.
2. Sensor coordinate alignment, gyro/accel calibrated 3×3 matrices including cross-axis terms, magnetometer hard-/soft-iron calibrated under final mounted configuration, baro accuracy and useful altitude sensing.
3. Deterministic control-loop budget, failsafe/watchdog paths and sensor/receiver validity gating; only use yaw from a trustworthy estimator.
4. RC/companion communications reliability and link-loss behavior; prove the control path is valid without depending on ROS2/N150 initially.
5. Explicit motor numbering, rotation direction, thrust sign, ESC protocol and arming state machine; **motor-off** tests first.
6. Safe prop-off actuator tests, controlled low-power restrained tests under appropriate supervision, then only a bounded outdoor test if all prior gates pass. **No evidence in current files shows a verified first flight.**

## 8. Calibration science and quality expectations

- **Gyroscope:** stationary offset and noise PSD/Allan variance where sufficient long static records, temperature drift, bias stability. Offset estimation needs a stationary, level-free condition; vibration and temperature contaminate results.
- **Accelerometer:** calibrate a 3×3 linear map plus offset from six or more independent gravity orientations, **include cross-axis terms** and estimate repeatability/uncertainty. Diagonal-only calibration is a weak assumption; user explicitly requested cross-axis terms. Verify transformed six-face norms and independent orientations rather than just optimizing training residuals.
- **Magnetometer:** rigorous 3D ellipsoid/affine calibration to handle hard-iron vector and full 3×3 soft-iron/cross-axis matrix. Require broad 3D angular coverage, repeated independent sessions, enough **fresh** readings, outlier detection, matrix conditioning and realistic field-magnitude validation. A yaw-only circle is insufficient. Compare board unpowered, USB powered, and flight electronics powered only under safe controlled conditions. Do not invent field-vector scale or yaw accuracy.
- **BMP280:** pressure and temperature compensation (use the actual library math), thermal drift, reference pressure for altitude, and measurement noise; sensor read timing is slower than IMU loop. Altitude accuracy depends on atmospheric pressure reference and environment.
- **LiDAR:** separate UART instrument / likely downward, compare controlled ranges 0.25/0.50/1/2 m, valid-return flags and model-specific limits; do not claim direct ESP32 UART integration until implemented. TOFSense F2 Mini typically has limited output rate compared with 500 Hz IMU; consult datasheet before choosing settings.
- **Fusion/control:** time stamps from original sensor acquisition; account for different update rates, latency, gyro integration, long-term bias, mag trust/innovation gating and baro/LiDAR altitude aiding. Existing Mahony/Kalman/PID projects are **development concepts**, not flight-qualified tuning.
- **Noise/error measures:** log raw data, timestamps, task execution time, failures, environmental conditions, temperature and calibration matrix condition numbers. Keep train/test data separate and don't report MC confidence intervals without input uncertainty assumptions.

## 9. Frame, mass, inertia and optional AI companion

**These are engineering estimates, not measured flight data.** The lower PCB is described as ~7.5×10 cm. Battery is above lower PCB; 4 motors at ends, 4 ESCs around mid-arms, LiDAR near lower PCB center/bottom facing down. Optional N150 mini-PC with display and battery removed is centered horizontally near lower PCB, changing vertical CG and inertia. Prior assistant-generated model assumed:

- **Configuration A (no N150):** mass approximately **1.104 kg**; estimated CoM `z ≈ +18.66 mm` relative to lower board center; diagonal inertia estimate `[0.009781, 0.010390, 0.019170] kg·m²`.
- **Configuration B (+ ~200 g N150 module and mounting):** mass approximately **1.319 kg**; estimated CoM `z ≈ +11.85 mm`; diagonal inertia `[0.010300, 0.010913, 0.019562] kg·m²`.
- **Do not adopt these in the FC as measured truth**: component masses, dimensions, CoM offsets, motor axial geometry and uncertainties were assumed, not weighed; off-diagonal inertia was set to zero by ideal symmetry. Recompute from real hardware or CAD and establish uncertainties. A code model exists in `reference_docs/IntelQCopter_inertia_model.py` and matching illustrative JSON; review their definitions before using.

## 10. Known problems in launchers and diagnostic UX

- PowerShell 5.1 may fail on UTF-8 without BOM containing `—` and yield a misleading missing quote terminator; prefer plain ASCII PS1 (or UTF-8 BOM).
- Old wrapper forwarded parameters positionally, causing `-SkipLidar` to be parsed as integer `GyroSamples`. Correct version uses **named parameter splatting**.
- Old launcher used empty `--upload-port`, generating `Option '--upload-port' requires an argument`. Corrected launcher resolves a nonempty COM port (CH343 first) before upload; COM port may change after reset.
- v16 runner demanded physical 10-second power cycle and timed out waiting for COM after 60 seconds; user wants no arbitrary short limit. v17 automated suite used warm reset but encountered serial `PermissionError(13)` after several uploads; use `try/finally` for serial closure and stop on sustained port busy. Avoid pretending a warm reset is physical power removal.
- One `run_auto_tests.ps1` invocation previously launched an **interactive Python shell** instead of the test runner; v17.1/v18 script changes target this. Verify exact executable invocation on user's Windows machine.
- 921600 serial can be reliable for 52-byte binary frames at 500 Hz: theoretical raw stream `52*500=26000 B/s`, 8N1 UART line requirement ~260000 bit/s. However actual dropped frames/buffering/OS scheduling must be measured. **921600 has margin but does not explain QMC failure at 115200.** The PC uses CH343, so validate driver, DTR/RTS and port enumeration independently.
- The user's immediate benchmark is truthful **sensor success** with no Wi-Fi and optional LiDAR, not a new GUI or exhaustive combinations that delay flight preparation.

## 11. Suggested division of work between two AIs

**Local AI (owns checkout and code modifications):** read v18 source & v17 logs; perform small patch/compile/tests and run diagnostic firmware under user supervision; keep all tests reproducible, ensure PowerShell 5.1 and CH343 COM behavior, generate logs and a strict issue tracker. When QMC actually produces data, port the minimal fix into a clean clone of full calibration v3 and verify 921600 binary preflight. Build tests for packet decoder/CRC/calibration outputs.

**ChatGPT (design/review partner):** review new v18 `SESSION_*_SEND_THIS.zip` traces, compare differential bus behavior, help validate sensor science and physical flight readiness, audit code and proposed fixes, review 500 Hz timing, estimator and control safety. Do not assign the same physical COM port to two AIs at once.

**Shared artifacts contract:** each change should save a small `CHANGELOG.md` entry with date, source file, config diff, exact test command, hardware power state (warm vs true cold), whether code built/flashed, `good/fail/unknown`, evidence path and next question. Never replace a previous working baseline without a backup. A log with zero samples is a failed result, never an ellipsoid dataset.

## 12. Priorities for Nov 3–5 competition window

**P0 — establish communications:** isolate the QMC at 50 or 100 Hz, identify why post-enable reads fail via v18 triage; verify real fresh XYZ, not just ACK. If the failure persists, **do not produce fake mag calibration**. A vehicle can in principle use IMU-only roll/pitch and gyro-only yaw in controlled architectures, but that requires a fully validated independent control system; it is not proven safe on this quad.

**P1 — complete UART sensor data and calibration:** full v3 project, Wi-Fi off, CH343 921600, gyro 30k, 6-face accel 5k/face, MAG 2k/axis & 8k tumble if working, BMP 1k; optional LiDAR skip in GUI. Validate time series, covariance, physical orientation and calibration residuals.

**P2 — 500 Hz benchmark:** run the separate no-motor bench to measure achieved IMU rate, deadlines and timing with baro/mag; compare 100k vs 400k only after sensor bus stable. Import actual calibrated parameters only after review.

**P3 — FC and motor safety:** code can progress, but **no motor/prop flight attempt until** hardware mapping, disarm, failsafe, estimator, battery/power, RC link and prop-off/controlled checks pass. User's competition deadline is not evidence of flight readiness.

**Defer:** 200 Hz QMC optimization, Wi-Fi telemetry, onboard N150 ROS2+YOLO, advanced autonomy, complicated mass modeling, high-rate textual logging, and LiDAR if it blocks a working baseline.

## 13. Exact package map (all preserved unmodified)

| Path in this handoff bundle | What it contains / why use it |
|---|---|
| `reference_packages/IntelQCopter_QMC5883P_FixedBootProfiles_v13.zip` | Actual **working isolated 100 Hz** firmware and v11/v12 forensic docs, register code and logging |
| `reference_packages/IntelQCopter_QMC5883P_AutoDiagnostics_v18_NoPurchase_Triage.zip` | **Newest diagnostic design** for post-start I²C read-method differentiation; CLI source/runner/test suite |
| `reference_packages/SESSION_20261103_184453_23344_SEND_THIS.zip` | **Actual v17 captured evidence**: 7 failed MAG measurements and 14 COM upload errors, with manifests and raw logs |
| `reference_packages/IntelQCopter_100Hz_UART_MAG_REPAIR_v3.zip` | Latest **complete calibration** source, UI/optional LiDAR, 921600 binary protocol, fixes not yet hardware-verified |
| `reference_packages/IntelQCopter_FC_500Hz_Bench_v1.zip` | Timing/attitude-only **motor-disabled** ESP32-S3 bench firmware and UART capture tools |
| `reference_packages/IntelQCopter_2026-11-02_Repo_FC_Patch.zip` | Separate historical FC repository patch; do not mix into calibration firmware blindly |
| `reference_docs/IntelQCopter_Engineering_Study_2026-11-01.md` | Broader control architecture, branch review, physics/modeling/validation recommendations |
| `reference_docs/MAGNETOMETER_VALIDATION_METHOD.md` | Magnetometer calibration/validation method from earlier work |
| `reference_docs/IntelQCopter_inertia_model.py` + `IntelQCopter_inertia_results.json` | Assumption-driven mass/inertia model, **not measured** |

## 14. Technical references / provenance

- Primary direct evidence: user-provided v13 50/100/200 boot console logs and bundled `SESSION_20261103_184453_23344_SEND_THIS.zip` (v17), supplemented by v16 CLI capture. Source manifests are inside the packages.
- QST **QMC5883P Datasheet Rev. E** (register map and 50/100/200Hz modes): https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf. Verify source availability and exact timing if changing modes.
- ESP32 Arduino repeated-START issue discussion: https://github.com/espressif/arduino-esp32/issues/9617 (possible mechanism, not proof of the issue on this board).
- ESP32 Arduino `Wire.cpp`: https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.cpp.
- GitHub historical repository: https://github.com/BenMohStem/IntelQCopter (consult live repo before claiming any branch is up to date).
- Attached engineering-study and validation files in `reference_docs` provide additional derivations and historical discussion. They are not substitutes for real airframe measurements or flight checks.

### Most important instruction to the continuing AI

**Stop treating a build, upload, `ACK=1`, `regs_ok` flag, a simulated test, or a version number as proof the magnetometer works. Start with actual recorded fresh XYZ; keep the full 100 Hz no-Wi-Fi UART calibration as the user’s preferred end state; keep all motors disabled until a separate, demonstrably safe FC exists.**
