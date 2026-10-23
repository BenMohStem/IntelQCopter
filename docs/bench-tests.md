# First real bench tests

No hardware acceptance has been recorded at branch creation. Remove propellers and leave all ESC signal/power connections disconnected for these tests. USB powers the ESP32; do not simultaneously feed the DevKit5V pin.

## 00 — Inspect material and wiring first

1. Record ESP32 board marking, sensor labels/chip markings and photos that show only the hardware. Fill `config/hardware.json` when actual identities/wiring are known; changing it invalidates old acceptance.
2. With power removed, inspect solder joints, wire insulation/strain relief, pin order, polarity and accidental bridges. Measure common-GND continuity and check for power-rail shorts. A capacitor may briefly charge from an ohmmeter; record resistance and trend rather than equating any beep with a short.
3. Power the logic only. Measure3.3V at the DevKit and the supply at each sensor chip after its breakout regulator. Record meter/range/uncertainty and readings.3.20–3.40V is an initial **bench acceptance target**, not a universal module specification. Verify the actual module datasheet if its internal voltage differs.
4. Measure the I2C high level: it must remain in the3.3V domain. Confirm SDA=GPIO9, SCL=GPIO8 and common GND. Start at100kHz. Record effective pull-ups; do not blindly add another resistor in parallel with each breakout.
5. Use the current labels: MPU VCC/GND/SCL/SDA, AD0=GND; BMP VCC/GND/SCL/SDA, CSB high and SDO low. Power sensor wire ports at3.3V initially, then verify the chip supply after any module LDO. Do not assume a clone behaves like a different breakout.
6. Record frame/arm fasteners, motor mounting,9047 CW/CCW prop inventory, battery condition, actual battery mass and whole-aircraft mass if available. Mechanical/thrust/ESC characterization is **not passed** just because visual inspection succeeds. Do not bend-test or load-test the LiPo.

Copy `measurements/bench-template.csv` to a dated folder under `artifacts/hardware/`, fill actual values, retain photos/meter readings, and write a short acceptance note. The gate tool requires an operator and hardware identity; it cannot independently prove an operator performed a measurement.

## 01 — Board

Build, deliberately select the correct COM port, upload the bench program, then capture at least30seconds and one repeat boot. Check chip identity, reset reason, uptime progression and inactive motor pins. Serial heartbeat proves firmware progress, not an electrical measurement of a pin; verify motor/ESC outputs remain inactive with a meter/scope before later output work.

## 02 — I2C

Send `i`. Keep the original scan/ID output. MPU6050 WHO_AM_I must identify the MPU6050 family; BMP280 chip ID must be0x58. A responding address alone is not sufficient. HMC5883L's H43 identity is reported if present;0x0D is reported as an unidentified possible QMC device, not silently treated as HMC.

## 03 — Sensor samples

Keep the board still and send `m` for500 fresh IMU readings at a nominal100Hz. Repeat in several known orientations; for a level board one axis should see approximately1g depending on mounting. Do not label sensor axes as drone axes yet. Gyro means estimate stationary bias, not temperature-invariant calibration. Repeat after warm-up and record temperature.

Send `b` for100 pressure/temperature observations at a nominal20Hz. Compare pressure/temperature with an independent reference where available and raise/lower the sensor by a measured distance. Pressure-derived relative height is not a truth reference. The firmware's range/identity checks and sample summaries are **observations**, not automatic stage acceptance.

The program reports raw samples and sample standard deviation: `s = sqrt(sum((x-mean)^2)/(n-1))`. Noise depends on bandwidth, sample rate, temperature, vibration and mounting; report those conditions. It does not equal absolute accuracy or drift. No made-up noise threshold automatically passes a sensor.

## Sources

- [Espressif DevKitC-1 header/supply reference](https://documentation.espressif.com/api/resource/path/docs/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide.html).
- [Bosch BMP280 datasheet and ID/I2C interface](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf).
- MPU6050 registers/scales: installed ElectronicCats MPU6050 library sources, pinned in PlatformIO; inspect its WHO_AM_I and full-scale constants with the raw register readback.

##04 � Calibration diagnostics (prepared offline)

Use `m` while truly stationary. It records500 fresh samples, prints per-axis mean/sample standard deviation, and proposes a gyro bias in RAM. Provisional motion screening is accelerometer sigma<=0.02g, gyro sigma<=0.25deg/s, bias magnitude<=5deg/s per axis, and gravity-vector norm0.85�1.15g. These are initial diagnostic bounds, not measured sensor specifications or proof the aircraft is still.

Capture the six sensor orientations with `0=+X`, `1=-X`, `2=+Y`, `3=-Y`, `4=+Z`, `5=-Z` (the selected axis measures the stated acceleration sign). Each command collects100 fresh samples. Align the selected axis with gravity; wait for `CAL_FACE,CAPTURED` before moving. A rejected/repeated face clears the old fit. All six valid faces are required. This is a diagonal bias/scale fit: offset=(positive+negative)/2; scale=2/(positive-negative); corrected_g=(raw_g-offset)*scale. It does not estimate cross-axis misalignment or temperature dependence.

`w` writes only when a valid six-face fit and a gyro verification from this boot are present. The `qstep_bench` Preferences namespace is separate from the original FC's calibration namespace. `CAL_SAVE,BYTE_READBACK_OK` verifies immediate bytes, board identity, version and CRC. It does not prove persistence across power loss: power-cycle physically and use `v`/boot logs to verify reload, then use `m` again. `v` reloads the bench record and resets this-boot gyro verification and unsaved face captures. `e` deliberately erases **only** this bench record. No old-flight-controller settings are read or overwritten.

When testing again, compare corrected accelerometer samples with each face's expected vector and gyro-minus-bias while stationary. Record residuals and repeats; the current firmware prints the proposed parameters rather than claiming a flight estimator is calibrated. NVS interruption/corruption and reboot cases remain **physical tests pending**.

### Other sensors

`h` collects64 HMC5883L raw field vectors only after its H43 identity matches. A possible QMC at0x0D is identified as unsupported, without guessing a driver. This is not hard/soft-iron or heading calibration. Keep magnets/motors/power wiring effects in the measurement plan.

`l` listens for10seconds to TOFSense UART at921600baud on FC RX6/TX7. It sends no query/control commands and never uses the ROS UART. It retains all16 frame bytes in hex, checksum failures, module status, distance and signal strength. Compare readings with a ruler/tape at several distances and target colours/angles. A valid checksum is not a claim of correct distance; query-mode sensors or changed baud rates may produce no active frames. Confirm the actual module/model/status interpretation first.

[Official F2 Mini voltage/default baud reference](https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_en.pdf) and [UART frame format](https://support.nooploop.com/tofsense/protocol/). The LiDAR and magnetometer are not used in a flight controller in this diagnostic build.

` t ` (send the single character `t`, without spaces) captures200 new samples and prints corrected per-axis means/std and gravity-vector norm for residual testing. Repeat each known face and stationary gyro test before/after saving and rebooting. No pass is automatic. The hardware assembly is reported by the user, not inspected; it is unavailable now, so there are no physical readings.
