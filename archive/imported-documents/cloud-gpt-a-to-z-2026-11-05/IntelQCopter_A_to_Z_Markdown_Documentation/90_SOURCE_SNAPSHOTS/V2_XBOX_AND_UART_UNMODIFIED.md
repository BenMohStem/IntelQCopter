# Pilot interface (bench only)

A wired PC reads the Xbox controller via pygame and writes 50 Hz checksummed lines to the ESP32's **same CH343 COM port**. The firmware does not implement Xbox pairing. Xbox controllers using BLE firmware v5+ may be compatible with ESP32-S3 through a future Bluepad32 integration; older Bluetooth Classic firmware is not. The actual pilot transmitter, arming procedure and independent failsafe are not yet defined. **This PC-to-USB path is not a flight receiver.**

Frame on UART 921600: `$IQ,<unsigned_seq>,<throttle_0_to_1>,<roll_-1_to_1>,<pitch_-1_to_1>,<yaw_-1_to_1>,<mask_0_to_255>*<4-digit-upper-case-CRC16>\n`, with CRC16-CCITT-FALSE over bytes after `$` and before `*`.

Buttons mask bits:

- 0x01: D-pad Up
- 0x02: left bumper LB (shoulder)
- 0x04: right bumper RB (shoulder)
- `0x07`: Up+LB+RB. Only rising edges trigger a pressure-zero request, subject to a 900ms debounce and current pressure validity.

A valid frame expires after 200 ms. Duplicate sequence numbers and bad CRC are rejected. Serial parsing is nonblocking and bounded each loop. The Windows gamepad mapping **must** be inspected; SDL mappings vary. Axis defaults follow the historical `esp32-xbox` convention: **left X for yaw, right X/Y for roll/pitch**. SDL index defaults are guesses until you use `--inspect`. The default bridge deliberately sends `throttle=0` until you opt in using `--enable-throttle-preview`. Even when you do, the code can only calculate virtual motor commands.

The FC prints one ASCII diagnostic line/second; do not open PlatformIO monitor or the legacy calibration GUI at the same time as the Python bridge on the same COM port.

Historical branch difference: `esp32-xbox` uses D-pad Down + A/B for mode selection and Down+LT hold for a latched disarm. This development build adds a separate Up+LB+RB disarmed pressure-zero chord; it does not import the historical arm/mode protocol or prove Xbox BLE pairing. The two implementations must be reconciled before a real receiver is integrated.
