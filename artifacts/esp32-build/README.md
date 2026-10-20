# Original ESP32-S3 build evidence

Executed on 20 October 2026, on branch esp32-xbox.

| Operation | Result | Original output |
|---|---|---|
| PlatformIO dependency resolution | Completed | dependency-install.log |
| First full ESP32-S3 firmware build | Exit 0, with an application I²C overload warning | build-attempt-1.log |
| Rebuild after explicit I²C overload / dependency pins | Exit 0 | build-attempt-2.log |
| Source/header rebuild | Exit 0 | build-attempt-3.log |
| Final source-format cleanup build | Exit 0 | build-final.log |

Command: C:\Users\Mohamed\.platformio\penv\Scripts\platformio.exe run -e esp32-s3.
Dependencies were resolved first with the same executable's pkg install command.

Final PlatformIO report:

- Static RAM: 31,724 / 327,680 bytes (9.7%).
- Program flash: 588,761 / 3,342,336 bytes (17.6%).
- Generated firmware.bin: 589,120 bytes.
- These are build measurements, not peak heap usage, task stack margin or deadline measurements.

The first build's ambiguous Wire.requestFrom overload was corrected with explicit
matching argument types. Full dependency builds also report upstream NimBLE
enum-in-boolean-context warnings. They were retained, not filtered out.
No application warning appears in the final incremental source rebuild.

manifest.json records source and local binary SHA256 values.
Generated binaries remain in .pio/build/esp32-s3/ and are excluded from Git.

## Preservation evidence

preservation.json verifies:

- All 591 original tracked evidence files under the prior artifacts tree remain
  byte-identical to the baseline Git blobs after moving into archive/rp2350/.
- The inspected Drone main.cpp and platformio.ini still match their original
  hashes; the original folder was not changed.
- Main remains at 38034376a9fcac1550fe46aed25f8f0be2fb9370.

The Drone snapshot includes its pre-existing working edits, not just its older
7bd54c1 Git baseline. See archive/reference-drone/provenance.json.

## Limits

No board upload, Bluetooth pairing, real sensor/calibration capture, NVS reboot or
power-loss experiment, ESC action, flight, new simulation, or implementation test
suite was executed. Physical motor output remains disabled.
Do not use historical Pico/emulator/ROS recordings as proof of ESP32 execution.
