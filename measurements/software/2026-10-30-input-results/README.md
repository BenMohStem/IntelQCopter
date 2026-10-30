# Xbox input results and LED clarification — 30 October 2026

Actual ESP32-S3 recording, analyzed offline; no new firmware uploaded. Original recording preserved privately; SHA256 and statistics are in analysis.json.

466 snapshots at the configured 5 Hz diagnostic rate. All 11 logged button fields have nonzero samples. Each stick axis reached 0–65535; LT/RT reached 0–1023. This demonstrates observed decoding and endpoint coverage, not exact centers, deadband, physical label identity, BLE packet latency or an accepted flight input interface. Snapshot counts are not press counts. No calibration/PID/motors changed.

User confirms completion. Their previous physical Menu/Y mismatch is retained as operator evidence; application swaps these two roles for this controller. Full label validation still requires associating each press with its physical label, rather than inferring identity from arbitrary event order.

## LED decision

User intended the small blue flashing/activity LED or the adjacent small LED, not the large RGB. Exact board circuit remains unidentified (CH343 USB bridge previously observed). It may be USB activity or power, so no guessed GPIO is configured. RGB code remains as previously installed pending identification; this iteration does not claim the requested small LED was made programmable. A clear board photo showing both LEDs/nearby markings or the exact schematic is needed.

Official Espressif DevKitC-1 reference documents power and RGB LEDs; a clone's USB/activity LED cannot be inferred from that reference: https://documentation.espressif.com/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html

## Future battery behavior (not implemented)

Reserve the large RGB for battery indication, including red at estimated 20% remaining. User now proposes safe landing or GPS-assisted return/landing in the future. This supersedes the earlier requested 20% immediate-disarm behavior as a future direction; exact flight response remains to be designed and validated. No voltage/current sensors or GPS yet: no charge estimate, threshold action or landing is enabled. GPS alone is insufficient for a validated return/landing system. Physical motor outputs remain disabled in this bench stage.

## Next

Input endpoint observation is complete; do not repeat the four artificial-pause tests just for repetition. Next meaningful physical gate: corrected gyro signs and scale against known gentle rotations, followed by attitude estimation. This menu harness is not the calibrated attitude firmware; do not use its raw acquisition samples as calibrated drone rates. Review the unapproved production gyro coefficient edit before restoring that firmware. No PID flight acceptance follows from this input test.
