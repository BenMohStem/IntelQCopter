# Failure history and recovered engineering lessons

## Magnetometer

v13 100 Hz `0xC9` occasionally worked ~90 Hz, other settings failed after configuration. v14–v18 attempted read modes, power-cycle control, Wi-Fi stress, bus loading and diagnostics; all tested combinations were not uniformly healthy. A later v13 test proved the chip could still output at ~90 Hz. Transplanting the original driver into full code was followed by the successful 4 Nov capture using **`0x3D` ~195 Hz**. This supports a configuration/interference sensitivity and successful integrated operation, not a completely determined root cause.

## Serial toolchain

PowerShell UTF-8 em dash parser error; malformed argument forwarding `-SkipLidar` into integer; empty `--upload-port`; COM port changing after USB reset; “port unavailable after 60 s”; COM busy, pyserial exclusive handle and Windows CH343 driver; `pio device monitor` incompatible with redirected stdin/stdout. Every error requires a targeted workflow fix, not a sensor reflash.

## Calibration math

Original full-ellipsoid MAG fit permitted a trivial solution and saved the wrong matrix interpretation. Original corrected plot collapsed due to multiplying by an ellipsoid equation matrix. Independent fit used all legitimate low-norm samples and validated on held-out rotations. A GUI axis reflection was caught and must not be copied into physical orientation correction.

## Modeling

Initial Claude book used HMC5883L as a described sensor and an illustrative 1.2 kg/6 N motor plant; real measured MAG is QMC5883P and actual aircraft mass/propulsion remain unknown. Earlier synthetic PI design produced excessive overshoot under assumed motor lag. These are model-review issues, not evidence of a flying system.
