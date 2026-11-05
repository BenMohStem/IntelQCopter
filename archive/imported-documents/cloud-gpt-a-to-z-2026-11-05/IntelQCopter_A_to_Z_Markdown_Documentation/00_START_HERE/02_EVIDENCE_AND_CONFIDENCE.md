# Evidence rules, verification levels and claim ledger

Use these terms consistently in issues, PRs, reports and AI prompts.

| Label | Meaning | Example |
|---|---|---|
| **MEASURED** | Observed on the real board with a saved instrument trace | 4 Nov calibration CSV has 98,868 records |
| **REPRODUCED** | Independently recomputed using the raw measurement and another method | 3×3 QMC calibration reproduced by nonlinear and algebraic fits |
| **CONFIGURED** | Value appears in actual program settings | `IQ_MAG_CONTROL1=0x3D` in successful snapshot |
| **IMPLEMENTED** | Algorithm present in source, host tests may pass | FIFO and altitude KF in FC v2 |
| **HOST TESTED** | Unit test or mock compilation without ESP32 hardware | Bosch integer compensation tests |
| **SIMULATED** | Output of a declared idealized plant | Controller overshoot, hover-command statistics |
| **ASSUMED** | Unverified numeric parameter | 6 N/motor max thrust, 40 ms lag, vehicle mass prior |
| **OPEN** | Unverified or contradictory | Motor order, actual axis map, LiDAR UART RX pin |

## A test's chain of evidence

`source commit -> build flags -> toolchain -> board and wiring -> instrument/log -> parser -> metrics -> acceptance criterion`.

Missing steps limit conclusions. An I²C ACK is not a valid sample. A register readback is not a sustained sampling test. An IMU count of 500 Hz is not evidence that all FC computations completed before 2 ms. A valid Monte Carlo output is conditional on its input distributions.

## Warning examples

- *Incorrect:* “QMC hardware is certified at 200 Hz.”
- *Defensible:* “The QMC provided ~195 fresh samples/s during the Nov 4 calibration capture with `0x3D`; earlier startup failures were documented.”
- *Incorrect:* “The drone weighs 1.104 kg.”
- *Defensible:* “The geometry model estimates 1.104 ± 0.051 kg before real weighing.”
- *Incorrect:* “PID v2 is tuned.”
- *Defensible:* “All 21 gains exist in source; candidate rates are derived from assumed inertia and validated in a synthetic plant only.”

Never silently carry an assumption from a study PDF into a flight-control constant without retaining its provenance.
