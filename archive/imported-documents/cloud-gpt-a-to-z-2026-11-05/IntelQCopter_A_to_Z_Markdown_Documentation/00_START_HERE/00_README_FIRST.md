# IntelQCopter — complete project knowledge base

**Snapshot:** 5 November 2026. **Scope:** the entire documented engineering effort through the **ESP32-S3 500 Hz fast calibrated FC v2** source and the successful 4 November sensor-calibration capture. This collection is intended for the project owner, local coding agents, reviewers and future contributors. It is not a flight clearance document.

## Where to begin

1. Read `01_CURRENT_STATUS.md`, `02_EVIDENCE_AND_CONFIDENCE.md` and `03_TIMELINE.md` in this folder.
2. For actual current firmware, start at `../04_FIRMWARE/03_FAST_FC_V2_ARCHITECTURE.md` and `../04_FIRMWARE/05_BUILD_CONFIGURATION.md`.
3. For physical measurements, read `../03_CALIBRATION/01_NOV4_CAPTURE_OVERVIEW.md` and the individual sensor audit documents.
4. For mathematics and the seven-loop controller, read `../05_ESTIMATION/` and `../06_CONTROL/`.
5. For the next experiments, use `../07_TESTING/01_BENCH_TEST_ORDER.md` and `../09_ROADMAP/01_IMMEDIATE_PRIORITIES.md`.

## Nonnegotiable distinctions

- **Observed:** The 4 Nov full calibration capture contains 98,868 records with live MPU6050, QMC5883P and BMP280 data. Its actual QMC control value was **0x3D**, nominal 200 Hz, despite an earlier package being named *100hz*.
- **Implemented in source:** FC v2 contains a 500 Hz nominal scheduler, 12-byte MPU FIFO acquisition, calibrated matrices, boot offset handling, quaternion estimator, 3-state altitude Kalman, seven virtual PID loops, Xbox-to-UART bench interface and safety checks.
- **Not demonstrated on physical hardware:** FC v2 executing its complete fast loop, gains controlling real motors, thrust mapping, ESC arming/failsafe, verified body-frame signs, integrated LiDAR and any successful free flight.
- **Estimated:** Vehicle mass, inertia, motor lag, thrust capacity and Monte Carlo uncertainty depend on assumptions rather than hardware weighing or a thrust stand.

## Quick source map

- `IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip` — most recent v2 bench-control source.
- `IntelQCopter_Calibration_20261104.zip` — successful sensor capture, original fitting program and saved outputs.
- `IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip` — independently refitted candidate constants and validation.
- `IntelQCopter_100Hz_Original_v13_Driver_Full_Calibration.zip` — transition from isolated working magnetometer to full GUI calibration.
- `IntelQCopter_Volume_I_Study_Edition2.pdf` — user-supplied theoretical study; **some sensor names and assumed plant constants do not describe the measured aircraft**.

## What this knowledge base is not

It does not silently modify firmware or GitHub. It is a readable, searchable set of `.md` documents. Statements about builds, simulation and motor-off tests are not evidence of flight suitability. Preserve your original raw files and commits; make changes on branches and retain logs.
