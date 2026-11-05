# Calibration file formats and provenance

## Full session files

- `iq_500hz_20261104_100907.csv`: authoritative raw/engineering-unit time series from the successful capture.
- `calibration_20261104_100907.json`: output of original calibration program; **not automatically authoritative for MAG 3×3** because of its fitting error.
- `iq_500hz_20261104_100907.skipped.json`: skipped-stage metadata; stage status should not be inferred merely from missing CSV columns.
- `calibration_20261104_100907.mag.png`: original plot (MAG correction method later found to be wrong).
- `calibrate_sensors.py`: original algorithm, useful for history and regression, not a validation oracle.

## Independently computed files

- `calibration_candidate_UNVERIFIED.json`: carefully unit-labeled gyro, accel and MAG candidate constants.
- `independent_calibration_results.json`: detailed fit statistics and validation.
- `calibration_candidate_UNVERIFIED.hpp`: C++ constants for integration after axis validation.
- `corrected_capture_CANDIDATE.csv.gz`: derived outputs for all 98,868 rows.
- `IntelQCopter_Independent_Raw_Calibration_Audit_20261104.pdf`: methods and warnings.

## Storage rules

Keep originals immutable; preserve timestamps, sample flags, raw int16 registers and units. Name transformed outputs as **derived** and pin the source commit/config in metadata. Do not mix `C_accel` correction matrix with the inverse of a least-squares normal matrix; do not mix normalized MAG shape calibration with a truly calibrated absolute field strength.
