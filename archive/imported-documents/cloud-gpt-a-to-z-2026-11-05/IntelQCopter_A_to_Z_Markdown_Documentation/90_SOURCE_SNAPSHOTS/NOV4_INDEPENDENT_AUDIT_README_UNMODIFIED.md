# IntelQCopter - Independent Calibration Audit (2026-11-04)

This archive is an independent numeric re-analysis of the supplied raw session.
It does NOT import or reuse saved calibration parameters, the user's existing
`calibrate_sensors.py` fit routines, or another AI's numerical results.

Start with:
- `results/IntelQCopter_Independent_Raw_Calibration_Audit_20261104.pdf`
- `results/AUDIT_REPORT.md`
- `results/calibration_candidate_UNVERIFIED.json` and `.hpp` (NOT flight-verified)
- `results/independent_calibration_results.json` (full metrics)
- `results/corrected_capture_CANDIDATE.csv.gz` (derived 98,868 rows)
- `results/independent_crosschecks.json` (numeric integrity crosschecks)

Raw data:
- `inputs/iq_500hz_20261104_100907.csv` exactly the supplied main capture.
- `inputs/firmware/captures/iq_500hz_20261104_025733.csv` previous full session.
- `inputs/firmware/captures/iq_500hz_20261104_031731.csv` previous full session.
- `inputs/firmware/captures/iq_500hz_20261104_040827.csv` REST-only data; not a full MAG validation.
- `inputs/firmware/src/main.cpp`, `inputs/firmware/platformio.ini`,
  `inputs/firmware/sensor_config.json` to inspect exact logging configuration.

Recompute on a PC with Python 3.11+, NumPy, Pandas, SciPy and Matplotlib:

    python run_independent_audit.py
    python create_outputs.py
    python validate_outputs.py

For PDF generation, also install Mistune and WeasyPrint, then run:

    python make_report.py
    python build_pdf.py

The first script may take some time due to multiple robust fits and 50 block
bootstrap resamples. The `results/` directory contains finished outputs.

Key cautions:
1. The actual `platformio.ini` sets QMC5883P CTRL1=0x3D (200 Hz low OSR),
   despite the firmware ZIP name containing `100hz`. This is independently
   corroborated by ~195.5 Hz fresh captures.
2. The candidate 3x3 magnetometer matrix is a symmetric relative correction
   with determinant 1. It does not determine physical orientation, absolute
   Earth-field strength, magnetic declination, or in-flight motor interference.
3. GUI X/Y swap is a display-only reflection, not a body-frame rotation.
4. The six-face accelerometer correction assumes six EXACT orthogonal poses;
   hand-placed poses have unmeasured alignment errors.
5. The 60-sec gyro calibration is valid near its recorded operating temperature.
   Separate runs show bias shifts bigger than statistical noise.
6. NO AUTOMATIC FC CONFIG CHANGE is performed. Propellers off for further tests.
