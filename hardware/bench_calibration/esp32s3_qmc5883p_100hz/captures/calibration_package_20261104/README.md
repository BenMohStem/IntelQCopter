# IntelQCopter Calibration Package - 2026-11-04
## Session Summary

### Files Included:
1. **iq_500hz_20261104_100907.csv** - Raw calibration capture (33 MB, 98,868 rows)
2. **iq_500hz_20261104_100907.skipped.json** - Skipped stages (none skipped)
3. **calibration_20261104_100907.json** - Computed calibration parameters
4. **calibration_20261104_100907.mag.png** - Magnetometer 3D plot
5. **calibrate_sensors.py** - Calibration analysis script
6. **thought-database-2026-11-03.md** - Session context & safety rules

### Calibration Results:

**Accelerometer:**
- Bias: [1.2647, -0.0591, -0.6787] m/s²
- Scale matrix: 3x3 diagonal ~1.0 with small cross-coupling
- RMS residual: 0.0536 m/s²
- Max error: 0.2129 m/s²

**Magnetometer (diagonal fit - full ellipsoid was degenerate):**
- Hard-iron offset: [-30.29, -18.75, -36.25] µT
- Soft-iron gains: [0.9846, 0.9792, 1.0383]
- Fitted field radius: 40.79 ± 1.64 µT
- Raw field magnitude: 55.88 ± 20.85 µT

### Sensor Performance (from preflight):
- MPU: 500 Hz ✅
- MAG: ~196 Hz fresh at 400 kHz I2C ✅
- BMP: ~151 Hz (target 200 Hz, limited by conversion time)
- CRC errors: 0
- Sequence loss: 0

### Safety Notes:
- Physical motor arming: DISABLED
- Motor pins: INPUT ONLY
- Propellers: Removed for all captures
- No ESC output generated
