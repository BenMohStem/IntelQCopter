# Archived ESP32-S3 calibration notes

These two short notes were committed with the original calibration bench on
2026-11-03. They are retained for history and provenance; they are not current
launch instructions or proof of active firmware behavior.

- `LIDAR_GUI_CHANGES.txt` describes a historical version that exposed LiDAR
  range stages by default. The current GUI has 12 IMU/MAG/BMP phases and defers
  LiDAR calibration at the user's request.
- `MAG_REPAIR_NOTES.txt` describes proposed QMC repair changes. It does not
  establish that those changes were merged, flashed or validated. Use the
  chronological bench `VALIDATION_LOG.md` and source history for evidence.

SHA-256 values at archival time:

| File | SHA-256 |
|---|---|
| `LIDAR_GUI_CHANGES.txt` | `48E972C13C53D62018840F8B6796B4961885CC0F0D358F497B2CAEAEDDB967B7` |
| `MAG_REPAIR_NOTES.txt` | `3942DE0D8A943738F7C3AB186579C322CF1F0CF2608AF111CA1E4D9B7BBC0248` |
