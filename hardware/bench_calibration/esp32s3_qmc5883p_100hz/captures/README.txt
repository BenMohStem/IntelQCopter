This folder retains diagnostic captures, failed trials, audit packages and
other historical evidence. Legacy root-level `iq_500hz_*.csv` GUI sessions
and their old skip manifests were moved to
`measurements/calibration/esp32s3/archive/legacy_gui_captures/` with a SHA-256
index. New GUI sessions are written under `measurements/calibration/esp32s3/`.
Run tools/calibrate.py on a completed session CSV to create calibration outputs
under `measurements/calibration/esp32s3/analysis/`.
