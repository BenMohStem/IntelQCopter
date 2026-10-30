# Live attitude preview, first visible step

This first preview displays calibrated accelerometer gravity tilt and corrected body yaw rate. It is NOT yet the fused Kalman attitude estimate: translational acceleration corrupts gravity tilt. It reuses the calibrated runtime in an isolated single main.cpp; the user's production coefficient edit is preserved. This snapshot uses the historical proper-rotation coefficient0.288364175, consistent with config/gyro-frame.json. The altered0.2808364175 value is not an accepted rotation update.

Build: .\qcopter.ps1 build-attitude-preview
Upload when ready: .\qcopter.ps1 upload-attitude-preview --upload-port COM3
View: .\qcopter.ps1 view-attitude --port COM3

Close other serial users. Gently tilt left/right and forward/back, then return. Stop once the curves respond and return near their starting values. One short session, no calibration recording required. Logs save automatically; closing the plot ends recording. Physical axis labels remain provisional. No Xbox integration, Kalman fusion, PID or motor outputs in this preview. Next small change is gyro/accelerometer fusion after this display works.
