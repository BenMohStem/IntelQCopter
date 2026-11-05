# Build and test the current FC v2 safely

## System requirements

Windows PowerShell 5.1+, Python 3, pyserial, optional pygame for Xbox, VS Code/PlatformIO (`espressif32@7.1.3`, `esp32-s3-devkitc-1`), USB CH343 driver and reliable USB power. The original project source is C++17 with modular headers.

## Safe recipe

1. Extract **v2** source to a new folder, no duplicate nested manual edits.
2. Confirm `IQ_MOTOR_OUTPUTS_ENABLE=0` and `IQ_WIFI_ENABLED=0` in `platformio.ini`.
3. Run `py -3 tests/test_project.py`, plus source/host tests documented in README.
4. Verify every sensor/ESC signal pin and remove propellers; disconnect ESC signal leads.
5. Run `.\tools\run_fc.ps1 -UploadFirmware` once; capture boot diagnostic and 30 seconds running.
6. Run `.\tools\run_fc.ps1` for repeats, and compare long-run timing.
7. Use `py -3 tools\xbox_uart_bridge.py --inspect` for input mapping before `-Xbox` virtual mode.

## What passing tests mean

Pure C++ numerical tests passed in prior authoring environment, including quaternion, Kalman, PID, CRC and compensation routines. Source syntax checks were performed off-device, not native physical ESP32 timing. PlatformIO compilation and board operation are still required. A successful capture is valuable data, not authorization to arm motors.

## If sensor regression appears

Reflash the preserved full calibration golden source and compare results on the same power state and wiring. If it works and FC v2 fails, the problem is in the new FIFO/clock/scheduling integration or its electrical interaction; don't immediately declare the QMC dead.
