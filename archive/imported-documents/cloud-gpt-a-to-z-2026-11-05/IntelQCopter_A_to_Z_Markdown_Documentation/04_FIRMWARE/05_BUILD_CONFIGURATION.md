# PlatformIO v2 build flags and Windows workflow

## PlatformIO source settings

```ini
[platformio]
default_envs = iq-fc-safe
[env:iq-fc-safe]
platform = espressif32@7.1.3
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 921600
build_unflags = -std=gnu++11
build_flags =
  -std=gnu++17
  -DCORE_DEBUG_LEVEL=0
  -DIQ_I2C_HZ=400000
  -DIQ_WIFI_ENABLED=0
  -DIQ_MAG_CONTROL1=0x3D
  -DIQ_MOTOR_OUTPUTS_ENABLE=0
  -DIQ_AIRFRAME_PROFILE=0
  -DIQ_LIDAR_ENABLE=0
  -DIQ_LIDAR_RX_PIN=-1
```

## Usage — safe bench

```powershell
py -3 -m pip install -r requirements.txt
.\tools\run_fc.ps1 -UploadFirmware  # build, upload, log
.\tools\run_fc.ps1                  # log only
.\tools\run_fc.ps1 -Port COM3       # override auto-detection
py -3 tools\xbox_uart_bridge.py --inspect
.\tools\run_fc.ps1 -Xbox           # gamepad virtual-input mode
```

No root `START_*.ps1` is required. Close every other COM program before flashing or running the bench bridge; only one process can own the CH343 at a time. The USB upload baud of 460800 and UART running baud of 921600 are separate channels/settings. Windows PowerShell 5.1 encoding (UTF-8 BOM vs em dash) caused older script parsing failures; v2 retains an ASCII-friendly launcher.

## Do not silently change

Disable `IQ_MOTOR_OUTPUTS_ENABLE` guard only after independent actuation design/review; the project is expressly not ready for that. Do not set LiDAR enable with unknown RX pin. Do not force 200 Hz BMP fresh status by just polling faster. Preserve the proven QMC profile while benchmarking bus speed.
