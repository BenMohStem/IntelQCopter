# Windows tooling, launchers, COM detection and known regressions

## Preferred user experience

Only use the scripts inside `tools/`, not a collection of root `START_*.ps1` files. For full sensor calibration: `tools/start_calibration.ps1`. For FC v2 bench: `tools/run_fc.ps1`. Both support auto-selecting the CH343 when visible, with optional `-Port COM3` override.

## Historical bugs worth remembering

| Problem | Cause/correct response |
|---|---|
| PowerShell “string is missing terminator” | Windows PowerShell 5.1 misinterpreted non-ASCII UTF-8 without BOM; use ASCII/BOM scripts |
| `-SkipLidar` interpreted as `GyroSamples` int | Bad positional forwarding; use named options or named splatting |
| `--upload-port requires an argument` | Launcher passed blank variable; resolve port *before* building command |
| GUI didn't offer LiDAR | Launcher forced skip when second COM unspecified; GUI now has skip control |
| `pio device monitor` refused redirection | It expects an interactive terminal; use pyserial logger when saving to file |
| COM3 vanished after cold boot | Windows USB re-enumeration/reset; detect again and don't pretend fixed COM number |
| USB serial busy/permission denied | Another process has exclusive COM handle; close it and use `finally` cleanup |

## How to debug

Confirm driver in Device Manager, enumerate `pyserial.tools.list_ports`, close monitors, use a sane reset/power-cycle procedure and log exact errors. Distinguish failure to open a serial port from a firmware sensor-read failure. A program that prints “upload success” but then zero frames is not equivalent to a broken magnetometer.
