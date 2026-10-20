@echo off
cd /d "%~dp0.."
C:\WinLibs\pixi\pixi.exe run --manifest-path C:\dev\pixi.toml cmd /d /c tools\ros2_windows.cmd ros2 launch qcopter_description view.launch.py
