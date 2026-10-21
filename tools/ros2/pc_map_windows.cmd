@echo off
call "%~dp0ros2_windows.cmd" python "%~dp0..\..\companion\pc_mission_map.py" --ros-args -r __ns:=/drone
exit /b %errorlevel%
