@echo off
if defined PIXI_PROJECT_ROOT goto activated
"C:\WinLibs\pixi\pixi.exe" run --manifest-path C:\dev\pixi.toml cmd /d /c "%~f0" %*
exit /b %errorlevel%
:activated
rem Run through pixi run --manifest-path C:\dev\pixi.toml cmd /c this-file ...
call C:\dev\jazzy\install\local_setup.bat
if exist "%~dp0..\..\build\ros2-install\local_setup.bat" call "%~dp0..\..\build\ros2-install\local_setup.bat"
set QT_QPA_PLATFORM_PLUGIN_PATH=C:\dev\.pixi\envs\default\Library\plugins\platforms
set ROS_DOMAIN_ID=64
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
call %*
exit /b %errorlevel%
