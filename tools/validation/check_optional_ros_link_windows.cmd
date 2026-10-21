@echo off
call "%~dp0..\ros2\ros2_windows.cmd" python "%~dp0..\..\simulation\gazebo\check_optional_ros_link.py" %*
exit /b %errorlevel%
