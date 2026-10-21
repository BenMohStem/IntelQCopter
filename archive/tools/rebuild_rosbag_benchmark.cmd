@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
call C:\dev\jazzy\install\local_setup.bat
cd /d C:\dev\jazzy
rem Keep the existing install, use a shorter object path for these long package names.
colcon --log-base C:\dev\benchmark-log build --build-base C:\dev\b --merge-install --packages-select rosbag2_performance_benchmarking_msgs rosbag2_performance_benchmarking --executor sequential --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_OBJECT_PATH_MAX=180
exit /b %errorlevel%
