@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
if errorlevel 1 exit /b 1
call C:\dev\jazzy\install\local_setup.bat
set CMAKE_BUILD_PARALLEL_LEVEL=6
set taskRoot=%~dp0..
cd /d "%taskRoot%"
colcon --log-base build\companion-log build --base-paths companion\ros2_ws\src C:\dev\jazzy\src\vision_opencv\cv_bridge --build-base build\ros2 --install-base build\ros2-install --merge-install --executor sequential --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCV_BRIDGE_DISABLE_PYTHON=ON -DOpenCV_DIR=C:/dev/.pixi/envs/default/Library/cmake/x64/vc16/lib -DCMAKE_SYSTEM_PROCESSOR=AMD64
exit /b %errorlevel%
