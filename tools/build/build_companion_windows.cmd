@echo off
if defined PIXI_PROJECT_ROOT goto activated
"C:\WinLibs\pixi\pixi.exe" run --manifest-path C:\dev\pixi.toml cmd /d /c "%~f0" %*
exit /b %errorlevel%
:activated
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
if errorlevel 1 exit /b 1
call C:\dev\jazzy\install\local_setup.bat
set CMAKE_BUILD_PARALLEL_LEVEL=6
set taskRoot=%~dp0..\..
cd /d "%taskRoot%"
colcon --log-base build\companion-log build --base-paths companion\ros2_ws\src --build-base build\ros2 --install-base build\ros2-install --merge-install --executor sequential --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DOpenCV_DIR=C:/dev/.pixi/envs/default/Library/cmake/x64/vc16/lib -DCMAKE_SYSTEM_PROCESSOR=AMD64
exit /b %errorlevel%
