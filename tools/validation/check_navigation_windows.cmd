@echo off
cd /d "%~dp0..\.."
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /W4 /I companion\ros2_ws\src\qcopter_autonomy\include companion\tests\navigation_regression.cpp companion\ros2_ws\src\qcopter_autonomy\src\navigation.cpp /Fe:build\navigation-regression.exe /Fo:build\
if errorlevel 1 exit /b 1
build\navigation-regression.exe
