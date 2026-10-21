@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
if errorlevel 1 exit /b 1
cd /d C:\dev\jazzy
set CMAKE_BUILD_PARALLEL_LEVEL=6
call C:\dev\jazzy\install\local_setup.bat
rem gz-cmake installs under share/cmake, which this native vendor build did not discover.
set gz-cmake3_DIR=C:/dev/jazzy/install/opt/gz_cmake_vendor/share/cmake/gz-cmake3
set CMAKE_PREFIX_PATH=C:\dev\jazzy\install\opt\gz_cmake_vendor;C:\dev\jazzy\install\opt\gz_utils_vendor;C:\dev\jazzy\install\opt\gz_math_vendor;C:\dev\jazzy\install\opt\rviz_ogre_vendor;%CMAKE_PREFIX_PATH%
colcon build --merge-install --executor sequential --event-handlers console_direct+ --packages-select gz_cmake_vendor gz_utils_vendor gz_math_vendor rviz_ogre_vendor rviz_rendering rviz_rendering_tests rviz_common rviz_visual_testing_framework rviz_default_plugins rviz2 --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
exit /b %errorlevel%
