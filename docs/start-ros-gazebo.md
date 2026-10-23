# Open ROS2, RViz and Gazebo yourself

The original blocks below use **Command Prompt (`cmd.exe`)**. If your terminal says `PS`, use the PowerShell alternative in section 1 instead, or enter `cmd` first. Enter the commands in order, waiting for `pixi shell` to show its activated prompt before continuing.

There is no Python `import` here: Pixi activates an environment, and the ROS batch file loads ROS commands into that terminal.

## 1. ROS2 and RViz — first terminal

```bat
cd /d C:\dev
C:\WinLibs\pixi\pixi.exe shell
call C:\dev\jazzy\install\local_setup.bat
set "PATH=C:\dev\jazzy\install\opt\gz_math_vendor\bin;%PATH%"
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set "QT_QPA_PLATFORM_PLUGIN_PATH=C:\dev\.pixi\envs\default\Library\plugins\platforms"
ros2 run rviz2 rviz2
```

RViz should open. RViz visualises ROS data; it is not a physics simulator. An empty view or missing fixed-frame warning is expected before a robot/TF/data publisher is configured. No drone is loaded by these commands.

For another ROS2 terminal, repeat the activation/setup lines, then use commands such as `ros2 topic list` or `ros2 node list`. Keep the same middleware/domain settings for nodes you want to communicate.

### PowerShell alternative for ROS2/RViz

In a fresh PowerShell terminal, enter these first two lines and wait for the activated Pixi prompt:

```powershell
cd C:\dev
& C:\WinLibs\pixi\pixi.exe shell
```

Then enter:

```powershell
. C:\dev\jazzy\install\local_setup.ps1
$env:PATH = "C:\dev\jazzy\install\opt\gz_math_vendor\bin;" + $env:PATH
$env:RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
$env:QT_QPA_PLATFORM_PLUGIN_PATH = "C:\dev\.pixi\envs\default\Library\plugins\platforms"
ros2 run rviz2 rviz2
```

The leading dot loads the ROS PowerShell setup into this session. `$env:NAME = "value"` sets an environment variable inherited by programs launched here. CMD's `call` and `set NAME=value` are not the PowerShell equivalents. These settings affect this terminal only.

### If RViz reports a GLSL/OpenGL error

The latest supplied console output contains `rviz/glsl120/black.frag` followed by repeated `Error using GLSL Program Object : GL_INVALID_OPERATION`. It no longer contains the earlier DLL-load error126. This is a rendering failure; the text `Validation successful` does not cancel the accompanying OpenGL error. Its root cause has not been established.

First relaunch with the correct environment block for your terminal above. If the same error returns, close RViz and launch with Ogre logging:

```powershell
ros2 run rviz2 rviz2 -l
```

The local RViz source confirms `-l` enables Ogre logging. Preserve the first error and the Ogre renderer/vendor/version information for diagnosis. Do not publish full logs without reviewing private paths and environment details. This is a diagnostic run, not a verified graphics fix. No graphics driver or ROS installation was modified. The installed CLI does not expose the older RViz `--opengl 210` option.

## 2. Activate Gazebo — two fresh terminals

Open **two new Command Prompt windows**. Run this same block in **both**:

```bat
cd /d C:\Users\Mohamed\Desktop\Work\QCopter\sim\gazebo
C:\WinLibs\pixi\pixi.exe shell
set "QT_QPA_PLATFORM_PLUGIN_PATH=%PIXI_PROJECT_ROOT%\.pixi\envs\default\Library\lib\qt6\plugins\platforms"
set "QML2_IMPORT_PATH=%PIXI_PROJECT_ROOT%\.pixi\envs\default\Library\lib\qt6\qml"
```

This reuses your installed Gazebo environment. It is a runtime installation location, not the new firmware source folder. The two `set` lines correct the GUI paths **in this terminal only**: the installed manifest currently leaves `$CONDA_PREFIX` as literal text. No installation files need to be edited for this lesson.

In the **first Gazebo terminal**, start the physics server with the supplied example world:

```bat
gz sim -v 4 -s -r shapes.sdf
```

Leave it running. In the **second Gazebo terminal**, open the window:

```bat
gz sim -g
```

You should see the built-in shapes world. `-s` means server; `-g` means GUI; `-r` starts time running; `-v 4` shows detailed logs. This is a packaged example, not a drone simulation or proof of the FC.

## 3. Stop when finished

Close the Gazebo/RViz window. Press `Ctrl+C` in a running server terminal to stop it. Use `exit` to leave a Pixi shell.

## What is and is not connected

ROS2/RViz and Gazebo are separate at this stage. Opening both does **not** connect ROS topics to Gazebo or run the ESP32 code. The bridge, robot model, sensors and controller connection will each be a later small lesson that you build yourself. No custom world, bridge or flight simulation was generated here.

Local inspection on23October2026 found the ROS2 setup batch, RViz executable, Gazebo Qt plugin, and shapes.sdf. `ros2 pkg prefix rviz2` returned `C:\dev\jazzy\install`; `gz sim --versions` returned `10.5.0`. No GUI or simulation was launched to validate these instructions. Gazebo10is Jetty; do not assume a Jazzy/Harmonic bridge binary automatically matches this installation.

References: [ROS environment setup](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html), [Gazebo CLI](https://gazebosim.org/libs/sim/), [Windows server/GUI instructions](https://gazebosim.org/docs/harmonic/install_windows/). The Windows page describes Harmonic; the local installation/version and paths above were inspected separately.

## If RViz reports LoadLibrary error126

This installation has `gz-math7.dll` in a vendor subfolder that the current setup does not add to PATH. In the already activated ROS2 Command Prompt, after closing the failed RViz window, run:

```bat
set "PATH=C:\dev\jazzy\install\opt\gz_math_vendor\bin;%PATH%"
ros2 run rviz2 rviz2
```

PATH tells Windows where to find runtime DLLs; it is separate from the Qt display-plugin path. The DLL is already present, so this correction adds no package and rebuilds nothing. It affects this terminal only. Repeat it after `local_setup.bat` in new ROS terminals (included above).

Local DLL-load verification changed from failure126 to success0 after adding only that folder. Original console output and diagnostic logs are retained privately under `.local/rviz-loader-20261023/`; they are not published. The full GUI was not launched by the agent. The `Stereo is NOT SUPPORTED` line is not the failed DLL-load report shown here.
