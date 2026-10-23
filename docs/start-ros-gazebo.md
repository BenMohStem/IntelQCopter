# Open ROS2, RViz and Gazebo yourself

Use **Command Prompt (`cmd.exe`)**, not PowerShell, for these commands. If your terminal says `PS`, enter `cmd` first. Enter the commands in order, waiting for `pixi shell` to show its activated prompt before continuing.

There is no Python `import` here: Pixi activates an environment, and the ROS batch file loads ROS commands into that terminal.

## 1. ROS2 and RViz — first terminal

```bat
cd /d C:\dev
C:\WinLibs\pixi\pixi.exe shell
call C:\dev\jazzy\install\local_setup.bat
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set "QT_QPA_PLATFORM_PLUGIN_PATH=C:\dev\.pixi\envs\default\Library\plugins\platforms"
ros2 run rviz2 rviz2
```

RViz should open. RViz visualises ROS data; it is not a physics simulator. An empty view or missing fixed-frame warning is expected before a robot/TF/data publisher is configured. No drone is loaded by these commands.

For another ROS2 terminal, repeat the activation/setup lines, then use commands such as `ros2 topic list` or `ros2 node list`. Keep the same middleware/domain settings for nodes you want to communicate.

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
