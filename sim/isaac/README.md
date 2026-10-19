# QCopter Isaac Sim tests

This folder defines the Isaac Sim path for testing QCopter. Isaac Sim 4.1.0 is installed in the project-local Python environment at `../../.venv-isaacsim` (Python 3.10) with its extension caches. The minimal Kit app reported `app ready`, but the full and headless Isaac Sim experiences both crashed during extension startup with Windows exit code `0xC0000005`; no quadcopter physics simulation has run. No existing USD/USDA/URDF scene was found, so a starter URDF is provided below.

The host is Windows 10 with an RTX 3050 Laptop GPU (about 4 GB VRAM) and about 28 GB system memory. NVIDIA lists RTX 3070/8 GB VRAM and 32 GB RAM as Isaac Sim 4.1 minimums. Current releases require still newer Windows/GPU hardware. See NVIDIA's [4.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/4.1.0/installation/requirements.html) and [6.0.0 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html). The system therefore falls below the minimum in GPU memory and system RAM.

## Local install and startup status

The project-local environment has Isaac Sim 4.1.0, the 4.1 extension-cache packages, CUDA-enabled PyTorch 2.2.0+cu121, and boto3/botocore versions matched to the bundled pip archive. The separate TensorFlow environment was not modified. To retry the graphical application after addressing the hardware/runtime issue, open PowerShell in the project root and run:

```powershell
$env:OMNI_KIT_ACCEPT_EULA = "YES"
.\.venv-isaacsim\Scripts\isaacsim.exe omni.isaac.sim
```

The EULA variable only records the user's acceptance for the process. Tested headless experiences terminated with `0xC0000005` during startup, so importing the quadcopter URDF or ROS 2 testing has not yet been possible. Isaac Sim 4.1 remains installed for a future supported workstation; if this PC cannot run it reliably, use Gazebo for local physics/ROS 2 testing and keep Isaac Sim for a compatible RTX workstation.

## Starting point

An initial URDF has been added at [`assets/qcopter.urdf`](assets/qcopter.urdf). The editable parametric visual model is [`../model/qcopter_f450.scad`](../model/qcopter_f450.scad); it renders the center plates, four colored arms, motors, 9.4 inch visual prop discs, battery, controller, and two skids. These are visual dimensions, not a calibrated physics model. The URDF gives a visual X-frame with motor link locations based on a provisional 450 mm motor-to-motor diagonal. Its 1 kg body mass and inertia values are placeholders, propeller geometry/motor dynamics are not physically modeled, and the URDF by itself does not create functional rotors. In Isaac Sim, import it as a robot, check orientation/dimensions, replace geometry/mass/inertia with measurements, then add four rotor actuator/force models and simulated IMU, barometer, magnetometer, and range sensors.

For current NVIDIA releases, this PC cannot run Isaac Sim locally: Windows 10 and the RTX 3050 Laptop GPU with 4 GB VRAM fall below current minimums (Isaac Sim 6.0.0 lists Windows 11 and an RTX 4080 with 16 GB; the older 4.1 minimum is RTX 3070/8 GB and 32 GB system RAM). See [6.0.0 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html) and [4.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/4.1.0/installation/requirements.html). Installing the large simulator package here would not make it runnable. The practical path is to install Isaac Sim on a supported Windows 11/Linux workstation with a compatible RTX GPU, then open this URDF there; the rest of this repository can be shared with that machine.

Use NVIDIA's [Isaac Lab multirotor contribution](https://github.com/isaac-sim/IsaacLab/tree/develop/source/isaaclab_contrib) for a four-thruster vehicle with motor response and thrust allocation. NVIDIA currently describes this contribution as community/experimental code, and its full Isaac Sim workflows depend on matched Isaac Sim and Isaac Lab versions. Pin a compatible release pair before adding it as a dependency.

For an existing visual quadrotor model and MAVLink/PX4 HIL route, [Robotsix-UAV/rs_isaac_uav_sim](https://github.com/Robotsix-UAV/rs_isaac_uav_sim) is another reference. QCopter is not PX4 firmware, so that repo's MAVLink bridge is not directly compatible; reuse its aircraft/scene ideas, not its FC interface.

Isaac Sim's physics IMU provides simulated acceleration and angular velocity. It can be sampled at physics steps and noise can be applied in a post-processing layer. Simulate the BMP280 and HMC5883L as software sensor models derived from ground truth, with realistic update rates, noise, bias, quantization, and dropout. The CC1101 and LoRa RF links should initially be modeled as packet delay/loss/latency, not electromagnetic propagation.

## Recommended HIL boundary

Keep all flight logic shared between hardware and simulation. Add a compile-time `SIM_IO` backend behind the same sensor/motor interfaces:

1. RP2350 firmware receives timestamped simulated sensor frames over USB CDC/UART and feeds them through the normal estimator/controller path.
2. Firmware emits four normalized motor commands plus status/telemetry frames.
3. A host bridge in Isaac Sim maps the motor commands to simulated thruster speeds and sends IMU, pressure/altitude, and magnetic field observations back to firmware.
4. Hardware builds use the real I2C sensors and ESC output backend; simulation builds must not drive real ESC outputs.

This tests the actual compiled C control loop in a HIL-style setup. It does not model I2C bus timing, actual RF propagation, ESC commutation, or the precise motor/propeller thrust curve unless those models are added and calibrated.

## Scenario checklist

1. **Bench/arming:** boot, sensor startup, arm/disarm transitions, low-throttle gating, invalid-sensor rejection.
2. **Rate inner loops:** roll, pitch, and yaw rate steps; measure overshoot, settling time, saturation, and integrator recovery.
3. **Angle mode:** bounded roll/pitch setpoints, disturbance rejection, and recovery from a tilt impulse.
4. **Yaw reference:** yaw-rate commands with magnetometer heading corrections; inject magnetic disturbance and compass dropout.
5. **Altitude hold:** capture altitude, climb/descend step, barometer noise/bias, pressure spike, and acceleration during tilt.
6. **Motor/link faults:** one motor saturation/failure, delayed/lost/duplicated RC packets, stale controller input, and sensor disconnect.
7. **Timing:** run the controller at 500 Hz; log measured loop intervals, missed deadlines, sensor age, and worst-case execution time. Simulator wall-clock speed is not a substitute for MCU cycle/deadline measurements.
8. **Autonomy/companion faults:** send bounded ROS 2 setpoints, then interrupt the companion link; inject YOLO false/missing detections, lidar dropout/invalid range, GPS dropout and stale position; confirm the flight controller rejects unsafe setpoints and enters the configured degraded mode.

Do not tune based on an uncalibrated vehicle model. The 40 A ESC label gives current rating only; thrust limits and motor dynamics require the actual motor/propeller data or bench measurements.

## Version and execution notes

- Confirm the user's installed Isaac Sim version and host GPU/driver before choosing an Isaac Lab tag. The multirotor contribution changes with Isaac Lab releases.
- The separate [c1570/rp2350js](https://github.com/c1570/rp2350js) emulator remains useful for RP2350 boot/firmware-level checks. It is not Isaac Sim and cannot supply quadrotor physics or external I2C devices.
- The ROS 2 topic/frame contract for the simulator and companion interface is in [`ros2_interface.md`](ros2_interface.md). ROS 2 is not installed in the current environment, and the full simulator did not start, so no ROS/Isaac simulation result is claimed.
- A local boot-only check for the RP2350 build is available at `sim/rp2350js/boot_check.cjs`; it uses the emulator API directly so it avoids the daemon's Unix-socket CLI path on Windows. Run it with `node sim/rp2350js/boot_check.cjs build/flight-rp2350/qcopter.hex 1000000`. The emulator's unsupported timer/interrupt/PWM behavior prevents this from validating the 500 Hz loop or motor outputs.
- Isaac Sim supports scripted scene control and physics IMU sensors. Begin with the already available scene, configure its mass as a clearly marked provisional value, and test software behavior independent of final tuning. Update mass, inertia, motor/propeller thrust curves, and geometry when measured. The HIL bridge and runnable firmware scenarios require the flight firmware and simulation I/O protocol first.
