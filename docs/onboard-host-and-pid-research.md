# Onboard N150 Plus and PID tuning research - 2026-10-28

## Feasibility decision

The user proposes mounting the Samsung N150 Plus motherboard onboard, without its own battery and display, using regulated flight-battery power, SSH, wireless ROS 2 and its built-in camera. Keep this as a conditional engineering candidate rather than a selected flight computer.

The exact model/CPU/RAM are not identified. If it is the Atom N450 variant, Intel specifies a 1.66 GHz single-core/two-thread x86-64 CPU and up to 2 GB DDR2-667. A 32-bit installed OS does not prove a 32-bit-only CPU. Confirm the model suffix, `lscpu` and installed RAM before buying memory; do not promise more than that CPU's limit. [Intel product brief](https://www.intel.com/content/dam/www/public/us/en/documents/product-briefs/atom-netbooks-brief.pdf). Samsung lists model-specific camera/software variants, but that is not Linux support evidence for this particular webcam: [Samsung N150 support example](https://www.samsung.com/uk/support/model/NP-N150-JP05DE/).

Proposed workload: minimal 64-bit Linux, SSH, telemetry/parameter bridge and low-rate camera acquisition. Start with a camera-advertised low-resolution mode and approximately 5 fps as an experiment, then measure CPU, RAM, dropped frames, end-to-end age and thermal throttling. This is a proposed test setting, not measured capability. Avoid a local desktop, Gazebo, RViz and full-stack builds on 1 GB. Cross-build or build compatible artifacts on the stronger host, without compiling for ISA extensions the Atom lacks. RAM expansion helps capacity, not CPU instruction support or processing throughput.

Jazzy targets Ubuntu 24.04 amd64/arm64 at Tier 1; Debian 12 amd64 is Tier 3. Do not mix Ubuntu ROS apt repositories into Debian. Identify Debian version before choosing a source build or matching supported Ubuntu container. Debian 13 is not the same supported Jazzy target. The N150 fallback is lightweight Linux capture/telemetry with ROS processing on the stronger host. A container does not add CPU instructions or RAM. [Official REP-2000 source](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst). Standard TensorFlow binaries use AVX and may not run on old CPUs; the N150 is not the Windows GPU TensorFlow machine. [TensorFlow hardware requirements](https://www.tensorflow.org/install/pip).

Debian 13 no longer provides an official i386 kernel/installer. Verify 64-bit CPU support before choosing an image; do not confuse low RAM with CPU bitness. [Debian i386 status](https://www.debian.org/ports/i386/).

## Display removal and camera

First demonstrate reliable headless boot, network recovery and SSH on the bench with cooling intact. Identify/retain required power-button circuitry and Wi-Fi antenna leads; avoid accidentally discarding lid-mounted antenna or webcam hardware when removing the display. Verify whether removal changes lid/power behavior.

Use `lsusb`, `lspci -nn`, `v4l2-ctl --list-devices` and supported camera format enumeration. UVC-compatible devices can use the Linux uvcvideo/V4L2 stack, but the module's actual interface/driver must be identified. Keep the original camera board/cable until its connector and supply are known; do not assume a detached camera accepts 5 V or 19 V. [Linux UVC driver documentation](https://www.kernel.org/doc/html/latest/userspace-api/media/drivers/uvcvideo.html).

Then calibrate lens intrinsics/distortion and camera-to-drone pose, record acquisition timestamps and characterize exposure/rolling shutter, vibration, blur, compression delay and packet loss. Webcam frames alone do not establish metric drift-free localization or arbitrary-obstacle clearance. Preserve USB/device inventories and original camera samples locally; no image capture has yet been performed.

## Power, payload and DC converter gate

The user's 19 V / 2.1 A label implies a 39.9 W adapter rating; it is not a measurement of continuous board consumption. Confirm connector polarity/input tolerance and measure boot, CPU, camera and Wi-Fi load peaks with a bench supply before selecting a converter.

A boost-only converter is suitable only if the entire input range stays below the required output. If the pack spans both sides of 19 V, evaluate a regulated buck-boost; a higher-voltage pack instead needs a suitable buck design. Cell count, chemistry, charged voltage and minimum allowed voltage are still unknown. Use `I_input = P_output / (efficiency * V_pack_min)` for an initial budget, followed by actual transient/thermal tests. As an illustration only, 39.9 W at 9 V and 85% efficiency is about 5.2 A input before transient margin; this is not the actual battery or a module recommendation.

Retain cooling; assess fusing, insulation, strain relief, EMI/ripple, shared USB/UART grounding and possible power backfeed. Measure stripped mass, mounting, CG and inertia and update the flight model only from those measurements. Verify adequate thrust/hover margin under 45/65% limits; replacing the battery/display with a converter does not automatically make the payload viable. Do not wire or buy a converter from the adapter label alone.

## PID tuning server plan

The common server is a Linux/Windows host application, not an assumed HTTP stack inside the RP2350. It can initially run on the stronger workstation and later on the N150 if benchmarks allow. Both MCU variants retain their native control loops; the host talks through interchangeable ESP32/RP2350 adapters over verified USB/UART framing.

1. Read-only milestone: publish firmware/profile hash, loop period, calibration status, pilot freshness, arming/fault state, gain snapshot, setpoint/measured rate/angle, PID terms, normalized motor output, saturation and execution timing. Record original acquisition times/sequence numbers; decimate the dashboard independently of identification logs. Use bounded queues and never stall the FC for a slow client.
2. Wire contract: version, message type, sequence, bounded payload, CRC, ACK/error and explicit units/axis convention. Budget transport capacity before promising full-rate logs; e.g. 64-byte records at 500 Hz and 10 serial bits/byte need 320 kbit/s before margin. The existing low-rate serial printouts are not this protocol. RP GPIO/radio/ESC adapters and production configuration writers remain pending.
3. Candidate transaction: obtain the active gain revision; stage finite bounded values with units and source/profile identity; preview a diff; require matching revision and **FC-confirmed disarmed state** to atomically apply. MCU verifies and returns readback/CRC before persistence. Reset relevant integrators safely and retain a known-good snapshot. Reject stale, malformed, cross-profile or partial updates. A host failure cannot arm, change caps or grant motor authority.
4. Proposed local API: `GET /v1/status`, `GET /v1/pid`, bounded telemetry stream, `POST /v1/pid/validate`, disarmed-only `POST /v1/pid/apply`, snapshot/restore and offline candidate export. These are designs, not running endpoints. No arm/direct-motor endpoint. Keep 10/45/65% policy outside tuning permissions.
5. Wireless access: bind to localhost initially and use SSH tunneling from the ground station. Do not require a modern browser on 1 GB N150. Store credentials outside Git. Validate disconnect/reconnect, source/revision mismatch, malformed input, queue overflow, save failure and power-loss rollback before a write-capable service is enabled.

## Auto-PID approaches and decision

| Approach | What research supports | Project decision |
|---|---|---|
| Bounded excitation + model identification | Estimate per-axis rate dynamics/delay from input/output records, then assess candidate loop gains and stability/robustness. PX4's documented autotune is a useful reference. | Preferred research path: offline identification and held-out tests first; fixture/airframe differences must be explicit. No generic gains without measured dynamics. |
| Relay/limit-cycle identification | Astrom/Hagglund estimate critical gain/frequency using an automatically generated excitation. | Compare in a model or suitable controlled fixture; deliberately inducing oscillations on the damaged aircraft is not the initial experiment. |
| IMC/SIMC model-based rules | Skogestad derives interpretable tuning from reduced time-delay models. | Candidate baseline only if the fitted model class and coupling/delay assumptions are defensible; test sensitivity to uncertainty and saturation. |
| Autopilot flight autotune | ArduPilot/PX4 start from an already stabilizing/flyable vehicle and use maneuvers to identify/tune it. | Not a recovery method for an unstable/damaged drone. Do not port the procedure blindly to this FC or run it under this planning request. |

Sources: [Astrom/Hagglund original paper](https://doi.org/10.1016/0005-1098(84)90014-1), [Skogestad original paper](https://doi.org/10.1016/S0959-1524(02)00062-8), [versioned PX4 autotune guide](https://docs.px4.io/v1.14/en/config/autotune), [current PX4 multicopter tuning guidance](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/config_mc/pid_tuning_guide_multicopter_basic.md), [ArduPilot AutoTune](https://docs.ardupilot.org/copter/docs/autotune.html).

Tune inner angular-rate loops first, then outer attitude loops; altitude tuning waits for a real validated altitude source. Compare tracking/overshoot, settling, noise amplification, saturation, control effort, timing and robustness on held-out data rather than a single best fit. Identify actuator/thrust nonlinearity and filter delays. Preserve every failed case and freeze source/gain versions with the data.

Propeller-off electronic tests verify commands, timing and stopping but do not identify the loaded free-flight rotational plant. Keep physical frame mapping, stable initial manual tune, airframe/propeller condition and sufficient control/thrust authority as prerequisites. No automatic online gain writes, excitation or flight test was implemented or authorized here.

Scope: primary-source research and planned architecture only. No physics/emulation engine, Linux hardware test, converter measurement or PID experiment was run. Proposed CPU, RAM, mass, camera characteristics, Wi-Fi, supply, plant dynamics and FC configuration adapters remain unvalidated.
