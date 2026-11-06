# TOFSense-F2 Mini geometry and altitude-fusion requirements

Updated: 2026-11-06 (Africa/Lagos)

## Hardware report and coordinate convention

The user reports a TOFSense-F2 Mini at the central underside of the F450 frame,
nominally centered over the center of gravity (COM). It is on the aircraft, not
the N150 companion computer. The N150/ROS2 floor-ray setup remains a simulation
alternative and is not the physical mounting model.

Use body axes +X forward, +Y left, +Z up. The optical axis is body −Z. Relative
to the lower PCB origin, the preliminary optical reference point is
`(0, 0, -0.008) m`; the preliminary COM is `(0, 0, +0.018658) m`. Therefore the
reported sensor position relative to COM is
`r_s = (0, 0, -0.026658) m`. These are estimates, not surveyed dimensions. Keep
the offset in a configuration record and verify it from the assembled aircraft
before using it for control.

The manufacturer lists a 0.02–7.8 m range, up to 50 Hz update rate, 2–3° field
of view, 4.3–5.2 V supply, 3.3 V TTL, and 921600 bps default UART. Those are
device specifications, not measured performance on this airframe. The device
datasheet reports 3 mm standard deviation and 3 cm RMS under its stated test
conditions; those are not guaranteed installed-system noise values. Do not copy
them directly into the filter covariance without bench characterization. The F2
Mini can also use I2C; the actual module identity, mode, line sequence, wiring,
configured baud, timestamp path, and valid-return handling still need bench
verification. Surfaces and multiple objects within the field of view can affect
the return, so test the actual underside and landing surfaces.

Sources: [TOFSense-F2 Mini datasheet](https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_zh.pdf),
[manufacturer FAQ](https://support.nooploop.com/tofsense/FAQ-for-tOfsense/).

## Geometry

Let `R = Rz(yaw) Ry(pitch) Rx(roll)` map body vectors into a world frame with
world +Z up. Let `d_s = (0, 0, -1)` be the unit beam direction in body axes, `r_s`
the sensor optical reference position relative to COM, and `D` the measured
slant range to the beam-hit point:

```text
H_COM = -[R r_s]_z - D [R d_s]_z
```

This follows from the beam-hit point `p_hit = p_COM + R r_s + D R d_s` and gives
the COM's world-vertical separation above that point. A horizontal surface is
not required to calculate the point's vertical coordinate; it is needed only
to equate the measurement with a simple height directly beneath the vehicle.
For a level aircraft and the preliminary centered mount,
`H_COM = D + 0.026658 m`. For this vertical sensor offset and down-facing beam,
the simplified expression is:

```text
H_COM ≈ (D + 0.026658) cos(roll) cos(pitch)
```

Roll and pitch must be radians in C/C++ trigonometric calls. The full vector
equation is the implementation reference; the simplified expression applies
only with the stated alignment assumptions. Store the offset
as a configurable 3D body vector. Do not bury the preliminary estimate in a
flight-control constant.

The range is to the beam footprint, not necessarily to ground directly under
the COM. Tilt shifts that footprint horizontally. Sloped, stepped, soft,
reflective, obstructed, or multiple surfaces in the small field of view can make
the return differ from local terrain under the aircraft. Attitude alignment and
terrain observability therefore remain part of measurement validity.

The body-frame hit displacement from COM is `r_s + D d_s`; its world-frame
footprint is `R(r_s + D d_s)`. The horizontal components are the footprint
offset from the COM projection. Use them when checking whether a range is
representative of the terrain patch needed by the controller.

The offline filter treats terrain height as a scalar at the observed footprint.
Without aircraft horizontal position and a terrain map, it cannot convert that
height into ground elevation beneath COM or estimate slope. Use the footprint
offset when judging whether a range represents the terrain patch needed by the
controller.

## Required estimator state and behavior

Keep separate states for:

1. Pressure altitude relative to a saved, stationary pressure reference.
2. LiDAR range and terrain-relative height at the beam hit.
3. Estimated vertical velocity.
4. Estimated local-navigation vertical position.
5. Estimated terrain height, only while the available observations make it observable.

Fuse time-aligned world-up acceleration (MPU6050 target 500 Hz), fresh BMP280
pressure at the highest verified configured rate (approximately 166.7 Hz is a
nominal configuration estimate, not a measured board rate), and valid TOFSense
observations (up to 50 Hz). Each observation needs its acquisition timestamp,
sample age, status, and uncertainty. Propagate covariance and apply innovation
gating; account for range noise, attitude uncertainty, lever-arm survey error,
latency, and correlation in a production estimator. A constant 0.07 m LiDAR
sigma in the offline replay helper is only a provisional software-test setting,
not a datasheet claim or fitted covariance.

Reject invalid-status/out-of-range returns, missing or stale timestamps,
excessive tilt, bad beam alignment, inconsistent innovations, and pressure
disturbances. Use a configurable tilt/beam gate, then establish its value by
analysis and bench data. Do not assume a LiDAR status bit alone proves the
ground return is geometrically useful.

At confirmed stationary ground startup, establish pressure reference and accept
only valid LiDAR observations for the initial terrain reference. Do not reset
altitude to zero on every new range. After landing on a different surface,
update terrain reference only after landing is confirmed and blend/update without
a discontinuity in local navigation state. On a suspected airborne reboot, do
not ground-zero, stationary-calibrate gyros, or arm motors automatically.

## Current implementation boundary

`hardware/bench_calibration/esp32s3_qmc5883p_100hz/tools/altitude_fusion.py`
is an offline replay prototype only. It applies the 3D sensor lever arm using
the user-reported preliminary offset as an overridable default, checks the
manufacturer range, requires logged LiDAR age, and applies a tilt gate. Its
five-state research KF keeps local vertical position and terrain height separate;
LiDAR observes their difference, while pressure observes local position plus
barometer bias. A ground datum can be established only from an explicit pressure
reference supplied by the caller or from 30+ fresh pressure rows marked both
stationary and ground-startup-confirmed. It does not automatically re-zero during
flight. This tool has no sensor acquisition and produces no flight-control output.
Its constant scalar range sigma and terrain process noise are assumed values;
it does not yet propagate attitude, timestamp, or mounting-survey covariance.

The calibration GUI remains the existing 12-step no-LiDAR workflow. This
geometry note does not reintroduce a LiDAR calibration GUI stage. Standalone
range calibration already has a separate optional bench script at
`hardware/bench_calibration/esp32s3_qmc5883p_100hz/tools/lidar_calibration.py`;
it is outside the main GUI and has not been validated for this physical mount.
Audit it only after the actual module, interface, wiring, attitude frame, and
measured reference distances are confirmed. A terrain-separated algorithm
illustration is available at
[`synthetic_terrain_separated_altitude.png`](../hardware/bench_calibration/esp32s3_qmc5883p_100hz/analysis/synthetic_terrain_separated_altitude.png);
it is generated from synthetic inputs and is not aircraft performance data.

## Verification gates

1. Measure lower-PCB-to-optical-reference and lower-PCB-to-COM vectors on the
   assembled frame; record the measurement method and uncertainty.
2. Confirm sensor SKU/authenticity, electrical mode, power/logic levels, pin
   order, UART baud or I2C address, and actual driver timestamps/status fields.
3. With motors disabled, test known target distances across the specified
   range and representative materials; record range, status, signal strength,
   age, and repeatability.
4. Verify body/world axis alignment and tilt projection using an inclinometer
   and a flat reference plane. Test the beam footprint location at tilt.
5. Measure residuals against independent reference heights, estimate range and
   attitude/geometry uncertainty, then validate innovation rejection and
   dropout continuity offline before considering a physical estimator.
6. Keep landing-on-different-terrain and airborne-reboot behavior separately
   tested. Physical arming remains disabled under the project policy.
