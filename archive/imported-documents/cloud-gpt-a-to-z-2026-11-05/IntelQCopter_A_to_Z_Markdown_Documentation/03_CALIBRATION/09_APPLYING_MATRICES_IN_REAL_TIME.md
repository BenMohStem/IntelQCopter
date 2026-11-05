# Applying hardware offsets and 3×3 matrices in the FC loop

**Correct order for a software-only reference:**

```cpp
// Conceptual pseudocode. Values and units must match actual ADC config.
Vec3 gyro_dps  = raw_gyro_to_dps(raw_gyro);
Vec3 accel_si  = raw_accel_to_mps2(raw_accel);
Vec3 mag_uT    = raw_mag_to_uT(raw_mag);
Vec3 gyro_cal  = gyro_dps - gyro_bias_dps;
Vec3 accel_cal = C_accel * (accel_si - accel_bias_si);
Vec3 mag_cal   = C_mag * (mag_uT - mag_bias_uT);
// Then apply *individually verified* sensor-to-body rotations.
```

## With gyro/accel hardware offset registers

The hardware outputs change before converting counts to SI. If a known bias is compensated by hardware, measure the effective delta; only the **remaining residual** belongs in software. A full cross-axis matrix still must run every new sensor sample. Preserve temperature-compensation bits and don't write magnetometer hard/soft iron into unsupported registers: QMC correction stays in software.

## Floating-point workload

A 3×3 matrix-vector product is nine multiplies and six adds, negligible compared with I²C or UART waits at 240 MHz. Every 2 ms loop may apply *latest calibrated* MAG, but it must only execute magnetic Kalman/heading corrections when a **fresh** MAG conversion appears. No benefit to re-correcting identical cached MAG bytes at 500 Hz as independent data.

## Full-scale caveat

All stored calibration quantities use sensor-frame engineering units. If MPU full-scale is raised later, nominal ADC sensitivities change; the same SI correction may remain a first candidate, but raw conversion, noise and hardware-offset scaling need revalidation. Cross-axes and biases can shift with mounting and temperature.
