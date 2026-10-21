# Provisional mass and roll-estimator uncertainty report

Run date: 2026-10-19
Reproducibility seed: `2350`
Detailed outputs: [`artifacts/test-runs/2026-10-19/`](../artifacts/test-runs/2026-10-19/)

> These are conditional software-model results, not measurements of the assembled aircraft. They must not be used as evidence that the quadcopter is flight-stable.

## Mass and inertia Monte Carlo

The Monte Carlo draws 20,000 independent parameter sets. It includes the F450 frame reference mass and the user's approximate battery mass only. DJI lists a 450 mm diagonal wheelbase and 282 g frame mass for the F450; a clone's actual mass and frame arrangement may differ. The battery is entered as 150 g with an **assumed** 15 g one-sigma uncertainty. The model also assumes a 5 mm wheelbase sigma, a 0.55 ± 0.10 arm-mass fraction (clipped to 0.25–0.85), battery center at (-18, 0, -8) mm with (10, 5, 5) mm one-sigma placement tolerances, and provisional 105 × 38 × 28 mm battery dimensions with 10% sigma. These are transparent priors, not calibrated tolerances. See [DJI F450 manual](https://dl.djicdn.com/downloads/flamewheel/en/F450_User_Manual_v2.2_en.pdf).

The frame approximation divides its arm share into four slender diagonal rods from the center to motor locations and treats the remaining frame mass as a point at center. The battery is a rectangular solid. Parallel-axis terms shift both to the combined center of gravity. Motors, ESCs, controller, sensors, wiring, fasteners not included in the reference frame mass, lidar, camera, GPS, and payload are omitted. Thus, the reported mass is a **known-subassembly subtotal**, and its inertia is only for the modeled subtotal.

| Property | Mean | 1σ | Central 95% of sampled model |
|---|---:|---:|---:|
| Frame + battery subtotal | 431.5 g | 32.2 g | 369.5–494.6 g |
| CG x | -6.27 mm | 3.54 mm | -13.33–0.52 mm |
| CG y | 0.003 mm | 1.76 mm | -3.46–3.50 mm |
| CG z | -2.77 mm | 1.76 mm | -6.26–0.64 mm |
| Ixx | 1.347 × 10⁻³ kg·m² | 0.274 × 10⁻³ | 0.844–1.909 × 10⁻³ |
| Iyy | 1.506 × 10⁻³ kg·m² | 0.280 × 10⁻³ | 0.989–2.082 × 10⁻³ |
| Izz | 2.816 × 10⁻³ kg·m² | 0.551 × 10⁻³ | 1.797–3.951 × 10⁻³ |

The analytically expected mean of the two mass inputs is 432 g. The Monte Carlo mean differs by about 0.5 g because it is a finite seeded sample. The file `mass-inertia-summary.json` contains the assumptions and full precision; `mass-inertia-monte-carlo.csv` contains all 20,000 draws.

### Battery energy arithmetic

For a 3S 4200 mAh pack, nominal energy is `11.1 V × 4.2 Ah = 46.62 Wh`. This is nominal stored energy, not a safe usable-energy figure. Battery discharge rating, actual pack voltage under load, propeller, motor current/thrust curve, and all-up mass are unknown, so flight time, hover throttle, thrust-to-weight ratio, and motor safety cannot yet be calculated.

## Roll Kalman Monte Carlo

This uses the production C `angle_kf_update()` function. For each of 500 seeded trials, it resets the filter to the same initial state and covariance, draws an initial roll error with σ = 1°, a gyro-bias prior with σ = 0.1°/s, adds gyro noise σ = 0.2°/s, accelerometer-angle noise σ = 2°, and bias random walk σ = 0.01°/s/√s. The deterministic truth trajectory stays level for 1 s, ramps to 8° over 0.7 s, then holds. Steps are 2 ms (500 Hz target) for 5 s. Noise values are taken from the current C constants interpreted as variances/process-noise terms; actual sensor noise has not been measured.

At 5 s, across the 500 simulated trials:

- Mean estimation error: **-0.0046°**.
- Empirical error standard deviation: **0.1031°**.
- Kalman-predicted `sqrt(P00)`: **0.1374°**.
- RMSE: **0.1031°**.
- Coverage inside predicted ±1σ: **81.2%**.
- Coverage inside predicted ±2σ: **99.8%**.

For a calibrated Gaussian posterior, nominal coverage would be about 68.3% within one sigma and 95.4% within two. This run's covariance is conservative at the 5-second endpoint: its predicted standard deviation is about 33% above the empirical error standard deviation. At startup, measured empirical error σ is 0.919° versus predicted 0.894°; calibration varies with time. These coverage estimates themselves have Monte Carlo sampling uncertainty (500 trials), and are only valid for this assumed noise model and trajectory.

The CSV contains 251 points, including mean error, empirical error sigma, Kalman sigma, RMSE, and empirical one-/two-sigma coverage at every 20 ms. The host trace is not hardware-in-the-loop: it does not include MPU6050 vibration, acceleration projection error, timing jitter, barometer, magnetometer/yaw, altitude covariance, or vehicle dynamics. It does not prove the closed-loop quad is stable or that 500 Hz can be met on RP2350.

## Reproduce

From the project root in PowerShell, with `.venv-tests` and Zig installed:

```powershell
.\tools\validation\run_flight_controller_tests.ps1 `
  -KalmanPath artifacts\test-runs\kalman-roll-monte-carlo.csv `
  -KalmanTrials 500
.\.venv-tests\Scripts\python.exe tools\model\model_uncertainty.py artifacts\test-runs\mass-model
```

Use measured part masses with a 0.1 g scale, measure motor diagonal and component coordinates, and then replace the priors. For the arm fraction/solid geometry, weigh the four arms and center plates separately. Replace the box dimensions and placement with the actual pack measurements. The distribution will then become measurement-based rather than prior-based.
