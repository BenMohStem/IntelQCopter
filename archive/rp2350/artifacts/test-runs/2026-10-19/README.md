# QCopter test evidence — 2026-10-19

## Results

- `flight-controller-host-tests.log`: shared C estimator/controller math host tests passed.
- `flight-control-trace.csv`: 2,500 recorded samples at 500 Hz over 5 seconds. The sensor readings are a deterministic scripted input; there is no vehicle dynamics model in this trace. All rows report `armed=0` and `sensor_fault=0`.
- `kalman-monte-carlo.log` and `kalman-roll-monte-carlo.csv`: 500 seeded trials through the production C roll Kalman update. At 5 seconds, mean error was -0.0046°, empirical error σ 0.1031°, predicted `sqrt(P00)` 0.1374°, RMSE 0.1031°, ±1σ coverage 81.2%, and ±2σ coverage 99.8%. Noise/prior assumptions are printed in the log and detailed in `docs/model-uncertainty.md`.
- `mass-inertia-run.log`, `mass-inertia-summary.json`, and `mass-inertia-monte-carlo.csv`: seeded 20,000-trial uncertainty run for the frame+battery subtotal only. It is not all-up mass; see the assumptions and missing components in the summary and model report.
- `rp2350js-boot.log`: original emulator console output, including its peripheral-support warnings and the final successful boot check. The log records 1,000,000 instruction steps, 1,024,880 simulated cycles, and core 0 at `0x10000f90`.
- `model-validation.log`: URDF XML parse passed.

## Limits

The Kalman test tracks the production C roll estimator's errors and covariance under its scripted Gaussian noise model. It does not validate real sensor noise or quadcopter stability. The RP2350 emulator result proves that this firmware image executes through the checked steps; it does not validate real 500 Hz wall-clock timing, I2C, RF, PWM/DShot, ESCs, or flight. No board was flashed and no motors were driven.

## Re-run

```powershell
.\tools\run_flight_controller_tests.ps1 -TracePath artifacts\test-runs\2026-10-19\flight-control-trace.csv
.\tools\run_flight_controller_tests.ps1 -KalmanPath artifacts\test-runs\2026-10-19\kalman-roll-monte-carlo.csv -KalmanTrials 500
python tools\model_uncertainty.py artifacts\test-runs\2026-10-19
$env:RP2350JS_PROGRESS_EVERY = '100000'
node sim\rp2350js\boot_check.cjs build\flight-rp2350\qcopter.hex 1000000 2>&1 | Tee-Object artifacts\test-runs\2026-10-19\rp2350js-boot.log
```
