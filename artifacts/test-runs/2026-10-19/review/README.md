# Review rerun evidence — 2026-10-19

This is a fresh rerun captured while organizing the project. Logs preserve the commands' console output; CSV and JSON files preserve the generated data. Files are UTF-8 text. The RP2350 build now compiles the battery monitor; the RP2040 build was incremental. The scripts produced no flash operation.

## Results

- [`rp2350-build.log`](rp2350-build.log): Pico SDK 2.3.0, `pico2` / RP2350 ARM-S configuration completed. No firmware flashed.
- [`battery-monitor-host-tests.log`](battery-monitor-host-tests.log): the added battery monitor code passed host checks for scalar pack-voltage filtering, SOC Kalman prediction/correction, SOC uncertainty reduction after a supplied rested-OCV observation, low voltage, critical-cell/BMS alerts, landed-disarm recommendation, and invalid sensor fail-safe.
- [`pcb-drc.log`](pcb-drc.log): KiCad geometry DRC result is zero geometry violations and 76 unconnected pads. The board is electrically unrouted and cannot be fabricated.
- [`isaacsim-install-record.md`](isaacsim-install-record.md): records the checksum-verified official package, shared extraction path, setup result, and why no simulator launch was attempted.
- [`rp2040-build.log`](rp2040-build.log): `pico` / RP2040 configuration completed. No firmware flashed.
- [`host-and-kalman-tests.log`](host-and-kalman-tests.log): C host checks passed. The production C roll filter ran 500 seeded trials, 2,500 steps each at 500 Hz, with synthetic sensor noise and a synthetic roll trajectory.
- [`kalman-roll-monte-carlo.csv`](kalman-roll-monte-carlo.csv): at 5.0 s, empirical error σ = 0.103118°, predicted Kalman σ = 0.137386°, mean error = -0.004643°, RMSE = 0.103119°, ±1σ coverage = 81.2%, ±2σ coverage = 99.8%.
- [`mass-monte-carlo.log`](mass-monte-carlo.log) and [`mass-model/`](mass-model/): 20,000 seeded draws. Frame plus approximate battery subtotal = 431.47 g mean, 32.20 g standard deviation, 369.55–494.60 g central 95% interval. Motors, ESCs, electronics, wiring, and payload are excluded. Many geometric/mass uncertainties are assumed priors, not measured uncertainty.
- [`rp2350js-boot.log`](rp2350js-boot.log): emulator executed 1,000,000 instruction steps (1,024,880 emulator cycles). Its warnings about unsupported peripherals and its final scope limitation are preserved in the log.
- [`flight-control-trace.csv`](flight-control-trace.csv): 2,500 synthetic 500 Hz math samples. This trace uses constant roll and throttle inputs, has equal normalized mixer values, and stays disarmed. It has **no vehicle dynamics**; it is not useful as a stability result.
- [`../visualizations/uncertainty-dashboard.html`](../../../visualizations/uncertainty-dashboard.html): standalone browser dashboard built from the review CSVs. Its play button replays aggregate Kalman error/covariance statistics over time; mass and inertia plots show the sampled distributions.
- [`kalman-uncertainty.png`](../../../visualizations/kalman-uncertainty.png): static graph exported from the same 251 recorded time bins for direct preview. The reproducible vector plot is [`kalman-uncertainty.svg`](../../../visualizations/kalman-uncertainty.svg).

## Reproduce

Run these commands in PowerShell from the repository root. They depend on the locally installed Pico SDK and `.venv-tests` Zig compiler for host tests; `.venv-tests` is ignored by Git.

```powershell
.\tools\build_firmware.ps1 -Target flight
.\tools\build_firmware.ps1 -Target controller
.\tools\run_flight_controller_tests.ps1 `
  -TracePath artifacts\test-runs\flight-control-trace.csv `
  -KalmanPath artifacts\test-runs\kalman-roll-monte-carlo.csv
.\.venv-tests\Scripts\python.exe tools\model_uncertainty.py artifacts\test-runs\mass-model
$env:RP2350JS_PROGRESS_EVERY = '100000'
node sim\rp2350js\boot_check.cjs build\flight-rp2350\qcopter.hex 1000000
```

The build/emulator and filter tests prove compilation/execution or calculation for the specific software path. They don't prove RP2350 real-time timing, hardware I²C/RF, ESC signaling, vehicle response, or flight safety. Isaac Sim's official 6.2.0 standalone files are checksum-verified and staged in the shared Windows Public folder, but the simulator did not run because this PC is below the supported OS/GPU minimum; see [`sim/isaac/README.md`](../../../../sim/isaac/README.md).
