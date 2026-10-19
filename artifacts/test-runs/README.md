# Test evidence index

All committed results are software checks or synthetic models. None prove a stable aircraft, working sensor/radio hardware, real-time 500 Hz deadlines, or flight readiness.

## Original recorded run

[`2026-10-19/`](2026-10-19/README.md) contains the original artifacts and a README with interpretation and limitations:

- `flight-controller-host-tests.log` — C host test result.
- `flight-control-trace.csv` — 2,500 samples at 500 Hz; constant scripted inputs and equal mixer outputs. No airframe model.
- `kalman-monte-carlo.log` and `kalman-roll-monte-carlo.csv` — 500 seeded trials of the production C roll filter with synthetic noise/trajectory.
- `mass-inertia-run.log`, `mass-inertia-summary.json`, `mass-inertia-monte-carlo.csv` — 20,000 draws for frame plus approximate battery only.
- `rp2350js-boot.log` — original RP2350 instruction-emulator console output.
- `model-validation.log` — URDF XML syntax parse only.

## Review rerun

[`2026-10-19/review/`](2026-10-19/review/README.md) stores fresh build, host-test, emulator, uncertainty logs, and input/output data generated during the repository review. Each log names its command and outcome. The HTML uncertainty dashboard is [`../visualizations/uncertainty-dashboard.html`](../visualizations/uncertainty-dashboard.html); it replays the recorded aggregate Kalman trace and plots the mass/inertia distributions. It is a data visualization, not a video capture or vehicle simulation.

The review folder now also includes the battery-monitor host test log and a fresh PCB DRC report after moving the external BMS header inward.

The latest input/radio-scaffold rerun adds separate RP2040 and RP2350 build logs, protocol host tests (CRC, range, sequence, and timeout behavior), and an RP2350JS one-million-instruction log. The command protocol is tested in C, but the LoRa hardware interface remains a stub.
