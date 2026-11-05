# FC v2 log fields — how to interpret them

Look at actual firmware `src/main.cpp` for canonical formatting. The following are the important fields, not a guarantee that your exact run has produced them.

## Startup

- `BOOT_BIAS`: stationary residual check, count, register status, means and SDs; must not blindly read `still=1` without checking motion.
- `BOOT_ATTITUDE`: roll/pitch initialized from measured gravity, yaw arbitrary.
- `BOOT_BARO`: pressure baseline and accepted count; `zero=1` indicates initial reference computed.
- `FC_BOOT`: sensor path/FIFO/I²C configuration.
- `FC_MODEL`: assumed mass and inertia, hover surrogate, PID parameters, `ESC=OFF`.

## Running `FC` line

- `hz`: observed loop rate over reporting window (not worst-case period).
- `imu`, `mag`, `bmp`: counts of measurement paths; compare with timestamps/fresh flags.
- `fifo`, `fifo_empty`, `fifo_lost`, `fifo_over`: health of 12-byte MPU buffer pipeline.
- `late`, `miss`: scheduler overruns and skipped deadlines.
- `parser_ok`, `parser_bad`, `link`: serial command parser and pilot freshness.
- `baro0`, `alt`, `vz`, `hsigma`: vertical reference, estimate and uncertainty.
- `roll`, `pitch`, `yaw`, `p`, `q`, `r`: estimated attitude/body rates; verify physical axes.
- `health`: bit mask of fault reasons; see `include/flight_core.hpp` enum.
- `virt`: four normalized virtual motor demands; **not GPIO output**.

## `FC_TIMING`

`imu_max_us`, `mag_max_us`, `bmp_max_us`, `loop_max_us`, `qmc_slow`, `bmp_identical`, `fifo_fail`, `baro_reject`, `lidar_ok`.

## Acceptance philosophy

An achieved average `hz=500` with a 5 ms `loop_max_us` is a failure of the strict 2 ms deadline objective. Health should be evaluated over long motion/cold-start traces, not a single first-second screen capture. Save complete logs under `results/` and record the exact build flags.
