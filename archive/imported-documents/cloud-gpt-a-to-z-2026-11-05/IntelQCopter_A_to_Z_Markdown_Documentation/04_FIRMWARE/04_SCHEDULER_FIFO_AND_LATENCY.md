# Scheduler, FIFO buffering and low-latency design

## Core target

500 Hz controller means one 2 ms period. FC v2 uses absolute microsecond deadlines and a bounded FIFO read path. `SMPLRT_DIV=1` with DLPF configured gives nominal 500 Hz internal IMU updates.

## FIFO contract

Each selected MPU FIFO frame contains **12 bytes**: accelerometer XYZ (6 bytes) and gyroscope XYZ (6 bytes). The v2 driver measures `FIFO_COUNT`, reads a limited burst, chooses the newest complete frame and records discarded/backlogged samples; resets when near overflow or misaligned. The temperature sample is cached, not included in that selected 12-byte frame.

## Why a FIFO isn't a speed multiplier

A sensor sample can be produced on time but processed late if the host is blocked by a QMC retry or BMP transaction. The latest frame might have less age than the oldest buffered frame, but dropping frames changes the true integration interval. Use actual sample timing/freshness and count missed deadlines. Host capture frame rate is not a substitute for controller timing measurement.

## Monitoring

Read the v2 logs: `hz`, `imu`, `fifo`, `fifo_empty`, `fifo_lost`, `fifo_over`, `late`, `miss`; `FC_TIMING` records sensor read maxima and loop max, QMC fallback, BMP duplicate count. A high average rate with a long tail of 5–10 ms stalls is not safe deterministic 500 Hz operation.

## Next optimization after measurement

If I²C is the bottleneck, reduce optional traffic, split bus domains where physically possible, decimate status reporting, use short bounded timeouts, and consider a SPI-capable future IMU revision rather than silently overrunning deadlines. Do not change a working shared bus without an A/B capture.
