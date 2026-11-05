# Timing design — sample clock, FIFO, callbacks and deadlines

An MCU scheduler should distinguish **sensor acquisition**, **estimator update**, **controller update** and **telemetry**. FC v2 uses a nominal absolute 2 ms scheduling loop. This avoids accumulated `delay(2)` drift but does not make all I²C calls bounded below 2 ms.

## Sources of latency

- MPU internal DLPF ~2 ms group delay, even before host I²C transaction and FIFO age.
- Host services FIFO late: newest sample age depends on backlog length and burst handling.
- QMC status polled every 4 ms may report a conversion that finished just after previous poll.
- BMP normal-mode conversion cycle and IIR introduce their own delay.
- UART console/USB buffering and Windows COM activity can burst unrelated to the main loop.

## Why no external INT/DRDY was requested

The actual board had unreliable or unavailable physical interrupt lines. The proven calibration path uses software polling and internal status bits. A future hardware revision can use MPU INT for deterministic FIFO wakeup, but do not claim it is connected now. `INT_PIN_CFG` in MPU config is a bypass-control register name, not physical interrupt wiring.

## Recommended log record

`{acq_timestamp_us, sensor_id, seq, age_us, flags, raw_values, calibrated_values, read_duration_us, fifo_level, missed_deadline}`.

Control should always consume a coherent snapshot and reject invalid/aged signals; it should never block while waiting for a missing MAG or BMP conversion. Keep a histogram of deadline overruns, not only average Hz.
