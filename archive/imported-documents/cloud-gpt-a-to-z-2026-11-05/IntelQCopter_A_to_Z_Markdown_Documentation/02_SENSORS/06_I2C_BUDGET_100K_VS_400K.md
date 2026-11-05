# I²C bandwidth, scheduling and transaction hazards

## Separate clocks

- **I²C clock**: electrical bus clock in Hz (100 kHz and 400 kHz tested/planned).
- **UART baud**: physical CH343 serial bit rate (921600 baud for binary capture and FC bench; 115200 for text-only v13 MAG probe).
- **Sensor ODR**: internal conversion frequency (500 Hz MPU, ~200 Hz QMC, ~167 Hz BMP in v2).
- **Control frequency**: how often the controller runs its calculations (target 500 Hz).

These are not interchangeable. A 921600-baud UART does not repair I²C register access.

## First-order bus budget

At 400 kHz, a 12-byte I²C FIFO read consumes at least about 108 wire-clock bits for the bytes alone (~270 µs), plus address phases, register pointer, repeated starts, transaction overhead and MCU latency. At 100 kHz the corresponding byte payload requires at least ~1.08 ms, already more than half of a 2 ms loop. MAG, pressure and setup/health reads further compete for bus time.

The above is a *lower bound*, not measured completion time; different repeated-START and STOP behaviors, errors, timeouts and FIFO backlog can dominate actual latency.

## Implemented approach

- MPU/BMP use 400 kHz primary bus in FC v2.
- QMC may fall back to 100 kHz transaction rate after confirmed problems.
- FIFO burst management is bounded rather than endlessly catching up.
- BMP factory trims cached and raw ADC acquired in one burst.
- Text reporting at about 1 Hz; do not dump multiple verbose CSV lines inside each 2 ms deadline.

## Required bench data

At least several minutes of `FC` and `FC_TIMING`: actual loop rate, minimum/mean/worst duration, FIFO drops, QMC fallback state, MAG and BMP fresh rates, bus error counts and P99 latency. Measure under motion and with representative UART traffic. A 5 ms timeout violates a 2 ms control deadline: fault handling must be bounded and take priority over convenience.
