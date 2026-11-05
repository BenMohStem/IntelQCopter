# What counts as success — measurable acceptance criteria

| Aspect | Example metric | Acceptance status now |
|---|---|---|
| UART binary calibration | CRC errors, sequence gaps, packet rate | 4 Nov reported successful capture; 500 Hz target observed |
| QMC real output | Fresh XYZ/sec, ID, status, range, repeatability | ~195 Hz in successful Nov 4 trace; earlier intermittent failures |
| MPU rate | FIFO actual frames/sec, overflow and backlog | FC v2 source implemented; on-device FIFO trace still missing |
| 500 Hz scheduler | Max/P99 loop time, missed deadline count | No real FC v2 trace yet; acceptance must be set before actuation |
| Gyro calibration | Stationary mean and SD after hardware offset | Raw 30k calibration done; new register path not yet tested |
| Attitude | Physically measured sign, roll/pitch transient, drift | Not independently signed-off |
| Barometric height | Drift, step delay, ruler comparison, NIS | Stationary SD measured; dynamic uncertainty unverified |
| Xbox | Command CRC, disarm/loss policy, failsafe latency | Bridge host tests only, no flight receiver validation |
| Motors | Thrust map, lag, current, order, direction, stop | Not measured; physical outputs disabled |

## How to compare two firmware revisions

Hold sensor placement/power/temperature and logging method constant. Change one configuration at a time (e.g. I²C clock). Log full distributions, not a single average. Note that a short test with zero failures is a weak claim about rare tail faults.

## Recommended report summary

`commit, firmware SHA, board, timestamp, power state, I2C Hz, UART baud, MAG C1, MPU DLPF/FIFO, BMP profile, boot offsets, average/p99/max loop us, missed/overflows, sensor fresh Hz, UART loss, anomalies, verdict, next action`.
