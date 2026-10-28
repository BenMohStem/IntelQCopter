# Real 1 kHz tests and next gyro-frame step

All completed captures below were replayed from original raw bytes, not trusted only from summaries. Source/configuration/calibration headers, sample order, FNV payload checksum, all 200 MCU diagnostics, final metadata and saved raw words agree. The malformed-serial trial reproduces its original failure and remains preserved. No motor outputs were enabled.

| Trial | Samples verified | Rate (Hz) | Maximum handler (us) | Overruns | Result |
|---|---:|---:|---:|---:|---|
| 1 | 20000 | 1000.67 | 1381 | not instrumented | Handler budget failed |
| 2 | 2687 partial | unavailable | unavailable | unavailable | Rejected original malformed record |
| 3 | 20000 | 1000.69 | 1249 | 214 | Handler budget failed |
| 4 | 20000 | 1000.68 | 814 | 0 | Observed handler budget passed |
| 5 | 20000 | 1000.68 | 819 | 0 | Observed handler budget passed |
| 6 | 20000 | 1000.68 | 812 | 0 | Observed handler budget passed |
| 7 | 20000 | 1000.69 | 822 | 0 | Observed handler budget passed |

![Actual timing before and after repair](actual-timing.png)

Synchronous logging caused real overruns despite average throughput near 1 kHz. The repaired path uses a bounded FreeRTOS packet queue: core 1 acquires/calculates; core 0 formats/transmits. Queue-full/drain failure rejects capture. Buffered Windows serial reads retain strict validation; the earlier malformed-record cause is not conclusively established. No record was ignored to manufacture a pass.

The successful handler budget does not establish a perfectly periodic hard deadline. Inter-service gaps above 1000 us remain, FIFO timestamps are nominal sample-clock positions, and clocks have not been calibrated against an external reference. No attitude filter/PID/actuator performance is inferred.

## Next implemented step: body-frame gyro rates

The latest source applies a **provisional rotation-only mounting transform** after sensor-frame gyro bias correction. It does not reuse the full accelerometer scale matrix. Actual MCU diagnostics include both sensor and body rates; their arithmetic is checked against original FIFO words. A stationary run validates computation/timing, not physical direction, gyro scale or dynamic attitude accuracy.

Known physical rotations are the next reference check. The proper-rotation candidate and its assumptions are recorded in config/gyro-frame.json and explained in [the frame guide](../../../docs/gyro-frame-stage.md). Gravity-angle conventions are unchanged; calibration biases and register targets are unchanged.

Original successful/failed logs and source fingerprints remain local; exact hashes and measured statistics are in [analysis.json](analysis.json).
