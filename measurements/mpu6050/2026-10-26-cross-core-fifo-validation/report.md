# Real cross-core 1 kHz acquisition results

Three actual 20000-sample COM3 captures pass source/configuration/readback, sequence, checksum and 200 MCU diagnostic checks each. Total: 60000 samples. Motors remain disabled. Calibration biases/matrix are unchanged.

| Run | Rate (Hz) | Maximum handler (us) | Over-1000-us handlers | Maximum FIFO bytes |
|---|---:|---:|---:|---:|
| 1 | 1000.6848 | 814 | 0 | 14 |
| 2 | 1000.6848 | 819 | 0 | 14 |
| 3 | 1000.6848 | 812 | 0 | 14 |

The initial synchronous-logging implementation failed the handler budget (1381 us; refined synchronous trial 1249 us/214 overruns). Those originals and the malformed-serial trial remain preserved. Moving UART formatting/transmission to a bounded FreeRTOS queue on core 0 reduced acquisition-core logging work to tens of microseconds. FIFO reads and calibrated state updates run on core 1. Queue-full or drain failure aborts capture rather than dropping data. Windows receives serial in chunks while retaining strict record/sequence/checksum checks.

Trim registers are checked before and after capture rather than inside the hot path. No intentional external register writer is present. Unexpected mid-run changes are not guaranteed to be detected immediately; no flight/PID acceptance is claimed.

Inter-service gaps up to 1278 us remain, even with every observed handler below 1000 us. Thus observed 1 kHz throughput and handler execution budget pass these finite tests; a strict periodic hard deadline is not proven. Timing is measured against the ESP32/MPU clocks, not a calibrated external clock. Raw payloads and exact hashes are retained locally; summaries are in analysis.json.
