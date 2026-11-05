# Design decisions, rationale and reversibility

| Decision | Rationale | Revisit trigger |
|---|---|---|
| ESP32-S3 current sensor-FC platform | Physical board and CH343 captures already exist | New board measured/validated |
| QMC5883P not HMC5883L | Chip ID/address and original captures | Hardware replacement/clear ID evidence |
| Keep Nov 4 MAG `0x3D` in v2 | Actual integrated high-rate capture worked | Controlled new A/B evidence |
| Wi-Fi disabled for FC bench | Previous interference and deterministic timing priority | Validated power, EMI and separate telemetry task |
| 921600 baud for calibration/bench | Successful binary acquisition, known CH343 | Measured CRC/overrun problems |
| Use full accel/MAG 3×3 matrices | Independent raw fitting and holdout | Independent physical mapping/temperature test |
| Use MPU offset registers with residual verification | Reduce coarse bias, retain software cross-axis | Register effect mismatch on hardware |
| MPU FIFO at 500 Hz | Buffer sensor timing and reduce host jitter | On-board overflow/latency regression |
| BMP ×1/×1 IIR×4 ~167 Hz | Freshness/latency compromise | Dynamic step/noise measurements |
| Seven cascaded PIDs | User's learning and inspectability objective | Measured plant shows strong MIMO coupling |
| Quaternion + 3-state vertical KF | Robust orientation, relative height/velocity tracking | New independent navigation sensors |
| Motor actuation off in v2 | No flight control or failsafe validation | Full independent hardware-readiness review |

Every changed decision should link to a source commit and measured evidence. Do not turn the “revisit trigger” into permission to skip safety.
