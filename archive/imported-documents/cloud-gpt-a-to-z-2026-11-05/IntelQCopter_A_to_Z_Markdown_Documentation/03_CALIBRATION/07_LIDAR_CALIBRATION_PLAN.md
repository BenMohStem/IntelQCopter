# Optional LiDAR calibration — complete plan for when available

The GUI intentionally supports skipping LiDAR while still collecting gyro, accel, MAG and barometer data. The 4 Nov session did not have any accepted LiDAR samples, and this should remain an explicit **pending** status.

## Planned stage targets

| Reference distance | Accepted readings |
|---|---:|
| 0.25 m | 300 |
| 0.50 m | 300 |
| 1.00 m | 300 |
| 2.00 m | 300 |

Use a rigid reference surface and an independent ruler/tape. Record sensor lens-to-target distance, mounting Z offset and known orientation. Use the original Nooploop UART frame protocol and compare status bits/precision fields, not raw distance alone.

## Fit and validation

Compute per-range mean, median, SD, dropouts and bias against the ruler. Test different target reflectances, angles and lighting as appropriate for the model. Fit a simple offset/scale only if the residuals support it. Do not force a single linear correction if distance errors change discontinuously or low-confidence readings dominate.

## ESP32 integration

Before using range in FC v2, physically identify RX GPIO, voltage levels and UART ownership. Do not confuse the separate GUI USB-UART port with the FC's disabled optional UART parser. Match timestamp and coordinate frame; use tilt compensation only when ground geometry is sensible and attitude estimate reliable. Use innovation gates in altitude Kalman and retain barometric fallback for missing range.
