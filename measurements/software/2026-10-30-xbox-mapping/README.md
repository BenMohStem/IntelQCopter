# Next small step: verify physical controller inputs

The user reports physical Menu shows information and physical Y advances selection. The two library fields are therefore exchanged in the bench application's action mapping for this controller. This is operator evidence, not proof of a universal Xbox mapping. A/B and all analog axes still need observation. Upstream decoder is unchanged.

Latest operator session 20261030T185510Z contains menu events and connected status, but no START/DATA/DONE captures at inspection time. Earlier closed operator sessions contain zero runs. The recorder could still have been open; absence of completed results is not an accepted acquisition test. Original logs are preserved privately. The four previous automatic trials remain separate hardware evidence from an older source.

## Why these tests existed

Direct/FIFO normal tests establish acquisition throughput and observed service time. Artificial-pause tests ask whether delayed work loses readable samples or queues older samples. Xbox selection checks the human-to-BLE-to-request-queue path. Together these address missing/delayed PID inputs, not PID gains, attitude correctness or flight stability. No need to repeat artificial-pause trials as a learning exercise now.

## One short input check before continuing

```powershell
.\qcopter.ps1 record-xbox-menu --port COM3 --inputs-only
```

No tests start in this mode. Press and release A, B, X, Y, Menu separately; then D-pad, LB/RB, sticks and triggers one at a time. Observe which raw field changes. rawY/rawStart are intentionally library labels: physical Menu is currently mapped from rawY, physical Y from rawStart. Report mismatches rather than assuming A/B are correct. Sticks should be near 32767 at rest; exact direction and endpoint limits require observation. Trigger endpoints are library/device dependent and must be measured. Ctrl+C saves original logs and returns to test-menu mode. Values print at 5 Hz; BLE input processing continues independently.

After this, validate gyro rotation directions with known gentle propeller-free rotations, then the attitude estimator, then PID calculations. No altitude acceptance until the barometer is available. No motors/PWM/PID enabled here. Current production firmware/calibration unchanged; its existing stale fingerprint is preserved.

Build and upload succeeded. Device source SHA matched and MONITOR_ONLY_NO_TESTS mode acknowledged; see boot.json. Recorder Python syntax checked without writing bytecode. Physical button action validation remains pending.

