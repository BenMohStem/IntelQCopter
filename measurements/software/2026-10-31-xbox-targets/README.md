# Next small feature: Xbox targets, motors disabled

One-file preview now reads Xbox on core0 alongside telemetry; IMU/fusion remains core1. Same pinned Xbox library as the earlier bench lesson. Xbox connection attempts run in their own core0 task; timestamped targets pass through a one-slot queue to the separate serial task. Serial independently rejects old packets. Queue overflow in the acquisition path still stops capture; integration timing must be measured on ESP32. No hard deadline guarantee.

Right stick: roll/pitch targets within±20deg. Left horizontal: yaw rate within±30deg/s. Left vertical: normalized vertical request±1 (no altitude estimator or control; barometer absent). 8%deadband rescaled to full travel;0.6s+0.4s³ gives gentler response. Intended signs follow existing angle convention, but physical stick directions still require operator confirmation. RT/LT boost and mode gestures deferred.

New T records contain valid, roll_deg, pitch_deg, yaw_deg_s, vertical_normalized. A recent report within300ms and sticks observed neutral are required before accepting values. Disconnect/stale data zeros all display targets and clears neutral readiness. Idle controllers may send infrequent reports: if valid drops at rest this conservative bench guard must be diagnosed from notification cadence before flight, not relaxed blindly. Recenter to requalify. Connection alone does not authorize motors. Currently these values only go to graphs; there is no PID/mixer/output adapter.

Upload-attitude-preview then view-attitude commands unchanged. Close existing viewer first. One short session: connect, center sticks, move each separately and release, then disconnect. Check dashed targets, neutral return and validity. No sensor calibration needed. This is planned operator validation, not an already measured pass.

Validation: ESP32-S3 build passed. Actual stick function tested at all65536input codes for bounds/monotonicity plus startup, neutral, release, disconnect/recenter and stale guard cases using native host stubs. Actual quaternion helper+library regression passed9cases. These are host tests, not ESP timing measurements. New combined firmware not uploaded or physically validated. Operator's accepted shaking run applies only to previous FUSION_PREVIEW_V2 source.

