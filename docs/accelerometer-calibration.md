# Six-pose accelerometer calibration and flash storage

## What changed

The user confirmed that pose labels refer to DRONE axes, while the readings are in MPU sensor axes. The +X pose predominantly excites sensor +Y; the +Y pose predominantly excites sensor -X. Therefore three separate per-sensor-axis scale factors cannot, by themselves, produce drone-frame acceleration. This implementation applies a provisional affine correction containing bias, scale/cross-axis and mounting effects together. It is not a multivariable flight controller.

The original collection programme, all six averaged measurements, fit coefficients and residuals are preserved in [the measurement snapshot](../measurements/mpu6050/2026-10-24-six-pose/calculation.json). No source data were discarded. The old single-pose averaging at every boot was removed; it was not a six-pose calibration. Raw and corrected output are both retained.

## Derivation

Let r be the raw sensor vector in m/s², a the desired drone-frame vector, b the sensor-coordinate offset and A the combined response. At rest in a correctly aligned pose:

    r = b + A a

For each drone axis j, the two targets are +g e_j and -g e_j, using g = 9.80665 m/s². Add each pair to cancel gravity:

    c_j = (r_(+j) + r_(-j)) / 2
    b = (c_X + c_Y + c_Z) / 3

Subtract each pair to cancel the offset:

    d_j = (r_(+j) - r_(-j)) / 2 = g A e_j
    D = [d_X d_Y d_Z] = g A
    C = g inverse(D)
    a_corrected = C (r - b)

This is the unconstrained least-squares affine fit for these six symmetric target vectors when expressed as raw response versus target, then inverted. It assumes accurate stationary pose orientations. Every output row is only three products added together, as shown in correct_acceleration. It is not a Kalman filter, and it neither estimates nor subtracts gravity from the corrected acceleration.

Calculated offset b in sensor coordinates (m/s²):

```text
1.293084598, -0.079166657, -0.639507624
```

Calculated C, mapping centred sensor readings to drone axes:

```text
 0.046659658  0.992539802 -0.045743511
-0.955388252  0.053316466  0.289431328
 0.272504170  0.023832247  0.951963666
```

The three opposite-pose midpoints differ: see calculation.json. Their disagreement and the residuals can reflect alignment error, noise, temperature change or model limitations. They do not establish standard deviation, and the three +/− pairs are not independent validation data.

## Persistence, line by line

The firmware uses Arduino Preferences, backed by ESP32 nonvolatile storage. The MPU6050 itself has no user-calibration nonvolatile memory. No MPU offset registers are written: correction is applied once in software, also allowing the mounting transform. Do not add MPU offset writes on top of it without reworking the calibration.

1. AccelCalibration holds a format version, a measurement-set ID, three bias values and a 3-by-3 correction matrix.
2. measured_calibration contains the computed candidate coefficients. Updating these requires reviewing a new measurement set; changing the sensor or mounting invalidates this candidate.
3. load_or_save_calibration opens namespace mpu6050 and reads the accel-v1 blob if its byte length matches.
4. An exact comparison against this firmware's candidate checks the complete record, including version, ID and coefficient bytes. Storage uses NVS's own integrity protection too. A different, absent or unreadable record is replaced with this candidate once.
5. A write is accepted only if putBytes reports the full size and an immediate getBytes returns the identical record. Failure leaves corrected output disabled and raw output available. Review the status message; this is not flight-ready health handling.
6. Matching records print LOADED_FROM_FLASH and cause no flash rewrite. A fresh candidate prints SAVED_AND_READ_BACK. Persistence is called only during setup, never in the sampling loop.

This procedure installs the provisional uploaded candidate automatically on first boot and on a changed candidate; it does not perform automatic physical recalibration. Recalibration requires collecting six new means, calculating/reviewing new coefficients and updating the candidate/measurement ID. Booting while tilted does not replace the saved calibration with a single-pose average. Flash erase can delete it, after which the embedded candidate will reinstall. Use only on the measured sensor/mount: this lesson cannot detect a substituted module automatically.

## Your real test - not performed by the assistant

Keep propellers removed. Close Monitor, upload deliberately, reopen at 115200 baud and press RESET/EN if needed to capture startup. First installation should say:

```text
CALIBRATION,SAVED_AND_READ_BACK
```

Press RESET/EN again, then separately unplug/reconnect power. Both should say:

```text
CALIBRATION,LOADED_FROM_FLASH
```

If it says saved on every unchanged reboot, or reports any failure, preserve the actual log and stop for diagnosis. A reset alone is not a full power-cycle test.

Now use a stable fixture and repeat all six DRONE-axis poses. The corrected aligned axis should be near +9.80665 or -9.80665 m/s², with the other two near zero. Record all three values, variation over time and how each pose was aligned. Also check several NEW stationary tilted orientations: the corrected vector magnitude should stay near g. Do not interpret the training fit as proof that these new checks will pass.

Training residual vector norms, in m/s²:

- +Z: 0.107519
- -Z: 0.107519
- +X: 0.142461
- -X: 0.142461
- +Y: 0.043839
- -Y: 0.043839

No acceptance tolerance was invented from these means. Without original sample logs or their variability we cannot compute reliable confidence intervals. Failing independent checks means recollecting/revising calibration, not tuning PIDs around the error.

## Limits and next step

This calibrates only acceleration provisionally. Gyro bias, gyro-to-drone mounting rotation, thermal drift, sensor disconnect detection and attitude estimation are not implemented. Do not apply C directly to gyro readings: its accelerometer scale/shear terms are not a pure rigid rotation. The one-second report rate is for bench inspection only. Motor outputs remain disabled. Mass/inertia and gains remain deferred to standalone-FC Gazebo work.

No build, flash, power-cycle or sensor acceptance was performed by the assistant. The numerical fit and training residuals were calculated from the user-supplied means; these are calculations, not new measurements. Calibration remains the current phase until the user verifies restoration and independent pose checks.
