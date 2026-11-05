# Sensor axes, mounting transforms and sign validation

The independent calibration matrices operate in **each sensor's native XYZ axes**. They are not a statement that sensor X is aircraft roll X. A valid physical mounting matrix `R_BS` transforms sensor-frame vectors into the body frame and must satisfy:

\[
R_{BS}^T R_{BS}=I,\qquad \det R_{BS}=+1.
\]

## Recommended order

1. Decode raw sensor XYZ in chip register order.
2. Convert to engineering units (rad/s, m/s², µT).
3. Apply offsets and full 3×3 calibration *in the sensor frame*.
4. Rotate with the individually verified `R_BS` for gyro/accel/MAG.
5. Feed coherent body-frame vectors into quaternion propagation, accelerometer gravity correction and yaw innovation logic.

## Key historical issue

A simple GUI display X/Y exchange was found to have determinant −1 if treated as a 3D physical rotation. A reflection is not a legal rigid-body orientation. A display convenience must never enter the estimator or mixer.

## Propeller-free verification

- Place aircraft level in the declared FLU reference; static specific force should point roughly +body Z.
- Pitch the nose down and check the **sign of attitude and gyro rate**, not just magnitude.
- Roll to the right, yaw manually, and compare magnetic yaw trend with gyro yaw.
- Rotate MAG in a level circle and verify heading angle continuity and magnetic magnitude before any motor test.

The v2 source currently uses identity sensor-to-body placeholders. **Unverified identity mappings block physical flight**.
