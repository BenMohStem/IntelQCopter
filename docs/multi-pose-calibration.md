# Calibration that remains valid when the drone tilts

User clarification, 25 October 2026: the sensor stays fixed to the drone. Calibration must work across drone orientations, not merely make the current stationary pose read level. The current one-pose refinement is provisional. Do not accept it or progress to control from a level-only result. No firmware changes were made for this clarification.

## What the last experiment can and cannot establish

The last refinement applies a fixed rotation to the previous six-pose calibration. It preserves vector length and changes the coordinate frame; it does not continuously zero the measured tilt. A correct fixed mounting rotation remains useful when the drone tilts. However, residual bias, scale error and fixture error can also cause a tilted reading in one pose. That experiment cannot distinguish them. A temporal holdout at the same pose checks repeatability, not accuracy at new orientations.

Use one fixed calibration for every pose:

\[
\mathbf a_{body}=R_{mount}S(\mathbf a_{raw}-\mathbf b).
\]

Here b is sensor bias, S corrects scale/cross-axis errors, and R_mount maps sensor axes to drone axes. A general fit may estimate their combined matrix rather than uniquely separate them. Hardware offset correction must be accounted for exactly once. Current MPU_BIAS_CORRECTED logs already include register correction; they are not untouched raw samples.

## Small practical measurement steps

1. Keep the MPU fixed in its final mounting. Rotate the whole rigid assembly, not the MPU independently. Support the drone safely with propellers removed; motor outputs remain disabled.
2. Keep range, filter and calibration/register state identical throughout. Record firmware identity, startup readbacks and temperature. Allow temperature to settle and note its change; do not assume a fixed warm-up duration guarantees stability.
3. Start with +Z up. Check the drone reference plate independently, then record 60 seconds without touching it. At the current approximately 2.42 Hz reporting rate this gives about 145 observations, not 30,000 independent observations.
4. Repeat for -Z, +X, -X, +Y and -Y up, using a square/support to establish each reference. Capture all three acceleration components in every pose. Keep movement and settling periods separate from each stationary recording.
5. Repeat the six poses in a different order, repositioning between captures. Repositioning reveals reference/alignment repeatability that a long unmoving recording misses.
6. Reserve separate measured tilted orientations for validation: for example +30 and -30 degrees roll, +30 and -30 degrees pitch, and two combined tilts. Measure actual reference angles independently. Do not use MPU-estimated angles as the reference or fit to these reserved records.
7. Fit once using the training poses, then freeze the coefficients. Compare both the previous calibration and the candidate on every reserved pose. If validation informs a new fit, collect fresh validation poses afterward.

If the supports cannot establish accurate reference directions, say so: an arbitrary-orientation ellipsoid fit can assess bias and scale from the gravity magnitude, but cannot establish drone-frame alignment by itself.

## Statistical calculation and acceptance

For each stationary capture, calculate the three-axis mean, covariance, temperature range and drift. Average settled blocks; estimate uncertainty using blocks/repeated captures rather than assuming every sample is independent. Report excluded movement periods and preserve original logs. Robust fitting may reduce outlier influence but must not hide a consistently bad pose.

Known reference vectors allow a weighted affine fit:

\[
\min_{A,c}\sum_i (A\bar{\mathbf a}_i+c-\mathbf g_i)^T
W_i(A\bar{\mathbf a}_i+c-\mathbf g_i).
\]

Each reference vector g_i is gravity expressed in the independently measured drone orientation. W_i accounts for uncertainty of both the measured mean and the reference, with a nonzero reference-uncertainty floor. For the six axis-up poses, targets are the corresponding signed axis times g. Reference-angle errors remain even after averaging many samples.

For every reserved pose report vector error, magnitude error and gravity-direction angle error, plus uncertainty and repeatability after a power cycle. Choose the acceptable angle error together before judging the results; there is no demonstrated flight-ready tolerance yet. Do not approve a candidate just because its level error is smaller if other orientations get worse.

This is a stationary calibration experiment, not a flight/noise model validation: acceleration during motion is not just gravity. No simulator was used, and the physical offset-register response, temperature behavior and multi-orientation accuracy still require testing.

Method reference: [NXP AN4399, high-precision three-axis accelerometer calibration](https://www.nxp.com/docs/en/application-note/AN4399.pdf).
