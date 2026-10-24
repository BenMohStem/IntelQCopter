# Parameters reserved for the simulation phase

The user supplied `params-2026-10-24.txt` on 24 October 2026. It is preserved byte-for-byte, not regenerated or treated as instructions to implement its suggested estimator.

- Total mass mean: 1.038 kg; supplied standard deviation: 16.0312 g.
- Supplied principal inertias: Ixx = Iyy = 0.006421 kg m^2; Izz = 0.011853 kg m^2.
- These are user-supplied simulation assumptions, not independently validated physical measurements.
- This snapshot uses a 320 g battery estimate, replacing the earlier approximate 150 g estimate for this proposed simulation model only.

## Deferred work

Calculate controller/estimator gains and test the standalone FC in Gazebo first, without an onboard ROS companion computer (Pi/Jetson). Desktop tools may still run the simulation. No gains, Q matrix or simulation configuration have been changed from these values now. Firmware lessons remain paused at step05 while the user tests.

Before applying the numbers, review the component mass/variance totals, the correlation assumptions behind combined variances, motor numbering and coordinate conventions, origin versus actual centre of mass, and the inertia derivation. For example, listed hub component variances sum to 205 g^2 while the supplied hub summary says 195 g^2. Inertia covariance and discrete process noise require their own derivation; the file's Q guidance is not automatically accepted as a valid implementation.

SHA-256 of the original saved file:

```text
6ccc10772bdf0332fce448bd71c8faf4cbc887d8e84300c5b55e4872a44341dc
```
