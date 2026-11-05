# Monte Carlo mass/inertia/PID uncertainty — correct interpretation

FC v2 provides a 100,000-draw study using assumed distributions for mass, inertia, motor diagonal, motor maximum thrust, yaw leverage and thrust lag. Outputs include means, SDs, percentiles for gains and surrogate hover fractions, and idealized stability checks.

## Main modeled inputs

| Parameter | Basic prior center | Prior 1σ |
|---|---:|---:|
| Mass m | 1.104 kg | 0.0506 kg |
| Jxx | 0.009781 kg·m² | 0.000446 |
| Jyy | 0.010390 kg·m² | 0.000454 |
| Jzz | 0.019170 kg·m² | 0.000831 |
| Tmax per motor | 6 N | 1.5 N |
| Motor lag | 0.040 s | 0.015 s |
| Motor diagonal | 0.450 m | 0.010 m |

## Results to interpret carefully

The modeled median hover normalized fraction is around **45.2%** with the assumed 6 N thrust; a significant fraction of cases exceed a historical 45% cap. This is evidence of an **uncertain or possibly infeasible assumed operating point**; not evidence of real throttle position.

## Improvements needed for scientific credibility

Obtain measured component mass distributions, measured motor force curves and delay, estimate covariance between inertia components, and use physically consistent correlated variations in Monte Carlo. Report conditional distributions, not “95% aircraft confidence” when the input priors are guesses. Propagate calibration parameter covariance into attitude/altitude filter performance as well as torque gains.
