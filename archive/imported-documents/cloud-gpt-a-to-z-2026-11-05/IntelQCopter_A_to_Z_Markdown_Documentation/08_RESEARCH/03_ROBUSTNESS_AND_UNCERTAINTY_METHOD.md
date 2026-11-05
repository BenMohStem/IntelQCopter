# Robustness, statistics and what an error bar means

A physical drone model is not accurate merely because it has many decimal places. This project has excellent repeated sensor data but unmeasured propulsion and some uncertain geometry. Use transparent uncertainties rather than masking gaps.

## Suggested hierarchy

- **Type A:** estimate noise/variance and covariance from repeated measurements, accounting for temporal correlation, Allan slope and between-session changes.
- **Type B:** manufacturer tolerances, ruler/scale resolution, fixture orientation and sensor mounting error.
- **Structural model error:** missing motor lag, nonideal mixer, time delay, power droop, thrust curve and propwash, which cannot be encoded by just smaller Gaussian σ.
- **Empirical verification:** new independent measurements at relevant operating conditions.

## Model propagation

For function y=f(x), local first-order covariance is `Σ_y≈J_f Σ_x J_fᵀ`. For strong nonlinearities (hover ∝m/Tmax), Monte Carlo over bounded physically plausible priors is more useful; retain correlations and report rejected infeasible models. A 100,000-draw result remains a *conditional* distribution if input variances are assumed.

## Model validation

Use calibration holdout sessions; do not train on the same identical samples used to judge performance. Evaluate estimator NIS/NEES if ground truth exists, rate-step transient vs measured motor plant, and worst-case loop latency under stress. State uncertainty in units and never claim sub-centimeter hover solely from stationary barometer SD.
