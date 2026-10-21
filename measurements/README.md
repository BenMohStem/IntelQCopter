# Measure the aircraft before calling its gains precise

**[Open the formatted reading view](guide.html)** if your Markdown viewer shows raw equations.

CSV files in templates/ contain **headers only**. Copy one into a dated session folder,
preserve raw readings, and record reference uncertainty and configuration alongside it.
Do not replace unknown measurements with teaching examples.

## Order of work

| Order | Quantity | Method | What it improves |
|---|---|---|---|
| 1 | Complete flying mass and centre of mass | Checked scale, repeated assembly measurements, two-support reactions | Hover thrust and lever arms |
| 2 | Dimensions, component positions, prop direction | Ruler/calipers and labelled photos | Geometry and allocation signs |
| 3 | IMU bias/noise versus temperature | Stationary raw capture, six faces; record rate/ranges/DLPF | Bias and Kalman Q/R |
| 4 | Voltage/current | Suitable reference measurements across operating range | Coulomb counting and alarms |
| 5 | Static thrust vs command and voltage | Secured motor/prop thrust stand, load cell, voltage/current/RPM log | Hover command and actuator slope |
| 6 | Motor delay / transient response | Timestamp command steps and thrust with suitable bandwidth | Poles and delay margin |
| 7 | Inertia about x/y/z | Component model then bifilar pendulum, subtract fixture | Angular gains |
| 8 | Rotor reaction torque | Torque stand / known lever arm and force | Yaw allocation |
| 9 | Height/range, camera calibration and latency | Known distances, camera target, timestamped messages | Altitude and navigation |
| 10 | Effective battery capacity | Appropriate controlled charger/discharger, integrate current | SOC and warning timing |

For rotating-prop measurements use a rigid rated stand, remote operation and suitable protection.
Do not hold a powered motor by hand. Verify mechanical bandwidth: a fast ADC does not make a slow scale fast.

## Basic calculations

For $N$ repeated readings $y_1,\ldots,y_N$:

$$
\bar{y}=\frac{1}{N}\sum_{k=1}^{N} y_k,\qquad
s=\sqrt{\frac{\sum_{k=1}^{N}(y_k-\bar{y})^2}{N-1}},\qquad
u_A(\bar{y})=\frac{s}{\sqrt{N}}.
$$

Here $\bar{y}$ is the mean, $s$ is the sample standard deviation, and $u_A(\bar{y})$
is the standard uncertainty of the mean from repeatability.
The last formula assumes independent samples. Filtering often creates correlation.
A fixed assembly has a fixed mass; repeated scale readings fluctuate.
Their standard deviation does not include every calibration bias.

For separate independent uncertainties, combine their variances.
For example:

$$
u_c=\sqrt{u_A^2+u_{\mathrm{reference}}^2+u_{\mathrm{resolution}}^2}.
$$

Avoid counting the same contribution twice. For correlated inputs:

$$
u^2(f)\simeq \nabla f^T\Sigma\nabla f.
$$

For resolution step $q$, a uniform rounding model gives

$$
u_{\mathrm{resolution}}=\frac{q}{\sqrt{12}}.
$$

Use that equation only when the uniform model is justified.
A quoted ± limit alone does not establish a Gaussian standard deviation.
See the [full mathematical guide](../docs/reports/IntelQcopter-model-guide.pdf) for correlated samples,
Allan variance, apparatus procedures, propagation and worked examples.

## Session notes

Record date, apparatus photos, raw file hash, software commit, units, instrument model/calibration,
temperature, battery ID/voltage, exact motor/prop model, ESC settings,
sample frequency, filtering, axis/sign convention, mounting and rejected samples.
Reference measurements also have uncertainty; record it instead of calling them exact truth.
Measure the final flying assembly again after component changes.
