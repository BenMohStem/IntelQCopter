# Preparing the angle filter with fewer physical tests

This step is **PC-only preparation**. The ESP32 firmware, calibration registers,
calibration coefficients and motor-disablement are unchanged. The user will do
physical reference rotations later. No new stationary recording or upload is
needed now.

[Results and graphs](../measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md)
show what we learned. The calculation is in
[prepare_attitude.py](../tools/estimation/prepare_attitude.py); its small
`kalman_angle()` function is the candidate update. It is not enabled in main.cpp.

## 1. The technique: Design of Experiments (DOE)

This may be the technique you remember from chemistry/chemical engineering.
Instead of changing factors without a plan, choose the factors, levels and
responses first. A **full factorial** experiment tests every combination of the
chosen levels. A **fractional factorial** uses fewer combinations but can mix
together effects that cannot then be separated.

Here only two model factors are varied, so four combinations are enough to
separate their main effects and their interaction **at these chosen levels**:

| Case | Added gyro bias | Added body-Y acceleration |
|---|---:|---:|
| 1 | 0 deg/s | 0 g |
| 2 | 0 deg/s | 0.1 g |
| 3 | 0.1 deg/s | 0 g |
| 4 | 0.1 deg/s | 0.1 g |

These are deliberately chosen stress levels, not newly measured drone properties.
All cases use the same assumed motion and same noise draws. Thus differences
between cases are not caused by using a different random noise sequence. The
experiment describes these four cases; it does not identify temperature effects,
vibration, all acceleration directions, intermediate levels or rare failures.

For a response such as RMSE, code the low/high levels as -1/+1:

$$
y=\beta_0+\beta_A A+\beta_B B+\beta_{AB}AB.
$$

The four design rows form

$$
X=\begin{bmatrix}
1&-1&-1&+1\\
1&-1&+1&-1\\
1&+1&-1&-1\\
1&+1&+1&+1
\end{bmatrix},\qquad X^TX=4I.
$$

Its columns are independent, so these four coefficients are identifiable. There
are no replicate degrees of freedom in four responses/four coefficients: we do
not calculate experimental p-values or claim a physical confidence interval.
The cheap software fault tests stay in place: each tests a different failure,
and DOE does not replace those assertions.

See [NIST design choices](https://www.itl.nist.gov/div898/handbook/pri/section3/pri3.htm)
and [why fractional designs alias effects](https://www.itl.nist.gov/div898/handbook/pri/section3/pri3343.htm).

## 2. Reuse the real data; reserve a check before calculating gains

Three repaired captures (60,000 samples) estimate stationary noise. The fourth
capture (20,000 samples) is reserved **for this filter study**. It was already
inspected for timing in the previous step, so it is not a globally blind dataset.
The program freezes Q and R before reading its measurements. No later adjustment
minimizes its displayed fluctuation or subtracts its residual mean.

Original serial hashes, source fingerprints, sequence/checksum, MCU calculations,
final metadata and stored raw words are checked again using the existing verifier.
Calibration is identical across these captures; firmware differs because the
last one added the provisional gyro rotation. The PC applies the same recorded
rotation candidate to every sensor-frame gyro vector. Physical direction and
scale are still unverified.

For run j, subtract that run's own mean before estimating covariance:

$$
S=\frac{\sum_j\sum_{i=1}^{N_j}
(x_{ji}-\bar x_j)(x_{ji}-\bar x_j)^T}
{\sum_j(N_j-1)}.
$$

This prevents a small difference in support angle between runs from being counted
as high-frequency noise. It does not remove bias in the filter input or prove
stationarity. Separate mean and one-second block-mean spans remain in the results.
The support was assumed stationary, with no independently measured level angle.

## 3. The candidate: one angle and one uncertainty

Use radians inside calculations. Let x be either roll or pitch, u its gyro rate
input, and z the angle inferred from the calibrated accelerometer:

$$
z_{roll}=\operatorname{atan2}(a_y,a_z),\qquad
z_{pitch}=\operatorname{atan2}(-a_x,\sqrt{a_y^2+a_z^2}).
$$

Near level, use body gyro p for roll rate and q for pitch rate. This is a local
approximation: arbitrary simultaneous rotations require the proper body-rate to
attitude equations. The model experiment uses pure roll, where this particular
rate relationship is exact. Yaw has no gravity-based absolute heading observation.

The discrete prediction is

$$
\hat x_k^-=\hat x_{k-1}+\Delta t\,u_k,\qquad
P_k^-=P_{k-1}+Q,\qquad \Delta t=0.001\ \mathrm{s}.
$$

P is the model's angle-error variance, in rad². The rate input is interpreted as
the rate for this interval. If its zero-mean error is independent of the previous
state error and of other intervals, then

$$
\operatorname{Var}(\Delta t\,e_u)=(\Delta t)^2\sigma_u^2,
\qquad Q_{white\ rate}=(\Delta t)^2\sigma_u^2.
$$

This Q is **only an initial white-rate model**. It leaves out changing bias,
unmodeled movement, sampling uncertainty and temporal correlation. It is not a
complete process-noise identification. R is the corresponding diagonal element
of the stationary angle covariance, not an estimate of angle error in flight.

The accelerometer correction is

$$
K_k=\frac{P_k^-}{P_k^-+R},\qquad
\hat x_k=\hat x_k^-+K_k(z_k-\hat x_k^-).
$$

Angles and their differences are wrapped to the interval [-pi, pi) in code.
The following uncertainty formula assumes prediction and measurement errors are
uncorrelated:

$$
P_k=(1-K_k)^2P_k^-+K_k^2R.
$$

Why this K? Differentiate the expression with respect to K:

$$
\frac{dP_k}{dK}=-2(1-K)P_k^-+2KR=0
\quad\Longrightarrow\quad K(P_k^-+R)=P_k^-.
$$

Since the second derivative is 2(P_k^-+R)>0, this is a minimum **under those
covariance assumptions**. Gaussianity is not necessary for this best linear
weight calculation. A Gaussian, correctly specified model provides stronger
distributional interpretations, which have not been established for this drone.

For constant positive Q and R, the limiting corrected covariance satisfies

$$
P=\frac{R(P+Q)}{P+Q+R}
\Rightarrow P^2+QP-QR=0
\Rightarrow P_\infty=\frac{-Q+\sqrt{Q^2+4QR}}{2}.
$$

The code uses the equivalent numerically stable positive-root expression
2QR/(sqrt(Q²+4QR)+Q). Then K_inf=(P_inf+Q)/(P_inf+Q+R).
The associated accelerometer-correction pole is 1-K_inf, giving
tau=-dt/log(1-K_inf). This is a correction time constant; it is **not** a delay
on the gyro prediction. Initial x=z[0], P=R; the first observation is not counted
twice. The first two seconds are omitted from descriptive/model comparison
metrics by a fixed choice, not because all transients are guaranteed over then.

## 4. Why a smooth angle can still be wrong

An accelerometer measures specific force. Horizontal acceleration contributes to
the vector used by atan2. For a level drone with the model's added body-Y signal
0.1 g, the apparent roll is

$$
z_{roll}=\operatorname{atan2}(0.1g,g)=\arctan(0.1)
\approx5.71^\circ.
$$

Yet its magnitude is only sqrt(1+0.1²)g, about 1.005g. A simple test that the norm
is close to g can miss this error. A generic low-pass filter cannot uniquely
separate sustained acceleration from a different gravity direction. Reducing K
slows this error but increases sensitivity to gyro drift. Do not tune only for
the smallest stationary standard deviation.

The model uses 10° sinusoidal roll at 0.5 Hz, exact discrete interval rates,
and optional acceleration from 8 to 12 seconds. Gaussian white noise is an
explicit modeling assumption for this sensitivity experiment; the real measured
lag correlations do not support treating the actual samples as white. This is
neither a Gazebo flight nor an execution of ESP32 firmware.

## 5. Next steps and commands

Review the reserved-data graph and the four modeled outcomes. The offline filter
is a starting point; it has no flight-acceptance threshold or certified accuracy.
Keep physical reference rotations pending. Later, use one bidirectional session
per drone axis with clear start/stop/reference notes. A known measured angle is
needed for scale; visual direction alone checks only sign/dominant axis.

After that gate, improve dynamic measurement handling and choose the estimator
model, then add one small update to main.cpp and remeasure the actual 1 kHz budget.
BMP280/altitude and PID work remain separate later steps.

From the Stepwise folder:

```powershell
.\qcopter.ps1 check-attitude
.\qcopter.ps1 check-all
.\qcopter.ps1 prepare-attitude --output .local/attitude-replay-01
```

Choose a new output directory each time. Original private capture folders must
exist to reproduce the real-data analysis; the public repo contains reviewed
derived evidence. The analytical checks run without hardware or private logs.

## Next small offline lesson: identical readings, different attitudes

Run `python tools/estimation/acceleration_ambiguity.py` (standard library only).
This analytic model compares a stationary 10-degree roll with a level vehicle
under constant body-Y and body-Z acceleration. Both have zero angular rate,
identical accelerometer readings and exactly 1 g measured magnitude. Therefore
an acceleration gate based only on magnitude cannot reject the misleading case.
An accelerometer-only angle reports 10 degrees for both; its level-case error is
10 degrees. These are constructed assumptions, not measured drone motion.

Initial attitude is unknown and transitions are excluded. Known initial attitude,
reliable gyro integration over a finite interval, or independent velocity/position
information may help. A more elaborate filter cannot recover information absent
from identical inputs. Next compare a bounded acceleration disturbance and recovery
before choosing an offline candidate. Do not activate the MCU filter yet.

2026-10-29 checkout review also found an existing uncommitted gyro matrix change:
`0.288364175f` became `0.2808364175f` in main.cpp. It is preserved, not accepted as
a new calibration, and is inconsistent with config/gyro-frame.json. Existing
source fingerprints must remain stale until that change is resolved explicitly.
The existing gyro-frame test could not start under the default system Python:
`ModuleNotFoundError: No module named 'serial'`. This failure is retained here;
it is not a passing test or evidence against the historical rotation model.
