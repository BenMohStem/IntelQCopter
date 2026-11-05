# Height, vertical speed and acceleration-bias Kalman filter

## State definition

\[
x=[z, v_z, b_a]^T
\]

where z is relative world-up height in meters, v_z vertical velocity m/s and b_a vertical accelerometer bias m/s².

Quaternion-rotated, gravity-compensated specific force is the input `u = (R_WB(q) a_body)_z - g`. For sample interval dt,

\[
F=\begin{bmatrix}1&dt&-dt^2/2\\0&1&-dt\\0&0&1\end{bmatrix},\quad
B=\begin{bmatrix}dt^2/2\\dt\\0\end{bmatrix},\quad
x_{k+1}=Fx_k+Bu.
\]

Covariance predicts via `P'=F P Fᵀ+B σ_a² Bᵀ + Q_bias`. The current code uses `sigmaAcceleration=0.6 m/s²` and `sigmaBiasRandomWalk=0.035` in an assumed process-noise representation; tune via residual statistics rather than laboratory fantasies.

## Measurement updates

- BMP: `z_baro = pressureAltitude(P;P0)`, measurement row H=[1,0,0], provisional operating SD 0.22 m.
- Optional LiDAR: relative ground range corrected by tilt and baseline, higher trust only with valid range/quality.
- Sequential updates on fresh arrivals; innovation gating approximately 5σ BMP and 4σ LiDAR.
- Joseph covariance update helps preserve positive numerical covariance and symmetry after correction.

## Observability limits

A single barometer gives height only relative to drifting pressure reference; dynamic pressure and temperature changes can mimic height. A bias-augmented state can reduce drift but cannot manufacture an absolute vertical reference. Near-ground LiDAR, once calibrated, is a valuable independent height observation. Without horizontal velocities/position sensors there is no XY hold.
