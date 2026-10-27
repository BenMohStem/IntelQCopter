# Prepared gyro-rotation review tool

The PC can review a future 20-second `operator_rotation` FIFO capture. It
independently compares the stored samples, source, gyro frame, residual trim,
duration and checksum against the original serial records before calculating.
It does not write calibration, flash a board or enable motor outputs.

## Math

For each raw post-trim gyro word $r_{k,j}$ and stored software remainder $b_j$,
the sensor-axis angular rate in radians/second is

$$
\omega_{s,k,j}=r_{k,j}\frac{\pi}{180\cdot65.5}-b_j.
$$

The provisional rigid mounting matrix $R$ maps sensor axes to drone axes:

$$
\boldsymbol\omega_{b,k}=R\boldsymbol\omega_{s,k}.
$$

The FIFO log has **no individual sample timestamps**. If its complete trailer
reports elapsed time $T$ for $N=20000$ samples, the tool approximates

$$
\Delta t\approx\frac{T}{N},\qquad
I_j=\frac{180}{\pi}\sum_{k=1}^{N}\omega_{b,k,j}\Delta t.
$$

Each $I_j$ is an **integral of a body-rate component**, in degrees. For a
roughly fixed single-axis rotation, it can be compared with the independently
observed angle. In general 3D motion the three integrals are not Euler angles:
rotations about different moving axes do not commute. A turn and return can
also cancel in the net integral. The plot therefore shows the body rates over
time, averaged in groups of ten samples for readability (~100 Hz); all
20,000 original samples are used in the integral.

For an operator-specified axis and sign, the tool also reports the signed
component and $\sqrt{I_\perp^2+I_{\perp 2}^2}$. This cross-axis number is a
diagnostic, not a proof of cross-axis calibration error: off-axis hand motion
can produce it. If the user supplies an independent angle magnitude $\theta_r$,
the displayed difference is $I_{expected}-\theta_r$. The tool deliberately
does **not** provide a confidence interval: sensor bias drift, temperature,
correlation, timing and reference uncertainty have not been established here.

## For the later physical test

With motor outputs disabled and propellers removed, use a controlled reference
to turn the drone about one known drone axis in one known direction, then hold
the final position until the capture ends. The right-hand rule defines positive
rotation: thumb points along the positive axis, curled fingers show positive
rotation. Record the real motion with the existing command:

```powershell
.\qcopter.ps1 record-1khz --port COM3 --condition operator_rotation --note "X positive; measured reference 90 degrees"
```

Use the actual saved folder printed by that command in the next command:

```powershell
.\qcopter.ps1 analyze-gyro-motion --recording "C:\path\to\recording" --axis x --direction positive --reference-deg 90
```

Replace the axis, direction and reference with the real observation. Omit
`--reference-deg` when no independent angle was measured. Results are saved
to a new timestamped `artifacts/gyro-motion-analysis` folder as `analysis.json`
and `gyro-rates.svg`. Open the SVG in a browser. Never relabel a stationary
recording as a rotation; retain the original evidence.

## What was tested now

[Original synthetic unit-test log](synthetic-tests-original.log) contains five
passing controls: known positive and negative single-axis signals, cross-axis
response, corrupted samples/serial, invalid rate, and changed frame metadata.
The tool also rechecked all 20,000 records from a previously collected real
stationary log against its serial stream and correctly rejected that recording
for rotation analysis. [Machine-readable validation](validation.json).

No new physical motion was performed. Physical gyro direction and scale remain
pending, as do dynamic attitude, PID and flight acceptance. No MCU source,
calibration constants or hardware test was changed by this tool.

## Graph from a real earlier stationary capture

[Open the real stationary gyro graph](real-stationary-rates.svg) ·
[PNG preview](real-stationary-rates.png). It uses the actual 20,000 raw samples
from the previously collected recording `20261026T083102215018Z`, after the
stored gyro bias remainder and provisional body-frame rotation. Each displayed
point averages ten samples. The device was assumed stationary on its support;
this is not a new capture or a physical rotation test. The original serial and
sample files remain in the private `artifacts/fifo1000-recordings` folder.
