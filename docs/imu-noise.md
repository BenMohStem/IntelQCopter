# Measure noise before setting Kalman gains

## Configuration now selected

| Parameter | Current setting |
|---|---|
| Acceleration range | ±8g |
| Gyro range | ±500 degrees/s |
| DLPF mode | 0: 260 Hz acceleration / 256 Hz gyro |
| Datasheet filter delay | nominally 0 ms acceleration / 0.98 ms gyro (filter only) |
| Divider | 15: 8000/(1+15) = 500 Hz MPU output |
| Current bench read/report | About 10 Hz, using delay(100) |

The user selected the higher-bandwidth setting to reduce filter delay. It provides less high-frequency attenuation and is not strong anti-alias filtering for a 250 Hz Nyquist limit. It is not a proven best choice for this drone. Its delay, vibration rejection and aliasing must be measured during later timing/model/controller work. The 500 Hz output setting does not create a 500 Hz FC or lossless 500 Hz recording. The ADC range and filtering changed, so repeat the calibration pose checks rather than assuming old coefficients remain equally accurate.

Source: [manufacturer CONFIG/DLPF table](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf). No high-rate serial streaming, timing ISR or new PID is added in this lesson. The existing hardware bias restoration and software mounting correction remain active.

## Why no standard deviation number is supplied yet

The six-pose file contains means, not the individual readings or within-pose variances. They cannot identify sensor noise. A fitting residual measures disagreement with calibration targets, not stationary temporal noise. Manufacturer noise density, sample variance, standard error of a mean and mass uncertainty are also different quantities.

Do not replace missing data with zero noise or a guessed “precise” sigma.

## Your recording

1. Upload yourself. Confirm startup configuration/offset readback succeeds. Keep motor outputs disabled and propellers removed.
2. Let the sensor temperature settle. Fix the drone on a stable surface, upright with drone +Z upward, away from handling and vibration.
3. Record at least 60 seconds of original Monitor output at 115200 baud. Preserve the log as plain text, including startup and failures. Do not touch the drone during the selected stationary interval.
4. The new IMU lines are CSV:

```text
IMU,uptime_us,ax_drone,ay_drone,az_drone,gx_sensor,gy_sensor,gz_sensor
```

The actual firmware prints numerical rows without the header. Acceleration is in m/s² and angular speed in rad/s. Accelerometer data is corrected into drone coordinates; gyro data still uses sensor coordinates and has not been calibrated or rotated. This distinction prevents silently using sensor gyro noise for a different drone axis.

5. Send the original stationary log and say which part was motionless, or use tools/calibration/imu_noise.py on the saved log. It ignores non-IMU lines and produces a report. Do not mix different orientations, warmup or deliberate movement into a stationary-noise estimate.

The current report interval is about 100 ms; this is an initial subsampled estimate of the filtered output's marginal variation. It does not measure the spectrum, correlations or performance of a 500 Hz read loop. Later we must repeat with time-checked fresh 500 Hz acquisition before treating a high-rate model as characterized. The Adafruit event read also lacks reliable per-sample I2C failure propagation; no continuous health guarantee is claimed.

## Calculations

For one channel with N samples:

$$
\mu=\frac1N\sum_i x_i,\qquad
s^2=\frac{\sum_i(x_i-\mu)^2}{N-1},\qquad s=\sqrt{s^2}.
$$

The mean gyro value is a candidate stationary bias; its standard deviation measures fluctuation around that mean. Acceleration mean contains gravity as well as residual bias. Do not subtract its entire stationary mean as a new calibration.

The analysis also derives upright stationary roll/pitch observations from the corrected acceleration:

$$
\phi_a=\operatorname{atan2}(a_y,a_z),\qquad
\theta_a=\operatorname{atan2}(-a_x,\sqrt{a_y^2+a_z^2}).
$$

It reports their noise in radians and the sample roll/pitch covariance. Stay upright during this test; a roll angle near the ±pi branch cut would need circular statistics instead. These angles are not ground-truth attitude, and dynamic acceleration will change their error during flight.

## Using results in a Kalman model

For a scalar angle observation, an initial stationary measurement variance can be:

$$
R_\phi=s_{\phi_a}^2,\qquad R_\theta=s_{\theta_a}^2.
$$

Those are angle variances in rad², not acceleration variances in (m/s²)². For a joint roll/pitch observation use the covariance matrix instead of assuming independence without checking it.

If a simple angle predictor has discrete independent rate error with variance s_omega², then its one-step angle-noise contribution would be dt²*s_omega². That is a model assumption, not a universal Q formula. Continuous noise-density models discretize differently, and filtered adjacent samples are correlated. Gyro-to-drone rotation must be established before using its axis variances for drone roll/pitch. Gyro bias drift requires additional characterization/modeling; a stationary standard deviation does not describe it completely.

For altitude, barometer/vertical-acceleration noise, gravity compensation, orientation uncertainty and dt are required. We have no barometer noise log yet. The 2D altitude Q/R and final Kalman gains cannot be derived from these IMU means alone. Do not use an accelerometer norm as altitude or as a substitute for barometer noise.

The report gives candidate statistics, not validated gains or flight acceptance. Original samples are the evidence; preserve them when repeating tests at other temperatures or filter settings.


## Run the analysis yourself

From the stepwise project folder, after saving the actual stationary log as measurements/stationary.txt:

```powershell
.\qcopter.ps1 imu-noise .\measurements\stationary.txt --output .\measurements\stationary-noise.json
```

Choose a new output filename for each run. Keep the original log with the report. This command does not connect to the board or upload firmware. No real standard deviation, Q/R or gain is claimed until a valid stationary recording has been supplied. The updated configuration and analyzer have not been built or exercised in this iteration; previous numerical/build results describe their earlier source revisions.
