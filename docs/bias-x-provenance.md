# Where does the 1.312 m/s² correction come from?

25 October 2026 · IntelQCopter · X-bias provenance report

This report explains why the expected X-reading decrease is approximately 1.312 m/s². It traces that number from your original six-pose measurements to a fitted bias, then to the integer MPU6050 offset word. The bias estimate and the predicted register effect are different quantities.

**STATUS: calculated prediction. The register's physical response and reset/power-cycle behavior still need bench verification. This report performs no new sensor measurement, upload or firmware change.**

## 1. Which measurements produced the X bias?

The completed secured-mount session contains 18 records × 20,000 samples = 360,000 samples. Only repetitions 1 and 2 were used to fit the candidate: 12 pose means from 240,000 samples. Repetition 3 and the later four tilted poses were kept out of this fit. The four tilted records were reviewed using your clarified raised-side order and approximate ±1° hand-held reference; they did not change the bias.

Each row below shows the mean SENSOR-X reading while the DRONE axis named in the first column pointed upward. All numbers are m/s². These are not three different axes of one sample: every numeric entry is the X channel measured in a different physical pose.

| Drone pose | Repetition 1: sensor X | Repetition 2: sensor X | Mean of the two |
| --- | --- | --- | --- |
| X+ | 10.594872901279 | 10.608765615543 | 10.601819258411 |
| X− | -8.094564173977 | -8.093103471545 | -8.093833822761 |
| Y+ | 2.301335605582 | 2.324001991539 | 2.312668798561 |
| Y− | 0.530471410543 | 0.534481099773 | 0.532476255158 |
| Z+ | 4.096255182672 | 4.077099294899 | 4.086677238786 |
| Z− | -1.588812811814 | -1.586601766581 | -1.587707289197 |

$$
\bar r_{X+}=\frac{10.594872901279+10.608765615543}{2}\approx10.601819258411
$$

## 2. Why do opposite poses reveal the bias?

A mounted sensor can mix all three drone axes. For its X output, use the forward model below. The A coefficients describe mounting and sensor response; e denotes the remaining measurement/model error. We assume the labelled reference directions are accurate enough for a provisional fit.

$$
r_x=b_x+A_{11}a_x+A_{12}a_y+A_{13}a_z+e_x
$$

In the +X and −X poses, the target body vectors are (+g,0,0) and (−g,0,0). Ignoring mean error for this derivation, the measured X values satisfy:

$$
\bar r_{X+}=b_x+gA_{11},\qquad \bar r_{X-}=b_x-gA_{11}
$$

Adding the pair cancels the gravity response, even when the sensor is tilted relative to the drone:

$$
c_X=\frac{\bar r_{X+}+\bar r_{X-}}{2}\approx b_x
$$

$$
c_X=\frac{10.601819258411-8.093833822761}{2}\approx1.253992717825
$$

Repeat this operation using the Y and Z opposite-pose pairs. In ideal data the three answers agree. Your measurements give:

| Opposite-pose pair | Sensor-X midpoint (m/s²) |
| --- | --- |
| X | 1.253992717825 |
| Y | 1.422572526859 |
| Z | 1.249484974794 |

With equal weights for these balanced poses, average the three midpoint estimates:

$$
b_x=\frac{c_X+c_Y+c_Z}{3}
$$

$$
b_x\approx\frac{1.253992717825+1.422572526859+1.249484974794}{3}
$$

$$
b_x\approx1.308683406493\ \mathrm{m/s^2}
$$

This reproduces the stored forward least-squares fit. To see why, minimize the sum of squared residuals r_i − b − A a_i over the 12 training means. Differentiating with respect to b gives the following normal equation:

$$
0=-2\sum_{i=1}^{12}(r_{x,i}-b_x-A_xa_i)
$$

The symmetric target vectors sum to zero. Therefore the fitted intercept equals the mean of the 12 measured X means:

$$
\sum_i a_i=0\quad\Longrightarrow\quad b_x=\frac{1}{12}\sum_{i=1}^{12}r_{x,i}
$$

**The midpoint estimates differ by about 0.173 m/s². That disagreement can reflect pose-reference error, repositioning, temperature or model limitations. Many decimal places describe reproducible arithmetic, not equally precise knowledge of the true bias. No Gaussian or independent-sample assumption is needed for this algebra.**

## 3. Convert the bias to an MPU offset word

The established MPU6050 library convention uses a nominal accelerometer offset-word scale of 2048 units/g. This is separate from the measurement ADC scale, which is 4096 counts/g at our ±8g setting. The offset conversion and preservation of compensation bit 0 are implemented in established MPU6050 drivers; the public register map does not fully specify these trim semantics. Applying this convention to our particular module is a prediction that the bench test must verify.

$$
1\ \mathrm{offset\ word\ unit}\ \longleftrightarrow\ \frac{g}{2048}\ \mathrm{m/s^2}
$$

We preserve compensation bit 0 by changing the whole word by an even integer. Hence the correction grid is two word units per step:

$$
q=\frac{2g}{2048}=\frac{g}{1024}=0.009576806640625\ \mathrm{m/s^2}
$$

Calculate how many of those steps would remove the fitted bias:

$$
n_{ideal}=\frac{b_x}{q}=\frac{1024b_x}{g}\approx136.651334375
$$

$$
n=\mathrm{round}(136.651334375)=137,\qquad d=2n=274
$$

The neighbouring allowed corrections show why 137 steps were selected:

| Steps | Word decrement | Predicted decrease (m/s²) | Remaining bias (m/s²) |
| --- | --- | --- | --- |
| 136 | 272 | 1.302445703125 | +0.006237703368 |
| 137 | 274 | 1.312022509766 | -0.003339103273 |

## 4. Why write −2348 rather than −274?

Your original recording headers reported an X offset word of −2074 after initialization. That existing baseline is retained and adjusted; it is not replaced by the decrement alone. It is a register value, not an acceleration reading.

$$
o_{new}=o_{baseline}-d=-2074-274=-2348
$$

The positive fitted bias calls for a negative offset-word change under the nominal convention. The firmware verifies the known baseline, writes the absolute target and reads it back. Repeating a command sets the same target; it does not subtract another 274. An unexpected baseline stops the lesson instead of accumulating a correction.

```cpp
static const float accel_bias_x = 1.308683406f;
static const int16_t expected_baseline_x = -2074;
static const int32_t offset_decrement = 2 * lroundf(accel_bias_x * 1024.0f / 9.80665f);
static const int32_t corrected_offset_x = expected_baseline_x - offset_decrement;
```

## 5. The origin of 1.312022510 m/s²

Convert the selected word decrement back into its nominal physical effect:

$$
\Delta a_x=d\frac{g}{2048}=274\frac{9.80665}{2048}
$$

$$
\Delta a_x=1.312022509765625\ \mathrm{m/s^2}\approx1.312\ \mathrm{m/s^2}
$$

The fitted bias is 1.308683406 m/s²; the allowed register grid makes the selected predicted correction slightly larger. The modeled residual is:

$$
b_{after}=b_x-\Delta a_x\approx-0.003339103273\ \mathrm{m/s^2}
$$

Nearest-step rounding alone gives a maximum magnitude q/2, assuming the nominal conversion. This is only a quantization bound; it does not bound the true calibration, reference or sensor error.

$$
|e_{round}|\leq\frac{q}{2}=0.0047884033203125\ \mathrm{m/s^2}
$$

At ±8g, the measurement ADC has 4096 counts/g. The same predicted decrease therefore corresponds to:

$$
\Delta\mathrm{ADC}=\frac{\Delta a_x}{g}\,4096=\frac{274}{2048}\,4096=548\ \mathrm{counts}
$$

## 6. What must the real bench test demonstrate?

Keep the drone fixed in one orientation. The baseline and corrected X readings both contain the same real gravity component. Subtracting their means should cancel that common component, leaving the register's effect:

$$
\bar r_{x,baseline}-\bar r_{x,corrected}\approx1.312022510\ \mathrm{m/s^2}
$$

The final X reading need not be zero. For example, with a hypothetical constant real X gravity component of 2.8 m/s², the model predicts baseline X = 2.8 + 1.308683406 = 4.108683406, and corrected X = 4.108683406 − 1.312022510 = 2.796660896 m/s². These example readings are invented to illustrate the arithmetic; they are not recorded results.

The prepared PC test measures baseline, corrected and baseline again, interpolating between the two baseline means to reduce linear drift. Readback verifies the bits; the observed change checks actual sensor response. Neither test alone proves complete three-axis calibration or flight readiness. Reset and full power-cycle checks remain separate.

| Quantity | Origin | Evidence status |
| --- | --- | --- |
| 1.308683406 m/s² | Fit of real training-pose means | Provisional estimate; reference uncertainty remains |
| −2074 | Recorded initial offset word | Seen in original raw-session metadata |
| 274 and −2348 | Rounded decrement and absolute target | Computed and compiled |
| 1.312022510 m/s² / 548 ADC counts | Nominal register-response model | Prediction; not a measured response |
| Actual decrease and reboot behavior | Physical bench comparison | Not established by this report |

## Sources and reproduction

Data: the secured-mount analysis JSON and the recorded post-reset baseline. Method: balanced forward affine least squares. Assumed standard gravity is 9.80665 m/s²; no local-gravity measurement is claimed. Processing: Python arithmetic and Matplotlib SVG equation rendering; no simulator or new sensor reading. Original raw logs remain local. Numerical output is saved alongside this report for independent checking.

- [Measured-data fit JSON](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json)
- [Tilt validation review](../measurements/mpu6050/2026-10-25-tilt-validation/report.md)
- [Numerical provenance](../measurements/mpu6050/2026-10-25-bias-x-provenance/calculation.json)
- [Bench procedure](bench-x-bias.md)
- [Current firmware](../firmware/main.cpp)
- [TDK register map: ADC sensitivities](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [MPU6050 library: word trims and bit-zero preservation](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp)
- [MPU6050 example: nominal 2048 offset units/g](https://github.com/kriswiner/MPU6050HMC5883AHRS/blob/master/MPU6050HMC5883AHRS.ino)
