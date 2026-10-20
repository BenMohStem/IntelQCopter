"""Publish a source-bound teaching guide; does not execute or alter the firmware."""
from pathlib import Path
import hashlib
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'flight_controller/flight_controller.c'
DEST = ROOT / 'docs/flight-controller-line-by-line.md'
SOURCE_REF = '68619b5'
source_bytes = subprocess.check_output([
    'git', '-c', f'safe.directory={ROOT.as_posix()}', 'show',
    f'{SOURCE_REF}:flight_controller/flight_controller.c'], cwd=ROOT)
lines = source_bytes.decode('utf-8').splitlines()
notes = {}

def explain(start, end, text):
    """A multiline C statement has one explanation, referenced by each physical line."""
    for number in range(start, end + 1):
        notes[number] = text if number == start else f'Continuation of the statement at L{start}: {text}'

DATA = r'''
1-9|Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming.
10|Include math declarations for sin/cos, atan2, square roots, finite-value checks and min/max operations.
11|Include C Boolean types and the true/false constants.
12|Include puts and printf for the Pico USB/serial status messages.
13|Include fixed-width integer types used in packets, timestamps and counters.
14-15|Document the command protocol duplicated in the handheld's one-file firmware. Serialization explicitly chooses byte order instead of transmitting an in-memory C struct with possible padding.
16|Include stdbool again. Repeating a standard header is harmless but redundant here.
17|Include size_t for buffer lengths and NULL for pointer checks.
18|Include stdint again; this is redundant but harmless.
20|A complete command packet occupies exactly 22 bytes; the unsigned suffix u makes this an unsigned integer constant.
21|The packet format version is 1. Both sender and receiver must agree.
22|Signed roll, pitch and yaw commands use integer scale 10000: -10000..10000 corresponds to -1..1 before conversion to physical command units.
23|Each trigger uses integer scale 1000: 0..1000 means released..fully pressed.
24|Throttle uses 0..10000. The midpoint 5000 will become 0.5, meaning zero requested vertical speed.
26|Begin an anonymous enumeration that names the six supported button bits.
27|LB occupies bit 0: 1u shifted left zero positions gives mask 0x01.
28|RB occupies bit 1, mask 0x02.
29|North occupies bit 2, mask 0x04.
30|East occupies bit 3, mask 0x08.
31|South occupies bit 4, mask 0x10.
32|West occupies bit 5, mask 0x20.
33|End the button-mask enumeration.
35|Begin the C structure describing decoded packet fields, not the on-air bytes.
36|Store a 16-bit packet sequence number, wrapping from 65535 back to 0.
37|Store the signed roll-stick command in scaled integer units.
38|Store the signed pitch-stick command in scaled integer units.
39|Store the signed yaw-rate-stick command in scaled integer units.
40|Store the unsigned centered throttle command; its meaning is vertical speed, not direct motor power.
41|Store the left trigger value.
42|Store the right trigger value; later it supplies the boost command.
43|Store the six button states as a bit mask.
44|Name this structure type qcopter_command_frame_t.
46|Define the internal CRC function. data points to the bytes; length says how many bytes to check. static keeps the function private to this translation unit; inline permits inlining but does not require it.
47|Initialize the 16-bit CRC accumulator to hexadecimal FFFF.
48|Visit every input byte, from index 0 through length-1.
49|Move the next byte into the CRC's upper eight bits and XOR it into the accumulator.
50|Process the eight bits contributed by that byte.
51-52|If the outgoing top bit is 1, shift and XOR polynomial 0x1021; otherwise only shift. Casting back to uint16_t keeps the low 16 bits. This is a bitwise integrity checksum, not encryption or authentication.
53|Close the per-byte loop.
54|Return the final checksum.
55|End qcopter_crc16.
57|Define a helper that writes a 16-bit unsigned integer to a two-byte destination.
58|Write its low eight bits first.
59|Write its high eight bits second: the wire format is little-endian.
60|End the write helper.
62|Define the inverse helper that reads two little-endian bytes.
63|Combine byte 0 with byte 1 shifted eight positions left; return the unsigned 16-bit value.
64|End the read helper.
66|Define semantic validation for a decoded frame, separate from CRC/header validation.
67|Require a non-null frame before reading any field. The following && operations short-circuit.
68|Require roll in the signed integer range -10000..10000.
69|Require pitch in the same range.
70|Require yaw-rate command in the same range.
71|Require throttle no greater than 10000; its unsigned type already excludes negative values.
72|Require both triggers no greater than 1000.
73|Reject any button bits outside the supported low six bits. ~ inverts the mask; & isolates unsupported bits.
74|End frame-value validation.
76-77|Declare the packet encoder: read a const decoded frame and write a 22-byte output buffer. The array parameter does not independently prove the caller actually allocated 22 bytes.
78|Return false if the output pointer is null or the values are invalid.
79|Write the first header marker, 0x51, the ASCII letter Q.
80|Write the second marker, 0x43, the ASCII letter C.
81|Write protocol version 1.
82|Write a reserved byte as zero; decoding requires it to stay zero.
83|Encode the sequence into bytes 4 and 5.
84|Encode roll into bytes 6 and 7. Converting a negative int16_t to uint16_t produces its modulo-65536 unsigned representation.
85|Encode pitch into bytes 8 and 9.
86|Encode yaw rate into bytes 10 and 11.
87|Encode throttle into bytes 12 and 13.
88|Encode LT into bytes 14 and 15.
89|Encode RT into bytes 16 and 17.
90|Encode the button mask into bytes 18 and 19.
91|Compute the checksum over bytes 0..19 and place its two bytes at offsets 20 and 21.
92|Report successful encoding.
93|End the encoder.
95-96|Declare the decoder: inspect a byte buffer and length, and write a decoded structure only after successful validation.
97|Reject null pointers or any length other than 22 before reading packet fields.
98|Reject a packet whose two header markers are not Q and C.
99|Reject an unsupported version or nonzero reserved byte.
100|Reject the packet if its supplied CRC does not equal the CRC recomputed over the first 20 bytes.
101|Return false on any preceding failed packet check.
102|Create a temporary decoded structure, so a rejected packet does not partially overwrite the caller's result.
103|Decode the sequence number.
104|Decode roll, converting its unsigned wire representation back to int16_t. This relies on the target compiler's signed conversion behavior for values above INT16_MAX; it works on the project's usual two's-complement targets but is not a portable-C serialization proof.
105|Decode signed pitch using the same conversion.
106|Decode signed yaw-rate command using the same conversion.
107|Decode unsigned throttle.
108|Decode LT.
109|Decode RT.
110|Decode the button mask.
111|Finish the temporary decoded structure initializer.
112|Reject semantically invalid commands even if header and CRC passed.
113|Copy the fully validated temporary structure to the caller's output object.
114|Report successful decoding.
115|End the decoder.
117|Document the half-sequence-space assumption needed to compare wrapping sequence numbers.
118|Declare the sequence comparator.
119|Treat a positive signed 16-bit difference as newer. For example 0 follows 65535 under the target's signed conversion behavior. Exactly half a cycle and longer ambiguities are not resolved; a sender restart is not explicitly negotiated.
120|End the comparator.
121|Mark the end of the intentionally duplicated wire protocol.
124|Compile Pico-specific includes only when building firmware, not a host simulation/test library.
125|Include Pico timing, sleep and stdio APIs.
126|Include the API used to launch core 1.
127|Include Pico's inter-core queue API.
128|End that conditional include block.
130|Introduce the pack-monitoring types, kept in the same C file as requested.
131|Include stdbool once more; again redundant.
133|Begin the enumeration of requested battery actions.
134|Normal battery operation has numeric value zero.
135|LAND is a request for controlled landing, not an immediate motor cut.
136|LANDED_DISARM requests disarming only after the supplied landed flag is true.
137|CONFIGURATION_FAULT reports invalid monitor settings.
138|SENSOR_FAULT reports invalid measurements or sampling time.
139|Name the enumeration battery_action_t.
141|Begin the structure supplied to a battery monitor update.
142|Pack voltage in volts, from a future calibrated voltage measurement path.
143|Pack current in amperes; positive current means discharge.
144|Optional minimum individual-cell voltage. No cell monitor is implemented here.
145|Optional SOC observation from a calibrated rested open-circuit voltage curve, expressed in percentage points.
146|Flag that pack-voltage data is valid.
147|Flag that current data is valid; the current implementation requires this for SOC updates.
148|Flag that the optional individual-cell voltage is available.
149|Flag that a rested-pack SOC observation is available.
150|Optional external BMS warning input; declaring this flag does not provide a hardware BMS.
151|Caller-supplied landed status; the battery function does not detect landing itself.
152|Name the battery sample type.
154|Begin the battery configuration structure.
155|Capacity in ampere-hours; the intended 4200 mAh pack corresponds to 4.2 Ah.
156|Initial SOC percentage, which must be supplied from a credible starting condition.
157|SOC threshold at which the monitor requests landing. The function has no hard-coded 20% default; the caller must configure it.
158|Optional low individual-cell voltage threshold.
159|Optional critical individual-cell voltage threshold, no greater than the low-cell threshold.
160|Low total-pack voltage threshold.
161|Maximum permitted measured discharge current. A battery's printed C rating is not used here.
162|Pack-voltage measurement variance R in V squared.
163|Pack-voltage process variance Q, added once per accepted monitor update, not multiplied by dt.
164|Initial SOC uncertainty variance in percentage-points squared.
165|SOC process-uncertainty growth per second.
166|Variance R for the optional rested SOC measurement.
167|Largest accepted interval between battery updates.
168|Name the battery configuration type.
170|Begin the battery status/estimator state structure.
171|Estimated filtered total-pack voltage.
172|Standard deviation sigma of that voltage estimate, in volts; its square is the covariance.
173|Estimated SOC percentage maintained by coulomb counting and optional correction.
174|SOC standard deviation, in percentage points.
175|Current requested battery action.
176|Whether the voltage estimate has been initialized from a measurement.
177|Whether an initial SOC estimate has been supplied.
178|Whether SOC uncertainty has been initialized.
179|Whether the last attempted update detected a sensor fault.
180|Name the status type.
182|Declare initialization before its later definition; parameters are a writable status pointer and starting SOC.
183-186|Declare the monitor-update function. It modifies status, reads sample/config through const pointers, takes elapsed seconds, and returns the selected action.
187|Clarify that this is monitoring/warning logic with no connected hardware power switch.
190|Repeat the math include; no behavioral change.
191|Repeat the size_t/NULL include; no behavioral change.
193|Declare a small helper for checking physically positive numeric configuration values.
194|Return true only for finite numbers strictly greater than zero; NaN, infinity, zero and negative values fail.
195|End positive_finite.
197|Declare validation of a battery configuration.
198|Require a configuration pointer and positive finite capacity.
199|Require finite initial SOC.
200|Bound initial SOC to 0..100 percent.
201|Require finite low-SOC threshold.
202|Bound the low-SOC threshold to 0..100 percent.
203|Require positive finite critical-cell threshold.
204|Require the low-cell threshold to be at least the critical-cell threshold.
205|Require positive finite low-cell threshold.
206|Require positive finite total-pack low-voltage threshold.
207|Require positive finite current limit.
208|Require positive finite voltage measurement variance, avoiding a zero observation-noise denominator.
209|Allow voltage process variance to be zero, but not negative.
210|Also reject NaN/infinite voltage process variance.
211|Require positive finite initial SOC variance.
212|Allow nonnegative SOC process-variance growth.
213|Require that SOC process-variance growth be finite.
214|Require positive finite SOC observation variance.
215|Require a positive finite maximum update interval, finishing the long Boolean expression.
216|End configuration validation.
218|Define battery initialization.
219|If the status pointer is null, return without dereferencing it.
220|Overwrite every status member with its zero value using a compound literal; enum action zero means NORMAL.
221-222|Accept the requested initial SOC only if finite and within 0..100.
223|Store the accepted initial SOC.
224|Mark SOC initialized, but leave covariance initialization for the first valid update.
225|Close the optional valid-SOC initialization block.
226|End initialization.
228-231|Define one battery update. dt_s is time elapsed since the previous sample, in seconds.
232|A null status pointer returns a configuration-fault enum because no status object can be written.
233|Clear the last sensor-fault flag before assessing the new update.
234|Branch if configuration validation fails.
235|Store the configuration-fault action.
236|Return immediately without applying any measurements.
237|Close the configuration-fault branch.
238|Start rejection of null samples or nonfinite/nonpositive elapsed time.
239|Also reject elapsed time beyond the configured maximum.
240|Require both pack-voltage and current validity flags. A voltage divider alone does not satisfy this SOC monitor's interface.
241|Require positive finite pack voltage.
242|Require finite current; negative current is allowed mathematically and represents charging.
243-245|If individual-cell data is flagged valid, require positive finite minimum-cell voltage. If not flagged valid, this optional value is ignored.
246-249|If rested SOC is flagged valid, require a finite percentage in 0..100. The function trusts the caller that the pack really was rested and calibrated.
250|Latch a sensor fault for this failed update.
251|Select SENSOR_FAULT.
252|Return before modifying the voltage/SOC estimate.
253|Close sample validation.
255|Introduce the scalar pack-voltage Kalman filter. It estimates voltage, not capacity by itself.
256|Branch on the first valid voltage measurement.
257|Initialize estimated voltage directly to the sample.
258-259|Initialize its standard deviation to sqrt(R), where R is measurement variance.
260|Mark voltage filter initialized.
261|For subsequent samples, perform predict/correct instead of reinitializing.
262|Recover covariance P by squaring stored sigma.
263|Predict uncertainty: P_minus = P + Q. Here Q is per accepted update.
264|Compute scalar gain K = P_minus/(P_minus + R).
265-266|Correct the voltage estimate: V_hat += K*(V_measured - V_hat).
267|Update scalar covariance P_plus = (1-K)*P_minus.
268|Store sigma = sqrt(max(P_plus,0)); max protects against a small negative roundoff result.
269|End the voltage-filter branch.
271|Introduce SOC prediction: current integration supplies the process model.
272|If SOC was not initialized earlier, use configuration's initial value.
273|Copy initial SOC from configuration.
274|Mark its state initialized.
275|Close SOC state initialization.
276|Initialize SOC uncertainty only once.
277-278|Set initial SOC sigma to the square root of the configured initial variance.
279|Mark SOC covariance initialized.
280|Close SOC covariance initialization.
281-282|Subtract discharged capacity: delta_SOC = 100*I*dt/(capacity_Ah*3600). I*dt is ampere-seconds; 3600 converts Ah to ampere-seconds. At 10 A, 1 s, 4.2 Ah, the decrease is approximately 0.0661 percentage points.
283-285|Predict SOC variance by squaring current sigma and adding configured variance growth times elapsed seconds. This is a simplified uncertainty model, not a complete identified current-bias/capacity model.
286-287|Explain why voltage under motor load is not directly converted to SOC: load sag is not the same thing as rested open-circuit voltage.
288|Apply an SOC measurement correction only when the optional rested observation is declared valid.
289-290|Calculate SOC gain K = P_minus/(P_minus + R_soc).
291-293|Move the predicted SOC toward the independent rested SOC observation by K times their difference.
294|Reduce its covariance by multiplying by 1-K.
295|Close optional SOC correction.
296-297|Clamp the reported estimate into the physically meaningful interval 0..100 percent.
298|Convert the nonnegative updated variance back to sigma.
300|Document warning priority: optional BMS/cell-critical conditions are evaluated before lower-priority warnings.
301-303|If an external BMS alert is active, or available minimum-cell voltage is critical, select the critical action branch.
304-305|If already landed, request LANDED_DISARM; otherwise request LAND. The ternary condition chooses one enum; it does not stop motors here.
306|Otherwise check low SOC against the configured threshold.
307|Also check low filtered total-pack voltage.
308-309|Also check optional low-cell voltage.
310|Also check measured current above the configured discharge limit.
311-312|For any of these warnings, request landing in air or disarming when already landed.
313|If none of the thresholds fired, take the normal branch.
314|Set the normal action.
315|Close action selection. There is no latching or hysteresis here; actions may clear on a later normal sample.
316|Return the current action enum.
317|End the battery monitor. It is not called from the firmware main loop below; monitoring math is present, but integration is unfinished.
319|Set the intended control-loop frequency to 500 Hz.
320|Set the corresponding scheduling period to 2000 microseconds.
321|Define nominal dt as 1/500 = 0.002 seconds; this macro is not the actual measured dt used in main.
322|Reject control intervals over 10 ms. At 500 Hz, that is five nominal periods.
323|Approximate 180/pi for converting radians to degrees.
324|Approximate pi/180 for converting degrees to radians.
325|Assume normalized hover collective 0.50 until a caller supplies a better value. This is not measured thrust or necessarily physical ESC duty.
326|Set full-stick baseline vertical-speed magnitude to 2 m/s; RT can extend it to 3 m/s.
327|Commands become stale after 150000 microseconds = 150 ms.
328|Declare a sensor-rate target of 1000 Hz. This constant does not create a sensor task and is not used to acquire real IMU samples.
330-335|Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence.
336|Begin online three-axis calibration statistics.
337|Number of samples accumulated so far.
338|Per-axis double-precision mean and centered sum of squared deviations M2. M2 is not yet divided by sample count.
339|Name the statistics type.
341|Begin the stored IMU calibration parameters.
342|Accelerometer bias offsets in g and multiplicative axis gains.
343|Uncertainty of accelerometer bias and gain calibration parameters.
344|Gyroscope zero-rate offsets in degrees/s and sample noise variances in (degrees/s) squared.
345|Uncertainty variance of the estimated gyro mean bias, distinct from individual-sample noise.
346|Number of samples used for gyro calibration.
347|Validity flags for accelerometer and gyro calibration fits.
348|Name the IMU calibration type.
350|Group stationary accelerometer and gyro statistics so capture stability can be checked jointly.
352|Declare three-component finite-value validation.
353|Require an array pointer and finite x, y and z components.
354|End the validation helper.
356|Declare Welford's online statistics update for a three-axis vector. The caller must already have validated the pointers and overflow.
357|Increment the sample count.
358|Repeat the statistics update independently for axes 0, 1 and 2.
359|Calculate delta = new sample - old mean.
360|Update mean: mean_new = mean_old + delta/n.
361|Update M2 with delta*(sample - mean_new), using the new mean. This avoids subtracting two very large raw sums.
362|End the axis loop.
363|End the online update.
365|Declare conversion from accumulated M2 to sample variance.
366|Use unbiased sample variance M2/(n-1) if at least two samples exist; otherwise report infinity so later validation rejects insufficient data.
367|End variance calculation.
369-370|Accept one stationary IMU calibration pair. disarmed must be true; this API expects samples already converted from raw register counts to g and degrees/s.
371-373|Reject null statistics, an armed capture, nonfinite input vectors or counters at UINT32_MAX.
374-375|Calculate squared accelerometer magnitude ax²+ay²+az², in g squared.
376|Require squared magnitude between 0.64 and 1.44: magnitude 0.8..1.2 g, an initial loose stationary plausibility check.
377|Inspect all gyro axes.
378|Reject any instantaneous rotation over 10 degrees/s in magnitude.
379|Add accelerometer data to stationary statistics.
380|Add gyro data to stationary statistics.
381|Report acceptance of this sample pair; this alone does not complete calibration.
382|End stationary capture.
384-385|Finalize the gyro bias/noise fit from a stationary capture, writing output only after aggregate checks.
386-387|Require pointers, disarmed capture, at least 500 samples and equal accelerometer/gyro sample counts.
388|Start the squared norm of mean acceleration at zero.
389|Validate each axis's aggregate measurements.
390|Accumulate squared mean acceleration components.
391|Reject a nonfinite mean gyro reading or stationary mean over 5 degrees/s.
392-393|Require finite accelerometer and gyro sample variances.
394-395|Require both variances nonnegative.
396|Require accelerometer sample variance no larger than 0.0004 g², corresponding to sigma <=0.02 g per axis.
397|Require gyro variance no larger than 0.01 (degrees/s)², corresponding to sigma <=0.1 degrees/s per axis.
398|Finish per-axis stability checks.
399|Require finite mean acceleration magnitude 0.9..1.1 g; the code compares its square to 0.81..1.21.
400|Now write each accepted gyro calibration parameter.
401|Estimate zero-rate bias as that axis's sample mean.
402|Store individual-sample gyro variance.
403|Estimate variance of the mean bias as sample variance/n, assuming effectively independent stationary samples.
404|Finish writing all axes.
405|Record sample count for provenance.
406|Mark gyro calibration valid.
407|Report success.
408|End gyro finalization.
410|Document six accelerometer poses: +X,-X,+Y,-Y,+Z,-Z in sensor axes, referring to specific force rather than world position.
411-412|Accept one accelerometer sample into one selected face's statistics while disarmed.
413-415|Require a valid faces array, face index below six, disarmed capture, finite sample and no count overflow.
416|Accumulate this sample in that face's online statistics.
417|Report successful sample accumulation.
418|End face capture. Orientation/stability checks happen on finalization; this function trusts the supplied face label initially.
420-421|Finalize the six-face accelerometer calibration.
422|Require valid input/output pointers and disarmed state.
423|Prepare temporary bias and gain arrays before committing any fit.
424|Inspect each of the six faces.
425|Require at least 100 samples per face.
426|Inspect all three axes for that face.
427|Require finite per-axis mean.
428-430|Require finite, nonnegative sample variance <=0.0004 g².
431-432|Axes other than the intended face axis must have mean magnitude <=0.25 g, rejecting strongly tilted or mislabeled faces. Integer face/2 selects the intended axis.
433|End per-axis checks for this face.
434|End all-face validation.
435|Fit one bias/gain pair per axis.
436|Read the positive-face mean for this axis.
437|Read the negative-face mean for this axis.
438|Calculate measured span = positive mean - negative mean; ideal span is 2 g.
439-440|Require a sufficiently positive positive-face reading, negative negative-face reading and a span in 1.5..2.5 g.
441|Bias is the midpoint (positive+negative)/2.
442|Gain is ideal span/measured span = 2/span, so corrected positive and negative readings approach +1 and -1 g.
443|Reject absolute bias exceeding 0.25 g.
444|End preliminary fit for all axes.
445|Commit the accepted calibration and calculate parameter uncertainties per axis.
446|Store the estimated bias.
447|Store the estimated scale gain.
448|Take pointers to this axis's positive and negative statistics.
449-450|Sum variances of the two face means: sampleVariance_pos/n_pos + sampleVariance_neg/n_neg.
451|Recalculate the positive-minus-negative mean span.
452|Bias variance is one quarter of that sum, from differentiating (pos+neg)/2.
453|Gain variance is 4*meanVariance/span^4, from differentiating 2/(pos-neg), assuming independent face means.
454|Finish writing axis fits.
455|Mark accelerometer calibration valid.
456|Report success.
457|End six-face finalization. This is a diagonal gain/offset model, not a full cross-axis alignment fit.
459-461|Apply stored IMU calibration to input acceleration and gyro vectors, returning corrected vectors through output pointers.
462-465|Require a calibration object with both validity flags, finite input vectors and non-null output arrays.
466|Use temporary corrected arrays so a failure does not partially overwrite caller outputs.
467|Correct each axis independently.
468-470|Require a finite positive accelerometer gain and finite accelerometer/gyro biases.
471|Correct acceleration as (raw_g - bias_g)*gain.
472|Correct gyro rate as raw_dps - stationary_bias_dps.
473|Reject nonfinite corrected results.
474|End calculation loop.
475|After all calculations succeeded, copy the temporary outputs to the caller.
476|Write corrected acceleration for this axis.
477|Write corrected gyro rate for this axis.
478|End copy loop.
479|Report success.
480|End IMU correction.
482|Begin state accumulated during a magnetometer sweep.
483|Hold online three-axis statistics.
484|Track minimum and maximum magnetic readings per axis, in microtesla.
485|Track centered xy,xz,yz cross-products to reject a sweep confined approximately to a plane.
486|Name sweep-statistics type.
488|Define a diagonal magnetic calibration: per-axis hard-iron offset, scale gain and a validity flag.
490|Accept one magnetic field vector during a disarmed sweep.
491-492|Require a statistics pointer, disarmed state, finite field vector and no sample-count overflow.
493|Prepare double-precision differences from the old mean.
494|Inspect each axis.
495|Reject individual components exceeding 1000 microtesla in magnitude.
496|Save sample-minus-old-mean for covariance accumulation.
497|End difference collection.
498|Update minimum and maximum values for each axis.
499-500|Initialize the minimum on the first sample, or replace it if this reading is smaller.
501-502|Initialize the maximum on the first sample, or replace it if this reading is larger.
503|End extrema collection.
504|Update means and diagonal centered sums with the shared Welford helper.
505|Update centered xy cross-product using old x difference and new y mean.
506|Update xz cross-product.
507|Update yz cross-product.
508|Report accepted sweep sample.
509|End magnetic sample collection.
511|Explicitly state model limits: min/max hard-iron plus diagonal scaling, not a full soft-iron ellipsoid/matrix fit.
512-513|Finalize magnetic calibration, requiring both caller-confirmed full rotation and disarmed state.
514-515|Reject missing pointers, armed/incomplete rotation captures or fewer than 300 samples.
516|Prepare three measured half-ranges and their average.
517|Evaluate each magnetic axis.
518|Radius is half of maximum-minus-minimum span.
519-521|Require a finite radius 5..100 microtesla and a finite positive centered sum of squares.
522|Accumulate one third of each radius to obtain the average radius.
523|Finish per-axis radius validation.
524|Compute normalized xy correlation from the cross-product and diagonal sums.
525|Compute xz correlation.
526|Compute yz correlation.
527|Compute determinant of the 3x3 correlation matrix: 1+2*rxy*rxz*ryz-rxy²-rxz²-ryz². Near-zero determinant indicates inadequate three-dimensional variation.
528|Require finite determinant at least 0.05. This heuristic is not a complete proof of uniform sphere coverage or magnetic cleanliness.
529-531|Require every proposed diagonal gain mean_radius/radius_i within 0.5..2.0.
532|Commit the three accepted axis fits.
533|Offset is midpoint of the observed min and max on this axis.
534|Scale the axis radius toward the mean radius.
535|Finish axis fit storage.
536|Mark magnetic calibration valid.
537|Report success.
538|End magnetic finalization. These calibration parameters are not used by the current attitude/control step.
540-541|Apply a previously accepted magnetic offset/scale calibration.
542-543|Require valid calibration, finite input vector and output storage.
544|Prepare temporary corrected vector.
545|Correct each axis.
546-547|Require finite offset and finite positive gain.
548|Compute corrected magnetic field (raw-offset)*gain.
549|Reject nonfinite results.
550|Finish calculation loop.
551|Copy corrected vector only after all axes succeeded.
552|Report success.
553|End magnetic correction. No heading, tilt compensation, declination or yaw fusion is performed here.
555-557|Introduce BMP280 factory-trim compensation. This is separate from learning local ground pressure. No I2C register reading is implemented by these arithmetic functions.
558|Begin storage for the BMP280 factory coefficients.
559|T1 and P1 are unsigned 16-bit coefficients.
560|T2,T3 and P2..P9 are signed 16-bit coefficients; signedness matters for decoding.
561|Name the trim structure.
563|Decode the 24 factory-calibration bytes into trim coefficients.
564|Require pointers and exactly 24 input bytes.
565|Build a temporary trim object.
566|Read T1 at offsets 0..1 and signed T2 at 2..3.
567|Read signed T3 at 4..5 and unsigned P1 at 6..7.
568|Read signed P2 at 8..9 and P3 at 10..11.
569|Read signed P4 at 12..13 and P5 at 14..15.
570|Read signed P6 at 16..17 and P7 at 18..19.
571|Read signed P8 at 20..21 and P9 at 22..23.
572|Finish temporary coefficient initialization.
573-574|Reject all-zero/all-one sentinel values for essential coefficients T1 and P1.
575|Commit the validated coefficients.
576|Report successful decoding.
577|End trim decoder.
579-581|Convert raw BMP280 temperature/pressure ADC values to degrees Celsius and pascals through output pointers, using factory trim.
582-585|Require valid pointers, essential nonzero trim values, 20-bit raw values, and reject the 0x80000 skipped-measurement sentinel.
586|Form a scaled raw-temperature difference used in the quadratic temperature correction. This local variable dt is not a time step.
587-588|Compute the temperature fine term: the linear temperature difference times T2 plus the squared difference term times T3.
589|Convert fine temperature to degrees Celsius by dividing by 5120.
590|Initialize the first pressure intermediate from temperature fine compensation.
591|Calculate the quadratic/linear temperature contribution using P6 and P5.
592|Scale that contribution and add P4's offset term.
593|Calculate a second temperature-dependent pressure-scale intermediate using P3 and P2.
594|Finish the pressure denominator using P1.
595|Reject a nonfinite or nearly zero denominator before dividing.
596|Calculate the first compensated pressure estimate from raw pressure and the temperature correction terms.
597|Calculate pressure-squared correction using P9.
598|Calculate linear pressure correction using P8.
599|Add those terms and P7's offset to pressure.
600-601|Reject nonfinite output, temperatures outside -40..85 Celsius and pressures outside 30000..110000 Pa. These are broad plausibility bounds, not a guarantee of a good sensor measurement.
602|Write compensated temperature.
603|Write compensated pressure.
604|Report success.
605|End compensation. Calling this does not automatically feed the altitude filter; integration must supply its result.
607|Define ground-reference running statistics: count, mean pressure and M2 in pressure-squared units.
608|Begin stored ground-reference calibration.
609|Store ground mean pressure, variance of that mean reference, and individual pressure-measurement variance.
610|Flag whether a valid ground reference exists.
611|Name the ground-reference calibration type.
613-614|Add one compensated ground-pressure sample while disarmed.
615-617|Require pointer, disarmed status, finite pressure in broad allowed range and no count overflow.
618|Calculate pressure-minus-old-mean.
619|Increment count.
620|Update pressure mean by delta/n.
621|Update centered squared pressure deviations using the new mean.
622|Report sample acceptance.
623|End ground-reference capture.
625-626|Finalize a pressure reference, with a caller-specified maximum allowed pressure standard deviation.
627-630|Require pointers, disarmed status, at least 100 samples, positive finite sigma limit and a plausible finite mean pressure.
631|Estimate individual-sample pressure variance with M2/(n-1).
632-633|Reject invalid/negative variance or variance above maximum_sigma_pa squared.
634|Store mean ground pressure, reference-mean variance variance/n, individual-measurement variance and valid=true.
635|Report success.
636|End ground-reference fitting.
638-639|Convert compensated pressure into height relative to calibrated ground pressure and propagate pressure uncertainty to a height sigma.
640-646|Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances.
647|Set exponent for the pressure-ratio standard-atmosphere approximation.
648|Compute pressure ratio p/p0, where p0 is the local calibrated ground reference.
649|Calculate a shared magnitude factor 44330*a*(p/p0)^(a-1) used in the height derivatives.
650|Derivative dh/dp = -factor/p0: higher pressure means lower inferred altitude.
651-652|Derivative dh/dp0 = factor*p/p0², accounting for uncertain ground reference.
653|Calculate relative height h = 44330*(1-(p/p0)^a). It assumes an atmospheric model; it is not a measured weather correction.
654-655|Propagate independent pressure/reference variances: sigma_h = sqrt((dh/dp)²*Var(p)+(dh/dp0)²*Var(p0)). Correlation and atmospheric-model uncertainty are not included.
656|Return true only for finite output height and sigma.
657|End relative-height conversion.
659-662|Introduce a reusable linear reference calibration y=gain*x+offset for voltage/current/lidar. Reference data must come from independently known physical values.
663|Store online paired statistics: n, means x/y, centered sums xx/yy and cross-product xy.
664|Store fitted gain, offset, residual standard deviation and validity.
666-667|Accumulate one calibration input/reference pair while disarmed.
668-670|Require pointer, disarmed state, finite values bounded in magnitude by 1e9 and no count overflow.
671|Compute input and reference differences from their old means.
672|Increment count.
673|Update input mean.
674|Update reference mean.
675|Update centered input sum of squares.
676|Update centered reference sum of squares.
677|Update centered input/reference cross-product using the old input difference and new reference mean.
678|Report successful accumulation.
679|End paired capture.
681-682|Fit a straight line and reject excessive residual scatter, taking the maximum acceptable sigma from the caller.
683-687|Require valid pointers/disarmed state, at least three pairs, finite statistics, nonnegative reference variation, sufficient input variation and a positive finite residual limit.
688|Least-squares gain is Sxy/Sxx; offset is mean_y - gain*mean_x.
689|Estimate residual sigma = sqrt(max(0,Syy-gain*Sxy)/(n-2)). Two fitted parameters use two degrees of freedom.
690-692|Reject a near-zero/nonfinite gain, nonfinite offset or residual, or residual sigma above the caller's bound. Negative gains are permitted for inverted electrical relationships.
693|Commit the accepted gain, offset, residual and validity.
694|Report success.
695|End line fitting.
697|Apply a fitted linear sensor conversion.
698-699|Require valid calibration/input/output and finite gain/offset.
700|Calculate corrected = gain*raw + offset.
701|Reject nonfinite converted output.
702|Write the result only after successful conversion.
703|Report success.
704|End the generic reference conversion.
706|Begin generic PID state/configuration type.
707|Proportional gain Kp.
708|Integral gain Ki, multiplying accumulated error-times-seconds.
709|Derivative gain Kd; the generic PID differentiates measurement, not setpoint.
710|Stored integral of error, before multiplying by Ki.
711|Previous measured value, needed to estimate its time derivative.
712|Magnitude bound on stored error integral, not directly on the I-term output.
713|Symmetric magnitude bound on the complete PID output.
714|Whether a previous measurement exists, suppressing the first derivative spike.
715|Name the PID type. Six instances follow; some use zero I/D gains.
717|Begin the per-axis angle/bias Kalman state.
718|Estimated roll or pitch angle in degrees.
719|Estimated residual gyro bias in degrees/s. This is additional to optional fixed calibration bias removal.
720|P00: variance of estimated angle, in degrees squared.
721|P01: covariance between angle and gyro bias.
722|P10: transpose covariance; ideally equals P01.
723|P11: gyro-bias variance in (degrees/s) squared.
724|Per-sample gyro-rate measurement variance; its contribution to angle variance is multiplied by dt squared.
725|Continuous bias random-walk intensity, in (degrees/s) squared per second.
726|Accelerometer-derived angle observation variance R in degrees squared; despite the name it is not acceleration variance in g squared.
727|Last angle/bias gains, innovation and innovation variance, retained for diagnostics/graphs.
728|Name the angle filter type. Although one axis is estimated, it contains TWO states: angle and bias.
730|Begin the altitude/vertical-velocity Kalman state.
731|Estimated upward-positive height in metres.
732|Estimated upward-positive vertical velocity in metres/s.
733|P00: height variance in m squared.
734|P01: height/vertical-speed covariance.
735|P10: transpose covariance.
736|P11: vertical-speed variance in (m/s) squared.
737|Acceleration noise standard deviation in m/s squared; prediction squares it to obtain variance. This differs from angle_kf_t's gyro_noise convention.
738|Height-measurement variance R in m squared, despite the name baro_noise.
739|Last height/velocity gains, height innovation and innovation variance for diagnostics.
740|Name the two-state altitude filter type.
742|Begin the internal pilot/companion command type after packet scaling.
743|Requested roll angle in degrees.
744|Requested pitch angle in degrees.
745|Requested yaw angular rate in degrees/s; no absolute-yaw heading target exists.
746|Centered throttle in 0..1; 0.5 requests zero vertical speed.
747|RT boost in 0..1, which expands climb/collective limits without selecting a new flight mode.
748|Whether the command is valid/fresh according to its source.
749|Reserved arm-request field. Current math/runtime never uses it to arm hardware.
750|Name the internal command type.
752|Begin one sensor sample as consumed by the control core.
753|Body/sensor-frame x acceleration in g, after the agreed frame mapping and calibration.
754|y acceleration in g.
755|z acceleration in g.
756|x-axis angular rate in degrees/s.
757|y-axis angular rate in degrees/s.
758|z-axis angular rate in degrees/s; used as the yaw-rate measurement.
759|Separately supplied upward world acceleration in m/s squared AFTER rotation and gravity compensation; the core does not calculate this rotation itself.
760|Relative barometer height in metres, already pressure-compensated and referenced by its producer.
761|Flag that IMU data is valid; there is no sample timestamp in this structure.
762|Flag that IMU axis scale/bias calibration has been applied.
763|Flag that height aiding is valid.
764|Flag that this is a new independent pressure sample. It prevents correcting the filter repeatedly with one stale repeated measurement.
765|Name the sensor sample type.
767-768|Apply IMU calibration in-place to one sample before control.
769|Require a sample with valid IMU, and reject already-calibrated samples to avoid correcting the same data twice.
770|Copy acceleration components into an input vector.
771|Copy angular rates into another input vector.
772|Prepare temporary corrected outputs.
773|Use the earlier imu_cal_apply helper; return false if calibration/inputs are invalid.
774|Write corrected accelerometer components back to the sample.
775|Write corrected gyro components back to the sample.
776|Mark this sample calibrated.
777|Document that world-up acceleration must be derived after correction AND frame rotation. This function does not perform that extra step.
778|Report success.
779|End in-place calibration.
781|Begin control result structure.
782|Four normalized motor demands, mathematical results only. These are not connected ESC pulses and need not be linear in physical thrust on real hardware.
783|Hardware-arming flag; it is always false in this scaffold.
784|Fault flag, also used for invalid pilot command/height aiding rather than exclusively IMU faults.
785|Name the control result type.
787|Begin persistent pilot-link tracking state.
788|Store the most recently accepted physical-unit command.
789|Store its sequence number for stale/duplicate ordering checks.
790|Store local reception time in microseconds.
791|Record whether any frame has ever been accepted.
792|Name the link state.
794-795|Accept one received radio packet and its local timestamp into link state; this function does not itself read the radio.
796|Prepare a temporary decoded frame.
797|Require a link pointer and successfully decoded, semantically valid packet.
798-800|After the first frame, reject duplicates/older sequence numbers using the wrap comparator.
801|Construct the new internal pilot command.
802|Map full signed roll stick to -35..+35 degrees.
803|Map full signed pitch stick to -35..+35 degrees.
804|Map full signed yaw stick to -180..+180 degrees/s.
805|Convert integer throttle to 0..1; midpoint 0.5 means hold current height reference.
806|Map RT to boost 0..1. LT/buttons are validated in the frame but not used here for flight control.
807|Mark the just-decoded command valid.
808|Force arm_request false; no physical arming gesture is enabled.
809|Complete the command initializer.
810|Save the sequence of the accepted frame.
811|Save reception timestamp.
812|Mark that a frame was received.
813|Report acceptance.
814|End frame acceptance.
816-817|Return the current pilot command if still fresh at the supplied local time.
818|Initialize a zero-filled command, including valid=false and throttle=0.
819-820|Require an existing link, at least one frame, non-backward local time and packet age <=150 ms.
821|For a fresh packet, return its saved command.
822|Otherwise take the stale/no-link branch.
823|Explicitly mark the zero command invalid. Its zero throttle is not treated as a valid descent request because valid=false is checked downstream.
824|Return the chosen command.
825|End link freshness handling. The current physical scaffold has no airborne recovery/landing strategy for a stale link.
827|Create one persistent roll angle/bias estimator, private to this file.
828|Initialize angle variance to 1 degree squared and bias variance to 0.01 (degrees/s) squared; unspecified members begin at zero.
829|Set gyro-rate variance 0.04, bias random-walk intensity 0.0001 and accelerometer-angle variance 4 degrees squared. These are provisional noise assumptions.
830|End roll estimator initializer.
831|Create a separate persistent pitch angle/bias estimator.
832|Give pitch the same initial diagonal covariance.
833|Give pitch the same provisional noise parameters.
834|End pitch initialization.
835|Create the persistent height/velocity estimator.
836|Initialize height and velocity variances to 1 in their respective squared units.
837|Set acceleration sigma to 1.5 m/s² and height observation variance to 2 m². Host simulation reset later uses a different barometer variance; do not confuse these defaults.
838|End altitude-estimator initializer.
840|Label the PID gains as simulation/bench candidates rather than identified real-aircraft gains.
841|Create the outer roll-angle controller.
842|Set Kp=3, Ki=Kd=0: this instance is presently a P controller despite using pid_t.
843|Disable its integrator and cap its angular-rate output at +/-180 degrees/s.
844|End outer-roll initialization.
845|Create the outer pitch-angle controller.
846|Use the same P-only gain 3.
847|Disable integral accumulation and limit pitch-rate command to +/-180 degrees/s.
848|End outer-pitch initialization.
849|Create the inner roll-rate controller.
850|Set provisional Kp=0.000829338, Ki=0.003870245 and Kd=0: currently PI.
851|Bound its error integral to +/-15 and normalized roll correction to +/-0.35.
852|End inner-roll initialization.
853|Create the inner pitch-rate controller.
854|Use the same provisional PI gains as roll rate.
855|Use the same integral and output limits.
856|End inner-pitch initialization.
857|Create the yaw-rate controller, without an outer yaw-angle controller.
858|Set provisional yaw PI gains Kp=0.005375614, Ki=0.008362066, Kd=0.
859|Limit yaw error integral to +/-8 and normalized yaw correction to +/-0.35.
860|End yaw initialization.
861|Create the sole altitude PID. Vertical speed feedback is its D/error-derivative term, not a seventh PID instance.
862|Set provisional height gains Kp=0.108045, Ki=0.03087, Kd=0.1225.
863|Bound height-error integral to +/-2 metre-seconds and normalized collective correction to +/-0.35.
864|End altitude-PID initialization.
865|Persistent target height in metres; initially zero by static initialization, then captured from the estimator when control first becomes valid.
866|Persistent hover feed-forward, initially the unmeasured 0.50 normalized value.
867|Persistent smoothed boost value, initially zero.
868|Flag that height reference has been initialized.
870|Declare a floating-point clamp helper.
871|Return the lower bound if value is too small.
872|Return the upper bound if value is too large.
873|Otherwise return value unchanged. This function alone does not reject NaN; callers must validate their data.
874|End clamp helper.
876|Reset a PID's memory without changing gains or limits.
877|Clear the accumulated error integral.
878|Clear the previous measurement.
879|Mark derivative history uninitialized.
880|End PID reset.
882|Reset all control-memory state, but not the three Kalman estimators or hover feed-forward.
883|Immediately reset the boost ramp to zero.
884|Reset the outer roll PID memory.
885|Reset the outer pitch PID memory.
886|Reset inner roll-rate memory.
887|Reset inner pitch-rate memory.
888|Reset yaw-rate memory.
889|Reset altitude-PID memory.
890|Require a fresh capture of the height reference when valid control resumes.
891|End control reset.
893|Explain derivative-on-measurement: changing a target does not directly create a derivative spike.
894|Define a generic PID update, taking state/config, target, measurement and elapsed seconds.
895|Reject dt that is not positive (including NaN in this comparison) or exceeds 10 ms. The outer control step also checks finiteness explicitly.
896|On invalid timing reset this PID's memory.
897|Return zero correction on invalid timing.
898|End invalid-time branch.
899|Compute control error e = setpoint - measurement.
900-901|If derivative history exists, compute measurement slope (y_k-y_previous)/dt; otherwise use zero for the first sample.
902-904|Form a bounded candidate integral I_candidate = clamp(I_old+e*dt, +/-integrator_limit).
905-906|Calculate raw output u_raw=Kp*e+Ki*I_candidate-Kd*measurement_slope. The derivative has a minus sign because increasing measurement should oppose positive error.
907|Clamp raw correction to +/-output_limit.
908|Introduce conditional-integration anti-windup.
909-910|Accept integral accumulation when unsaturated, or when error drives a saturated command back toward the allowed range. This checks this PID's own limiter, not later collective/mixer saturation.
911|Commit the candidate integral only under that condition.
912|End integral-acceptance branch.
913|Save the current measurement for next derivative calculation.
914|Mark derivative history initialized.
915|Return the bounded correction.
916|End the generic PID. There is no derivative low-pass filter in this implementation.
918|Introduce Joseph-form covariance correction for a two-state filter observed through its first component: H=[1,0].
919-920|Receive writable covariance entries, two Kalman gains and scalar observation variance R. Pointer arguments allow changing the caller's filter in-place.
921|Copy old diagonal entries and average the two off-diagonals, removing small symmetry discrepancies before correction.
922|Set z=1-K0, the first diagonal term of I-KH.
923|Compute P00_plus=z²*P00_old+K0²*R.
924|Compute P01_plus=z*(P01_old-K1*P00_old)+K0*K1*R.
925|Set P10_plus exactly equal to P01_plus to preserve covariance symmetry.
926|Compute P11_plus=P11_old-2*K1*P01_old+K1²*P00_old+K1²*R.
927|End covariance correction. Algebraically this is (I-KH)P(I-KH)^T + K R K^T.
929-930|Define one angle/bias filter update for an individual axis, using gyro rate, accelerometer-derived angle and elapsed seconds.
931|Predict angle by integrating bias-corrected rate: angle_minus=angle_old+dt*(gyro-bias_old). Bias prediction is constant and therefore needs no assignment.
932|Save all old covariance entries before modifying any of them.
933|Read bias random-walk intensity qb.
934|Document noise units: gyro_noise is rate variance per sample; bias_walk is variance growth per second.
935|Predict angle variance: a-dt*(b+c)+dt²*d + gyroVariance*dt² + qb*dt³/3. The negative cross terms come from subtracting gyro bias in angle propagation.
936|Predict angle/bias cross covariance b-dt*d-qb*dt²/2.
937|Predict its transpose using old c; the process covariance cross term has the same negative sign.
938|Predict bias variance d+qb*dt.
939|Calculate innovation variance S=P00_minus+R_angle.
940|Calculate angle gain K_angle=P00_minus/S.
941|Calculate bias gain K_bias=P10_minus/S. This gain is generally not constrained to 0..1; its units differ from the angle gain.
942|Innovation nu=accelerometer_angle-predicted_angle.
943|Correct angle by K_angle*nu.
944|Correct residual gyro bias by K_bias*nu.
945-946|Use Joseph correction with these gains and angle observation variance.
947|End angle update. There is no acceleration-magnitude rejection/gain adaptation here; translational acceleration can corrupt the gravity-angle measurement.
949|Introduce altitude state [height,vertical_speed]. Acceleration is an external input, not a third estimated state.
950-951|Predict height and velocity from supplied upward gravity-compensated acceleration.
952-953|Update height with h_minus=h_old+v_old*dt+0.5*a*dt²; old velocity is used before it is modified.
954|Update vertical velocity with v_minus=v_old+a*dt.
956|Copy old height variance.
957|Copy old h/v covariance.
958|Copy old v/h covariance.
959|Copy old velocity variance.
960|Square acceleration-noise sigma to obtain acceleration variance q.
961-962|Predict height variance P00_plus=P00+dt*(P01+P10)+dt²*P11+0.25*dt^4*q.
963|Predict h/v covariance P01_plus=P01+dt*P11+0.5*dt³*q.
964|Predict transpose covariance with the corresponding old entry.
965|Predict velocity variance P11_plus=P11+dt²*q. This is a per-step acceleration-error model with Q=G*q*G^T, G=[dt²/2,dt]^T.
966|End altitude prediction.
968|Correct height/velocity from one independent scalar height measurement. Despite its name, this helper can also fuse a caller-supplied lidar-derived height.
969|Calculate height innovation variance S=P00_minus+R_height.
970|Height gain K_h=P00_minus/S.
971|Velocity gain K_v=P10_minus/S: a height measurement can also correct velocity through cross covariance.
972|Innovation nu=measured_height-predicted_height.
973|Correct height by K_h*nu.
974|Correct vertical speed by K_v*nu.
975-976|Apply Joseph covariance update using height observation variance.
977|End height correction. Gains vary with covariance; they are not fixed tuning constants like PID gains.
979|Document that altitude uses a single PID whose D term is desired-minus-measured vertical velocity.
980-981|Declare the specialized altitude update: reference height, measured height, desired vertical speed, estimated vertical speed and dt.
982|Calculate height error reference-height.
983-984|Form a bounded candidate integral of height error.
985-986|Compute collective correction Kp*(h_ref-h)+Ki*I+Kd*(v_cmd-v_est). Since h_ref is generated by integrating v_cmd, its derivative is approximately v_cmd when the reference limiter is inactive.
987|Clamp correction to the altitude PID's output magnitude limit.
988-989|Conditionally accept integration only if this PID is not saturating farther in the error direction. Later tilt/collective/mixer saturation is not fed back here.
990|Return normalized collective correction.
991|End altitude PID. This helper relies on its caller to validate dt.
993|Declare the IMU health predicate used at the start of the control step.
994|Require a non-null sample and both valid/calibrated IMU flags.
995-997|Require every accelerometer and gyro component finite. This does not check sensor age, plausible magnitude or zero-vector acceleration.
998|End the health predicate.
1000-1003|Document that the following step is mathematical control only and that motor ordering/signs are provisional until physically validated.
1004-1006|Define one control update from sensor sample, command and measured dt.
1007-1008|Start from four zero motor demands, armed=false, sensor_fault=false. Early returns therefore have zero mathematical demands.
1009-1010|Reject unhealthy IMU or dt that is nonfinite, nonpositive or above 10 ms.
1011|Clear PID/reference/boost state on rejection; Kalman estimates remain as they were.
1012|Set the fault flag.
1013|Return zero demands while remaining disarmed. This scaffold behavior is not a completed airborne failsafe design.
1014|End early validation.
1016-1017|Derive roll observation atan2(ay,az) in radians, then convert to degrees. This interprets measured specific force as gravity-dominated and uses the source's axis/sign convention.
1018-1020|Derive pitch observation atan2(-ax,sqrt(ay²+az²)), then convert to degrees. It has the usual limited Euler-angle interpretation; this is not quaternion attitude estimation.
1021|Fuse x gyro rate with accelerometer-derived roll observation in the roll angle/bias filter.
1022|Fuse y gyro rate with accelerometer-derived pitch observation in the pitch filter. Body gyro axes only approximate Euler roll/pitch derivatives at modest coupled attitudes.
1024-1025|Allow altitude prediction/correction only if barometer-valid and finite height/world-up acceleration are provided.
1026-1027|Predict height/velocity on every accepted control step from external upward acceleration.
1028|Correct only when the producer flags a new independent barometer sample.
1029|Fuse that new height measurement. A repeated held sample with updated=false only participates in validity, not another correction.
1030|End altitude-estimation branch.
1032-1033|Reject null/invalid command or nonfinite roll, pitch, yaw rate, throttle or boost. Estimators above may already have advanced before this command rejection.
1034|Reset PID/reference/boost state.
1035|Set fault flag, even though the cause can be command validity rather than a physical sensor failure.
1036|Return zero demands and armed=false.
1037|End command validation.
1038|State that the only current mode requires height aiding; there is no alternative manual-thrust mode.
1039-1040|Reject invalid barometer aiding or nonfinite height/vertical acceleration.
1041|Reset control memory.
1042|Set the fault flag.
1043|Return zero mathematical demands. Sensor-loss landing/fallback authority remains future integration work.
1044|End height-aiding requirement.
1046|Introduce command bounds, separate from packet validation, because host/companion callers can bypass radio decoding.
1047|Clamp requested roll angle to +/-35 degrees.
1048|Clamp pitch angle to +/-35 degrees.
1049|Clamp requested yaw rate to +/-180 degrees/s.
1050|Clamp throttle to 0..1.
1051|Clamp RT boost target to 0..1.
1052|Slew boost: increase by at most 4*dt, decrease by at most 10*dt. A full rise takes at least 0.25 s and full fall at least 0.10 s. At dt=.002, per-step changes are +.008 or -.020.
1053|Use the smoothed boost value for this iteration.
1054-1055|Outer roll PID maps angle error to desired roll rate using filtered roll angle as measurement.
1056-1057|Outer pitch PID maps angle error to desired pitch rate.
1058-1059|Inner roll-rate PI maps desired-minus-measured x gyro rate to normalized roll mixer correction. It uses calibrated gyro_x directly, not gyro_x minus the Kalman filter's additionally estimated residual bias.
1060-1061|Inner pitch-rate PI similarly uses calibrated gyro_y directly.
1062-1063|Yaw-rate PI compares commanded rate with calibrated gyro_z; no magnetometer or absolute-heading state participates.
1065|If this is the first valid control step after startup/reset, initialize height hold.
1066|Capture current estimated height as the initial target, preventing an automatic jump to zero metres.
1067|Clear altitude PID memory at target capture.
1068|Mark height reference initialized.
1069|End target initialization.
1070-1071|Map throttle to upward velocity: v_cmd=(throttle-.5)*2*(2+boost). With boost=0, endpoints give -2,+2 m/s; boost=1 gives -3,+3 m/s. Center remains zero, so boost alone does not force a climb.
1072|Integrate desired velocity to move the height target: h_ref += v_cmd*dt. With centered throttle, the target remains fixed.
1073-1074|Keep reference within +/-2 m of current estimated height to bound target run-away. While limited, derivative of the stored reference is not necessarily exactly v_cmd.
1075-1076|Run the single altitude PID using height error plus desired/estimated vertical velocity; obtain a normalized collective correction.
1077-1078|Estimate vertical thrust projection cos(roll)*cos(pitch), with a floor of .5 to bound tilt compensation to at most a factor of two. It is an approximate geometric correction.
1079|Add hover feed-forward and altitude correction, divide by tilt projection, then clamp pre-mixer collective to 0..(.65+.15*boost): .65 normally, .80 at full boost. This reserves nominal room for attitude corrections but is not a universal post-mixer headroom guarantee.
1081-1083|Document the provisional X-frame order FL,FR,RR,RL and the lack of a connected ESC driver.
1084|Create four differential corrections: FL=roll-pitch-yaw; FR=-roll-pitch+yaw; RR=-roll+pitch-yaw; RL=roll+pitch+yaw. Each axis's four signs sum to zero before limits.
1085|Start the minimum and maximum correction at the first motor's value.
1086|Scan the other three motors to get the full differential-correction range.
1087-1088|Explain allocation strategy: preserve correction ratios through uniform scaling, then move collective into a feasible interval; separate per-motor clipping would change requested torque ratios.
1089|If maxCorrection-minCorrection exceeds normalized motor range 1, scale all corrections by its reciprocal; otherwise scale=1.
1090|Choose common allocated collective in [-low*scale, 1-high*scale], ensuring every shifted correction fits 0..1. This common shift may change the pre-mixer collective, including pushing it above its earlier cap; the cap is not a hard final total-demand/current limit.
1091|Add each scaled correction to the common collective and clamp to 0..1 for roundoff protection. Final motor demands are ordered FL,FR,RR,RL.
1093|Document that no hardware arm/output path follows the mathematical mixer.
1094|Explicitly force armed=false even when mathematical motor demands are nonzero.
1095|Return the calculated result.
1096|End the control step.
1098|Compile the following simulation ABI only when QCOPTER_HOST_TEST is defined.
1099|Document host-only C entry points: same control equations, no physical arming/ESC path.
1100|If additionally building the Windows SITL DLL, export the following symbols.
1101|Map QC_SIM_EXPORT to Windows __declspec(dllexport), making functions visible for ctypes/library loading.
1102|Otherwise select the non-DLL host path.
1103|Define an empty export macro for that path.
1104|End export-macro selection.
1105|Export simulation reset: caller supplies hover feed-forward, initial height and optionally 18 gain values. Pointers/array lengths are a trusted ABI contract, not fully checked here.
1106|Clear boost state.
1107-1108|Reset roll estimator angle/bias/covariance/noise using a compound literal; omitted members become zero.
1109|Copy that initial angle-filter state to pitch.
1110-1111|Reset altitude estimate to caller-supplied height, P00=.04 m², P11=.1 (m/s)², acceleration sigma=1.5 and barometer variance=.0025 m². That height R corresponds to .05 m measurement sigma, unlike the firmware-global default R=2.
1112-1113|Build the six-PID pointer order expected by the gain array: roll angle, roll rate, pitch angle, pitch rate, yaw rate, altitude.
1114|Visit all six PID instances.
1115|If a gains pointer was supplied, replace gains; otherwise retain current values, not necessarily the original source defaults after an earlier reset call.
1116|For loop i, read Kp at gains[3*i] and Ki at gains[3*i+1].
1117|Read Kd at gains[3*i+2]. Gain finiteness and array length are not validated in this host helper.
1118|End optional gain replacement.
1119|Reset the selected PID's integral/measurement history.
1120|End loop over PID instances.
1121|Clamp supplied normalized hover feed-forward to .05.. .85. The host caller must supply a finite value.
1122|Require height-reference capture on next valid control step.
1123|End host reset.
1125-1127|Export one host control step. Input/command arrays enter; four motors and sixteen diagnostics leave through caller-allocated arrays. No I2C, SPI, radio or ESC transport occurs inside it.
1128|Create the internal sensor sample from the host's flat arrays and flags.
1129|input[0..2] supply accelerometer x,y,z in g.
1130|input[3..5] supply gyro x,y,z in degrees/s.
1131|input[6] supplies upward gravity-compensated acceleration and input[7] supplies barometer height.
1132|input[8]>.5 flags a new barometer sample; flags bit 0 supplies IMU validity.
1133|The same flags bit 0 also marks host IMU data already calibrated; flags bit 1 supplies barometer validity. The simulation assumes its producer did the unit/frame/calibration work.
1134|command[0] is requested roll angle and command[1] requested pitch angle.
1135|command[2] is yaw rate, command[3] centered throttle; command[4] boost is read only when flags bit 3 is set. Flags bit 2 controls command validity.
1136|Call the same flight_control_step used by firmware.
1137|Copy four normalized motor demands to the host's motor array.
1138|Begin sixteen diagnostic values: indices 0 and 1 are estimated roll and pitch angles.
1139|Diagnostic 2 is estimated height; 3 is estimated vertical velocity.
1140|Diagnostics 4 and 5 are roll angle and residual-bias Kalman gains.
1141|Diagnostics 6 and 7 are altitude height and velocity gains.
1142|Diagnostic 8 is roll-angle sigma = sqrt(max(P00_roll,0)).
1143|Diagnostic 9 is height sigma = sqrt(max(P00_height,0)).
1144|Diagnostics 10 and 11 are roll and height innovations.
1145|Diagnostic 12 is stored height target; 13 is armed (always 0); 14 is the fault flag converted to float.
1146|Diagnostic 15 is roll innovation variance S; complete pitch covariance, yaw uncertainty and battery state are not exposed in this array.
1147|Copy all sixteen diagnostic values into the host output buffer.
1148|End the exported simulation control step.
1149|Export an extra scalar height-observation update, used by the simulation to provide lidar-derived height with caller-specified variance.
1150|Only accept finite height and finite strictly positive variance. Range, timestamp, alignment and outlier checks are not implemented by this helper.
1151|Save the normal barometer observation variance.
1152|Temporarily use the caller's observation variance for this measurement.
1153|Fuse this height with the shared height/velocity Kalman correction.
1154|Restore the original barometer variance afterward, so lidar does not permanently replace its noise setting.
1155|End accepted observation branch.
1156|End host-only extra-height helper.
1157|End all QCOPTER_HOST_TEST-only entry points.
1159|Compile the following radio/core-1 scaffold only for Pico firmware.
1160|Begin the object passed through the inter-core receive queue.
1161|Store a complete 22-byte packet buffer.
1162|Store actual received length as size_t.
1163|Store local reception time in microseconds.
1164|Name the queued receive-item type.
1166|Allocate one persistent Pico queue object; its storage is initialized in main.
1168|Explicitly label radio_receive_frame as a placeholder, not an operational LoRa driver.
1169|Declare a nonblocking radio read interface: destination, capacity and returned length.
1170|Cast the unused destination parameter to void to suppress compiler warnings.
1171|Likewise suppress the unused capacity warning.
1172|Likewise suppress the unused length-pointer warning; no length is written.
1173|Always report no packet received. No LoRa or CC1101 peripheral is initialized or used.
1174|End the radio stub.
1176|Define the function that core 1 will execute.
1177|Run core 1 forever.
1178-1181|Document intended ownership of slow/nonblocking radio and sensor work. Only the radio placeholder/polling loop exists; slower sensor acquisition is not implemented.
1182|Create a fresh zero-initialized queue item on the core-1 stack.
1183-1184|If a future radio driver returns data and its reported length fits the buffer, process that received item.
1185|Timestamp accepted received data using the local 64-bit microsecond clock.
1186|Try to enqueue without blocking; ignore the Boolean result. If the four-item queue is full, this item is dropped with no drop counter.
1187|End received-item branch.
1188|Sleep roughly 1 ms before polling again. This is not a precisely scheduled 1000 Hz sensor task.
1189|Close the endless polling loop.
1190|End the core-1 function.
1191|End Pico-only radio/core-1 definitions.
1193|Compile main only for Pico firmware, excluding it from the host DLL.
1194|Define firmware entry point.
1195|Initialize Pico's configured stdio interfaces; actual USB/UART enablement comes from the build configuration.
1196|Initialize a queue with capacity for four radio_rx_item_t objects. Queue use is the explicit communication channel between cores here.
1197|Launch slow_core on core 1; the current core continues as core 0.
1198|Wait 1500 ms before startup messages/loop setup, commonly useful for a console connection; this does not calibrate sensors.
1199|Print that 500 Hz is the target and hardware radio/sensors/ESC outputs are not connected.
1200|Print the radio-stub and disarmed status.
1202|Start with no accepted pilot-link state and invalid latest command.
1203|Read the timestamp used to calculate the first measured control interval.
1204-1205|Schedule the first control deadline 2000 us after current absolute time.
1206|Initialize iteration count for periodic status reporting.
1207|Initialize late-deadline count for the next reporting window.
1208|Enter the endless core-0 control loop.
1209|Sleep until the scheduled absolute deadline; if already late, sleep_until does not restore time already lost.
1210|Advance the planned deadline by exactly one 2000 us period, rather than scheduling from current completion time. This limits accumulating phase drift but has no explicit missed-period skip/resynchronization policy.
1211|Read actual wake-up time in microseconds.
1212-1213|Recover the deadline for the iteration just started by subtracting one period from the newly advanced deadline.
1214|Count the iteration late if wake-up is more than 100 us past that deadline. This measures start lateness, not complete execution time or a certified hard real-time guarantee.
1215|Calculate actual dt=(now_us-previous_us)*1e-6 seconds, rather than assuming exactly .002.
1216|Store this wake-up timestamp for next interval calculation.
1218|Prepare a receive item for draining core 1's queue.
1219|Remove queued items until the queue is empty, without blocking.
1220-1221|Attempt packet validation/sequence acceptance for each item with its original receive timestamp; ignore the Boolean return. Accepted later queue entries replace earlier ones.
1223|Document that no sensor acquisition exists and therefore the following input intentionally fails sensor-health checks.
1224|Construct an all-zero sample, including imu_valid=false, imu_calibrated=false and barometer_valid=false.
1225|Retrieve the last pilot command only if the link timestamp is fresh.
1226-1227|Call flight_control_step with the invalid no_sensors sample and measured dt. It therefore follows the early fault/zero-demand path on every firmware iteration.
1228|Discard the returned status to suppress an unused-variable warning. No function writes PWM/DShot/ESC output.
1229|Increment iteration count and report after 500 iterations. This is about once per second at the target rate, not necessarily exactly once per wall-clock second.
1230|Print the target-frequency label, late-start count and command freshness; the string does not prove measured loop frequency.
1231|Cast the late-start count for printf's unsigned-long format.
1232|Print fresh if the command is valid; otherwise failsafe. This is a status word, not proof of an implemented airborne failsafe maneuver.
1233|Reset iteration counter for the next reporting window.
1234|Reset late-start counter for that window.
1235|Close periodic printing branch. Printing can itself consume time; no worst-case logging budget is enforced here.
1236|Close the endless control loop.
1237|End firmware main; the endless loop normally never returns.
1238|End the firmware-only conditional compilation block.
'''

for row in DATA.strip().splitlines():
    location, text = row.split('|', 1)
    start, _, end = location.partition('-')
    explain(int(start), int(end or start), text)

CHAPTERS = [
    (1, 'Purpose, C includes and the 22-byte command protocol'),
    (124, 'Conditional Pico includes and battery-monitor data'),
    (193, 'Battery configuration, voltage Kalman filter and SOC'),
    (319, 'Control timing and calibration data'),
    (352, 'Online statistics, stationary gyro and six-face accelerometer calibration'),
    (482, 'Magnetometer calibration — not yet flight fusion'),
    (555, 'BMP280 factory compensation and ground-reference altitude'),
    (659, 'Generic linear reference calibration'),
    (706, 'PID, estimator, command and sensor structures'),
    (794, 'Pilot packet scaling, freshness and initial control settings'),
    (870, 'Clamping, resets and generic PID'),
    (918, 'Kalman prediction and correction mathematics'),
    (979, 'Altitude PID and complete flight-control step'),
    (1098, 'Host simulation ABI and diagnostics'),
    (1159, 'Pico core 1, receive queue and core-0 main loop'),
]

INTRO = r'''# flight_controller.c — complete line-by-line teaching guide

**Historical source snapshot at commit 68619b5, before flash persistence and the
35% warning / 20% disarm policy.** For current behavior, see
[the model/integration guide](current-model-and-fc-integration.md) and
[calibration flash guide](calibration-flash.md). Old LAND and volatile-calibration
descriptions below apply only to this snapshot.

This guide explains that snapshot in order. It does not change the firmware
or rerun implementation tests. Every nonblank physical source line appears below;
blank lines are spacing only and are omitted. For a C statement spanning several
lines, the first line explains the complete operation and subsequent lines identify
the continuation. Comments and braces are included. Line numbers refer to this
exact source snapshot, not every future revision.

## Read this first: what actually runs

- **On a Pico:** `main()` creates a core-1 receive queue and schedules a 500 Hz
  target loop. Radio reception is a stub. It passes invalid zero sensor data into
  the math core, which returns zero mathematical demands and a fault. There is no
  ESC driver, physical sensor acquisition, hardware arming, or actual calibrated
  flight pipeline.
- **In simulation:** the host entry points `qc_sitl_reset`, `qc_sitl_step` and
  `qc_sitl_height_observation` let the workstation call the same C control math.
  The simulation supplies measurements and separately uses the resulting normalized
  motor demands to drive its virtual plant. `out.armed` remains false.
- Battery and calibration functions are available utilities in this file. Their
  presence does not mean they are called or wired to hardware in `main()`.
- HMC5883L-style magnetic calibration is present, but neither magnetometer samples
  nor a heading estimate enter `flight_control_step`. There is no absolute-yaw hold.
- The opening Isaac comment is historical. The current active simulation is Gazebo.

## C syntax used repeatedly

| Syntax | Meaning here |
|---|---|
| `float`, `double` | Floating-point numbers; `double` is used for more accurate calibration accumulation. |
| `uint16_t`, `int16_t` | Unsigned/signed integers with exactly 16 bits. |
| `0.5f`, `500u` | Float literal and unsigned integer literal. |
| `typedef struct { ... } name;` | Define a named record type with multiple fields. |
| `const sensor_sample_t *s` | Read through a pointer to a sample; do not modify it through that pointer. |
| `s->field` | Access a member through a pointer; equivalent to `(*s).field`. |
| `&value`, `*pointer` | Take an address; read/write the object at an address. |
| `static` at file scope | Private to this C translation unit; a static object retains its state between calls. |
| `{0}` | Zero-initialize every member, including false Boolean flags. |
| `.kp = ...` | Designated structure-member initialization. |
| `(type){...}` | A compound literal: construct a whole typed object for assignment. |
| `&&`, `||`, `!` | Boolean AND, OR and NOT; AND/OR short-circuit left to right. |
| `&`, `|`, `^`, `~`, `<<` | Bitwise AND, OR, XOR, complement and left shift. |
| `condition ? a : b` | Choose a when condition is true, otherwise b. |
| `return false` | Reject this operation; the calling function decides what happens next. |
| `#if`, `#ifdef`, `#ifndef` | Select which code is compiled; these are not runtime branches. |
| `(void)unused` | Suppress an unused-value warning; it does not transmit, execute or validate anything. |

## Follow the control path before memorizing helpers

```text
sensor sample -> validity -> roll/pitch angle observations -> angle/bias Kalman filters
                         -> height/velocity prediction + fresh height correction
pilot/companion command -> validity + limits -> outer angle P -> inner rate PI
                                           -> yaw rate PI
                                           -> integrate vertical-speed command into height target
                                           -> one altitude PID -> tilt compensation
                                           -> common collective + 4-motor torque allocation
                                           -> mathematical demands, physical armed=false
```

The battery monitor and calibration collectors are separate branches, not steps
automatically executed inside this path. The current file is not re-entrant: its
global filter/PID state represents one aircraft. The simulation/firmware calling
context must serialize calls.

## The six controllers and their units

There are six `pid_t` objects, but not all have nonzero P, I and D gains:

| Controller | Error/input | Output | Source default Kp, Ki, Kd |
|---|---|---|---|
| Roll angle | degrees | desired degrees/s | 3, 0, 0 |
| Roll rate | degrees/s | normalized roll correction | .000829338, .003870245, 0 |
| Pitch angle | degrees | desired degrees/s | 3, 0, 0 |
| Pitch rate | degrees/s | normalized pitch correction | .000829338, .003870245, 0 |
| Yaw rate | degrees/s | normalized yaw correction | .005375614, .008362066, 0 |
| Altitude | metres, with vertical-speed error | normalized collective correction | .108045, .03087, .1225 |

These are the C initialization values. A simulation caller can overwrite all 18
gains through `qc_sitl_reset`; a report's gain scale is not encoded by changing
these initializer lines. They are not identified real-airframe tuning values.

For roll, a target of 10 degrees with estimated angle 7 degrees gives an outer
P output `3*(10-7)=9 degrees/s`. If measured gyro x is 4 degrees/s, the inner
error is `9-4=5 degrees/s`. Its PI calculates a differential motor correction;
9 degrees/s is not a motor duty and 10 degrees is not a gyro-rate command.

For height control, define `e_h = h_ref-h`, then:

```text
v_cmd = (throttle - .5)*2*(2 + boost)
h_ref_new = h_ref_old + v_cmd*dt           [then limited near estimated height]
u_h = Kp*e_h + Ki*integral(e_h dt) + Kd*(v_cmd-v_est)
```

At throttle .5, `v_cmd=0`: the stored reference stops moving, so the controller
holds that reference height. It does not merely hold a fixed motor percentage.
At throttle .75 with no boost, `v_cmd=1 m/s`; at dt=.002 the height target advances
by .002 m per step. There is no seventh vertical-speed PID.

## Why the angle filter is not literally one state

Each axis uses `x=[angle, residual_gyro_bias]^T`. Its prediction is:

```text
angle_minus = angle + dt*(gyro - bias)
bias_minus = bias
F = [[1,-dt],[0,1]]
```

Writing the old covariance as `P=[[a,b],[c,d]]`, direct multiplication gives:

```text
F*P = [[a-dt*c, b-dt*d],[c,d]]
F*P*F^T = [[a-dt*(b+c)+dt²*d, b-dt*d],[c-dt*d,d]]
```

This explains the signs in lines 935–938. Gyro measurement variance contributes
`gyro_noise*dt²` to angle uncertainty. A continuous random-walk bias with intensity
`qb` contributes `[[qb*dt³/3,-qb*dt²/2],[-qb*dt²/2,qb*dt]]`.
The observation is the accelerometer-derived angle with `H=[1,0]`:

```text
S = P00_minus + R
K = [P00_minus, P10_minus]^T / S
innovation = measured_angle - predicted_angle
x_plus = x_minus + K*innovation
```

Thus the usual phrase “1D roll/pitch Kalman” means independent angular axes here;
the implementation is a two-state angle/bias filter for each axis. Kalman gains
are recalculated each update, not manually fixed like PID gains. Their uncertainty
meaning depends on the input/noise assumptions actually being reasonable.

## Altitude filter and covariance correction

Altitude uses `x=[h,v]^T`, `F=[[1,dt],[0,1]]`, external input acceleration `a_up`
and `G=[dt²/2,dt]^T`. Therefore:

```text
x_minus = F*x + G*a_up
P_minus = F*P*F^T + G*q*G^T
q = accel_noise²
G*q*G^T = q*[[dt^4/4,dt³/2],[dt³/2,dt²]]
```

This is exactly the component form at lines 952–965. Acceleration is treated as
an input, not a third state; no vertical acceleration-bias state exists. A fresh
barometer or supplied lidar-derived height then observes `H=[1,0]` and corrects
both height and velocity through the cross covariance.

`covariance_correct` uses Joseph form:

```text
A = I-K*H = [[1-K0,0],[-K1,1]]
P_plus = A*P_minus*A^T + K*R*K^T
```

Keeping the additional `K*R*K^T` term and explicitly restoring off-diagonal
symmetry improves the numerical form of the covariance calculation. It does not
compensate for an incorrect sensor model or prove statistically calibrated errors.

## Scope and unfinished connections to keep in mind

1. No physical MPU6050, BMP280, HMC5883L, LoRa, CC1101, ADC/current sensor,
   TOFSense or ESC driver is called from this file's `main()`.
2. A voltage divider provides voltage, not measured discharge current; current
   input is required by the present coulomb-count SOC function.
3. The battery action enum is not connected to an automatic landing controller.
   It requests LAND in air rather than cutting motors at a low-SOC threshold.
4. Calibration capture results are not automatically loaded into the live filter
   noise parameters, stored in flash, or passed to real sensor acquisition.
5. Gyro rate is used as Euler-axis rate approximately; accelerometer tilt assumes
   gravity-dominated motion. There is no full quaternion/heading estimator here.
6. `sample_is_healthy` checks flags/finiteness, not timestamps or vibration quality.
   There is no complete physical sensor-loss/radio-loss recovery state machine.
7. Individual PID anti-windup does not account for later collective/mixer limiting.
   The mixer preserves relative torque ratios by shifting collective, so the .65/.80
   pre-mixer cap is not an unconditional final throttle/current/headroom guarantee.
8. `main()` aims for 500 Hz, but no physical worst-case execution-time measurement
   or hard deadline guarantee is established. Replaying a simulated .002 s step
   is not evidence that hardware always finishes its work in .002 s.

## Source lines

In each table, source is copied verbatim with Markdown escaping. A continuation
entry refers to the complete multi-line statement, not an additional operation.
'''

chapter_map = dict(CHAPTERS)
output = [INTRO, f'\nSource: [flight_controller.c at {SOURCE_REF}](https://github.com/BenMohStem/IntelQCopter/blob/{SOURCE_REF}/flight_controller/flight_controller.c) · {len(lines)} physical lines.\n',
          f'\nGit blob content SHA256: `{hashlib.sha256(source_bytes).hexdigest()}`.\n\n']
missing = []
for number, source in enumerate(lines, 1):
    if number in chapter_map:
        output.append(f'\n### L{number}: {chapter_map[number]}\n\n| Line | Exact C source | Explanation |\n|---:|---|---|\n')
    if not source.strip():
        continue
    note = notes.get(number)
    if note is None:
        missing.append(number)
        note = 'UNEXPLAINED — author must complete this entry.'
    escaped = source.replace('&','&amp;').replace('<','&lt;').replace('>','&gt;').replace('|','&#124;').replace('`','&#96;')
    output.append(f'| {number} | <code>{escaped}</code> | {note.replace("|", "&#124;")} |\n')
if missing:
    raise SystemExit(f'Missing line explanations: {missing}')
DEST.write_text(''.join(output), encoding='utf-8', newline='\n')
print(f'Wrote {DEST}: {sum(bool(line.strip()) for line in lines)} nonblank source lines explained; firmware unchanged.')
