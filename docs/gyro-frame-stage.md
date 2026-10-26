# Current step: gyro sensor axes to drone axes

The acquisition stage now has actual hardware evidence: three runs, 60,000 samples, maximum handlers 814/819/812 us and no reported overruns. Core 1 handles acquisition; a bounded FreeRTOS queue sends slow UART formatting/transmission to core 0. Inter-service jitter remains, so this is a finite observed budget result, not a hard real-time or flight-readiness certificate.

The next source, FIFO1000_V2, computes corrected gyro rates in drone coordinates:

\[
\boldsymbol\omega_{body}=R\boldsymbol\omega_{sensor,corrected}.
\]

The full accelerometer matrix C includes accelerometer scale/shear as well as mounting orientation. It must not be applied directly to gyro rates. Its singular-value decomposition gives a closest proper rotation candidate:

\[
C=U\Sigma V^T,\qquad R=U\operatorname{diag}(1,1,\det(UV^T))V^T.
\]

The candidate has determinant +1 and preserves vector norm. Values and assumptions are in [config/gyro-frame.json](../config/gyro-frame.json). The fixed coefficients are compiled into main.cpp; JSON is not read by the MCU at runtime. Sensor bias is corrected before rotation, exactly once. Accelerometer calibration and MPU target words are unchanged.

The physical interpretation is **provisional**. An accelerometer fit cannot uniquely separate every sensor cross-axis error, reference error and rigid mounting rotation. No gyro scale is inferred from stationary measurements. The [manufacturer axes/polarity diagram](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf) defines package axes; reference rotations must validate their relationship to the drone.

## Tests of this step

The source compiles. Forty-eight unique offline checks and nineteen actual-source host cases pass, including three sensor-basis cases. Host peripherals/scheduling are modeled; those are not hardware timing results.

A new actual 20,000-sample capture completed at about 1000.69 Hz with maximum handler 822 us. Every raw sequence/checksum and all 200 emitted body-rate diagnostics passed independent raw-to-output recomputation. This verifies computation and observed overhead, not physical axis direction or dynamic attitude accuracy.

[Actual trials and timing graph](../measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md) retain the initial overruns, the rejected serial trial and repaired runs. No record was discarded to manufacture a pass. Current D records retain sensor rates and append body rates; state updates occur at every FIFO sample.

## Physical reference check remains pending

The user will perform known rotations later. No stationary or synthetic test substitutes for this check. Motor outputs stay disabled; no Kalman integration or PID is enabled using assumed physical validation.

For later operator motion, record-1khz accepts `--condition operator_rotation --note "axis/direction/reference"`. Each capture lasts about 20 seconds and saves originals. Positive/negative rotations about each defined drone axis are needed; an independent angular reference is needed to assess scale rather than only sign. Motion records must not enter stationary bias fitting.

Next: review the user's rotations, correct the mapping if needed, then add the small attitude filter to the verified acquisition path. BMP280 and altitude remain deferred. Calibration residuals are not zeroed or refitted from these validation runs.
