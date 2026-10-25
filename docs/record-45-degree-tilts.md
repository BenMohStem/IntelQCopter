# Four 45-degree tilt recordings

Use the already uploaded 20,000-sample firmware. **No reflashing is needed.** Keep the FC in exactly the same mounting as the completed six-pose session.

Close PlatformIO Serial Monitor and run in PowerShell:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-tilts --port COM3
```

There are four runs, each with five seconds of settling and 20 seconds of sampling. Progress prints every 1,000 samples. The next pose waits for you. Total sampling is 80 seconds, plus settling and positioning.

## Before starting

The script asks what independent reference you use (for example, a 45-degree jig or angle gauge) and its uncertainty if known. Enter can leave these unknown, but then results cannot establish absolute angle accuracy. Do not use this MPU's own calculated angle as the reference.

Press Enter at the mounting confirmation only if the FC has stayed firmly fixed since the six-pose session. Moving the FC relative to the drone invalidates this comparison. Propellers remain removed; motor outputs remain disabled.

## Pose order and signs

Use the same DRONE axes as the six-pose measurements, not the axis markings on the MPU. Start with the drone level, Z upward. The signs below define this recorder's convention explicitly:

| Run | Physical orientation from level | Nominal roll,pitch entry |
|---|---|---|
| Roll +45 | Raise the +Y side by 45 degrees; keep X horizontal | `45,0` |
| Roll -45 | Raise the -Y side by 45 degrees; keep X horizontal | `-45,0` |
| Pitch +45 | Raise the -X side by 45 degrees; keep Y horizontal | `0,45` |
| Pitch -45 | Raise the +X side by 45 degrees; keep Y horizontal | `0,-45` |

At each prompt, position the drone, check your independent reference and enter the actual measured `roll,pitch`. For example, enter `44.7,0.3` if those are the measured angles. Pressing Enter alone uses the nominal target and marks it **nominal only**, rather than pretending it was measured. Check your angle instrument's sign convention against the physical side-raised descriptions above.

Once entered, keep the drone motionless until **Saved** appears. Then reposition for the next prompt. No new recording starts without your input.

The expected stationary acceleration in these body axes is:

$$
\mathbf a_{reference}=g\begin{bmatrix}
-\sin\theta\\
\sin\phi\cos\theta\\
\cos\phi\cos\theta
\end{bmatrix},\qquad g=9.80665\ \mathrm{m/s^2}.
$$

Here phi is roll and theta is pitch. Thus roll +45 expects `(0,+g/sqrt(2),+g/sqrt(2))`; pitch +45 expects `(-g/sqrt(2),0,+g/sqrt(2))`. Standard gravity is an assumed model value, not a measured local gravity value.

## Automatic files and interrupted runs

Everything is saved in a new `artifacts/tilt-recordings/SESSION/` folder, kept local and excluded from Git. Each pose gets a fresh attempt folder containing:

- `reference.json`: declared angles, their source, reference method and uncertainty.
- `serial.log`: original sensor samples and configuration.
- `summary.json`: sample counts, checksums, averages, standard deviations and timing checks.
- `validation.json`: offline comparison of that run's mean against the declared angles using the frozen candidate calibration.

The candidate is frozen in the session manifest; these recordings do not refit it. The script checks the same firmware source, board/configuration and offset baselines as the original six-pose session. It applies the existing serial-noise synchronization and strict sample checks. Source/configuration/board continuity cannot detect physical mounting movement, so the mounting confirmation matters.

If interrupted, keep the folder intact and copy the exact **Resume** command printed in the terminal. Completed poses are skipped and failed attempts are preserved. The resume command includes the source session, candidate file and target angle, preventing accidental mixing. Defaults refer to the current completed session and candidate in this checkout.

After all four runs, send the folder path for analysis. A printed direction difference is a provisional comparison against your declared reference, not automatic calibration acceptance. Angle uncertainty, pose consistency, transients and register restoration still need review.

For another nominal angle later, use `--angle 30` or another value strictly between 0 and 90. Do not change the target angle when resuming an existing session.
