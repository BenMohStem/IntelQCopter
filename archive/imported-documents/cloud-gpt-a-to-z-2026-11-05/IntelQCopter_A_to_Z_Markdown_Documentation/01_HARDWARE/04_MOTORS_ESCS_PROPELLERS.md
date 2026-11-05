# Motors, ESCs, propellers and propulsion limitations

Hardware described: four orange A2212/10T-class motors, generally called 1400 KV by the project owner, four 40 A ESCs, 3S LiPo, F450 frame. Some older documents mention 1000 KV alternatives and 8045 propellers; **currently installed prop geometry and actual motor winding type remain unverified**.

## What the present numerical model assumes

Idealized maximum thrust **6 N per motor** with 1σ prior 1.5 N, first-order thrust lag 40 ms with 1σ prior 15 ms, yaw moment-per-thrust-arm ratio 0.015 m with uncertainty, and thrust linearly proportional to normalized motor command. These are assumptions from simulation, not real ESC characteristics. A 40 A ESC rating does not supply thrust data.

## Identification sequence

With a suitable guarded, restrained thrust stand, a calibrated force gauge and propeller safety, measure each motor separately at low and medium commands. Record supply voltage under load, command method, stable thrust, current, response lag and motor heating. Verify idle, spin direction, propeller handedness and ESC startup only after an independent hardware cut-off exists. Never infer free-flight thrust margin from an unmeasured “45% throttle” value.

## Critical limit from model

For hypothetical 1.104 kg mass and 4×6 N thrust, a **linear thrust surrogate** predicts hover around 45%. The historical 45% command cap may prevent hover or leave no stabilization headroom. ESC PWM values and actual thrust fractions are not necessarily the same. Calculate thrust margin using measured thrust curves and actual total mass before changing output limits.

## Still open

Motor numbering/order, spin direction, PWM or DShot capabilities, physical ESC signal GPIOs, signal ground continuity, battery voltage/current measurement, guarded operation, arming and independent kill path.
