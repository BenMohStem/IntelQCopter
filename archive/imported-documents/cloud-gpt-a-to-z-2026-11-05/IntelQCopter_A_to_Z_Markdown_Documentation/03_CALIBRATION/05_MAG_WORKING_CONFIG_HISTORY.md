# Why the magnetometer finally worked — the history without oversimplification

## What failed

Multiple isolated and full GUI branches previously detected QMC5883P ID=0x80 and ACKed writes, then failed reads on status `0x09` after starting measurement mode. The failing v13 200 Hz `0xCD` attempt recorded zero fresh samples. v17 unattended tests had seven completed variants with no fresh MAG data and additional experiments blocked by COM busy. Wi-Fi made low-rate behavior worse in some observations, but problems persisted with Wi-Fi off.

## What worked

- Original isolated v13 source: **`0xC9` 100 Hz**, readback `0xC9`/`0x08`, measured ~90 fresh Hz during the Nov 3 probe. A later probe was not indefinitely stable.
- Original v13 driver transplanted into full firmware: recovered a path to valid integrated MAG data.
- Successful 4 Nov capture: **`0x3D`** low-oversampling nominal 200 Hz, ~195 new readings/s, together with MPU/BMP. The successful capture's firmware config is more authoritative than its `100hz` archive name.

## Lessons

1. Do not replace a working transaction pattern with an elaborate fallback just because a datasheet theoretically permits it.
2. Chip ID/readback alone does not prove valid measurements; capture actual XYZ sample count and timestamps.
3. One working session does not establish reliability under Wi-Fi, hot boot, motor EMI or bus saturation.
4. Cold power removal and firmware-only reset are different experiments.
5. Do not re-label the physical `0x3D` success as 100 Hz to fit a previous assumption.

## Future change discipline

Preserve the Nov 4 successful source as a golden baseline. Put any new 400 kHz FIFO/FC migration in a separate branch. Use a regression trace with MAG rate, I²C errors, control readbacks, actual power state and MPU/BMP load before merging changes.
