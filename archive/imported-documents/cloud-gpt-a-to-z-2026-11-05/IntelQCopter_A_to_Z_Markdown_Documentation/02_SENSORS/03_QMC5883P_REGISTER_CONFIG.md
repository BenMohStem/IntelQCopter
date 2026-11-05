# QMC5883P — actual compass identity, register map and working profiles

## Chip identity

This is **QMC5883P** at I²C **0x2C**, identification byte **0x80** at register `0x00`. Earlier diagrams or books mentioning HMC5883L/QMC5883L are not the hardware identification. Never use the HMC5883L register order, address or gain constants for these captures.

## Registers referenced in experiments

| Register | Meaning | Known values |
|---|---|---|
| `0x00` | ID | 0x80 |
| `0x01..0x06` | XYZ little-endian signed counts | Three int16s |
| `0x09` | Internal data status (DRDY/overflow) | Polled via I²C, no external DRDY wire |
| `0x0A` | Control 1 | Profiles below |
| `0x0B` | Control 2 | 0x08 (±8 G setting) |
| `0x29` | XYZ sign/control | 0x06 in working programs |

## The two genuinely successful histories

- **v13 standalone:** `CTRL1=0xC9`, 100 Hz nominal, often **~90 fresh Hz**, with some sporadic I²C read failures. It once failed after several seconds; it was not perfectly reliable.
- **4 Nov full calibration:** `CTRL1=0x3D`, nominal 200 Hz low oversampling and **~195 Hz fresh** in the successful capture. The ZIP name containing “100Hz” is not evidence of actual register settings. The later v2 source follows `0x3D`.

200 Hz profile `0xCD` failed in earlier v13 tests. Frequency, oversampling bits, initialization ordering, I²C electrical rate, power state and other bus activity all changed across versions. The claim is “this particular combination worked in this capture,” **not** “every 200 Hz mode works.”

## Data conversion and calibration

In the working ±8 G setting, the observed configuration uses nominal 3750 counts/G (check QST revision). Convert raw counts to µT at that configured sensitivity before applying full 3×3 correction. Matrix and hard-iron offset are sensor-frame quantities; they do not establish physical heading until the magnetometer orientation relative to aircraft is measured.

## Guardrails

Check status freshness; discard invalid or overflowed XYZ; sanity-check vector norm and age; don't trust large abrupt magnetic-field disturbances. Do not automatically reinitialize on every missed reading: repeated writes can perturb a previously running sensor. Time-limit failed I²C transactions and preserve independent IMU acquisition.

Primary reference: QST QMC5883P Rev E datasheet, <https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf>.
