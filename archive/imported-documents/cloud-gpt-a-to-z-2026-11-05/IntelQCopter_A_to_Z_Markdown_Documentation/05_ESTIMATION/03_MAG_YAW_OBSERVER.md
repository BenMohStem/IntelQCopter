# Magnetometer heading observer and disturbances

v2 calibrates MAG in sensor coordinates and applies `RbodyMag`. MAG acceptance checks include fresh data age (<120 ms), finite vector and norm plausibility (roughly 15–90 µT in source). New measurements can correct yaw drift through a slow body-up-axis innovation.

## Relative yaw, not certified compass heading

The first accepted horizontal magnetic vector establishes a local world horizontal reference. This yields a relative yaw constraint; it does not automatically supply geographic north, declination or externally measured body heading. Correcting a reflected axis would create false yaw.

## Quality gates

- Check valid/overflow flags and internal QMC status `0x09`.
- Reject stale or implausible field magnitudes; include a coherent vector-direction innovation check in future revisions.
- Do not treat every 500 Hz loop as a new MAG observation when physical ODR is ~200 Hz.
- Compare yaw with gyro-only drift and manual orientation rotations in propeller-free tests.

## Motor magnetic interference

ESC/motor/battery currents induce fields that may swamp compass offset calibration. An ellipsoid fit with motors off does not predict powered heading quality. Future controlled tests should vary motor current under restraints and inspect MAG norm and direction; if untrustworthy, reduce yaw magnetic authority and use gyro yaw-rate control without pretending absolute compass hold.
