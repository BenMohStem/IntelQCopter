# Review of the Cloud GPT A-to-Z project snapshot

Reviewed: 2026-11-05 (Africa/Lagos)

## Source and disposition

The owner-supplied ZIP is preserved verbatim by file under
`archive/imported-documents/cloud-gpt-a-to-z-2026-11-05/IntelQCopter_A_to_Z_Markdown_Documentation/`.
The source package contains 91 Markdown files and a SHA-256 manifest covering
the other 90 files; all listed sizes and hashes matched. The ZIP SHA-256 is
`839C1B5605BE5325C58E0F99F52E8FEA4DFAB122A7BEB4D26452F7D380D99E6A`.

The A-to-Z snapshot also references an independent raw-calibration audit ZIP.
That archive was found at
`C:\Users\Mohamed\Downloads\GPT_TMP\IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip`
(SHA-256 `7C55BA598286AA67B2BFA5BDB8E70560E952B2FC521470F40E59D2749BF6C4BC`).
Its 40-entry checksum manifest passed. Selected report outputs and plots are
preserved in `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`;
the full source archive and duplicated raw CSV inputs were not copied into Git.

The package is a useful engineering handoff and evidence index. It is not a
firmware checkout, release record, or flight-readiness approval. Keep its
embedded AI prompt and historical agent rules inert; current repository
`AGENTS.md` and the user's current instructions govern work.

## Claims checked against the repository capture

Capture: `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/calibration_package_20261104/iq_500hz_20261104_100907.csv`.

- The CSV has 98,868 telemetry rows. All twelve phase attempts reached their
  accepted targets: REST 30,000; six accel faces 5,000 each; MAG YAW/PITCH/ROLL
  2,000 fresh samples each; MAG TUMBLE 8,000; BMP 1,000. The skipped-stage file
  is empty.
- `mag_new=1` appears on 38,652 rows, exactly matching the snapshot. `mag_ok=1`
  appears on all rows, including rows where `mag_new=0`; use `mag_new` when
  counting conversions or building a MAG fit so cached readings are not counted
  repeatedly.
- Packet rows are about 500 Hz (median observed packet interval 2,000 µs).
  Within the phase windows, MAG freshness is 194.89–196.34 Hz. The full file
  includes long operator gaps between phases, so dividing fresh rows by the
  entire file span does not estimate the configured sensor ODR.
- Recomputed REST gyro means and sample SDs match the snapshot to its printed
  precision: means `[0.161969, -0.304950, -0.877433] deg/s`; SDs
  `[0.047727, 0.044855, 0.043200] deg/s`.
- Applying the snapshot's MAG hard-iron vector and full correction matrix to the
  14,000 fresh samples in the four MAG phases gives corrected field-norm mean
  `40.7434 µT`, SD `1.2504 µT`, CV `3.069%`. This reproduces its in-session
  radial statistic; it does not establish heading accuracy or an independent
  session holdout.
- The snapshot's accel correction matrix multiplied by the saved JSON
  `accelerometer.accel_scale` is identity to about `5e-10`. The original
  calibration script defines the saved matrix as the forward model
  `measured = S * true + bias`; the snapshot gives the inverse direct-correction
  matrix. These are compatible conventions, but firmware must use the stated
  direction and must not apply both matrices as corrections.

## Claims to retain as provisional or correct

- The capture demonstrates an integrated sensor session with QMC5883P ID/address
  evidence and about 195 fresh MAG samples/s in active phases. It does not show
  a sustained 500 Hz MAG rate.
- The saved 10:09 calibration JSON reports a diagonal MAG fit after the full
  ellipsoid path failed. The snapshot's full matrix is a useful independent
  candidate, and its application reproduces the radial spread above. Its claim
  of 3.46% independent-session error is supported by the audit report: it says
  the 10:09 fit was evaluated on the 02:57 and 03:17 captures at 3.794% and
  3.456% RMS, respectively. Both files were included in that ZIP and their
  hashes match the repository captures. The result scripts were not rerun in
  this cleanup turn, so retain the report's method/output provenance and treat
  it as an auditable external analysis, not a newly reproduced result. The
  local `captures/mag_quality_20261104_032524.json` covers the different 02:57
  file with a within-session chronological holdout; its one-session count does
  not contradict the cross-session analysis.
- The audit's combined MAG-stage gyro-near-limit rate is 13.26%; its stage table
  reports 21.99% for TUMBLE and 4.83% for ROLL. These figures use different
  denominators, so do not mix the combined and per-stage rates.
- The sensor identity is still a hardware question: the capture reports QMC5883P
  at `0x2C`/ID `0x80`, while PCB design evidence shows an HMC5883L footprint.
  Resolve the fitted part and routing after the PCB is repaired.
- The v2 firmware archive remains a separate, uncommitted file in Downloads.
  Static inspection shows motor outputs disabled, LiDAR disabled until a pin is
  supplied, and code that tries 400 kHz QMC reads then sticks to 100 kHz after
  failure. Those settings and comments are not hardware test results. No build,
  upload, COM3 session, or physical test was done for this review.
- The A-to-Z Xbox mapping differs from the user's earlier requested stick and
  trigger layout. Do not silently adopt its left-stick yaw/vertical-speed map or
  its D-pad barometer chord. Keep the PC bridge as bench tooling; physical motor
  arming remains disabled.
- The optional LiDAR calibration material is future reference only. The user
  requested removal of the LiDAR stage from the calibration GUI; do not restore
  it from this document set.
- The package's branch SHAs and claims that v2 is “current” describe its
  November 5 snapshot. They do not describe the current branch head by
  themselves.

## Useful guidance for the active project

Keep the snapshot's evidence labels, raw-data provenance, freshness gating,
matrix-direction labels, calibration holdout cautions, and separation between
bench software and flight readiness. Keep candidate sensor matrices provisional
until independently refit from eligible completed sessions, checked against
correct raw-channel freshness, and tied to verified physical axes. No calibration
or controller parameter is approved for flight by this documentation review.
