# Independent calibration audit — 4 November 2026

This folder contains selected report outputs and figures from the independent
raw-data audit bundle. The original 31 MB source ZIP remains at
`C:\Users\Mohamed\Downloads\GPT_TMP\IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip`;
its SHA-256 is
`7C55BA598286AA67B2BFA5BDB8E70560E952B2FC521470F40E59D2749BF6C4BC`.
The package's 40-entry SHA-256 manifest was checked with no mismatches.

The audit ZIP's four raw CSV inputs were compared byte-for-byte with the
repository captures (10:09, 02:57, 03:17 and 04:08); all matched. Raw files are
not duplicated here. The main input is
`../iq_500hz_20261104_100907.csv`. The audit report and result JSON retain the
remaining input provenance and methods.

This folder includes the Markdown report, result JSON, calibration candidate,
corrected CSV derivative, stage comparison, and seven plots. Imported file
hashes are recorded in `IMPORT_MANIFEST.md`. The audit source scripts remain
inside the original ZIP; this folder alone is not a one-command reproduction
environment.

All parameter files are offline candidates. The saved MAG candidate has
determinant-normalized scale, not an independently measured absolute Earth-field
gain. Axis alignment, heading truth, temperature variation, motor interference,
and flight suitability remain unverified. Do not include the candidate header
in firmware without a separate calibration review.
