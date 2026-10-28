# IntelQCopter university report

**20-page edition 5.0 — 27 October 2026.** Concise prose, selected diagrams, equations with local symbol/unit definitions. Prepared for Professional and Personal Project.

- [PDF](technical-report.pdf) · [Standalone HTML](technical-report.html) · [Editable source](technical-report.md)
- [Full 38-page supporting edition](reference/pico-system-edition-v4.1.md)
- [37-entry materials CSV](bill-of-materials.csv) · [50 requirements](requirements-register.json)
- [Evidence hashes](evidence-manifest.json) · [Inspected sources](source-inventory.json) · [QA](validation.json)

RP2350 aircraft / RP2040 handheld is the target; ESP32-S3/Xbox is experimental. LoRa is primary; two CC1101 per side are reserved. Pi/Jetson companion is being sought. Historical ESP32 timing is not Pico timing; flight acceptance is pending and motor outputs remain disabled.

Rebuild with Python, ReportLab, Matplotlib, NumPy, Pillow and Arial fonts: `python tools/docs/build_university_report.py .`

Editorial principle: **ما قلّ ودلّ**. Supporting registers and original evidence retain their detail outside the 20-page submission. No firmware or calibration changes, uploads or new simulation/flight tests were made.
