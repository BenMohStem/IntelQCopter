# System specification handoff

Task: system-specification. Branch: codex/stepwise-esp32.

Delivered docs/specification/{cahier-des-charges.md,.html,.pdf}, README.md and validation.json; reproducible renderer tools/docs/build_specification.py. Plan links the deliverables. Fifteen pages,45 requirement IDs,12 tests; quantitative targets explicitly proposed. Covers current ESP32 scope and separate Pico evolution, calibration,1kHz timing, PID/model, Xbox compatibility gate, buses/bypass/lidar, optional ROS2, navigation observability, energy policy and staged acceptance.

Engine: ReportLab document generation; PyMuPDF text/bounds checks and rendered contact-sheet visual review. No new physical test, upload, simulation or firmware modification. Known physical rotations and all flight acceptance remain pending. Existing evidence is not relabeled. Document hashes are recorded in validation.json.

Checks: 15 nonempty pages; text bounds inside page; no unexpected source control characters; representative full-resolution table and all-page contact sheet inspected. Git diff whitespace and knowledge integrity checks required before publication. Requirement targets remain reviewable proposals, not verified performance.
