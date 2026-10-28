# Working agreement for the university-report branch

- Read knowledge/index.json and the relevant domain before editing.
- Keep essential dynamics/PID/calibration/variance/Kalman math in the short report, supported by illustrations; use Nyquist plus clear pole views; use 3D only when it adds information there. Bode and Black–Nichols belong in the full mathematical edition.
- Keep both current PDFs and the unchanged previous full edition available on this branch.
- Use one shared illustrative PID model. Mark assumed parameters and analysis engines; never label them measured flight gains or Gazebo acceptance.
- Physical motor arming remains disabled. This branch does not modify or flash firmware.
- After each prompt that changes durable facts, preferences, decisions or evidence, update its domain and append a compact event. Do not create empty commits or chat transcripts for acknowledgements with no new information.
- Split growing knowledge by domain when it exceeds the documented thresholds or mixes unrelated topics. Preserve IDs, provenance, decisions and original evidence; update the catalog and run its checker.
- Preserve historical failed trials and source identities. Never refresh a stale hardware fingerprint just to make a check pass.
- Commit and push reviewed project changes, preserve history, and exclude credentials/environments/SDKs.
- Run the model checks, both report checks and domain checker; render and visually review changed PDF pages before publication.
