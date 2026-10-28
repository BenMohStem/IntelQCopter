# IntelQCopter Autonomous Quadcopter

Professional and Personal Project ? visual mathematical edition3.0,28October2026.

- [PDF report](technical-report.pdf)
- [Standalone HTML report](technical-report.html)
- [Editable mathematical source](technical-report.md)
- [Materials and quotation inventory](bill-of-materials.csv)
- [45 requirements](requirements-register.json)
- [Figure provenance and assumptions](assets/visual/figure-manifest.json)
- [Source evidence hashes](evidence-manifest.json)
- [Cover artwork prompt and origin](image-provenance.json)
- [Document QA](validation.json)

30pages,32technical visuals including an original recording figure,34equation panels,45requirements,12verification cases and31BOMentries. Prose is reduced; calibration,matrix correction,covariance,Kalman,PID,continuous/discrete dynamics,timing failures,YOLO-to-avoidance,navigation and proof architecture are visual. Exact pinout and low-level syntax are outside the academic narrative. [The complete earlier engineering baseline](reference/engineering-baseline-v2.1.md) preserves detail.

The red-and-white F450 cover is generated concept artwork,not a photo of assembled or tested hardware. Every technical figure is identified as measured summary,existing model,analytic example,schematic or proposal. No new firmware,calibration,measurement,simulation or flight was performed. Motor outputs remain disabled.

Build from the repository root with Python,ReportLab,Matplotlib,NumPy,Pillow and Windows Arial fonts:

```powershell
python tools/docs/build_university_report.py .
```

The builder regenerates technical figures and embeds all images in the portable HTML. Manual inventory/quotation fields are preserved when IDs match. Student details,authentic reflection,actual rubric and dated supplier quotes must be completed before submission. No grade is guaranteed.
