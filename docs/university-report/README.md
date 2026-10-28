# IntelQCopter Autonomous Quadcopter

Professional and Personal Project ? Pico system edition4.0,28October2026.

- [PDF report](technical-report.pdf)
- [Standalone HTML](technical-report.html)
- [Editable mathematical source](technical-report.md)
- [Target materials and quotation inventory](bill-of-materials.csv)
- [Target materials source](materials-target.json)
- [Requirements register](requirements-register.json)
- [Exact inspected Pico/ROS source inventory](source-inventory.json)
- [Historical ESP32 evidence hashes](evidence-manifest.json)
- [Figure assumptions and provenance](assets/visual/figure-manifest.json)
- [QA](validation.json)

Target:RP2350 FC,RP2040 custom handheld,LoRa primary pilot link,two reserved CC1101 modules per side,and an optional Raspberry Pi or NVIDIA Jetson ROS2 computer still being sought. The report's GitHub link/QR and version map distinguish main,esp32-xbox and codex/stepwise-esp32. Runtime sources/branches are unchanged by this documentation revision.

38pages,40technical visuals,41equation panels,50requirements,13verification cases and37materials/equipment entries. Explains why the student selected PID to apply learned feedback concepts; derives inverse-variance fusion versus blind averaging with assumptions; details ROS2,localization,YOLO/range association,A* prototypes,n-node mission extension and companion selection. Existing ESP32 measurements are not labelled Pico timing. Inspected main Pico scaffold remains500Hz with hardware stubs; new RP2350 profiling is required.

The previous report source and ESP materials list are preserved under reference/. The realistic F450 image is labelled AI concept artwork; original evidence plots remain unchanged. Personal details,actual rubric,authentic reflection and quotations still require completion. Physical motor outputs remain disabled.

Build from repository root with Python,ReportLab,Matplotlib,NumPy,Pillow and Windows Arial fonts:

```powershell
python tools/docs/build_university_report.py .
```

The HTML embeds figures. Inventory fields are preserved only when both ID and component match,to avoid carrying prototype quotations onto different target hardware. No hardware,upload,flight or new simulator run was conducted for this report.

[After-report discussion reminder](discussion-backlog.md):Intel N-series computer plus camera budget idea; not part of the selected baseline.
