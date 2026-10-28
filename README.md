# IntelQCopter — Autonomous Quadcopter

**USTHB · FGE · Département Automatique · RSI**
Fourth-year engineering · Personal and Professional Project (RSI7.8)
Report edition: **27 October 2026**

## Read the report

**[Open the 20-page PDF](IntelQCopter.pdf)** · [Browser edition](report/index.html) · [Editable report](report/report.md)

1. **Flight controller:** physics, PID, sensors, calibration and estimation.
2. **ROS 2 and YOLO:** companion computer, perception and navigation.
3. **Gazebo and RViz:** model, environment and software validation.
4. **Resources:** hardware, software and professional method.
5. **Handheld and links:** RP2040, LoRa, CC1101 and companion communication.

The intended aircraft uses an RP2350 flight controller. A Raspberry Pi or Jetson companion is being sought. Figures distinguish measured evidence, analytic examples and proposed designs. Physical motor outputs remain disabled; flight acceptance is pending.

<p align="center"><img src="report/assets/f450-red-white-concept.png" width="600" alt="Red and white F450 concept illustration"></p>

*AI concept illustration; measured evidence and proposed tests are labelled in the report.*

## Supporting evidence

- [Materials inventory](report/bill-of-materials.csv)
- [Requirements](report/requirements-register.json)
- [Evidence provenance](report/evidence-manifest.json)
- [Inspected source identity](report/source-inventory.json)
- [PDF verification](report/validation.json)
- `evidence/`: selected published recordings-derived reports/data, including failures; not a replacement for private raw recordings.

## Rebuild

Python 3, ReportLab, Matplotlib, NumPy, Pillow and Windows Arial fonts are required.

```powershell
python -m pip install -r requirements.txt
python tools/build_report.py .
python tools/check_report.py
```

Student and supervisor names remain to complete. The programme identity is confirmed by the [official FGE listing](https://fge.usthb.dz/pages/Orientation-2026-2027). The [official RSI curriculum](https://fge.usthb.dz/storage/disk/Robotique-et-systemes-intelligents-MAJ-2025.pdf), PDF p. 11, lists **Projet Personnel Professionnel**, RSI7.8, semester 7, 2 credits and coefficient 1. The teacher's marking rubric remains to obtain.
