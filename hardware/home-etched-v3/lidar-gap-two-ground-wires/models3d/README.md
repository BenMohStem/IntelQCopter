# Portable 3D assembly models

Open `../QCopter.kicad_pcb` in PCB Editor, then press **Alt+3** (View > 3D Viewer).
If already open, reload the updated board first, preserving any unsaved edits.
The project must remain beside `models3d/`; model paths use `${KIPRJMOD}`.
No extra model download or global KiCad path setup is required.

Included: ESP32-S3 DevKitC-1, MPU6050/GY-521, BMP280, GY-271, four
3-pin ESC headers, four remote lidar wire stubs and both underside GND wires.
G2 includes the required middle solder tap. The lidar itself is remote.

These original meshes are **illustrative**, not manufacturer STEP models.
Pin positions/pitch and board envelopes match the current reference footprints.
Sensor envelopes are 21.2 x 16.4 mm (MPU), 15.5 x 11.8 mm (BMP), and
13.5 x 14.8 mm (GY-271). See the original [dimension sources](../../../module-library/module-dimensions.md).
Heights, chip details, antenna artwork and USB shells are approximate; sockets
are assumed 8.5 mm high, with module PCB bottoms 9 mm above the carrier.
Exact clone fit, socket height and connector/cable clearance need measurement.
The coloured lidar wires illustrate connections, not a verified factory cable colour order.
The ground-wire overlay is specific to this placement; regenerate after moving parts.

[Native KiCad top render](../output/3d-top.png) · [underside](../output/3d-bottom.png)

Recreate the renders from the repository root:
```powershell
.\qcopter.ps1 render-pcb-3d
```

`tools/hardware/add_module_3d.py` regenerates meshes and model references using
KiCad's bundled Python. It edits only model blocks. The original renderer log,
hashes and electrical-preservation proof are under `output/3d-*`.
Rendering does not validate mechanical fit, sensor interfaces or flight.
