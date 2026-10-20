# Active code and evidence map

| Start here | Responsibility |
|---|---|
| firmware/src/flight_controller.c | One C implementation for calibration, estimation, control, allocation and battery math |
| firmware/include/fc_api.h | Small public platform boundary |
| firmware/src/esp32_platform.cpp | Real-board I²C, Xbox, scheduling, serial telemetry and NVS |
| emulation/scenario.c | Synthetic input/check scenario executing on emulated Xtensa |
| emulation/main.cpp | Arduino startup for that scenario |
| emulation/flight_core.c | Compiles the active C file, without a copied implementation |
| tools/model_lti.py | Mathematical LTI calculations and figures; independent of QEMU |
| tools/knowledge_db.py | Validated project knowledge, history, evidence and search |
| tools/render_math_guides.cjs | Offline MathML reading view |

The C file is grouped into six numbered sections: battery math; calibration and
pressure conversion; control/estimator state; discrete PID/Kalman updates; control
step and allocation; public API and calibration records. These stay in one C file
as requested. The platform adapter is C++ because the selected hardware libraries
expose C++ interfaces.

Open `IntelQCopter.code-workspace` to focus the editor on active files. Its explorer
hides downloaded/build caches and the large historical RP2350 tree; search also
excludes archives and generated reading views. The historical files remain on disk
and in Git, accessible by path or by changing the editor exclusions.

`build/` is disposable local cache: emulator executable, firmware binaries,
documentation packages and the SQLite search index. `artifacts/` stores original
logs and reports. Do not treat evidence as cache or delete failed runs to improve
the apparent success rate. `archive/` retains earlier project material; the
original main branch remains its runnable reference.

The previous checked-in SQLite file is replaced by `build/knowledge/project.sqlite`.
It is generated from versioned JSON and no longer creates binary Git changes for
every knowledge update. Old schema/data remain recoverable from commit bb93f11.
