# Native KiCad 3D assembly

Open the PCB and press **Alt+3**. Keep this folder beside the board; all models
use `${KIPRJMOD}` relative paths. No additional model download is needed.
The four module bodies, ESC headers, lidar wire stubs and three new ground
jumpers are included. Jumper holes are outside the module courtyards.

Models are illustrative: nominal body dimensions and connector placement,
assumed 8.5 mm sockets and module PCB bottoms at 9 mm. G1 runs beneath part
of the raised BMP module; fit the wire first, keep it below the module and
verify the actual socket clearance. Exact clone dimensions/heights are unverified.
Wire colours are illustrative, not a factory lidar cable pinout.

Use `.\qcopter.ps1 render-pcb-3d` in the Stepwise checkout to reproduce native
previews. `build_drilled_jumpers.py` creates a fresh revision from the prior one;
do not run the old overlay-model generator on this new project.

[Top](../output/3d-top.png) · [Bottom](../output/3d-bottom.png) ·
[Native renderer log](../output/3d-native-render.log)
