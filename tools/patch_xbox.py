"""Add a guarded raw-report observer to the pinned, downloaded Xbox library.

Only .pio/libdeps is modified. This avoids racing on the library's public,
mutable xboxNotif object between the BLE callback and flight-control tasks.
The C++ adapter declares QCOPTER_XBOX_REPORT_HOOK before including the header.
"""
from pathlib import Path
Import("env")
base=Path(env.subst("$PROJECT_LIBDEPS_DIR"))/env.subst("$PIOENV")
header=base/"XboxSeriesXControllerESP32_asukiaaa/src/XboxSeriesXControllerESP32_asukiaaa.hpp"
if not header.is_file():
    raise RuntimeError("Resolve pinned dependencies first: pio pkg install")
text=header.read_text(encoding="utf-8")
marker="#ifdef QCOPTER_XBOX_REPORT_HOOK"
needle="      xboxNotif.update(pData, length);"
if marker not in text:
    if text.count(needle)!=1:
        raise RuntimeError("Xbox dependency changed; review report hook before building")
    replacement=needle+"\n#ifdef QCOPTER_XBOX_REPORT_HOOK\n      QCOPTER_XBOX_REPORT_HOOK(pData, length);\n#endif"
    header.write_text(text.replace(needle,replacement),encoding="utf-8",newline="\n")
    print("Added guarded raw Xbox report observer to local dependency cache")
