"""Embed exact source/configuration identity in the board's boot banner."""
Import('env')
from pathlib import Path
import hashlib
root=Path(env['PROJECT_DIR']);h=hashlib.sha256()
for name in ['firmware/main.cpp','platformio.ini','config/hardware.json','tools/stamp_build.py']:
    h.update(name.encode()+b'\0'+(root/name).read_bytes()+b'\0')
env.Append(CPPDEFINES=[('BENCH_SOURCE_SHA',env.StringifyMacro(h.hexdigest()))])
