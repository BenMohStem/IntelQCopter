"""Install the pinned official Windows QEMU into this checkout's ignored cache."""
from pathlib import Path
import hashlib
import tarfile
import urllib.request
import sys

ROOT=Path(__file__).resolve().parents[2]
URL='https://github.com/espressif/qemu/releases/download/esp-develop-9.2.2-20260417/qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-x86_64-w64-mingw32.tar.xz'
SHA='3c483d77f5350a568df1faf4d8dbc82c95d6bc2b826d0d4be910485e0a68ca2a'
DEST=ROOT/'build/qemu'
if sys.platform!='win32': raise SystemExit('This installer pins the Windows x86-64 build. Use Espressif tools for another host.')
DEST.mkdir(parents=True,exist_ok=True)
archive=DEST/'qemu.tar.xz'
if not archive.exists():
    print('Downloading',URL,flush=True)
    urllib.request.urlretrieve(URL,archive.with_suffix('.download'))
    archive.with_suffix('.download').replace(archive)
if hashlib.sha256(archive.read_bytes()).hexdigest()!=SHA:
    raise SystemExit('QEMU archive checksum mismatch; remove only the failed download and retry.')
with tarfile.open(archive) as bundle:
    for member in bundle.getmembers():
        target=(DEST/member.name).resolve()
        if not target.is_relative_to(DEST.resolve()) or not (member.isfile() or member.isdir()):
            raise SystemExit('Unsafe archive member: '+member.name)
    bundle.extractall(DEST)
print('Installed',DEST/'qemu/bin/qemu-system-xtensa.exe')
