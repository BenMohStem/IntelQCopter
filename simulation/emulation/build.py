"""Build emulator firmware and merge its flash image; never flash real hardware."""
from pathlib import Path
import argparse
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--pio',default=shutil.which('pio') or str(Path.home()/'.platformio/penv/Scripts/platformio.exe'))
args=parser.parse_args()
out=ROOT/'build';out.mkdir(exist_ok=True)
with (out/'emulation-build-latest.log').open('w',encoding='utf8') as log:
    proc=subprocess.Popen([args.pio,'run','-d',str(ROOT/'simulation/emulation')],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf8',errors='replace')
    for line in proc.stdout: print(line,end='');log.write(line)
    if proc.wait(): raise SystemExit('Emulator build failed; see build/emulation-build-latest.log')
# Use PlatformIO's own Python to load its esptool dependencies.
pio_python=Path(args.pio).resolve().with_name('python.exe' if sys.platform=='win32' else 'python')
packages=Path.home()/'.platformio/packages'
firmware=ROOT/'build/emulation-pio/esp32-s3-qemu'
command=[str(pio_python),str(packages/'tool-esptoolpy/esptool.py'),'--chip','esp32s3','merge_bin',
    '--flash_mode','dio','--flash_size','8MB','--flash_freq','80m','--fill-flash-size','8MB',
    '-o',str(ROOT/'build/emulation-flash.bin'),
    '0x0',str(firmware/'bootloader.bin'),'0x8000',str(firmware/'partitions.bin'),
    '0xe000',str(packages/'framework-arduinoespressif32/tools/partitions/boot_app0.bin'),
    '0x10000',str(firmware/'firmware.bin')]
subprocess.run(command,cwd=ROOT,check=True)
