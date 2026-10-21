"""Boot the merged ESP32-S3 image and retain raw UART, checks and source hashes."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--name', default='qemu-replay')
parser.add_argument('--timeout', type=float, default=45)
args = parser.parse_args()
if not 1 <= args.timeout <= 300: raise SystemExit('Timeout must be 1..300 seconds')
if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', args.name):
    raise SystemExit('Use a plain unique run name')
output = ROOT/'artifacts/emulation'/args.name
output.mkdir(parents=True, exist_ok=False)  # Never overwrite evidence.
qemu = ROOT/'build/qemu/qemu/bin/qemu-system-xtensa.exe'
flash = ROOT/'build/emulation-flash.bin'
command = [str(qemu), '-machine','esp32s3','-display','none','-monitor','none',
           '-serial','stdio','-no-reboot','-snapshot',
           '-drive',f'file={flash.as_posix()},if=mtd,format=raw']
finished = threading.Event()
rows, checks, lines = [], [], []
start = time.monotonic()
with (output/'uart.log').open('w', encoding='utf8', buffering=1) as log:
    proc = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding='utf8',errors='replace')
    def receive():
        for line in proc.stdout:
            log.write(line)
            lines.append(line)
            print(line, end='', flush=True)
            if line.startswith('QCOPTER_EMU_END,'): finished.set()
    thread = threading.Thread(target=receive, daemon=True)
    thread.start()
    completed = finished.wait(args.timeout)
    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    thread.join(timeout=5)
for line in lines:
    if line.startswith('CHECK,'):
        _, name, result = line.strip().split(',')
        checks.append({'name':name,'result':result})
    if line.startswith('DATA,'): rows.append(next(csv.reader([line.strip()]))[1:])
with (output/'telemetry.csv').open('w', newline='',encoding='utf8') as f:
    writer=csv.writer(f)
    writer.writerow(['time_us','phase','roll_deg','pitch_deg','height_m','FL','FR','RR','RL','fault','warn','disarm'])
    writer.writerows(rows)
files = ['firmware/src/flight_controller.cpp','firmware/include/fc_api.h',
         'simulation/emulation/scenario.c','simulation/emulation/main.cpp','simulation/emulation/flight_core.cpp','simulation/emulation/platformio.ini',
         'build/emulation-pio/esp32-s3-qemu/firmware.elf','build/emulation-flash.bin',
         'build/qemu/qemu/bin/qemu-system-xtensa.exe']
summary={'engine':'Espressif QEMU ESP32-S3','command':command,'completed':completed,
         'checks':checks,'host_elapsed_seconds':time.monotonic()-start,
         'hashes':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files},
         'scope':'Active C core compiled for Xtensa; synthetic input replay, not flight physics, BLE, sensor registers or real-time timing.'}
end=next((re.fullmatch(r'QCOPTER_EMU_END,checks=(\d+),failures=(\d+),ticks=(\d+)\s*',line) for line in lines if line.startswith('QCOPTER_EMU_END,')),None)
summary['passed']=bool(completed and end and int(end[1])==len(checks) and int(end[2])==0
    and {c['name'] for c in checks}==set(re.findall(r'check\("([^"]+)"',(ROOT/'simulation/emulation/scenario.c').read_text(encoding='utf8')))
    and len({c['name'] for c in checks})==len(checks)
    and all(c['result']=='PASS' for c in checks))
(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
print('Saved',output)
raise SystemExit(0 if summary['passed'] else 1)
