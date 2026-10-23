"""Compile without uploading; preserve original output and exact inputs."""
from pathlib import Path
import datetime,hashlib,json,subprocess,sys
R=Path(__file__).resolve().parents[1]
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
out=R/'artifacts/build'/stamp;out.mkdir(parents=True)
pio=Path.home()/'.platformio/penv/Scripts/platformio.exe'
command=[str(pio),'run','-e','bench']
with (out/'compiler.log').open('wb') as log:
    process=subprocess.Popen(command,cwd=R,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    for line in iter(process.stdout.readline,b''):
        log.write(line);log.flush();sys.stdout.buffer.write(line);sys.stdout.buffer.flush()
    code=process.wait()
sources=['firmware/main.cpp','platformio.ini','config/hardware.json','tools/stamp_build.py']
hashes={p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in sources}
h=hashlib.sha256()
for p in sources:h.update(p.encode()+b'\0'+(R/p).read_bytes()+b'\0')
report=dict(engine='PlatformIO espressif32@7.1.3 Arduino / ESP32-S3-DevKitC-1 bench build',command=command,exit_code=code,
            source_sha256=hashes,embedded_source_sha=h.hexdigest(),uploaded=False,physical_test=False)
binary=R/'.pio/build/bench/firmware.bin'
if code==0:report['firmware_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
(out/'build.json').write_text(json.dumps(report,indent=2)+'\n')
print('Original build evidence:',out)
raise SystemExit(code)
