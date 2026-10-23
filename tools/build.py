"""Build the current lesson without uploading; retain original compiler output."""
from pathlib import Path
import datetime,hashlib,json,subprocess,sys
R=Path(__file__).resolve().parents[1]
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
out=R/'artifacts/build'/stamp;out.mkdir(parents=True)
sources=['firmware/main.cpp','platformio.ini','config/hardware.json']
before={p:(R/p).read_bytes() for p in sources}
pio=Path.home()/'.platformio/penv/Scripts/platformio.exe'
command=[str(pio),'run','-e','bench']
with (out/'compiler.log').open('wb') as log:
    process=subprocess.Popen(command,cwd=R,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    for line in iter(process.stdout.readline,b''):
        log.write(line);log.flush();sys.stdout.buffer.write(line);sys.stdout.buffer.flush()
    code=process.wait()
unchanged=all((R/p).read_bytes()==data for p,data in before.items())
report=dict(engine='PlatformIO espressif32@7.1.3 Arduino ESP32-S3 lesson build',
    command=command,exit_code=code,source_sha256={p:hashlib.sha256(v).hexdigest() for p,v in before.items()},
    inputs_unchanged_during_build=unchanged,uploaded=False,physical_test=False,
    embedded_source_fingerprint=False)
binary=R/'.pio/build/bench/firmware.bin'
if code==0 and unchanged:report['firmware_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
(out/'build.json').write_text(json.dumps(report,indent=2)+'\n')
print('Original build evidence:',out)
if not unchanged:raise SystemExit('Source changed during build; no matching-build claim')
raise SystemExit(code)
