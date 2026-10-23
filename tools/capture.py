"""Explicit serial capture; no upload, no motor command, no automatic acceptance."""
from pathlib import Path
import argparse,datetime,hashlib,json,time
import serial
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',required=True);p.add_argument('--seconds',type=float,default=30)
p.add_argument('--send',choices=list('pimblh012345wve'),default='p');p.add_argument('--bench-id',required=True)
a=p.parse_args();assert 0<a.seconds<=600
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');out=R/'artifacts/hardware'/stamp;out.mkdir(parents=True)
report=dict(port=a.port,baud=115200,requested_seconds=a.seconds,command=a.send,bench_id=a.bench_id,
            kind='physical_serial_capture',acceptance='UNREVIEWED',uploaded=False)
s=serial.Serial();s.port=a.port;s.baudrate=115200;s.timeout=.1;s.dtr=False;s.rts=False
start=time.monotonic();count=0
try:
    s.open()
    with (out/'serial.log').open('wb') as log:
        # Retain boot output; do not flush away an identity banner.
        sent=False
        while time.monotonic()-start<a.seconds:
            if not sent and time.monotonic()-start>=2:
                s.write(a.send.encode());s.flush();sent=True
            raw=s.read(max(1,s.in_waiting))
            if raw:
                log.write(raw);log.flush();count+=len(raw);print(raw.decode('utf8',errors='replace'),end='',flush=True)
    report['bytes']=count;report['actual_seconds']=time.monotonic()-start
    report['serial_sha256']=hashlib.sha256((out/'serial.log').read_bytes()).hexdigest()
except Exception as e:
    report['error']=str(e);raise
finally:
    s.close();(out/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    print('\nOriginal unmodified serial bytes:',out)
