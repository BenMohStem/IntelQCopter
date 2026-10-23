"""Receive real serial output only. Never upload or send sensor/erase commands."""
from pathlib import Path
import argparse,datetime,json,time
import serial
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',required=True);p.add_argument('--seconds',type=float,default=15)
a=p.parse_args();assert 0<a.seconds<=600
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
out=R/'artifacts/hardware'/stamp;out.mkdir(parents=True)
device=serial.Serial();device.port=a.port;device.baudrate=115200;device.timeout=.1
device.dtr=False;device.rts=False
report=dict(port=a.port,baud=115200,acceptance='UNREVIEWED',transmitted_commands=False)
try:
    device.open();start=time.monotonic()
    with (out/'serial.log').open('wb') as log:
        while time.monotonic()-start<a.seconds:
            raw=device.read(max(1,device.in_waiting))
            if raw:
                log.write(raw);log.flush()
                print(raw.decode('utf8',errors='replace'),end='',flush=True)
except Exception as error:
    report['error']=str(error);raise
finally:
    device.close();(out/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Original serial evidence:',out)
