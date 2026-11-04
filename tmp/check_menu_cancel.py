from pathlib import Path
import serial
import time
import json

r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
output=r/'artifacts/acquisition-comparison/xbox-menu/20261030T125921Z/cancel-original.log'
device=serial.Serial()
device.port='COM3'
device.baudrate=921600
device.timeout=.5
device.dtr=False
device.rts=False
seen=[]
try:
    device.open()
    device.reset_input_buffer()
    device.write(b'd')
    time.sleep(.15)
    device.write(b'x')
    with output.open('xb') as log:
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            raw=device.readline()
            log.write(raw)
            if raw.startswith(b'START,') or raw.startswith(b'DONE,'):
                seen.append(raw.decode().strip())
            if raw.startswith(b'DONE,'):
                break
    assert seen==['START,d','DONE,d,USER_ABORT'], seen
    result={'engine':'Real ESP32-S3; serial d then x through slow core','observed':seen,'physical_Xbox_B_tested':False,'motor_outputs_enabled':False}
    (r/'measurements/software/2026-10-30-xbox-test-menu/cancellation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result)
finally:
    device.close()
