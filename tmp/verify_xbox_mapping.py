from pathlib import Path
import hashlib
import time
import serial
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
source=r/'lessons/xbox-test-menu/main.cpp'
sha=hashlib.sha256(source.read_bytes()).hexdigest()
compile((r/'tools/validation/record_xbox_menu.py').read_text(), 'record_xbox_menu.py', 'exec')
d=serial.Serial()
d.port='COM3'
d.baudrate=921600
d.timeout=.5
d.dtr=False
d.rts=False
lines=[]
try:
    d.open()
    d.reset_input_buffer()
    d.write(b'?i')
    until=time.monotonic()+3
    while time.monotonic()<until:
        raw=d.readline()
        if raw:
            lines.append(raw.decode('ascii',errors='replace').strip())
    d.write(b't')
finally:
    d.close()
assert any(line.startswith('ACQ_MENU_V1,'+sha) for line in lines), lines
assert 'INPUT_MODE,MONITOR_ONLY_NO_TESTS' in lines,lines
result={'engine':'Actual ESP32-S3 serial query and monitor-only command','source_sha256':sha,'identity_and_mode':[line for line in lines if line.startswith(('ACQ_MENU_V1,','CORE,','INPUT_MODE,'))],'input_rows_observed':sum(line.startswith('INPUT,') for line in lines),'physical_button_validation':'pending operator review; no physical button actions inferred'}
(r/'measurements/software/2026-10-30-xbox-mapping/boot.json').write_text(json.dumps(result,indent=2)+'\n')
print(result)
