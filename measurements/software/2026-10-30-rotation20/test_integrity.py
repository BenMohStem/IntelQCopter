import sys
import struct
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/validation'))
from record_xbox_menu import summarize_rotation
rows=[[i,1000,1,-2,3] for i in range(20000)]
checksum=2166136261
for row in rows:
    for byte in struct.pack('>3h',*row[2:]):checksum=((checksum^byte)*16777619)&0xffffffff
metadata={'elapsed_us':20000000,'samples':20000,'checksum':checksum,'tail_bytes':0}
assert summarize_rotation(rows,metadata)['read_rate_hz']==1000
controls=[(rows,{**metadata,'checksum':0}), (rows,{**metadata,'elapsed_us':4000000}), (rows,{**metadata,'samples':19999}), (rows,{**metadata,'tail_bytes':1024})]
duplicate=[row[:] for row in rows]
duplicate[10][0]=9
controls.append((duplicate,metadata))
gap=[row[:] for row in rows]
gap[0][1]=0
controls.append((gap,metadata))
for bad,meta in controls:
    try:summarize_rotation(bad,meta)
    except ValueError:continue
    raise AssertionError('Corruption accepted')
print('PASS: valid 20-second synthetic stream and 6 malformed controls; no physical motion acceptance')
