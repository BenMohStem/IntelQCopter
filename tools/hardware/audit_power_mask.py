"""Check final rendered copper, explicit wire links and independent power-pin expectations.
Run with NumPy/SciPy/Pillow after Poppler rasterizes the final PDF at 300dpi.
This checks a digital design, not a manufactured board or module internals.
"""
from pathlib import Path
import argparse, hashlib, json
import numpy as np
from PIL import Image
from scipy.ndimage import label
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc'
p=argparse.ArgumentParser();p.add_argument('--image',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
d=json.loads((H/'output/native-copper-geometry.json').read_text(encoding='utf8'))
m=json.loads((H/'components-and-nets.json').read_text(encoding='utf8'))
im=np.asarray(Image.open(a.image).convert('L'));sy,sx=im.shape[0]/297,im.shape[1]/210
regions,count=label(im<128);pads={(q['ref'],q['pin']):q for q in d['pads']}
def sample(q,dx=0,dy=0,left=5,top=37):
 return round((top+q['xy'][1]+dy)*sy),round((left+q['xy'][0]+dx)*sx)
def region(q):
 y,x=sample(q,(q['diameter']+q['drill'])/4);v=int(regions[y,x]);assert v,('missing copper',q);return v
net_regions={};region_nets={}
for q in pads.values():
 v=region(q);net_regions.setdefault(q['net'],set()).add(v);region_nets.setdefault(v,set()).add(q['net'])
assert all(len(s)==1 for s in net_regions.values()),('broken net',net_regions)
assert all(len(s)==1 for s in region_nets.values()),('bare-copper short',region_nets)
# Independent power-pin table: reference DevKitC-1, not generated from the model.
expected={('J1','1'):'3V3',('J1','2'):'3V3',('J1','21'):'5V_MCU',('J1','22'):'GND',
 ('J2','1'):'GND',('J2','21'):'GND',('J2','22'):'GND',('J3','1'):'GND',('J3','2'):'5V_EXT',
 ('J12','3'):'GND',('J13','1'):'GND',('J13','2'):'LIDAR_5V',('J14','1'):'GND',('J14','2'):'SENS_3V3'}
for j in ['J4','J5','J6']:expected.update({(j,'1'):'SENS_3V3',(j,'2'):'GND',(j,'3'):'I2C_SCL',(j,'4'):'I2C_SDA'})
expected.update({('J4','7'):'GND',('J5','5'):'BARO_CSB',('J5','6'):'GND'})
for key,net in expected.items():assert pads[key]['net']==net,(key,net,pads[key])
def assembled(remove=()):
 parent={v:v for v in region_nets}
 def root(v):
  while parent[v]!=v:v=parent[v]
  return v
 for link in m['wire_links']:
  if link['ref'] in remove:continue
  x=region(pads[(link['ref'],'1')]);y=region(pads[(link['ref'],'2')]);parent[root(x)]=root(y)
 return {net:root(next(iter(v))) for net,v in net_regions.items()}
on=assembled();usb=assembled(['W21'])
assert on['3V3']==on['SENS_3V3']==on['BARO_CSB']
signal_bridges={'GPIO9':'I2C_SDA','GPIO8':'I2C_SCL','GPIO6':'LIDAR_RX_FC',
 'GPIO7':'LIDAR_TX_FC','GPIO17':'COMP_TX','GPIO18':'COMP_RX','GPIO4':'VBAT_ADC',
 'GPIO5':'CURRENT_ADC','GPIO12':'GPIO12_ESC','GPIO14':'GPIO14_ESC'}
for gpio,signal in signal_bridges.items():assert on[gpio]==on[signal],('missing signal jumper',gpio,signal)
assert len({on[n] for n in ['I2C_SDA','I2C_SCL','LIDAR_RX_FC','LIDAR_TX_FC','COMP_TX','COMP_RX','VBAT_ADC','CURRENT_ADC','GPIO12_ESC','GPIO14_ESC','GND','3V3','5V_EXT']})==13,'assembled signal/rail short'
assert on['5V_EXT']==on['5V_MCU']==on['LIDAR_5V']
assert len({on[n] for n in ['GND','3V3','5V_EXT','VBAT_SENSE']})==4,'assembled rail short'
assert usb['5V_EXT']!=usb['5V_MCU'] and usb['5V_EXT']==usb['LIDAR_5V']
assert len({usb[n] for n in ['GND','3V3','5V_EXT','5V_MCU','VBAT_SENSE']})==5
holes=0
for left,top in [(5,37),(105,37),(55,147)]:
 for q in pads.values():
  y,x=sample(q,left=left,top=top);assert im[y,x]>245,('drill centre is not white',q,left,top);holes+=1
native=(H/'esp32_home_etch.kicad_pcb').read_text(encoding='utf8')
ground_count=sum(q['net']=='GND' for q in pads.values())
assert native.count('(zone_connect 2)')==ground_count
assert '(connect_pads yes' in native
report=dict(engine='Poppler 300dpi PDF + SciPy connected black pixels + explicit jumper graph + independent reference power-pin table',
 scope='Digital copper and carrier wiring only; component internals, regulator operation, manufactured board, sensor variants and fit unvalidated',
 passed=True,copper_nets=len(net_regions),pads=len(pads),ground_pads_solid=ground_count,white_drill_centres_verified=holes,
 all_nets_continuous_on_mask=True,signal_bridges_checked=signal_bridges,assembled_signal_groups_separate=True,no_different_copper_nets_short=True,wire_links=len(m['wire_links']),
 assembled_GND_3V3_5V_pack_separate=True,usb_W21_isolation_verified=True,expected_power_pins_checked=len(expected),
 pdf_sha256=hashlib.sha256((H/'output/pdf/etch-mask-3-copies-A4-1to1.pdf').read_bytes()).hexdigest(),
 board_sha256=d['source_sha256'],assumptions=['All fourteen insulated jumpers fitted as wire-links.csv; W21 removed for USB power',
 'No resistors, capacitors or DevKit regulator are modelled as zero-ohm wires',
 'Exact genuine DevKitC-1 and TOFSense-F2 Mini reference pinouts; pigtail adapters mapped by signal',
 'No physical voltage, current, insulation, thermal, RF or flight test performed'])
a.output.mkdir(parents=True,exist_ok=True);(a.output/'mask-power-audit.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(report,indent=2))
