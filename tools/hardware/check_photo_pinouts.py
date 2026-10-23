"""Independent intended header pinouts and GPIO nets, checked against native PCB."""
from pathlib import Path
import hashlib,json,pcbnew as k
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc';A=R/'artifacts/hardware/photo-headers-20261023'
b=k.LoadBoard(str(H/'esp32_home_etch.kicad_pcb'))
pads={(f.GetReference(),p.GetNumber()):p for f in b.GetFootprints() for p in f.Pads() if p.GetNumber()}
# From Espressif DevKitC-1 header table, not generated from board model.
expected={('J1','1'):'3V3',('J1','2'):'3V3',('J1','4'):'GPIO4',('J1','5'):'GPIO5',
 ('J1','6'):'GPIO6',('J1','7'):'GPIO7',('J1','10'):'GPIO17',('J1','11'):'GPIO18',
 ('J1','12'):'GPIO8',('J1','15'):'GPIO9',('J1','18'):'GPIO12',('J1','20'):'GPIO14',
 ('J1','21'):'5V_MCU',('J1','22'):'GND',('J2','1'):'GND',('J2','4'):'GPIO1',
 ('J2','5'):'GPIO2',('J2','21'):'GND',('J2','22'):'GND'}
# Signal labels from the supplied photos. Pin1 always starts at the VCC end.
headers={
 'J4':['SENS_3V3','GND','I2C_SCL','I2C_SDA',None,None,'GND',None],
 'J5':['SENS_3V3','GND','I2C_SCL','I2C_SDA','BARO_CSB','GND'],
 'J6':['SENS_3V3','GND','I2C_SCL','I2C_SDA',None],
 'J13':['GND','LIDAR_5V','LIDAR_RX_FC','LIDAR_TX_FC'],
 'J12':['COMP_TX','COMP_RX','GND']}
for ref,names in headers.items():
 assert len([p for (r,n),p in pads.items() if r==ref])==len(names),ref
 for i,net in enumerate(names,1):
  p=pads[(ref,str(i))]
  if net is None:assert p.GetNetname().startswith('unconnected-'),(ref,i,p.GetNetname())
  else:expected[(ref,str(i))]=net
  if ref in ['J4','J5','J6']:
   assert abs(k.ToMM(p.GetPosition().x)-(8+2.54*(i-1)))<1e-6
   assert abs(k.ToMM(p.GetDrillSize().x)-1.1)<1e-6
for key,net in expected.items():assert pads[key].GetNetname()==net,(key,net,pads[key].GetNetname())
for track in b.GetTracks():assert track.GetLayer()==k.B_Cu and type(track)==k.PCB_TRACK,'Only bottom straight tracks; no vias'
for z in b.Zones():assert z.GetLayer()==k.B_Cu and z.GetNetname()=='GND'
report=dict(engine='KiCad10.0.6 pcbnew native pad net and geometry inspection',passed=True,
 scope='Digital module header signal order and selected genuine DevKit header GPIOs; purchased board/body fit not measured',
 independent_expected_pin_nets=len(expected),sensor_pin_counts=[8,6,5],lidar_wire_pads=4,
 sensor_pitch_mm=2.54,only_bottom_copper_routes=True,no_vias=True,
 board_sha256=hashlib.sha256((H/'esp32_home_etch.kicad_pcb').read_bytes()).hexdigest(),
 physical_bench_test=False,source_references='sources.json')
(A/'pinout-verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2))
