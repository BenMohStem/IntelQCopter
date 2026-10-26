"""Independent native schematic/PCB connection and power separation checks."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('project',type=Path)
parser.add_argument('--netlist',type=Path,required=True)
args=parser.parse_args()
manifest=json.loads((args.project/'connections.json').read_text(encoding='utf-8'))
xml=ET.parse(args.netlist).getroot()
components={c.attrib['ref']:c for c in xml.findall('./components/comp')}
parts={p.attrib['part']:p for p in xml.findall('./libparts/libpart')}
connections={}
for net in xml.findall('./nets/net'):
    for node in net.findall('node'):
        key=(node.attrib['ref'],node.attrib['pin'])
        if key in connections:
            raise ValueError('Pin appears in multiple nets: '+str(key))
        connections[key]=net.attrib['name']
board=pcbnew.LoadBoard(str(args.project/'QCopter.kicad_pcb'))
fps={f.GetReference():f for f in board.GetFootprints()}
assert len(fps)==len(components)==manifest['expected_components']==17
assert sum(len(list(f.Pads())) for f in fps.values())==manifest['expected_pads']==98
for item in manifest['components']:
    ref=item['reference']
    assert ref in components and ref in fps
    assert fps[ref].GetPath().AsString().endswith('/'+item['uuid'])
    assert bool(fps[ref].GetAttributes() & pcbnew.FP_DNP)==item['dnp'],(ref,'DNP differs')
    pads={p.GetNumber():p for p in fps[ref].Pads()}
    symbol_name=components[ref].find('libsource').attrib['part']
    pins={p.attrib['num']:p.attrib['name'] for p in parts[symbol_name].findall('./pins/pin')}
    assert set(pins)==set(pads)
    if ref!='U1':
        assert set(item['pin_nets'])==set(pins),'A module/connector pin was left unused'
    for pin,expected in item['pin_nets'].items():
        assert connections.get((ref,pin))==expected,(ref,pin,expected,connections.get((ref,pin)))
        assert pads[pin].GetNetname()==expected,(ref,pin,'PCB differs')
    for pin in set(pins)-set(item['pin_nets']):
        assert not pads[pin].GetNetname(),'Unexpected unused GPIO net'
def ends(name):
    return {key for key,value in connections.items() if value==name}
assert ends('I2C_SDA')=={('U1','15'),('U2','4'),('U3','4'),('U4','4'),('R1','2')}
assert ends('I2C_SCL')=={('U1','12'),('U2','3'),('U3','3'),('U4','3'),('R2','2')}
assert connections['U3','5']=='3V3' and connections['U3','6']=='GND'
assert connections['U2','7']=='GND'
assert ends('MPU_AUX_SDA')=={('U2','5'),('J6','4')}
assert ends('MPU_AUX_SCL')=={('U2','6'),('J6','3')}
assert ends('FC_5V')=={('U1','21'),('J1','1'),('JP1','2'),('JP2','2'),('JP3','2'),('JP4','2')}
for number in range(1,5):
    assert ends(f'ESC{number}_BEC')=={('J'+str(number+1),'2'),('JP'+str(number),'1')}
power_pads={p.GetNumber():p for p in fps['U1'].Pads()}
assert len({power_pads[p].GetNetCode() for p in ['1','21','22']})==3
u1=parts[components['U1'].find('libsource').attrib['part']]
gpio_names={pin.attrib['name']:pin.attrib['num'] for pin in u1.findall('./pins/pin')}
expected_gpio={'I2C_SDA':9,'I2C_SCL':8,'MPU_INT':16,'MAG_DRDY':17,'LIDAR_TO_FC_RX':6,'FC_TX_TO_LIDAR':7,
               'FC_UART_RX':4,'FC_UART_TX':5,'ESC1_SIGNAL':12,'ESC2_SIGNAL':1,'ESC3_SIGNAL':2,'ESC4_SIGNAL':14}
for name,gpio in expected_gpio.items():
    assert connections['U1',gpio_names['GPIO'+str(gpio)]]==name,(name,gpio)
assert not list(board.GetTracks()) and not list(board.Zones()),'This checker is for the unrouted reference'
print('PASS: 17 components / 98 pads agree between native schematic netlist and PCB.')
print('PASS: shared I2C, BMP mode/address straps, interrupts, UART directions, AUX isolation and 4 ESC GPIOs.')
print('PASS: BEC feeds on separate nets through four open links; 5V/3V3/GND distinct; no motor power path.')
print('LIMIT: connections are net assignments, not copper routing or physical/firmware acceptance; jumpers must not be fitted together.')
