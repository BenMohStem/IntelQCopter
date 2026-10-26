"""Native netlist/pad agreement and assembled common-GND wire graph checks.
No upload, motor output, emulation or hardware acceptance is performed.
"""
import argparse,json,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import pcbnew as p
arg=argparse.ArgumentParser();arg.add_argument('project',type=Path);arg.add_argument('--netlist',type=Path,required=True);arg.add_argument('--drc',type=Path,required=True);arg.add_argument('--report',type=Path,required=True);args=arg.parse_args()
O=args.project;d=json.loads((O/'connections.json').read_text());xml=ET.parse(args.netlist).getroot()
by_pin={}
for n in xml.findall('./nets/net'):
    for pin in n.findall('node'):by_pin[(pin.attrib['ref'],pin.attrib['pin'])]=n.attrib['name']
b=p.LoadBoard(str(O/'QCopter.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
assert len(fps)==len(d['components'])==12
assert sum(len(list(f.Pads())) for f in fps.values())==86
holes=[a for f in fps.values() for a in f.Pads() if a.GetDrillSize().x>0]
assert len(holes)==86
assert all(t.GetLayer()==p.B_Cu and type(t)==p.PCB_TRACK and p.ToMM(t.GetWidth())>=1.2 for t in b.GetTracks())
assert all(z.GetLayer()==p.B_Cu and z.GetNetname() in ['GND','GND_RF_RETURN'] and z.GetPadConnection()==p.ZONE_CONNECTION_FULL for z in b.Zones() if not z.GetIsRuleArea())
parents={n:n for n in by_pin.values()}
def root(n):
    while parents[n]!=n:parents[n]=parents[parents[n]];n=parents[n]
    return n
bridge_refs=[]
for c in d['components']:
    ref=c['reference'];f=fps[ref]
    assert f.GetPath().AsString().endswith('/'+c['uuid'])
    for a in f.Pads():
        pin=a.GetNumber();actual=a.GetNetname();expected=c['pin_nets'].get(pin)
        if expected:assert actual==expected==by_pin[(ref,pin)],(ref,pin)
        else:assert actual==by_pin[(ref,pin)] and actual.startswith('unconnected-'),(ref,pin,'Unexpected unused GPIO connection')
    if c.get('dedicated_drilled_jumper'):
        bridge_refs.append(ref)
        assert len(list(f.Pads()))==(3 if ref=='G2' else 2)
        nets=list(c['pin_nets'].values())
        assert all(n.startswith('GND') for n in nets)
        for a in f.Pads():
            assert abs(p.ToMM(a.GetDrillSize().x)-1.2)<1e-6
            assert abs(p.ToMM(a.GetSize().x)-3.2)<1e-6
            assert a.GetAttribute()==p.PAD_ATTRIB_PTH
            assert sum(orig.GetPosition()==a.GetPosition() for orig in holes)==1,'Jumper must have its own drill hole'
        for n in nets[1:]:parents[root(nets[0])]=root(n)
assert set(bridge_refs)=={'G1','G2','G3'}
assert len(set((a.GetPosition().x,a.GetPosition().y) for a in holes))==86

groups={};gndpoints=[]
for c in d['components']:
    if 'logical_pin_nets' not in c:continue
    for pin,logical in c['logical_pin_nets'].items():
        groups.setdefault(logical,set()).add(root(by_pin[(c['reference'],pin)]))
        if logical=='GND':gndpoints.append(c['reference']+'.'+pin)
assert all(len(v)==1 for v in groups.values()),('Missing functional connection',groups)
assert len(set(next(iter(v)) for v in groups.values()))==len(groups),'Different logical nets shorted'
assert len(groups['GND'])==1
for ref in ['J5','J2','J3']:assert root(by_pin[(ref,'2')])!=root(by_pin[('U1','21')]),'Unselected BEC is paralleled'
assert root(by_pin[('J4','2')])==root(by_pin[('U1','21')])==root(by_pin[('J1','1')])
assert len({root(by_pin[('U1',pin)]) for pin in ['1','21','22']})==3
assert by_pin[('U3','5')]=='3V3' and root(by_pin[('U3','6')])==root(by_pin[('U1','22')])
assert root(by_pin[('U2','7')])==root(by_pin[('U1','22')])
gpio={'MPU_SDA':9,'MPU_SCL':8,'SLOW_SDA':41,'SLOW_SCL':40,'MPU_INT':13,'MAG_DRDY':47,'LIDAR_TO_FC_RX':10,'FC_TX_TO_LIDAR':11,'ESC1_SIGNAL':4,'ESC2_SIGNAL':1,'ESC3_SIGNAL':14,'ESC4_SIGNAL':21}
physical={'MPU_SDA':'15','MPU_SCL':'12','SLOW_SDA':'38','SLOW_SCL':'37','MPU_INT':'19','MAG_DRDY':'28','LIDAR_TO_FC_RX':'16','FC_TX_TO_LIDAR':'17','ESC1_SIGNAL':'4','ESC2_SIGNAL':'41','ESC3_SIGNAL':'20','ESC4_SIGNAL':'27'}
for n,pin in physical.items():assert by_pin[('U1',pin)]==n
# Primary Espressif GPIO matrix documentation lists these all as general I/O.
assert set(gpio.values()).isdisjoint({0,3,19,20,26,27,28,29,30,31,32,33,34,35,36,37,38,43,44,45,46,48})
assert len(set(gpio.values()))==len(gpio)
positions={a.GetNumber():[round(p.ToMM(a.GetPosition().x),3),round(p.ToMM(a.GetPosition().y),3)] for a in fps['J1'].Pads()}
assert positions=={'1':[47.0,95.5],'2':[55.0,95.5],'3':[59.0,95.5],'4':[64.08,95.5]}
rf=json.loads((O/'rf-clearance.json').read_text())
keepouts=[z for z in b.Zones() if z.GetIsRuleArea()]
assert len(keepouts)==1 and keepouts[0].GetDoNotAllowZoneFills() and keepouts[0].GetDoNotAllowTracks() and keepouts[0].GetDoNotAllowPads() and keepouts[0].GetDoNotAllowVias()
assert keepouts[0].GetLayerSet().Contains(p.F_Cu) and keepouts[0].GetLayerSet().Contains(p.B_Cu)
for z in b.Zones():
    if z.GetIsRuleArea():continue
    fill=z.GetFilledPolysList(p.B_Cu)
    assert all(p.ToMM(fill.COutline(i).CPoint(k).y)>=49-1e-6 for i in range(fill.OutlineCount()) for k in range(fill.COutline(i).PointCount())),'Copper in antenna edge strip'
assert all(p.ToMM(min(t.GetStart().y,t.GetEnd().y))-p.ToMM(t.GetWidth())/2>=49 for t in b.GetTracks())
assert all(p.ToMM(a.GetPosition().y)-p.ToMM(a.GetSize().y)/2>=49 for f in fps.values() for a in f.Pads())
drc=json.loads(args.drc.read_text())
assert not drc['violations'] and not drc['unconnected_items'] and not drc.get('schematic_parity')
pro=json.loads((O/'QCopter.kicad_pro').read_text());assert pro['board']['design_settings']['rules']['min_clearance']>=.5
assert all(c['clearance']>=.5 for c in pro['net_settings']['classes'])
report={'engine':'KiCad 10 native DRC+schematic parity, native XML netlist, independent pad/wire graph checks',
 'board_sha256':hashlib.sha256((O/'QCopter.kicad_pcb').read_bytes()).hexdigest(),'schematic_sha256':hashlib.sha256((O/'QCopter.kicad_sch').read_bytes()).hexdigest(),
 'cad_checks_pass':True,'violations':0,'unconnected_items':0,'schematic_parity_issues':0,'copper_layers_routed':['B.Cu'],
 'minimum_track_mm':min(p.ToMM(t.GetWidth()) for t in b.GetTracks()),'explicit_minimum_clearance_mm':.5,
 'physical_drill_holes':86,'electrical_pad_records':86,'power_jumpers':0,'continuous_ground_wires':3,'ground_wire_solder_taps':[2,3,2],'dedicated_jumper_holes':7,
 'all_ground_nodes_common_with_ground_wires_fitted':gndpoints,'voltage_rails_and_unselected_becs_separate':True,
 'gpio_map':gpio,'capability_check':'Selected GPIOs support GPIO-matrix I2C/UART/LEDC PWM. GPIO40/41 must not also be used for external JTAG.',
 'physical_fit_printer_scale_power_and_interfaces_tested':False,'flashed_or_armed':False,
 'limits':['Three separate GND wires are mandatory, G1/G3 use pairs; G2 uses three dedicated holes with a middle tap; no shared module/header holes','Verify actual clone dimensions, cable order, BEC 5V/current and 3.3V signal levels','Bench firmware MPU bus matches; future peripheral/PWM map differs and has not been flashed']}
args.report.write_text(json.dumps(report,indent=2)+'\n')
print('PASS: native DRC/parity all 0; all',len(gndpoints),'logical GND points common through the three dedicated-hole jumpers.')
print('PASS: 86 drill holes, no power jumper, only B.Cu tracks >=1.2mm, explicit 0.5mm copper clearance.')
print('PASS: I2C/UART/PWM GPIO eligibility and no strap/flash/USB GPIO collisions. Hardware/firmware integration pending.')
