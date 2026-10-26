"""Native schematic/PCB agreement and end-to-end logical wire-link verification."""
import argparse,json,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import pcbnew as p
arg=argparse.ArgumentParser();arg.add_argument('project',type=Path);arg.add_argument('--netlist',type=Path,required=True);arg.add_argument('--drc',type=Path,required=True);arg.add_argument('--report',type=Path);args=arg.parse_args()
O=args.project
d=json.loads((O/'connections.json').read_text())
xml=ET.parse(args.netlist).getroot()
components={c.attrib['ref']:c for c in xml.findall('./components/comp')}
netlist={}
for net in xml.findall('./nets/net'):
    for node in net.findall('node'):
        key=(node.attrib['ref'],node.attrib['pin'])
        assert key not in netlist
        netlist[key]=net.attrib['name']
b=p.LoadBoard(str(O/'QCopter.kicad_pcb'))
fps={f.GetReference():f for f in b.GetFootprints()}
assert len(components)==d['expected_components']==17+len(d['wire_links'])
assert len(fps)==len(components)+4
assert sum(len(list(f.Pads())) for f in fps.values())==98+2*len(d['wire_links'])+4
assert all(t.GetLayer()==p.B_Cu and type(t)==p.PCB_TRACK and p.ToMM(t.GetWidth())>=.6 for t in b.GetTracks())
assert len(list(b.Zones()))==1
zone=list(b.Zones())[0]
assert zone.GetLayer()==p.B_Cu and zone.GetNetname()=='GND'
assert zone.GetPadConnection()==p.ZONE_CONNECTION_FULL and zone.HasFilledPolysForLayer(p.B_Cu)
parents={n:n for n in set(netlist.values())}
def root(n):
    while parents[n]!=n:parents[n]=parents[parents[n]];n=parents[n]
    return n
for c in d['components']:
    ref=c['reference'];f=fps[ref]
    assert ref in components and f.GetPath().AsString().endswith('/'+c['uuid'])
    assert bool(f.GetAttributes()&p.FP_DNP)==c['dnp']
    for pad in f.Pads():
        pin=pad.GetNumber();expected=c['pin_nets'].get(pin,'')
        assert pad.GetNetname()==expected,(ref,pin,'board net')
        if expected:assert netlist[(ref,pin)]==expected,(ref,pin,'schematic net')
    if ref.startswith('W'):
        a=netlist[(ref,'1')];bb=netlist[(ref,'2')]
        parents[root(a)]=root(bb)
groups={}
for c in d['components']:
    if 'logical_pin_nets' not in c:continue
    for pin,logical in c['logical_pin_nets'].items():
        physical=root(netlist[(c['reference'],pin)])
        groups.setdefault(logical,set()).add(physical)
assert all(len(v)==1 for v in groups.values()),'Missing functional connection through fitted wires'
assert len(set(next(iter(v)) for v in groups.values()))==len(groups),'Different logical nets became shorted'
assert len({root(netlist[('U1',pin)]) for pin in ['1','21','22']})==3
for i in range(1,5):
    assert root(netlist[('J'+str(i+1),'2')])!=root(netlist[('JP'+str(i),'2')]),'BEC hardwired'
drc=json.loads(args.drc.read_text())
assert not drc['violations'] and not drc['unconnected_items'] and not drc.get('schematic_parity',[])
report={'engine':'KiCad 10.0.6 native DRC and native XML netlist; independent Python graph verification',
        'board_sha256':hashlib.sha256((O/'QCopter.kicad_pcb').read_bytes()).hexdigest(),
        'schematic_sha256':hashlib.sha256((O/'QCopter.kicad_sch').read_bytes()).hexdigest(),
        'schematic_components':len(components),'mounting_holes':4,'pcb_pads_total':98+2*len(d['wire_links'])+4,'fitted_insulated_links':len(d['wire_links']),
        'bottom_tracks':len(list(b.GetTracks())),'electrical_logical_nets':len(groups),'native_drc_violations':0,'native_unconnected_items':0,
        'no_top_copper_tracks_or_vias':True,'voltage_rails_and_four_bec_feeds_separate':True,
        'all_original_logical_connections_preserved_with_W_links':True,
        'solid_ground_zone_connections':True,'physical_fit_and_etched_continuity_verified':False,
        'limitations':['Modules must use elevated sockets; low insulated wires pass beneath them',
                       'No verified actual-clone dimensions, BEC output or F450 mounting pattern',
                       'One copper face means solder every lead on underside; through holes are unplated',
                       'No actuator, flight, battery-current or hardware acceptance']}
(args.report or O/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(f"PASS: {len(components)} schematic components + 4 mounts; {98+2*len(d['wire_links'])+4} holes/pads; all {len(d['wire_links'])} fitted wire links verified.")
print('PASS: all logical connections, separate 3V3/5V/GND/BEC nets; B.Cu only; solid GND pour.')
print('PASS: native KiCad DRC 0 violations / 0 unconnected items. Physical tests pending.')
