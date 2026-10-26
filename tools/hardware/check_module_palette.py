"""Cross-check native KiCad netlist pins against the actual loaded PCB pads."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('project',type=Path)
parser.add_argument('--netlist',type=Path)
args=parser.parse_args()
manifest=json.loads((args.project/'parts.json').read_text(encoding='utf-8'))
netlist=ET.parse(args.netlist or args.project/'parts.xml').getroot()
components={c.attrib['ref']:c for c in netlist.findall('./components/comp')}
libraries={c.attrib['part']:c for c in netlist.findall('./libparts/libpart')}
board=pcbnew.LoadBoard(str(args.project/'QCopter.kicad_pcb'))
footprints={fp.GetReference():fp for fp in board.GetFootprints()}
assert len(components)==len(footprints)==len(manifest['components'])==9
for item in manifest['components']:
    reference=item['reference']
    component=components[reference]
    assert component.findtext('footprint')==item['library_id']
    part=component.find('libsource').attrib['part']
    pins={p.attrib['num']:p.attrib['name'] for p in libraries[part].findall('./pins/pin')}
    assert pins==item['pins'],reference
    fp=footprints[reference]
    identifier=fp.GetFPID()
    assert str(identifier.GetLibNickname())+':'+str(identifier.GetLibItemName())==item['library_id']
    assert {p.GetNumber() for p in fp.Pads()}==set(pins)
    assert fp.GetPath().AsString().endswith('/'+item['schematic_uuid'])
assert sum(len(list(fp.Pads())) for fp in footprints.values())==79
assert not list(board.GetTracks()) and not list(board.Zones())
print('PASS: 9 matching schematic/PCB components, 79 matching pin names/numbers/pads, UUID links preserved; no tracks/zones.')
