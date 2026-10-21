"""Extract supported copper primitives from the actual KiCad board; KiCad Python required."""
from pathlib import Path
import hashlib,json,pcbnew as p
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc'
source=H/'esp32_home_etch.kicad_pcb';b=p.LoadBoard(str(source))
def xy(point): return [round(p.ToMM(point.x),6),round(p.ToMM(point.y),6)]
tracks=[];pads=[]
zones=[]
for zone in b.Zones():
    if zone.GetLayer()!=p.B_Cu or zone.GetNetname()!='GND':raise RuntimeError('Unexpected zone layer/net')
    if not zone.HasFilledPolysForLayer(p.B_Cu):raise RuntimeError('Unfilled GND zone')
    filled=zone.GetFilledPolysList(p.B_Cu)
    def points(chain):return [xy(chain.CPoint(i)) for i in range(chain.PointCount())]
    for i in range(filled.OutlineCount()):
        zones.append(dict(outline=points(filled.COutline(i)),holes=[points(filled.CHole(i,h)) for h in range(filled.HoleCount(i))]))
for t in b.GetTracks():
    if t.GetLayer()!=p.B_Cu or type(t)!=p.PCB_TRACK: raise RuntimeError('Only straight B.Cu tracks supported')
    tracks.append(dict(start=xy(t.GetStart()),end=xy(t.GetEnd()),width=p.ToMM(t.GetWidth())))
for f in b.GetFootprints():
    for a in f.Pads():
        if a.GetAttribute()==p.PAD_ATTRIB_NPTH: continue
        if not a.IsOnLayer(p.B_Cu) or a.GetShape()!=p.PAD_SHAPE_CIRCLE: raise RuntimeError('Only circular B.Cu pads supported')
        size=xy(a.GetSize());drill=xy(a.GetDrillSize())
        if size[0]!=size[1] or drill[0]!=drill[1]: raise RuntimeError('Noncircular pad/drill')
        pads.append(dict(ref=f.GetReference(),pin=a.GetNumber(),xy=xy(a.GetPosition()),diameter=size[0],drill=drill[0]))
# Independent match against the original coordinate model: do not print stale geometry.
model=json.loads((H/'components-and-nets.json').read_text())
expected={(c['ref'],a['number']):(a['x'],a['y'],c['pad_diameter_mm'],c['drill_mm']) for c in model['components'] for a in c['pads']}
actual={(a['ref'],a['pin']):(*a['xy'],a['diameter'],a['drill']) for a in pads}
if expected.keys()!=actual.keys() or any(any(abs(x-y)>1e-5 for x,y in zip(expected[k],actual[k])) for k in expected): raise RuntimeError('Model and native pad geometry differ')
def key(t):return (*sorted([tuple(round(v,6) for v in t['start']),tuple(round(v,6) for v in t['end'])]),round(t['width'],6))
if sorted(map(key,tracks))!=sorted(map(key,model['bottom_tracks'])):
    print('native only',set(map(key,tracks))-set(map(key,model['bottom_tracks'])))
    print('model only',set(map(key,model['bottom_tracks']))-set(map(key,tracks)))
    raise RuntimeError('Model and native tracks differ')
out=H/'output';out.mkdir(exist_ok=True)
data=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source='esp32_home_etch.kicad_pcb',units='mm',board_size=[100,100],tracks=tracks,pads=pads,zones=zones,geometry_matches_coordinate_model=True)
(out/'native-copper-geometry.json').write_bytes((json.dumps(data,indent=2)+'\n').encode())
print('Extracted native B.Cu:',len(tracks),'tracks,',len(pads),'circular pads;',len(zones),'GND polygons; model agreement PASS')
