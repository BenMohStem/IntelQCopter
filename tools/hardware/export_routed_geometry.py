"""Extract the manufactured B.Cu and drilled holes from native KiCad objects."""
import argparse,json,hashlib,csv
from pathlib import Path
import pcbnew as p
arg=argparse.ArgumentParser();arg.add_argument('project',type=Path);args=arg.parse_args();O=args.project
b=p.LoadBoard(str(O/'QCopter.kicad_pcb'))
xy=lambda q:[round(p.ToMM(q.x),6),round(p.ToMM(q.y),6)]
tracks=[];pads=[];holes=[];zones=[]
for f in b.GetFootprints():
    for a in f.Pads():
        assert a.GetShape()==p.PAD_SHAPE_CIRCLE and a.GetSize().x==a.GetSize().y
        row=dict(ref=f.GetReference(),pin=a.GetNumber(),net=a.GetNetname(),xy=xy(a.GetPosition()),diameter=p.ToMM(a.GetSize().x),drill=p.ToMM(a.GetDrillSize().x))
        if row['drill']>0:holes.append(row)
        if a.GetAttribute()!=p.PAD_ATTRIB_NPTH:pads.append(row)
for t in b.GetTracks():
    assert t.GetLayer()==p.B_Cu and type(t)==p.PCB_TRACK
    tracks.append(dict(start=xy(t.GetStart()),end=xy(t.GetEnd()),width=p.ToMM(t.GetWidth())))
def points(chain):return [xy(chain.CPoint(i)) for i in range(chain.PointCount())]
for z in b.Zones():
    assert z.GetLayer()==p.B_Cu and z.HasFilledPolysForLayer(p.B_Cu)
    fill=z.GetFilledPolysList(p.B_Cu)
    for i in range(fill.OutlineCount()):zones.append(dict(outline=points(fill.COutline(i)),holes=[points(fill.CHole(i,h)) for h in range(fill.HoleCount(i))]))
out=O/'output';out.mkdir(exist_ok=True)
data=dict(source='QCopter.kicad_pcb',source_sha256=hashlib.sha256((O/'QCopter.kicad_pcb').read_bytes()).hexdigest(),units='mm',board_size=json.loads((O/'connections.json').read_text()).get('board_mm',[90,90]),tracks=tracks,pads=pads,holes=holes,zones=zones)
(out/'native-copper-geometry.json').write_text(json.dumps(data,indent=2)+'\n')
with (out/'drill-map.csv').open('w',newline='') as stream:
    writer=csv.writer(stream);writer.writerow(['Reference','Pin','X_mm','Y_mm','Drill_mm','Net'])
    for a in sorted(holes,key=lambda a:(a['ref'],a['pin'])):writer.writerow([a['ref'],a['pin'],*a['xy'],a['drill'],a['net']])
print(len(tracks),'bottom tracks;',len(pads),'copper annuli;',len(holes),'drills;',len(zones),'filled GND contours')
