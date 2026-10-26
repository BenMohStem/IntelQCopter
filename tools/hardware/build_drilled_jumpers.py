"""Create a new revision with three dedicated two-hole insulated GND jumpers.
Run with KiCad's bundled Python; previous revisions are never overwritten.
"""
from pathlib import Path
import argparse, json, re, shutil, uuid, math, hashlib, sys
import pcbnew as p

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/hardware'))
from enrich_module_bodies import children
from create_parts import symbol, prop
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();S=args.source;O=args.output
assert not O.exists(),'Use a new directory to preserve the previous revision'
O.mkdir(parents=True)
for name in ['QCopter.kicad_pcb','QCopter.kicad_sch','QCopter.kicad_pro','QCopterModules.kicad_sym','fp-lib-table','sym-lib-table','connections.json','pin-audit.md']:
    shutil.copy2(S/name,O/name)
shutil.copytree(S/'models3d',O/'models3d',ignore=shutil.ignore_patterns('Ground_wires_assembly_visual.wrl','README.md','models.json'))
library=O/'QCopterModules.pretty';library.mkdir()
for f in (S/'QCopterModules.pretty').glob('*.kicad_mod'):
    if not f.name.startswith('Ground_Bridge'):shutil.copy2(f,library/f.name)
text=(O/'QCopter.kicad_pcb').read_text();edits=[]
for a,b in children(text):
    part=text[a:b]
    if not part.startswith('(footprint '):continue
    if re.search(r'\(property "Reference" "G[12]"',part):edits.append((a,b,''));continue
    if '(property "Reference" "J1"' in part:
        for ma,mb in reversed(children(part)):
            if part[ma:mb].startswith('(model ') and 'Ground_wires_assembly' in part[ma:mb]:part=part[:ma]+part[mb:]
        edits.append((a,b,part))
for a,b,v in reversed(edits):text=text[:a]+v+text[b:]
(O/'QCopter.kicad_pcb').write_text(text)
f=library/'Lidar_Free_Solder_Lands.kicad_mod';text=f.read_text()
for a,b in reversed(children(text)):
    if text[a:b].startswith('(model ') and 'Ground_wires_assembly' in text[a:b]:text=text[:a]+text[b:]
f.write_text(text)
b=p.LoadBoard(str(O/'QCopter.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
sheet=fps['U1'].GetPath().AsString().split('/')[1]
v=lambda xy:p.VECTOR2I(*[p.FromMM(q) for q in xy])
def track(net,points):
    for a,c in zip(points,points[1:]):
        t=p.PCB_TRACK(b);t.SetLayer(p.B_Cu);t.SetWidth(p.FromMM(1.2));t.SetNet(b.FindNet(net));t.SetStart(v(a));t.SetEnd(v(c));b.Add(t)

links=[dict(reference='G1',name='Ground_Jumper_G1',local='GND_SENSOR_RETURN',start=[112,79],end=[118.5,60.5],branch=[[110,79],[112,79]],purpose='BMP280 + magnetometer ground return'),
       dict(reference='G2',name='Ground_Jumper_G2',local='GND_ESC_RETURN',start=[103,108.6],end=[103,117.4],branch=[[103,108.8],[103,108.6]],purpose='ESP right GND + ESC4 ground return'),
       dict(reference='G3',name='Ground_Jumper_G3',local='GND_LEFT_ESC_RETURN',start=[55,108],end=[55,117.4],branch=[[58,107],[56,107],[55,108]],purpose='ESC3 isolated ground return')]
manifest=json.loads((S/'connections.json').read_text());comps=[c for c in manifest['components'] if c['reference'] not in ['G1','G2']]
def wire_model(name,dx,dy):
    # A coloured insulated wire with bare terminal legs, in KiCad legacy VRML units.
    text=['#VRML V2.0 utf8\n# Original illustrative fitted insulated jumper; units=0.1 inch']
    def tube(a,c,r,color):
        t=[c[k]-a[k] for k in range(3)];ln=math.sqrt(sum(x*x for x in t));t=[x/ln for x in t]
        h=[0,0,1] if abs(t[2])<.9 else [1,0,0]
        cross=lambda a,c:[a[1]*c[2]-a[2]*c[1],a[2]*c[0]-a[0]*c[2],a[0]*c[1]-a[1]*c[0]]
        u=cross(t,h);ln=math.sqrt(sum(x*x for x in u));u=[x/ln for x in u];w=cross(t,u);verts=[]
        for q in [a,c]:
            for i in range(12):
                angle=2*math.pi*i/12;verts.append([q[k]+r*(math.cos(angle)*u[k]+math.sin(angle)*w[k]) for k in range(3)])
        faces=[list(reversed(range(12))),list(range(12,24))]+[[i,(i+1)%12,(i+1)%12+12,i+12] for i in range(12)]
        coord=', '.join(f'{x/2.54:.6f} {-y/2.54:.6f} {z/2.54:.6f}' for x,y,z in verts)
        ids=', '.join(' '.join(map(str,reversed(f)))+' -1' for f in faces)
        text.append(f'Shape {{ appearance Appearance {{ material Material {{ diffuseColor {color} }} }} geometry IndexedFaceSet {{ coord Coordinate {{ point [ {coord} ] }} coordIndex [ {ids} ] }} }}')
    for x,y in [(0,0),(dx,dy)]:tube((x,y,-3),(x,y,1.5),.25,'.7 .7 .7');tube((x,y,1.5),(x,y,4),.45,'.1 .3 .8')
    tube((0,0,4),(dx,dy,4),.45,'.1 .3 .8')
    (O/'models3d'/name).write_text('\n'.join(text)+'\n')

for i,link in enumerate(links):
    ref=link['reference'];name=link['name'];a,c=link['start'],link['end'];dx=c[0]-a[0];dy=c[1]-a[1]
    wire_model(name+'.wrl',dx,dy)
    fp=f'''(footprint "{name}" (version 20240108) (generator "pcbnew") (layer "F.Cu") (attr through_hole)
    (descr "Dedicated two-hole insulated GND jumper. Fit wire above component side. 3.2mm round lands, 1.2mm drill.")
    (property "Reference" "REF**" (at {dx/2-2:g} {dy/2:g} 0) (layer "Dwgs.User") (effects (font (size 1 1))))
    (property "Value" "FIT_INSULATED_GND_WIRE" (at {dx/2:g} {dy/2+2:g} 0) (layer "F.Fab") (effects (font (size .8 .8))) hide)
    (fp_line (start 0 0) (end {dx:g} {dy:g}) (stroke (width .3) (type default)) (layer "Dwgs.User"))
    (pad "1" thru_hole circle (at 0 0) (size 3.2 3.2) (drill 1.2) (layers "*.Cu" "*.Mask"))
    (pad "2" thru_hole circle (at {dx:g} {dy:g}) (size 3.2 3.2) (drill 1.2) (layers "*.Cu" "*.Mask"))
    (model "${{KIPRJMOD}}/models3d/{name}.wrl" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0))))'''
    (library/(name+'.kicad_mod')).write_text(fp+'\n')
    f=p.FootprintLoad(str(library),name);f.SetReference(ref);f.SetValue('FIT_INSULATED_GND_WIRE');f.SetFPID(p.LIB_ID('QCopterModules',name));f.SetPosition(v(a))
    for pad in f.Pads():pad.SetNet(b.FindNet(link['local'] if pad.GetNumber()=='1' else 'GND'))
    uid=str(uuid.uuid4());f.SetPath(p.KIID_PATH('/'+sheet+'/'+uid));b.Add(f);fps[ref]=f
    track(link['local'],link['branch'])
    comps.append(dict(reference=ref,name=name,value='FIT_INSULATED_GND_WIRE',dnp=False,uuid=uid,symbol_xy=[65+105*i,250],board_xy=a,angle=0,pin_nets={'1':link['local'],'2':'GND'},dedicated_drilled_jumper=True,drill_mm=1.2,pad_diameter_mm=3.2))
# Rebuild matching native schematic with actual physical wire components.
defs={};source=(S/'QCopterModules.kicad_sym').read_text()
needed={c['name'] for c in comps}
for a,c in children(source):
    part=source[a:c]
    if not part.startswith('(symbol '):continue
    name=re.match(r'\(symbol "([^"]+)"',part)[1]
    if name in needed:defs[name]={m[2]:m[1] for m in re.finditer(r'\(pin passive line .*?\(name "([^"]+)".*?\(number "([^"]+)"',part,re.S)}
for link in links:defs[link['name']]={'1':'ISOLATED_GND','2':'COMMON_GND'}
(O/'QCopterModules.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator "kicad_symbol_editor")\n'+'\n'.join(symbol(n,q) for n,q in defs.items())+')\n')
cached='\n'.join(symbol(n,q).replace('(symbol '+json.dumps(n),'(symbol '+json.dumps('QCopterModules:'+n),1) for n,q in defs.items())
sch=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet}) (paper "A3") (lib_symbols {cached})']
for c in comps:
    ref,name=c['reference'],c['name'];sx,sy=c['symbol_xy'];pins=defs[name];top=27.94 if len(pins)==44 else (len(pins)-1)*1.27;h=top+2.54
    sch += [f'(symbol (lib_id "QCopterModules:{name}") (at {sx} {sy} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {c["uuid"]})',prop('Reference',ref,sx,sy-h-5.08),prop('Value',c['value'],sx,sy-h-2.54),prop('Footprint','QCopterModules:'+name,sx,sy,True),f'(instances (project "QCopter" (path "/{sheet}" (reference "{ref}") (unit 1)))))']
    for i,pin in enumerate(pins):
        side=1 if len(pins)==44 and i>=22 else -1;row=i-22 if side==1 else i;x=sx+side*20.32;y=sy-top+row*2.54
        if pin not in c['pin_nets']:sch.append(f'(no_connect (at {x:g} {y:g}) (uuid {uuid.uuid4()}))');continue
        ex=x+side*5.08;sch.append(f'(wire (pts (xy {x:g} {y:g}) (xy {ex:g} {y:g})) (stroke (width 0) (type default)) (uuid {uuid.uuid4()}))');sch.append(f'(global_label "{c["pin_nets"][pin]}" (shape bidirectional) (at {ex:g} {y:g} 0) (effects (font (size 1.016 1.016)) (justify {"left" if side==1 else "right"})) (uuid {uuid.uuid4()}))')
sch.append('(embedded_fonts no))');(O/'QCopter.kicad_sch').write_text('\n'.join(sch))
p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(O/'QCopter.kicad_pcb'),b)
# Keep the source revision's explicit home-etch rules after native serialization.
(O/'QCopter.kicad_pro').write_bytes((S/'QCopter.kicad_pro').read_bytes())
manifest.update(components=comps,wire_links=links,expected_components=12,expected_pads=85,drilled_holes=85,status='Native validation pending; fit three separate insulated GND jumpers in six dedicated holes')
(O/'connections.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Built three two-hole GND jumpers, short 1.2mm copper branches, six new 1.2mm drilled holes. Validation pending.')
