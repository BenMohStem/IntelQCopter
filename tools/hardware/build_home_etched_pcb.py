"""Turn reviewed route geometry into a native, self-contained KiCad project.
Wire links are real two-terminal schematic components, not hidden copper layers.
"""
import sys,json,uuid,shutil,math,re,argparse
from pathlib import Path
import pcbnew as p
R=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/'tools/hardware'))
from create_parts import symbol,prop
from enrich_module_bodies import children
uid=lambda:str(uuid.uuid4())
S=.254
arg=argparse.ArgumentParser();arg.add_argument('--output',type=Path,required=True);arg.add_argument('--routing',type=Path,default=R/'hardware/home-etched-v2/route-model.json');args=arg.parse_args()
O=args.output
if O.exists():raise RuntimeError('Never overwrite an existing design trial')
shutil.copytree(R/'hardware/connected-i2c',O)
for name in ['preview','verification.json','DRC-summary.json','validation.log']:
    target=O/name
    if target.is_dir():shutil.rmtree(target)
    elif target.exists():target.unlink()
d=json.loads(args.routing.read_text())
manifest=json.loads((O/'connections.json').read_text())
comps=manifest['components']
tracks=[]
def compress(points):
    out=[]
    for pt in points:
        if out and pt==out[-1]:continue
        if len(out)>1:
            a,b=out[-2:]
            if (a[0]==b[0]==pt[0]) or (a[1]==b[1]==pt[1]):out[-1]=pt;continue
        out.append(pt)
    return out
for route in d['routes']:
    run=[]
    def flush():
        pts=compress(run)
        for a,b in zip(pts,pts[1:]):tracks.append({'a':a,'b':b,'r':route['width']/2,'net':route['net']})
    for layer,x,y in route['path']:
        if layer==0:run.append([round(x*S,6),round(y*S,6)])
        elif run:flush();run=[]
    if run:flush()
links=[]
for i,wire in enumerate(d['wire_links'],1):
    ref='W'+str(i)
    a,b=wire['start'],wire['end']
    if a==b:raise RuntimeError('Zero length wire')
    links.append({'reference':ref,'net':wire['net'],'start':a,'end':b})
# Exact union of bottom copper primitives (orthogonal segments and circular pads).
items=[]
for pad in d['pads']:
    items.append(dict(a=pad['xy'],b=pad['xy'],r=pad['radius'],net=pad['net'],key=(pad['ref'],pad['pin'])))
for w in links:
    for pin,xy in [('1',w['start']),('2',w['end'])]:items.append(dict(a=xy,b=xy,r=1.2,net=w['net'],key=(w['reference'],pin)))
track_offset=len(items)
items.extend(tracks)
parent=list(range(len(items)))
def root(i):
    while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
    return i
def merge(i,j):parent[root(i)]=root(j)
def ptseg(q,a,b):
    dx=b[0]-a[0];dy=b[1]-a[1]
    t=0 if dx==dy==0 else max(0,min(1,((q[0]-a[0])*dx+(q[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(q[0]-a[0]-t*dx,q[1]-a[1]-t*dy)
def separation(a,b):
    if all(max(min(a['a'][k],a['b'][k]),min(b['a'][k],b['b'][k]))<=min(max(a['a'][k],a['b'][k]),max(b['a'][k],b['b'][k])) for k in [0,1]):return 0
    return min(ptseg(a['a'],b['a'],b['b']),ptseg(a['b'],b['a'],b['b']),ptseg(b['a'],a['a'],a['b']),ptseg(b['b'],a['a'],a['b']))
for i,a in enumerate(items):
    for j in range(i):
        b=items[j]
        if a['net'] and a['net']==b['net'] and separation(a,b)<=a['r']+b['r']+.00001:merge(i,j)
islands={}
for i,a in enumerate(items):
    if not a['net']:continue
    roots=islands.setdefault(a['net'],[])
    rt=root(i)
    if rt not in roots:roots.append(rt)
def net_of(i):
    a=items[i]
    if not a['net']:return ''
    n=islands[a['net']].index(root(i))
    return a['net'] if n==0 else a['net']+'__LINK_'+str(n)
padnets={a['key']:net_of(i) for i,a in enumerate(items) if 'key' in a}
for c in comps:
    c['logical_pin_nets']=dict(c['pin_nets'])
    c['pin_nets']={pin:padnets[(c['reference'],pin)] for pin in c['pin_nets']}
    c['board_xy']=d['positions'][c['reference']]
library=O/'QCopterModules.pretty'
defs={}
text=(O/'QCopterModules.kicad_sym').read_text()
for a,b in children(text):
    child=text[a:b]
    if child.startswith('(symbol '):
        name=re.match(r'\(symbol "([^"]+)"',child)[1]
        pins={}
        for pin in re.finditer(r'\(pin passive line .*?\(name "([^"]+)".*?\(number "([^"]+)"',child,re.S):pins[pin[2]]=pin[1]
        defs[name]=pins
for w in links:
    name='Insulated_Link_'+w['reference']
    defs[name]={'1':'FROM','2':'TO'}
    dx=w['end'][0]-w['start'][0];dy=w['end'][1]-w['start'][1]
    fp=f'''(footprint "{name}" (version 20240108) (generator "pcbnew") (layer "F.Cu")
    (attr through_hole) (descr "Insulated wire on component side; no top copper trace. Route wire clear of modules and USB.")
    (property "Reference" "REF**" (at 0 -2 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))
    (property "Value" "INSULATED_WIRE" (at 0 0 0) (layer "F.Fab") hide)
    (fp_line (start 0 0) (end {dx} {dy}) (stroke (width 0.2) (type default)) (layer "Dwgs.User"))
    (pad "1" thru_hole circle (at 0 0) (size 2.4 2.4) (drill 1.1) (layers "*.Cu" "*.Mask"))
    (pad "2" thru_hole circle (at {dx} {dy}) (size 2.4 2.4) (drill 1.1) (layers "*.Cu" "*.Mask")))'''
    # Property hide must be inside effects in native KiCad syntax.
    fp=fp.replace('(layer "F.Fab") hide)','(layer "F.Fab") (effects (font (size 0.8 0.8)) hide))')
    (library/(name+'.kicad_mod')).write_text(fp)
    comps.append({'reference':w['reference'],'name':name,'value':'INSULATED_WIRE','dnp':False,'uuid':uid(),'board_xy':w['start'],
                  'symbol_xy':[440+90*((int(w['reference'][1:])-1)//10),50+25*((int(w['reference'][1:])-1)%10)],
                  'pin_nets':{pin:padnets[(w['reference'],pin)] for pin in ['1','2']}})
# Make round copper annuli for every drill. Pin 1 remains identified in assembly files.
for path in library.glob('*.kicad_mod'):
    text=path.read_text().replace('thru_hole rect','thru_hole circle').replace('"F.SilkS"','"F.Fab"')
    if path.stem.startswith(('ESP32_','MPU6050_','BMP280_','Magnetometer_')):
        # Courtyard at the carrier's socket plane. Full module envelope remains
        # on F.Fab; raised modules clear low, insulated flying wires underneath.
        spans=[]
        for a,bb in children(text):
            chunk=text[a:bb]
            if chunk.startswith('(fp_rect ') and '"F.CrtYd"' in chunk:spans.append((a,bb))
        for a,bb in reversed(spans):text=text[:a]+text[bb:]
        count=44 if path.stem.startswith('ESP32_') else 8 if path.stem.startswith('MPU6050_') else 6 if path.stem.startswith('BMP280_') else 5
        rows=[(0,22),(22.86,22)] if count==44 else [(0,count)]
        extra=''
        for x,n in rows:extra+=f'\n(fp_rect (start {x-1.5} -1.5) (end {x+1.5} {(n-1)*2.54+1.5}) (stroke (width 0.05) (type default)) (fill none) (layer "F.CrtYd"))\n'
        if count==44:
            extra+='\n(fp_rect (start -1.27 -1.4) (end 24.13 61.34) (stroke (width 0.15) (type default)) (fill none) (layer "F.Fab"))\n'
            extra+='\n(fp_rect (start 2.54 -8) (end 20.32 -1.4) (stroke (width 0.15) (type default)) (fill none) (layer "F.Fab"))\n'
        text=text.rstrip()[:-1]+extra+')\n'
    path.write_text(text)
all_defs='\n'.join(symbol(name,pins) for name,pins in defs.items())
(O/'QCopterModules.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator "kicad_symbol_editor")\n'+all_defs+')\n')
cached='\n'.join(symbol(name,pins).replace('(symbol '+json.dumps(name),'(symbol '+json.dumps('QCopterModules:'+name),1) for name,pins in defs.items())
sheet=uid()
sch=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet}) (paper "A1") (lib_symbols {cached})']
b=p.BOARD()
netnames=sorted({net for c in comps for net in c['pin_nets'].values()})
nets={}
for name in netnames:
    net=p.NETINFO_ITEM(b,name);b.Add(net);nets[name]=net
for c in comps:
    ref,name=c['reference'],c['name'];sx,sy=c['symbol_xy'];pin_names=defs[name]
    top=27.94 if len(pin_names)==44 else (len(pin_names)-1)*1.27;height=top+2.54
    sch.extend([f'(symbol (lib_id "QCopterModules:{name}") (at {sx} {sy} 0) (unit 1) (in_bom yes) (on_board yes) (dnp {"yes" if c["dnp"] else "no"}) (uuid {c["uuid"]})',
                prop('Reference',ref,sx,sy-height-5.08),prop('Value',c['value'],sx,sy-height-2.54),prop('Footprint','QCopterModules:'+name,sx,sy,True),
                f'(instances (project "QCopter" (path "/{sheet}" (reference "{ref}") (unit 1)))))'])
    for index,pin in enumerate(pin_names):
        side=1 if len(pin_names)==44 and index>=22 else -1;row=index-22 if side==1 else index
        x=sx+side*20.32;y=sy-top+row*2.54
        if pin not in c['pin_nets']:sch.append(f'(no_connect (at {x:g} {y:g}) (uuid {uid()}))');continue
        endpoint=x+side*5.08
        sch.append(f'(wire (pts (xy {x:g} {y:g}) (xy {endpoint:g} {y:g})) (stroke (width 0) (type default)) (uuid {uid()}))')
        sch.append(f'(global_label "{c["pin_nets"][pin]}" (shape bidirectional) (at {endpoint:g} {y:g} 0) (effects (font (size 1.016 1.016)) (justify {"left" if side==1 else "right"})) (uuid {uid()}))')
    fp=p.FootprintLoad(str(library),name);fp.SetReference(ref);fp.SetValue(c['value']);fp.SetFPID(p.LIB_ID('QCopterModules',name))
    if c['dnp']:fp.SetAttributes(fp.GetAttributes()|p.FP_DNP)
    fp.SetPosition(p.VECTOR2I(*[p.FromMM(v) for v in c['board_xy']]))
    fp.SetPath(p.KIID_PATH('/'+sheet+'/'+c['uuid']))
    for pad in fp.Pads():
        if pad.GetNumber() in c['pin_nets']:pad.SetNet(nets[c['pin_nets'][pad.GetNumber()]])
    b.Add(fp)
for note,y in [('SINGLE B.Cu CARRIER - W1 onward are insulated fitted wire links, NOT top copper.',15),('JP1-JP4: FIT AT MOST ONE VERIFIED 5V BEC. REMOVE ALL BEFORE USB. MOTOR CURRENT ON SEPARATE PDB.',20),('Reference module bodies only; verify actual hardware against a 100% paper fit before etching.',25)]:
    sch.append(f'(text {json.dumps(note)} (at 15 {y} 0) (effects (font (size 1.27 1.27)) (justify left)) (uuid {uid()}))')
sch.append('(embedded_fonts no))')
(O/'QCopter.kicad_sch').write_text('\n'.join(sch)+'\n')
for i,t in enumerate(tracks):
    tr=p.PCB_TRACK(b);tr.SetStart(p.VECTOR2I(*[p.FromMM(v) for v in t['a']]));tr.SetEnd(p.VECTOR2I(*[p.FromMM(v) for v in t['b']]))
    tr.SetWidth(p.FromMM(t['r']*2));tr.SetLayer(p.B_Cu);tr.SetNet(nets[net_of(track_offset+i)]);b.Add(tr)
for a,bb in [((0,0),(90,0)),((90,0),(90,90)),((90,90),(0,90)),((0,90),(0,0))]:
    shape=p.PCB_SHAPE(b);shape.SetShape(p.SHAPE_T_SEGMENT);shape.SetStart(p.VECTOR2I(*[p.FromMM(v) for v in a]));shape.SetEnd(p.VECTOR2I(*[p.FromMM(v) for v in bb]));shape.SetLayer(p.Edge_Cuts);shape.SetWidth(p.FromMM(.05));b.Add(shape)
for i,(x,y) in enumerate([(4,4),(86,4),(4,86),(86,86)],1):
    fp=p.FOOTPRINT(b);fp.SetReference('H'+str(i));fp.SetValue('M3_CLEARANCE_NO_FRAME_PATTERN');fp.SetPosition(p.VECTOR2I(p.FromMM(x),p.FromMM(y)))
    pad=p.PAD(fp);pad.SetAttribute(p.PAD_ATTRIB_NPTH);pad.SetShape(p.PAD_SHAPE_CIRCLE);pad.SetSize(p.VECTOR2I(p.FromMM(3.2),p.FromMM(3.2)));pad.SetDrillSize(p.VECTOR2I(p.FromMM(3.2),p.FromMM(3.2)));pad.SetLayerSet(p.LSET.AllCuMask());pad.SetPosition(fp.GetPosition());fp.Add(pad);b.Add(fp)
z=p.ZONE(b);z.SetLayer(p.B_Cu);z.SetNet(nets['GND']);z.SetLocalClearance(p.FromMM(.5));z.SetPadConnection(p.ZONE_CONNECTION_FULL);z.SetMinThickness(p.FromMM(.6));z.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS)
poly=z.Outline();poly.NewOutline()
for x,y in [(1,10.16),(39,10.16),(39,1),(89,1),(89,89),(1,89)]:poly.Append(p.FromMM(x),p.FromMM(y))
b.Add(z)
p.SaveBoard(str(O/'QCopter.kicad_pcb'),b)
project=json.loads((O/'QCopter.kicad_pro').read_text())
settings=project['board']['design_settings']
settings['rules']['min_clearance']=.5
settings['rules']['min_track_width']=.6
settings['rules']['min_hole_clearance']=.5
settings['rules']['min_copper_edge_clearance']=.5
for cls in project['net_settings']['classes']:cls.update(clearance=.5,track_width=.6)
(O/'QCopter.kicad_pro').write_text(json.dumps(project,indent=2)+'\n')
manifest.update(status='Routed bottom copper with fitted insulated wire links; native validation pending',components=comps,expected_components=len(comps),expected_pads=98+2*len(links),wire_links=links,board_mm=[90,90],mounting_holes_mm=[[4,4],[86,4],[4,86],[86,86]],mounting_pattern='Chosen carrier holes; NOT verified F450 frame pattern',widths_mm=d['widths'])
(O/'connections.json').write_text(json.dumps(manifest,indent=2)+'\n')
# Reload after setting project clearances, then fill using native KiCad geometry.
b=p.LoadBoard(str(O/'QCopter.kicad_pcb'))
p.ZONE_FILLER(b).Fill(b.Zones())
p.SaveBoard(str(O/'QCopter.kicad_pcb'),b)
print('Native board:',len(list(b.GetFootprints())),'footprints;',len(list(b.GetTracks())),'bottom tracks;',len(links),'wire links')
