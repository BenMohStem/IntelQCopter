"""Independent mechanical checks of native KiCad pads against named references."""
from pathlib import Path
import argparse,hashlib,json,math,re
import pcbnew as k
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc'
p=argparse.ArgumentParser();p.add_argument('--reference-footprint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
b=k.LoadBoard(str(H/'esp32_home_etch.kicad_pcb'));model=json.loads((H/'components-and-nets.json').read_text(encoding='utf8'));refs=json.loads((H/'reference-dimensions.json').read_text(encoding='utf8'))
pads={(f.GetReference(),p.GetNumber()):p for f in b.GetFootprints() for p in f.Pads() if p.GetNumber()}
def xy(p):return [k.ToMM(p.GetPosition().x),k.ToMM(p.GetPosition().y)]
text=a.reference_footprint.read_text(encoding='utf8');official=re.findall(r'\(pad "(\d+)" thru_hole \w+ \(at ([\d.-]+) ([\d.-]+)',text)
assert len(official)==44
errors=[]
for number,x,y in official:
 n=int(number);key=('J1',str(n)) if n<=22 else ('J2',str(45-n));actual=xy(pads[key]);expected=[40+float(x),14+float(y)]
 error=math.dist(actual,expected);assert error<.01,(key,actual,expected);errors.append(error)
for ref in ['J1','J2']:
 row=sorted([p for (f,n),p in pads.items() if f==ref],key=lambda p:int(p.GetNumber()))
 assert len(row)==22
 for p in row:
  assert abs(k.ToMM(p.GetDrillSize().x)-1.1)<1e-6
  assert abs(k.ToMM(p.GetSize().x)-1.94)<1e-6
 for u,v in zip(row,row[1:]):assert abs(math.dist(xy(u),xy(v))-2.54)<1e-6
for ref in ['C1','C2']:assert abs(math.dist(xy(pads[(ref,'1')]),xy(pads[(ref,'2')]))-5)<1e-6
for ref in ['R'+str(i) for i in range(1,9)]:assert abs(math.dist(xy(pads[(ref,'1')]),xy(pads[(ref,'2')]))-10.16)<1e-6
def overlap(a,b):return min(a[2],b[2])>max(a[0],b[0]) and min(a[3],b[3])>max(a[1],b[1])
body=refs['devkit']['body_xy_mm'];usb=refs['usb_access']['rect_xy_mm'];passives=[]
for c in model['components']:
 if not c['ref'].startswith(('R','C')):continue
 cx=sum(p['x'] for p in c['pads'])/len(c['pads']);cy=sum(p['y'] for p in c['pads'])/len(c['pads'])
 w,h=(6.5,2.5) if c['ref'].startswith('R') else (4,2.6);box=[cx-w/2,cy-h/2,cx+w/2,cy+h/2]
 assert not overlap(box,body) and not overlap(box,usb),(c['ref'],box)
 passives.append(dict(reference=c['ref'],body_xy_mm=box))
for p in [pads[('C2','1')],pads[('C2','2')]]:
 x,y=xy(p);radius=k.ToMM(p.GetSize().x)/2;assert not overlap([x-radius,y-radius,x+radius,y+radius],usb)
report=dict(engine='KiCad 10.0.6 native pad geometry + independent Espressif footprint comparison + manufacturer dimensional envelopes',
 scope='Selected reference parts only; purchased modules, hand drilling, module/cable height and F450 mounting not measured',passed=True,
 official_reference_pad_positions_checked=44,maximum_official_pad_position_difference_mm=max(errors),nominal_pitch_mm=2.54,row_spacing_mm=22.86,
 socket_manual_drill_mm=1.1,socket_manufacturer_recommended_hole_mm=1.02,socket_pad_mm=1.94,socket_annulus_mm=.42,socket_gap_mm=.6,
 capacitor_pitches_mm=[5,5],resistor_pitches_mm=[10.16]*8,selected_passive_bodies=passives,
 passive_bodies_clear_of_devkit_and_usb=True,current_capacitor_pads_clear_of_usb=True,
 official_footprint_sha256=hashlib.sha256(a.reference_footprint.read_bytes()).hexdigest(),
 board_sha256=hashlib.sha256((H/'esp32_home_etch.kicad_pcb').read_bytes()).hexdigest())
a.output.mkdir(parents=True,exist_ok=True);(a.output/'mechanical-verification.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print('PASS: 44 reference pins, 2 socket rows, 8 resistors, 2 capacitors, passive/USB envelopes; reference-only mechanical verification.')
