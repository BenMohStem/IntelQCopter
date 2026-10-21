from pathlib import Path
import json
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.ndimage import label
import argparse
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc'
p=argparse.ArgumentParser();p.add_argument('--image',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args();A=args.output;A.mkdir(parents=True,exist_ok=True)
im=Image.open(args.image).convert('L');sx=im.width/210;sy=im.height/297
regions,count=label(np.asarray(im)<128)
data=json.loads((H/'output/native-copper-geometry.json').read_text())
def location(a):
 offset=(a['diameter']+a['drill'])/4
 return (round((5+a['xy'][0]+offset)*sx),round((37+a['xy'][1])*sy))
anchor=next(a for a in data['pads'] if a['ref']=='J3' and a['pin']=='1');x,y=location(anchor);ground_region=int(regions[y,x]);assert ground_region
checks=[];shorts=[];missed=[]
for a in data['pads']:
 x,y=location(a);same=int(regions[y,x])==ground_region
 if a['net']=='GND' and not same:missed.append([a['ref'],a['pin']])
 if a['net']!='GND' and same:shorts.append([a['ref'],a['pin'],a['net']])
 if (a['ref'],a['pin']) in [('J1','22'),('J2','1'),('J2','21'),('J2','22')]:checks.append(dict(reference=a['ref'],pin=a['pin'],net=a['net'],same_printed_copper_region_as_power_ground=same))
assert len(checks)==4 and all(c['same_printed_copper_region_as_power_ground'] for c in checks),checks
assert not missed and not shorts,(missed,shorts)
report=dict(engine='300dpi Poppler rendering of final print PDF + SciPy 4-connected black-pixel component analysis',scope='Independent digital mask continuity/no-short check; not a measurement of an etched or assembled board',threshold=128,reference='J3 pin 1 power GND pad annulus',pdf_sha256=__import__('hashlib').sha256((H/'output/pdf/etch-mask-3-copies-A4-1to1.pdf').read_bytes()).hexdigest(),esp32_ground_pads=checks,all_ground_pads_connected=not missed,non_ground_pads_shorting_to_plane=shorts,passed=True)
(A/'mask-ground-continuity.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
crop=im.crop((round(5*sx),round(37*sy),round(105*sx),round(137*sy))).convert('RGB').resize((1000,1000))
canvas=Image.new('RGB',(1400,1120),'#edf3f5');canvas.paste(crop,(30,80));draw=ImageDraw.Draw(canvas)
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19);large=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',24)
draw.text((30,20),'Direct common GND — all four ESP32 pads pass printed-copper continuity',fill='#183948',font=large)
targets={('J1','22'):(1060,700),('J2','1'):(1060,220),('J2','21'):(1060,600),('J2','22'):(1060,650)}
for a in data['pads']:
 key=(a['ref'],a['pin'])
 if key in targets:
  x,y=(30+10*a['xy'][0],80+10*a['xy'][1]);tx,ty=targets[key]
  draw.ellipse((x-15,y-15,x+15,y+15),outline='#00ad70',width=4);draw.line((x+15,y,tx-8,ty+10),fill='#00ad70',width=3);draw.text((tx,ty),f'{key[0]} pin {key[1]} = GND',fill='#15654e',font=font)
draw.text((1060,810),'No W22 required.\nSolid pad connections.\n12 other wire links remain.\n\nThis annotated proof is\nNOT the print mask.',fill='#24495c',font=font,spacing=9)
canvas.save(A/'ground-connection-proof.png')
print(json.dumps(report,indent=2))
