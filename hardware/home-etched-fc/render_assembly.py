"""Original assembly/fit drawing and schematic-to-PCB net comparison."""
from pathlib import Path
import json,csv,xml.etree.ElementTree as E,hashlib,math
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parent;R=D.parents[1];A=R/'artifacts/hardware/home-etch-20261021'
def main():
 data=json.loads((D/'components-and-nets.json').read_text());model={}
 for c in data['components']:
  for pad in c['pads']:
   if pad['net']:model.setdefault(pad['net'],set()).add((c['ref'],pad['number']))
 root=E.parse(A/'schematic-netlist-final.xml').getroot();actual={}
 for net in root.findall('./nets/net'):
  name=net.attrib['name'].removeprefix('/');actual[name]=set((n.attrib['ref'],n.attrib['pin']) for n in net.findall('node'))
 actual={k:v for k,v in actual.items() if not k.startswith('unconnected-')}
 mismatch={k:dict(model=sorted(model.get(k,set())),schematic=sorted(actual.get(k,set()))) for k in set(model)|set(actual) if model.get(k,set())!=actual.get(k,set())}
 drc=json.loads((A/'drc-release-candidate.json').read_text());erc=json.loads((A/'erc-bundled.json').read_text())
 report=dict(engine='KiCad 10.0.6 native DRC/ERC/netlist export; independent model-to-netlist comparison',DRC_violations=len(drc['violations']),unrouted=len(drc['unconnected_items']),schematic_parity_issues=len(drc.get('schematic_parity',[])),ERC_violations=sum(len(x['violations']) for x in erc['sheets']),schematic_matches_pad_nets=not mismatch,mismatches=mismatch,net_count=len(model),component_count=len(data['components']),insulated_links=len(data['wire_links']),assumptions=['Genuine/reference ESP32-S3-DevKitC-1 2x22 header geometry; purchased board must fit paper template','3.3V-compatible external sensor breakouts with mapped pigtails; no assumed clone pin order','External regulated5V; no motor current on PCB','Lidar supply disconnected; current sensor not selected; ADC/HMC/lidar firmware unfinished','100x100mm carrier and its mounting pattern not checked on actual F450 frame','DRC/ERC validate carrier connectivity/geometry; external modules are passive connectors, no voltage-domain or dynamic flight proof'],hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'esp32_home_etch.kicad_pcb',D/'esp32_home_etch.kicad_sch',D/'generate_board.py',D/'generate_schematic.py']})
 (A/'verification-summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
 if mismatch:raise RuntimeError('Schematic/model net mismatch: '+str(mismatch))
 with (D/'drill-table.csv').open('w',newline='',encoding='utf8') as f:
  w=csv.writer(f);w.writerow(['Reference','Pin','X_mm','Y_mm','Drill_mm','Net'])
  for c in data['components']:
   for a in c['pads']:w.writerow([c['ref'],a['number'],a['x'],a['y'],c['drill_mm'],a['net'] or 'unused'])
  for i,(x,y) in enumerate([(5,5),(95,5),(5,97),(95,97)]):w.writerow(['H'+str(i+1),'',x,y,3.2,'mounting'])
 with (D/'assembly-bom.csv').open('w',newline='',encoding='utf8') as f:
  w=csv.writer(f);w.writerow(['Reference','Role','Value','Pad_mm','Drill_mm'])
  for c in data['components']:w.writerow([c['ref'],c['role'],c['value'],c['pad_diameter_mm'],c['drill_mm']])
 # Exact-size, component-side drill/fit sheet. Annotation is not copper artwork.
 svg=['<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="100mm" viewBox="0 0 100 100"><rect x=".1" y=".1" width="99.8" height="99.8" fill="white" stroke="black" stroke-width=".15"/>']
 for c in data['components']:
  for a in c['pads']:svg.append(f'<circle cx="{a["x"]}" cy="{a["y"]}" r="{c["drill_mm"]/2}" fill="none" stroke="black" stroke-width=".1"/><path d="M{a["x"]-.8},{a["y"]}h1.6 M{a["x"]},{a["y"]-.8}v1.6" stroke="black" stroke-width=".08"/>')
 for x,y in [(5,5),(95,5),(5,97),(95,97)]:svg.append(f'<circle cx="{x}" cy="{y}" r="1.6" fill="none" stroke="black" stroke-width=".1"/>')
 svg.append('<path d="M40,11 H62.86 M40,10 V12 M62.86,10 V12 M45,95 H65 M45,94 V96 M65,94 V96" stroke="black" stroke-width=".15"/><text x="46" y="9" font-size="1.8">22.86 mm rows</text><text x="47" y="98" font-size="1.8">20 mm scale check</text><text x="13" y="5" font-size="2">TOP FIT / DRILL SHEET — PRINT 100%</text></svg>')
 (D/'fit-and-drill-top.svg').write_text(''.join(svg),encoding='utf8')
 # Readable assembly picture, derived from the routed component coordinate model.
 im=Image.new('RGB',(1510,1280),'#102233');draw=ImageDraw.Draw(im)
 def font(n):
  try:return ImageFont.truetype('C:/Windows/Fonts/arial.ttf',n)
  except OSError:return ImageFont.load_default()
 ox,oy,scale=55,135,10.3
 def xy(x,y):return (round(ox+x*scale),round(oy+y*scale))
 draw.text((55,22),'ESP32-S3 / Xbox competition carrier',(238,247,255),font=font(32))
 draw.text((55,65),'Component-side assembly view • 100 × 100 mm • bottom copper + 13 insulated links',(170,203,222),font=font(20))
 draw.rectangle([xy(0,0),xy(100,100)],fill='#e8d4a9',outline='white',width=2)
 draw.rectangle([xy(38.5,6.9),xy(64.36,76.8)],fill='#8b9b9e',outline='#243846',width=3)
 draw.rectangle([xy(38.5,0),xy(64.36,12)],fill='#243846');draw.text(xy(40,3),'ANTENNA',fill='white',font=font(15))
 for s in data['bottom_tracks']:draw.line([xy(*s['start']),xy(*s['end'])],fill='#b36d24',width=max(1,round(s['width']*scale)))
 colors=['#006e99','#973b9d','#057d65','#cf5340','#364fc1']
 for i,a in enumerate(data['wire_links']):
  st,en=xy(*a['from_xy_mm']),xy(*a['to_xy_mm']);draw.line([st,en],fill=colors[i%len(colors)],width=3)
  for t in (st,en):draw.ellipse([t[0]-4,t[1]-4,t[0]+4,t[1]+4],fill=colors[i%len(colors)])
 for c in data['components']:
  if c['ref'].startswith(('R','C')):
   pts=[xy(a['x'],a['y']) for a in c['pads']];draw.line(pts,fill='#385573',width=7)
  for a in c['pads']:
   x,y=xy(a['x'],a['y']);r=c['pad_diameter_mm']*scale/2;draw.ellipse((x-r,y-r,x+r,y+r),fill='#db9b48',outline='#72400d',width=1);r=c['drill_mm']*scale/2;draw.ellipse((x-r,y-r,x+r,y+r),fill='#102233')
  if not c['ref'].startswith('W'):
   a=c['pads'][0];draw.text(xy(a['x']-1.5,a['y']-3),c['ref'],fill='#102233',font=font(12))
 for i,(x,y) in enumerate([(5,5),(95,5),(5,97),(95,97)]):
  cx,cy=xy(x,y);draw.ellipse((cx-16,cy-16,cx+16,cy+16),fill='#102233')
 draw.text((1120,145),'ASSEMBLY KEY',fill='white',font=font(23));ly=190
 for c in data['components']:
  if c['ref'].startswith('W'):continue
  draw.text((1120,ly),c['ref']+'  '+c['role'].replace('J_','').replace('R_','').replace('C_',''),fill='#a9dcca',font=font(16));ly+=26
 draw.text((1120,ly+14),'Solid brown: bottom copper\nColoured lines: insulated wire\nWires may cross; do NOT\nreplace them with copper.\nRoute flat under socket gap.\n\n1.0 / 1.2 mm electrical drills\n3.2 mm mounting drills\n\nNo motor/ESC power paths.\nOptional ROS2 signal only.\nLidar VCC stays OPEN.',fill='#dce7ef',font=font(16),spacing=8)
 draw.text((55,1190),'PROTOTYPE: reference footprints only. Verify module fit, supply, connectors and frame before etching.',fill='#f8cd86',font=font(19))
 draw.text((55,1220),'Geometry/connection checks are not electrical bench or physical-flight validation. Physical motor arming is disabled.',fill='#bdd2e2',font=font(17))
 im.save(D/'assembly-preview.png')
 print(json.dumps({k:v for k,v in report.items() if k not in ['hashes','assumptions','mismatches']},indent=2))
if __name__=='__main__':main()
