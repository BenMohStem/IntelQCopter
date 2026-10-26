"""Three exact-size vector etch masks and explicit dedicated-hole wire diagrams."""
import sys,json,hashlib,html,math
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from pypdf import PdfReader
O=Path(sys.argv[1]);out=O/'output';out.mkdir(exist_ok=True)
d=json.loads((out/'native-copper-geometry.json').read_text());m=json.loads((O/'connections.json').read_text())
assert hashlib.sha256((O/'QCopter.kicad_pcb').read_bytes()).hexdigest()==d['source_sha256']
assert len(d['holes'])==86
pdf=out/'etch-mask-3-copies-A4-1to1.pdf';c=canvas.Canvas(str(pdf),pagesize=A4,pageCompression=1,invariant=1)
c.setTitle('IntelQCopter - three 1:1 masks - three dedicated GND jumpers');c.setViewerPreference('PrintScaling','None')
for left,bottom in [(59,203),(59,107.5),(59,12)]:
    c.saveState();c.translate(left*mm,(bottom+82)*mm);c.scale(mm,-mm);c.translate(-42,-38)
    c.setFillGray(0);c.setStrokeGray(0);c.setLineCap(1);c.setLineJoin(1)
    for z in d['zones']:
        q=c.beginPath()
        for ring in [z['outline']]+z['holes']:
            q.moveTo(*ring[0])
            for pt in ring[1:]:q.lineTo(*pt)
            q.close()
        c.drawPath(q,stroke=0,fill=1,fillMode=0)
    for t in d['tracks']:c.setLineWidth(t['width']);c.line(*t['start'],*t['end'])
    for a in d['pads']:c.circle(*a['xy'],a['diameter']/2,stroke=0,fill=1)
    c.setFillGray(1)
    for a in d['holes']:c.circle(*a['xy'],a['drill']/2,stroke=0,fill=1)
    c.restoreState()
c.showPage();c.save()
r=PdfReader(pdf);page=r.pages[0]
assert len(r.pages)==1 and not page.extract_text().strip()
assert abs(float(page.mediabox.width)-210*mm)<.001 and abs(float(page.mediabox.height)-297*mm)<.001
assert len([v for v,op in page.get_contents().operations if op==b'cm' and abs(float(v[0])-mm)<1e-5 and abs(float(v[3])+mm)<1e-5])==3
(out/'print-verification.json').write_text(json.dumps({'engine':'Native KiCad B.Cu copper / ReportLab vector PDF / pypdf','board_sha256':d['source_sha256'],'pdf_sha256':hashlib.sha256(pdf.read_bytes()).hexdigest(),'board_mm':[92,82],'page_mm':[210,297],'copies':3,'exact_mm_transforms_verified':True,'text_characters':0,'drill_voids_per_copy':86,'polarity':'Black retained copper, white drill holes','orientation':'Component coordinates: print unchanged, toner face down on underside; no extra mirror','ground_wires_required':3,'dedicated_jumper_holes':7,'physical_printer_scale_verified':False},indent=2)+'\n')

def diagram(back):
    xmap=lambda x:176-x if back else x
    s=['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="1260" viewBox="38 34 100 90">','<rect x="42" y="38" width="92" height="82" fill="#f4f7f5" stroke="#31564a" stroke-width=".25"/>']
    def label(x,y,t,size=1.3,color='#153c30'):
        s.append(f'<text x="{xmap(x)}" y="{y}" font-family="Arial" font-size="{size}" text-anchor="middle" fill="{color}" stroke="white" stroke-width=".5" paint-order="stroke">{html.escape(t)}</text>')
    for t in d['tracks']:
        a,b=t['start'],t['end'];s.append(f'<path d="M{xmap(a[0])} {a[1]} L{xmap(b[0])} {b[1]}" stroke="#c2d4cb" stroke-width="{t["width"]}" fill="none"/>')
    for x,y,w,h,title in [(74.23,51.6,25.4,62.74,'ESP32-S3'),(48.89,61.4,16.4,21.2,'MPU6050'),(103.43,60.25,11.8,15.5,'BMP280'),(112.69,83.75,14.8,13.5,'GY-271')]:
        xx=xmap(x+w) if back else x
        s.append(f'<rect x="{xx}" y="{y}" width="{w}" height="{h}" fill="#d9e6ec" opacity=".65" stroke="#627d87" stroke-width=".2"/>');label(x+w/2,y+h/2,title,1.4)
    s.append('<rect x="42" y="38" width="92" height="11" fill="#edf4ff" stroke="#4779b6" stroke-width=".2" stroke-dasharray="1 1"/>');label(88,44,'NO COPPER — antenna-side edge',1.6,'#28518c')
    for a in d['holes']:
        x,y=a['xy'];color='#5dba95' if a['net'].startswith('GND') else '#edb7a4' if '5V' in a['net'] else '#b9cbd6'
        s.append(f'<circle cx="{xmap(x)}" cy="{y}" r="{a["diameter"]/2}" fill="{color}" stroke="#4c6358" stroke-width=".12"/><circle cx="{xmap(x)}" cy="{y}" r="{a["drill"]/2}" fill="white"/>')
    for link,color in zip(m['wire_links'],['#bd2588','#205dd0','#d47013']):
        a,b=link['start'],link['end'];coords=link.get('taps',[a,b]);path='M'+' L'.join(f'{xmap(x)} {y}' for x,y in coords)
        s.append(f'<path d="{path}" stroke="white" stroke-width="1.4" fill="none"/><path d="{path}" stroke="{color}" stroke-width=".7" fill="none"/>')
        for x,y in coords:
            i=next(a['pin'] for a in d['holes'] if a['ref']==link['reference'] and abs(a['xy'][0]-x)<.001 and abs(a['xy'][1]-y)<.001)
            s.append(f'<circle cx="{xmap(x)}" cy="{y}" r="1.65" fill="white" stroke="{color}" stroke-width=".4"/><circle cx="{xmap(x)}" cy="{y}" r=".6" fill="#202f32"/>')
            label(x+(3.7 if link['reference']=='G1' else 3),y+.5,link['reference']+'.'+str(i),1.15,color)
    label(88,36,'SOLDER SIDE (mirrored)' if back else 'COMPONENT SIDE — three insulated wires',1.4)
    label(88,123,'G1/G3: two holes. G2: G2.3 to G2.1 (middle) to G2.2.',1.2)
    for x,y,t in [(57.5,50,'ESC1'),(123,50,'ESC2'),(63,104,'ESC3'),(123,102,'ESC4'),(54,92,'LIDAR wires')]:label(x,y,t,1.1)
    s.append('</svg>');return '\n'.join(s)
front=diagram(False);back=diagram(True)
(out/'assembly-component-side.svg').write_text(front,encoding='utf-8');(out/'assembly-solder-side.svg').write_text(back,encoding='utf-8')
rows=''.join(f'<tr><td>{html.escape(l["reference"])}</td><td>{html.escape(l["purpose"])}</td><td>{sum(math.dist(a,b) for a,b in zip(l.get("taps",[l["start"],l["end"]]),l.get("taps",[l["start"],l["end"]])[1:])):.1f} mm</td></tr>' for l in m['wire_links'])
page='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Three drilled GND jumpers</title><style>body{font:17px system-ui;max-width:1150px;margin:25px auto;padding:0 16px;color:#203e33;background:#fcfdfb}p,li{line-height:1.5}svg{width:100%;height:auto}table{border-collapse:collapse;width:100%}td,th{padding:12px;border-bottom:1px solid #d8e3dc;text-align:left}.notice{background:#edf3e7;padding:15px}button{padding:12px;margin:5px;cursor:pointer}</style>
<h1>Three dedicated GND jumpers</h1><p class="notice">Fit <b>all three</b> insulated wires on the component side, then solder underneath. G1/G3 join pad 1 to pad 2. G2 is ONE continuous wire: G2.3 to G2.1 (middle stripped/soldered tap) to G2.2. No wire shares a module/ESC header hole. Seven dedicated wire holes: 1.2 mm drill, 3.2 mm copper lands. Match the holes in the diagrams, not nearby sensor/header pins.</p>
<table><tr><th>Wire</th><th>Ground group joined to common GND</th><th>Hole-centre distance (add length for bends)</th></tr>ROWS</table>
<button onclick="document.getElementById('front').hidden=false;document.getElementById('back').hidden=true">Component view</button><button onclick="document.getElementById('front').hidden=true;document.getElementById('back').hidden=false">Solder view</button><div id="front">FRONT</div><div id="back" hidden>BACK</div>
<p>The short new copper track from ESC3 GND ends at G3.1; jumper G3 connects it to common GND at G3.2. G2 also joins the RF-side ground group through its new G2.3 hole. Use fine insulated wire (for example AWG24); fit its middle tap through the dedicated 1.2 mm hole with insulation stripped only at the solder points. Inspect the actual wire fit before soldering. G1 joins the shared BMP280/GY-271 return. Use insulated wire where it crosses other copper. G1 passes beneath a raised sensor module: fit the wire before the module and keep it below the assumed 9 mm module PCB height.</p>
<p>All fourteen module/header GND endpoints are common only after all three wires are fitted. Native DRC/parity and the independent assembled-net graph pass; these are CAD checks, not a physical continuity test. With power disconnected, check every GND against J1/lidar GND, then check 5 V and 3.3 V for shorts.</p>
<p>MPU6050 AD0 is intentionally grounded to select address 0x68. It is an address input, not another sensor GND supply pin. BMP280 SDO is also intentionally grounded (address 0x76).</p>
<p>Power: only J4/ESC3 pin 2 supplies the carrier from a verified regulated 5 V BEC; the other BEC positives remain isolated. Disconnect that positive lead before USB powering. Battery/motor current stays on a separate PDB. The antenna-side edge strip is copper-free; FR4 remains. This improves the intended antenna clearance but actual Bluetooth range, exact clone fit, paper scale, supply and interfaces remain untested. Firmware and disabled motor outputs are unchanged.</p>
<p>Use this revision's <a href="etch-mask-3-copies-A4-1to1.pdf">three-copy 1:1 mask</a> at Actual size / 100%, with no Fit to page or extra mirror. The PDF contains no text. Old masks lack the new RF ground hole and copper clearance and must not be used for this revision.</p></html>'''
(out/'assembly-guide.html').write_text(page.replace('ROWS',rows).replace('FRONT',front).replace('BACK',back),encoding='utf-8')
print('PASS: three 92x82mm masks on A4, 86 white drill holes per copy, no text; dedicated-wire diagrams generated')
