"""Vector toner-transfer masks and interactive assembly guide, from native copper.
Requires reportlab and pypdf; PDF generation has no printer scaling compensation.
"""
import argparse,sys,json,hashlib,math,html
from pathlib import Path
arg=argparse.ArgumentParser();arg.add_argument('project',type=Path);args=arg.parse_args();O=args.project
# Dependencies can be supplied by the caller's PYTHONPATH; none are vendored.
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from pypdf import PdfReader
out=O/'output';out.mkdir(exist_ok=True)
d=json.loads((out/'native-copper-geometry.json').read_text())
assert hashlib.sha256((O/d['source']).read_bytes()).hexdigest()==d['source_sha256']
manifest=json.loads((O/'connections.json').read_text())
pdf=out/'etch-mask-3-copies-A4-1to1.pdf'
c=canvas.Canvas(str(pdf),pagesize=A4,pageCompression=1,invariant=1)
c.setTitle('IntelQCopter - three actual-size toner-transfer masks')
c.setViewerPreference('PrintScaling','None')
placements=[(60,202),(60,103.5),(60,5)]
for left,bottom in placements:
    c.saveState();c.translate(left*mm,(bottom+90)*mm);c.scale(mm,-mm)
    c.setLineCap(1);c.setLineJoin(1);c.setFillGray(0);c.setStrokeGray(0)
    for zone in d['zones']:
        path=c.beginPath()
        for ring in [zone['outline']]+zone['holes']:
            path.moveTo(*ring[0])
            for point in ring[1:]:path.lineTo(*point)
            path.close()
        c.drawPath(path,stroke=0,fill=1,fillMode=0)
    for track in d['tracks']:
        c.setLineWidth(track['width']);c.line(*track['start'],*track['end'])
    for pad in d['pads']:c.circle(*pad['xy'],pad['diameter']/2,stroke=0,fill=1)
    c.setFillGray(1)
    for hole in d['holes']:c.circle(*hole['xy'],hole['drill']/2,stroke=0,fill=1)
    c.restoreState()
c.showPage();c.save()
r=PdfReader(pdf);page=r.pages[0]
assert len(r.pages)==1 and not page.extract_text().strip()
assert abs(float(page.mediabox.width)-210*mm)<.001 and abs(float(page.mediabox.height)-297*mm)<.001
# Confirm three exact mm transforms in the actual PDF instructions, not just settings.
operations=page.get_contents().operations
scales=[values for values,op in operations if op==b'cm' and abs(float(values[0])-mm)<1e-5 and abs(float(values[3])+mm)<1e-5]
assert len(scales)==3,scales
assert r.trailer['/Root']['/ViewerPreferences']['/PrintScaling']=='/None'
verification=dict(engine='Native KiCad B.Cu filled polygons / round pads / tracks; ReportLab vector PDF; pypdf checks',
    board_sha256=d['source_sha256'],pdf_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(),page_mm=[210,297],board_mm=[90,90],copies=3,
    placements_bottom_left_mm=placements,pdf_mm_transform_verified=True,text_characters=0,hole_voids_per_copy=len(d['holes']),
    polarity='Black retained copper; white removed copper and drill voids',
    orientation='Component-side coordinates, no additional X mirror: toner face-down against underside gives required reversal',
    physical_printer_scale_verified=False)
(out/'print-verification.json').write_text(json.dumps(verification,indent=2)+'\n')

# Assembly coordinate system: component side, X right / Y down, origin board top left.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="1100" viewBox="-5 -5 100 100">',
     '<rect x="0" y="0" width="90" height="90" rx="1" fill="#f2f6f3" stroke="#254437" stroke-width=".3"/>',
     '<path d="M13 70 H38 V90 H13 Z" fill="#dceeff" stroke="#6398c3" stroke-width=".15" stroke-dasharray="1 1"/>',
     '<text x="25.5" y="80" text-anchor="middle" font-family="Arial" font-size="1.5" fill="#226293">USB plug access</text>']
def rect(x,y,w,h,fill,label):
    svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx=".5" fill="{fill}" fill-opacity=".86" stroke="#425e5a" stroke-width=".18"/>')
    svg.append(f'<text x="{x+w/2}" y="{y+h/2}" text-anchor="middle" fill="#fff" font-family="Arial" font-size="1.5">{html.escape(label)}</text>')
rect(11.43,8.76,25.4,62.74,'#334d62','ESP32-S3')
rect(15.24,2.16,17.78,6.6,'#bd8543','ANTENNA')
rect(52.07,11.0,16.4,21.2,'#37767a','MPU6050')
rect(73.66,11.3,11.8,15.5,'#705194','BMP280')
rect(69.85,36.43,14.8,13.5,'#3d67a3','GY-271')
for x,y in manifest['mounting_holes_mm']:svg.append(f'<circle cx="{x}" cy="{y}" r="1.6" fill="white" stroke="#6b7280" stroke-width=".2"/>')
for item in manifest['components']:
    if item['reference'].startswith('U') or item['reference'].startswith('W'):continue
    x,y=item['board_xy'];label=item['reference']
    if label in ['J2','J3','J4','J5']:label+=' ESC'+str(int(label[1:])-1)
    if label=='J1':label+=' LIDAR'
    if label=='J7':label+=' ROS UART'
    if label=='J6':label+=' AUX'
    if label.startswith('JP'):label+=' BEC'
    lx,ly=(40,70.8) if item['reference']=='J7' else (x+2,y-1.8)
    svg.append(f'<text x="{lx}" y="{ly}" font-family="Arial" font-size="1.2" fill="#183e32">{label}</text>')
for a in d['pads']:
    x,y=a['xy'];color='#f0ba56' if not a['ref'].startswith('W') else '#a6c2ac'
    svg.append(f'<circle cx="{x}" cy="{y}" r="{a["diameter"]/2}" fill="{color}" stroke="#4d5a48" stroke-width=".12"/>')
    svg.append(f'<circle cx="{x}" cy="{y}" r="{a["drill"]/2}" fill="white"/>')
    if not a['ref'].startswith('W'):
        svg.append(f'<text x="{x-1.45}" y="{y+.35}" text-anchor="end" font-family="Arial" font-size=".85" fill="#19382e">{a["pin"]}</text>')
svg.append('<g id="wires">')
for w in manifest['wire_links']:
    ax,ay=w['start'];bx,by=w['end']
    svg.append(f'<g class="wire" data-wire="{w["reference"]}" opacity="0"><path d="M{ax} {ay} L{bx} {by}" stroke="#e94560" stroke-width=".5" fill="none"/><circle cx="{ax}" cy="{ay}" r="1.6" fill="none" stroke="#e94560" stroke-width=".35"/><circle cx="{bx}" cy="{by}" r="1.6" fill="none" stroke="#e94560" stroke-width=".35"/><text x="{ax+2}" y="{ay-1.4}" font-family="Arial" font-size="1.5" fill="#a11135">{w["reference"]} A</text><text x="{bx+2}" y="{by-1.4}" font-family="Arial" font-size="1.5" fill="#a11135">{w["reference"]} B</text></g>')
svg.extend(['</g>','</svg>'])
svg='\n'.join(svg)
(out/'assembly-component-view.svg').write_text(svg)
rows=[]
for w in manifest['wire_links']:
    rows.append(f'<tr data-wire="{w["reference"]}" tabindex="0"><td>{w["reference"]}</td><td>{html.escape(w["net"])}</td><td>{w["start"][0]:.3f}, {w["start"][1]:.3f}</td><td>{w["end"][0]:.3f}, {w["end"][1]:.3f}</td></tr>')
page='''<!doctype html><html lang="en"><meta charset="utf-8"><title>IntelQCopter PCB assembly</title>
<style>body{font:16px system-ui;margin:24px;color:#203e36;background:#fafcfb}h1{margin-bottom:8px}p{max-width:1000px;line-height:1.5}.layout{display:grid;grid-template-columns:minmax(500px,1fr) 620px;gap:20px}.board{position:sticky;top:8px;align-self:start}svg{width:100%;height:auto}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:9px;border-bottom:1px solid #dae4df;text-align:left}tr[data-wire]{cursor:pointer}tr.active{background:#ffe2e8}button{padding:10px;background:#234d3e;color:white;border:0;border-radius:5px}@media(max-width:1150px){.layout{display:block}.board{position:relative;max-width:850px}}</style>
<h1>IntelQCopter - component-side assembly</h1>
<p><b>90 × 90 mm, one B.Cu face. Click a wire row to highlight its two solder holes.</b> Coordinates are in mm from the top-left board corner: X right, Y down. Straight red lines identify endpoints; route the insulated wire as needed, flat against the board. Use raised female sockets for all four modules and verify at least 3 mm of free space above every wire. Fit all W links; JP1-JP4 are different: leave all open initially, then fit at most one verified 5 V BEC link.</p>
<p><b>Body dimensions are reference estimates for clone sensors, not a measured fit guarantee.</b> Match VCC/pin 1 to the numbered diagram, components upward. Do not mirror this assembly view. Keep USB plug access and antenna clear.</p>
<button id="clear">Hide wire highlights</button><div class="layout"><div class="board">SVG</div><div><table><thead><tr><th>Wire</th><th>Logical net</th><th>A (X, Y)</th><th>B (X, Y)</th></tr></thead><tbody>ROWS</tbody></table></div></div>
<script>function show(ref){document.querySelectorAll('.wire').forEach(el=>el.setAttribute('opacity',el.dataset.wire===ref?'1':'0'));document.querySelectorAll('tr[data-wire]').forEach(el=>el.classList.toggle('active',el.dataset.wire===ref))}document.querySelectorAll('tr[data-wire]').forEach(el=>{el.onclick=()=>show(el.dataset.wire);el.onfocus=()=>show(el.dataset.wire)});document.getElementById('clear').onclick=()=>show('');show('');</script></html>'''
(out/'assembly-guide.html').write_text(page.replace('SVG',svg).replace('ROWS','\n'.join(rows)),encoding='utf-8')
print('Created exact-scale three-copy PDF; no text; all drill voids. Created interactive assembly guide.')
