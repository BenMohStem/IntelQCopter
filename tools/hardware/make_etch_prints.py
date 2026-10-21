"""Vector 1:1 A4 etch masks from independently checked native KiCad geometry."""
from pathlib import Path
import hashlib,json,zipfile
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from pypdf import PdfReader
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc';O=H/'output/pdf';O.mkdir(parents=True,exist_ok=True)
d=json.loads((H/'output/native-copper-geometry.json').read_text())
if hashlib.sha256((H/d['source']).read_bytes()).hexdigest()!=d['source_sha256']:raise RuntimeError('Stale copper extraction')
def draw(name,mirror):
    path=O/name;c=canvas.Canvas(str(path),pagesize=A4,pageCompression=1)
    c.setTitle('IntelQCopter bottom copper - '+('direct solder-side view' if mirror else 'face-down transfer master'))
    c.setAuthor('IntelQCopter');c.setViewerPreference('PrintScaling','None')
    c.setFont('Helvetica-Bold',14);c.drawString(20*mm,276*mm,'ESP32-S3 / single bottom copper / 1:1')
    c.setFont('Helvetica',10)
    lines=['DIRECT SOLDER-SIDE VIEW - already mirrored.' if mirror else 'FACE-DOWN TRANSFER MASTER - component-view orientation.',
           'Black = copper to KEEP. White = copper to REMOVE. Positive artwork.',
           'Print Actual size / 100%. Turn OFF fit, borderless enlargement and printer mirroring.',
           'Use monochrome maximum-quality output. Verify the rulers on the physical print.',
           'No wire jumpers, silkscreen or page frame are inside the copper mask.']
    for i,line in enumerate(lines):c.drawString(20*mm,(267-6*i)*mm,line)
    # Board rectangle is x=55..155mm, y=110..210mm on A4. No automatic fit or scaling.
    c.saveState();c.translate(55*mm,210*mm);c.scale(mm,-mm)
    if mirror:c.translate(100,0);c.scale(-1,1)
    c.setLineCap(1);c.setLineJoin(1);c.setStrokeGray(0);c.setFillGray(0)
    for t in d['tracks']:
        c.setLineWidth(t['width']);c.line(*t['start'],*t['end'])
    for a in d['pads']:c.circle(*a['xy'],a['diameter']/2,stroke=0,fill=1)
    c.setFillGray(1)
    for a in d['pads']:c.circle(*a['xy'],a['drill']/2,stroke=0,fill=1)
    c.restoreState()
    # Corner crop marks OUTSIDE the board, never copper outlines inside the mask.
    c.setStrokeGray(0);c.setLineWidth(.15*mm)
    for x,y,sx,sy in [(55,110,-1,-1),(155,110,1,-1),(55,210,-1,1),(155,210,1,1)]:
        c.line((x+sx)*mm,y*mm,(x+sx*4)*mm,y*mm);c.line(x*mm,(y+sy)*mm,x*mm,(y+sy*4)*mm)
    c.setFont('Helvetica',9);c.drawString(55*mm,104*mm,'Board crop marks span 100.00 x 100.00 mm')
    for y,length in [(93,20),(81,100)]:
        c.line(55*mm,y*mm,(55+length)*mm,y*mm)
        for x in (55,55+length):c.line(x*mm,(y-1.5)*mm,x*mm,(y+1.5)*mm)
        c.drawString((58+length)*mm,(y-1)*mm,str(length)+' mm')
    c.setFont('Helvetica',9)
    notes=['Orientation check: battery-sense pad J7/1 is '+('near the upper RIGHT.' if mirror else 'near the upper LEFT.'),
           'Top wires: 13 INSULATED jumpers. Install separately using wire-links.csv.',
           'Prototype only: verify purchased module fit and print scale BEFORE etching.',
           'Keep bottom-copper order: a face-down transfer reverses the printed master.',
           'For a transparency/contact process, choose orientation by its final film contact face.',
           'Do not use unchanged with negative dry-film resist: it needs opposite polarity.',
           'Source SHA-256: '+d['source_sha256'][:32],
           'Remaining hash: '+d['source_sha256'][32:]]
    for i,line in enumerate(notes):c.drawString(20*mm,(62-5*i)*mm,line)
    c.showPage();c.save()
    page=PdfReader(path).pages[0]
    if abs(float(page.mediabox.width)-A4[0])>.01 or abs(float(page.mediabox.height)-A4[1])>.01:raise RuntimeError('Unexpected page size')
    return path
files=[draw('etch-mask-face-down-transfer-A4-1to1.pdf',False),draw('etch-mask-direct-solder-view-A4-1to1.pdf',True)]
package=H/'output/IntelQCopter-ESP32-home-etch-KiCad.zip'
with zipfile.ZipFile(package,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(H.rglob('*')):
        if not p.is_file() or p==package or p.suffix in ('.zip','.kicad_prl') or '-backups' in str(p):continue
        z.write(p,p.relative_to(H))
print('Created',*[str(p) for p in files],str(package),sep='\n')
