"""Three identical positive toner-transfer masks on A4; vector 1:1; no text."""
from pathlib import Path
import hashlib,json,zipfile
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from pypdf import PdfReader
R=Path(__file__).resolve().parents[2];H=R/'hardware/home-etched-fc';O=H/'output/pdf';O.mkdir(parents=True,exist_ok=True)
d=json.loads((H/'output/native-copper-geometry.json').read_text())
if hashlib.sha256((H/d['source']).read_bytes()).hexdigest()!=d['source_sha256']:raise RuntimeError('Stale copper extraction')
path=O/'etch-mask-3-copies-A4-1to1.pdf'
c=canvas.Canvas(str(path),pagesize=A4,pageCompression=1,invariant=1)
c.setTitle('Three identical 1:1 bottom-copper face-down transfer masks')
c.setAuthor('IntelQCopter');c.setViewerPreference('PrintScaling','None')
placements=[(5,160),(105,160),(55,50)]
for left,bottom in placements:
    # Native KiCad uses component-side X-right / Y-down coordinates.
    # Face-down contact to copper provides the one required reversal.
    c.saveState();c.translate(left*mm,(bottom+100)*mm);c.scale(mm,-mm)
    c.setLineCap(1);c.setLineJoin(1);c.setFillGray(0);c.setStrokeGray(0)
    for zone in d['zones']:
        q=c.beginPath()
        for ring in [zone['outline']]+zone['holes']:
            if len(ring)<3:raise RuntimeError('Degenerate filled polygon')
            q.moveTo(*ring[0])
            for point in ring[1:]:q.lineTo(*point)
            q.close()
        c.drawPath(q,stroke=0,fill=1,fillMode=0)
    for t in d['tracks']:
        c.setLineWidth(t['width']);c.line(*t['start'],*t['end'])
    for a in d['pads']:c.circle(*a['xy'],a['diameter']/2,stroke=0,fill=1)
    c.setFillGray(1)
    for a in d['pads']:c.circle(*a['xy'],a['drill']/2,stroke=0,fill=1)
    c.restoreState()
c.showPage();c.save()
r=PdfReader(path);page=r.pages[0]
if len(r.pages)!=1 or page.extract_text().strip():raise RuntimeError('Mask must be a single page without text')
if abs(float(page.mediabox.width)-A4[0])>.01 or abs(float(page.mediabox.height)-A4[1])>.01:raise RuntimeError('Unexpected page size')
proof=R/'artifacts/hardware/solid-ground-power-audit-20261021/print-verification.json'
report=dict(engine='KiCad 10.0.6 filled B.Cu extraction + ReportLab vector paths + pypdf',source_sha256=d['source_sha256'],pdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),page_mm=[210,297],board_mm=[100,100],copies=3,placements_mm=placements,text_characters=0,mirroring='component view; face-down toner transfer reverses once',polarity='black = retained copper; white = removed copper',connected_ground_polygons=len(d['zones']),tracks=len(d['tracks']),pads=len(d['pads']),physical_fit_and_print_scale_verified=False)
proof.write_bytes((json.dumps(report,indent=2)+'\n').encode())
package=H/'output/IntelQCopter-ESP32-home-etch-KiCad.zip'
with zipfile.ZipFile(package,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(H.rglob('*')):
        if not p.is_file() or p==package or p.suffix in ('.zip','.kicad_prl') or '-backups' in str(p):continue
        z.write(p,p.relative_to(H))
print('Created text-free A4 / 3 copies / 100% vector mask:',path)
