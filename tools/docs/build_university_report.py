"""Render the English report and procurement CSV without altering flight code."""
from pathlib import Path
import base64
import csv
import hashlib
import html
import io
import json
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image as PILImage
from reportlab.graphics.shapes import Drawing, Rect, String, Line, Circle, Polygon, Ellipse, Image as DrawingImage
from reportlab.graphics import renderPM
from reportlab.graphics.barcode.qr import QrCodeWidget
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Image, Flowable

ROOT = Path(sys.argv[1]).resolve()
OUT = ROOT / 'docs/university-report'
ASSETS = OUT / 'assets'
ASSETS.mkdir(parents=True, exist_ok=True)
SOURCE = OUT / 'technical-report.md'
TEXT = SOURCE.read_text(encoding='utf-8')
BASELINE = (OUT / 'reference/engineering-baseline-v2.1.md').read_text(encoding='utf-8')
from report_figures import build_figures
VISUALS = build_figures(ROOT)
REQUIREMENTS = json.loads((OUT/'requirements-register.json').read_text(encoding='utf-8'))
pdfmetrics.registerFont(TTFont('ReportArial', r'C:\Windows\Fonts\arial.ttf'))
pdfmetrics.registerFont(TTFont('ReportArialBold', r'C:\Windows\Fonts\arialbd.ttf'))
pdfmetrics.registerFontFamily('ReportArial', normal='ReportArial', bold='ReportArialBold')
INK = colors.HexColor('#18382c')
TEAL = colors.HexColor('#198548')
STYLES = {
    'title': ParagraphStyle('title', fontName='ReportArialBold', fontSize=29, leading=34, spaceAfter=17, textColor=colors.black),
    'h2': ParagraphStyle('h2', fontName='ReportArialBold', fontSize=18, leading=23, spaceAfter=13, textColor=INK),
    'h3': ParagraphStyle('h3', fontName='ReportArialBold', fontSize=11, leading=15, spaceBefore=8, spaceAfter=6, textColor=TEAL),
    'body': ParagraphStyle('body', fontName='ReportArial', fontSize=10.2, leading=14.4, spaceAfter=8, textColor=colors.HexColor('#293b32')),
    'cell': ParagraphStyle('cell', fontName='ReportArial', fontSize=8.2, leading=11.2),
    'toc': ParagraphStyle('toc', fontName='ReportArial', fontSize=8.8, leading=11.3, spaceAfter=1, textColor=INK),
    'caption': ParagraphStyle('caption', fontName='ReportArial', fontSize=8.7, leading=11.8, spaceAfter=9, textColor=colors.HexColor('#536c5d')),
}
HEADINGS = re.findall(r'^## (.+)$', TEXT, re.M)
PAGES = {}

def inline(text):
    text = html.escape(text)
    text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
    text = re.sub(r'`(.+?)`', r'<font color="#198548">\1</font>', text)
    return text

class Heading(Paragraph):
    def __init__(self, title):
        super().__init__(inline(title), STYLES['h2'])
        self.section_title = title

class ReportDoc(SimpleDocTemplate):
    def afterFlowable(self, flowable):
        if hasattr(flowable, 'section_title'):
            PAGES[flowable.section_title] = self.page
            key='section-'+str(HEADINGS.index(flowable.section_title))
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(flowable.section_title,key,level=0,closed=False)

def footer(c, doc):
    if doc.page == 1:
        c.linkURL('https://github.com/BenMohStem/IntelQCopter',(48,96,559,112),relative=0,thickness=0)
        return
    c.setStrokeColor(TEAL)
    c.setLineWidth(1.1)
    c.line(42, 805, 553, 805)
    c.setFillColor(TEAL)
    c.rect(42,805,26,3,fill=1,stroke=0)
    c.setFont('ReportArial', 8)
    c.setFillColor(colors.HexColor('#536c5d'))
    c.drawString(42, 816, 'IntelQCopter   /   Autonomous Quadcopter')
    c.drawRightString(553,816,'PICO SYSTEM REPORT   /   REV 5.0')
    c.setStrokeColor(colors.HexColor('#d2e4d7'))
    c.line(42,39,553,39)
    c.drawString(42, 25, 'Professional and Personal Project   /   27 October 2026')
    c.setFont('ReportArialBold',9)
    c.setFillColor(TEAL)
    c.drawRightString(553,25,f'{doc.page:02d}')

def cover():
    """Original cover layout with a labelled AI-generated F450 concept image."""
    d=Drawing(511,725)
    pale=colors.HexColor('#edf6ee')
    light=colors.HexColor('#d8ebdc')
    d.add(String(0,708,'PROFESSIONAL AND PERSONAL PROJECT',fontName='ReportArialBold',fontSize=9,fillColor=TEAL))
    d.add(String(511,708,'REVISION 5.0',fontName='ReportArial',fontSize=9,textAnchor='end',fillColor=TEAL))
    d.add(String(0,651,'IntelQCopter',fontName='ReportArialBold',fontSize=47,fillColor=INK))
    d.add(String(0,614,'Autonomous Quadcopter',fontName='ReportArial',fontSize=25,fillColor=TEAL))
    d.add(String(0,587,'TECHNICAL DESIGN AND VALIDATION REPORT',fontName='ReportArialBold',fontSize=10,fillColor=INK))
    d.add(Line(0,569,511,569,strokeColor=TEAL,strokeWidth=1.1))
    d.add(DrawingImage(45,282,420,280,str(ASSETS/'f450-red-white-concept.png')))
    d.add(String(0,273,'AI CONCEPT ILLUSTRATION  /  RED AND WHITE F450  /  NOT A HARDWARE TEST PHOTO',fontName='ReportArial',fontSize=7,fillColor=colors.HexColor('#65836c')))
    # A restrained product-summary strip rather than a collage of decorative cards.
    d.add(Line(0,250,511,250,strokeColor=TEAL,strokeWidth=.8))
    for x,a,b in [(0,'F450','X FRAME'),(138,'1 kHz','FC TARGET'),(280,'RP2350','FLIGHT CONTROLLER'),(432,'ROS 2','PI / JETSON TBD')]:
        d.add(String(x,227,a,fontName='ReportArialBold',fontSize=15,fillColor=INK))
        d.add(String(x,210,b,fontName='ReportArial',fontSize=8,fillColor=TEAL))
    d.add(Line(0,194,511,194,strokeColor=light,strokeWidth=.7))
    for y,t in [(170,'RP2350 aircraft + RP2040 handheld / LoRa + reserved CC1101 links.'),(153,'ROS 2 onboard computer: seeking a Raspberry Pi or NVIDIA Jetson.'),(136,'Models, sensor fusion and navigation design; flight acceptance pending.')]:
        d.add(String(0,y,t,fontName='ReportArial',fontSize=9.2,fillColor=INK))
    d.add(String(0,104,'IQC-PPP-002   /   27 OCTOBER 2026   /   UNIVERSITY SUBMISSION DRAFT',fontName='ReportArialBold',fontSize=8,fillColor=TEAL))
    d.add(String(0,78,'Student / ID / institution / supervisor / academic year: complete before submission.',fontName='ReportArial',fontSize=8.5,fillColor=INK))
    d.add(String(0,60,'Proposed targets and historical results are distinguished. Motor outputs remain disabled.',fontName='ReportArial',fontSize=8.5,fillColor=INK))
    d.add(String(0,44,'github.com/BenMohStem/IntelQCopter',fontName='ReportArialBold',fontSize=8,fillColor=TEAL))
    d.add(Line(0,33,511,33,strokeColor=TEAL,strokeWidth=1))
    d.add(String(0,16,'IntelQCopter',fontName='ReportArialBold',fontSize=10,fillColor=INK))
    d.add(String(511,16,'01',fontName='ReportArialBold',fontSize=9,textAnchor='end',fillColor=TEAL))
    return d

def architecture():
    d = Drawing(511, 153)
    boxes = [(0,90,112,45,'Xbox input','BLE to verify'), (148,90,172,45,'ESP32-S3 FC','Sensors / estimate / PID'), (358,90,153,45,'ESCs and motors','Outputs disabled'), (0,9,112,46,'Ground station','Approved mission'), (148,9,172,46,'ROS 2 companion','Optional perception / planner')]
    for x,y,w,h,a,b in boxes:
        d.add(Rect(x,y,w,h,fillColor=colors.HexColor('#edf6ee'),strokeColor=TEAL,strokeWidth=.8))
        d.add(String(x+w/2,y+28,a,fontName='ReportArialBold',fontSize=9.5,textAnchor='middle',fillColor=INK))
        d.add(String(x+w/2,y+12,b,fontName='ReportArial',fontSize=8.3,textAnchor='middle',fillColor=INK))
    for x1,y1,x2,y2 in [(112,112,148,112),(320,112,358,112),(112,32,148,32),(235,55,235,90)]:
        d.add(Line(x1,y1,x2,y2,strokeColor=TEAL,strokeWidth=1.3))
        if y1==y2:
            d.add(Polygon([x2,y2,x2-5,y2-3,x2-5,y2+3],fillColor=TEAL,strokeColor=TEAL))
        else:
            d.add(Polygon([x2,y2,x2-3,y2-5,x2+3,y2-5],fillColor=TEAL,strokeColor=TEAL))
    d.add(String(350,28,'Companion requests setpoints',fontName='ReportArial',fontSize=8,fillColor=INK))
    d.add(String(350,14,'FC owns all motor commands',fontName='ReportArial',fontSize=8,fillColor=INK))
    return d

def geometry():
    d=Drawing(511,145)
    cx,cy=125,72
    for dx,dy,lab in [(51,45,'(+a,+a)'),(-51,45,'(+a,-a)'),(-51,-45,'(-a,-a)'),(51,-45,'(-a,+a)')]:
        d.add(Line(cx,cy,cx+dx,cy+dy,strokeColor=TEAL,strokeWidth=7))
        d.add(Circle(cx+dx,cy+dy,14,fillColor=colors.white,strokeColor=INK))
        d.add(String(cx+dx+18,cy+dy-3,lab,fontName='ReportArial',fontSize=8,fillColor=INK))
    d.add(Circle(cx,cy,12,fillColor=INK,strokeColor=INK))
    d.add(String(270,113,'Motor diagonal: nominal 450 mm',fontName='ReportArialBold',fontSize=10,fillColor=INK))
    d.add(String(270,90,'Centre to motor: L = 225 mm',fontName='ReportArial',fontSize=9.5,fillColor=INK))
    d.add(String(270,68,'Body frame: x forward, y left, z up',fontName='ReportArial',fontSize=9.5,fillColor=INK))
    d.add(String(270,46,'Lever component: a = L / sqrt(2)',fontName='ReportArial',fontSize=9.5,fillColor=INK))
    d.add(String(270,24,'Measure actual geometry before mixing',fontName='ReportArial',fontSize=9.5,fillColor=INK))
    return d

def save_drawing(drawing, name):
    # SVG retains vector geometry in the browser; PDF uses the native Drawing.
    from reportlab.graphics import renderSVG
    path=ASSETS/(name+'.svg')
    renderSVG.drawToFile(drawing,str(path))
    return path

def html_image(path, alt):
    ext=path.suffix.lower()
    mime='image/svg+xml' if ext=='.svg' else 'image/png'
    encoded=base64.b64encode(path.read_bytes()).decode('ascii')
    return '<img alt="'+html.escape(alt,quote=True)+'" src="data:'+mime+';base64,'+encoded+'">'

def equation(latex, index):
    latex=re.sub(r'\\(mathbf|boldsymbol|widehat|dot|mathrm)\s*(\\[a-zA-Z]+|[a-zA-Z])',lambda m:'\\'+m.group(1)+'{'+m.group(2)+'}',latex)
    latex=latex.replace(r'\frac1N',r'\frac{1}{N}').replace(r'\frac1{',r'\frac{1}{').replace(r'\tfrac12',r'\frac{1}{2}')
    latex=re.sub(r'\\le\b',r'\\leq',latex)
    latex=re.sub(r'\\ge\b',r'\\geq',latex)
    fig=plt.figure(figsize=(9,0.65), dpi=220)
    fig.text(.5,.5,'$'+latex+'$',ha='center',va='center',fontsize=14,color='#18382c')
    path=ASSETS/f'equation-v5-{index:02d}.png'
    fig.savefig(path,bbox_inches='tight',pad_inches=.10,facecolor='white')
    plt.close(fig)
    return path

EVIDENCE = [
    ('P1','measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md'),
    ('P2','measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md'),
    ('P3','measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md'),
    ('P4','measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md'),
    ('P5','measurements/software/2026-10-27-input-guard/summary.json'),
    ('P1-data','measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json'),
    ('P4-data','measurements/mpu6050/2026-10-26-offline-attitude-preparation/analysis.json'),
]
manifest=[]
for label,rel in EVIDENCE:
    path=ROOT/rel
    assert path.is_file(),rel
    manifest.append({'reference':label,'path':rel,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'url':'https://github.com/BenMohStem/IntelQCopter/blob/codex/stepwise-esp32/'+rel})
(OUT/'evidence-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')

image_paths={}
for key,name in [('reserved','reserved-replay.png'),('factorial','factorial-model.png')]:
    src=ROOT/'measurements/mpu6050/2026-10-26-offline-attitude-preparation'/name
    dst=ASSETS/name
    dst.write_bytes(src.read_bytes())
    image_paths[key]=dst

# Convert material tables to a useful procurement inventory without inventing prices.
inventory_path=OUT/'bill-of-materials.csv'
existing_inventory={}
if inventory_path.exists():
    with inventory_path.open(encoding='utf-8-sig',newline='') as old:
        existing_inventory={row['id']:row for row in csv.DictReader(old)}
bom=[]
target_materials=json.loads((OUT/'materials-target.json').read_text(encoding='utf-8'))
for item in target_materials:
    row=dict(item)
    row.update({'exact_model':'TBD','owned_or_to_buy':'Confirm','supplier':'','currency':'','quote_date':'','unit_cost':'','shipping_allocation':'','verified_mass_g':''})
    old=existing_inventory.get(item['id'],{})
    if old.get('component')==item['component']:
        for field in ['exact_model','owned_or_to_buy','supplier','currency','quote_date','unit_cost','shipping_allocation','verified_mass_g']:
            if field in old:row[field]=old[field]
    bom.append(row)
with (OUT/'bill-of-materials.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(bom[0]),lineterminator="\n");writer.writeheader();writer.writerows(bom)

def make_story(toc_pages):
    story=[]; web=[]; eq=0
    lines=TEXT.splitlines();i=0
    while i<len(lines):
        line=lines[i].strip()
        if i==0:
            d=cover()
            story.append(d)
            web.append('<div class="cover">'+html_image(save_drawing(d,'cover'),'IntelQCopter Autonomous Quadcopter technical report cover')+'</div>')
            PAGES[HEADINGS[0]]=1
            while i<len(lines) and lines[i]!='---':i+=1
            continue
        if not line.strip():i+=1;continue
        if line=='---':story.append(PageBreak());web.append('</section><section>');i+=1;continue
        if line=='[[CONTENTS]]':
            toc=[]
            for title in HEADINGS[2:]:
                key='section-'+str(HEADINGS.index(title))
                toc.append([Paragraph('<link href="#'+key+'" color="#18382c">'+inline(title)+'</link>',STYLES['toc']),Paragraph(str(toc_pages.get(title,'')),STYLES['toc'])])
                web.append('<p class="toc"><a href="#'+str(HEADINGS.index(title))+'">'+inline(title)+'</a><span>'+str(toc_pages.get(title,''))+'</span></p>')
            table=Table(toc,colWidths=[480,31],hAlign='LEFT')
            table.setStyle(TableStyle([('LINEBELOW',(0,0),(-1,-1),.25,colors.HexColor('#dce9de')),('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0),('TOPPADDING',(0,0),(-1,-1),1),('BOTTOMPADDING',(0,0),(-1,-1),1)]))
            story.append(table)
        elif line=='[[REPO]]':
            url='https://github.com/BenMohStem/IntelQCopter'
            q=QrCodeWidget(url);b=q.getBounds();scale=70/(b[2]-b[0]);qr=Drawing(70,70,transform=[scale,0,0,scale,0,0]);qr.add(q)
            link=Paragraph('<b>Public engineering repository</b><br/><link href="'+url+'" color="#198548">'+url+'</link><br/>Source, versions, report, measurements and scoped commit history.',STYLES['body'])
            table=Table([[link,qr]],colWidths=[425,86]);table.setStyle(TableStyle([('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0)]));story.extend([table,Spacer(1,10)])
            web.append('<p><strong>Public engineering repository</strong><br><a href="'+url+'">'+url+'</a></p>'+html_image(save_drawing(qr,'repository-qr'),'GitHub repository QR code'))
        elif line.startswith('[[VIS:'):
            name=line[6:-2];rec=VISUALS[name];path=OUT/rec['file']
            w,h=PILImage.open(path).size;scale=min(511/w,245/h)
            story.extend([Image(str(path),width=w*scale,height=h*scale),Spacer(1,5)])
            note=rec['kind']+' — '+rec['caption']
            story.append(Paragraph(inline(note),STYLES['caption']))
            web.append(html_image(path,note)+'<p class="figure-caption">'+inline(note)+'</p>')
        elif line.startswith('[[BOM:') or line.startswith('[[REQ:'):
            if line.startswith('[[BOM:'):
                selected=[r for r in bom if (r['group']=='aircraft')==(line=='[[BOM:aircraft]]')]
                rows=[['ID / qty','Material','Purpose / acceptance']]+[[r['id']+' x'+r['quantity'],r['component'],r['purpose_and_checks']] for r in selected]
                widths=[65,170,276]
            else:
                cut=(len(REQUIREMENTS)+1)//2
                selected=REQUIREMENTS[:cut] if line=='[[REQ:one]]' else REQUIREMENTS[cut:]
                rows=[['ID / status','Requirement','Verify']]+[[r['id']+' '+r['status'],r['requirement'],r['verify']] for r in selected]
                widths=[75,355,81]
            hs=ParagraphStyle('compact-head',parent=STYLES['cell'],fontName='ReportArialBold',textColor=colors.white)
            data=[[Paragraph(inline(c),hs if j==0 else STYLES['cell']) for c in r] for j,r in enumerate(rows)]
            table=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
            table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#246b42')),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f7f1')]),('GRID',(0,0),(-1,-1),.3,colors.HexColor('#cddfcf')),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),4.5),('BOTTOMPADDING',(0,0),(-1,-1),4.5)]))
            story.extend([table,Spacer(1,9)])
            web.append('<table>'+''.join('<tr>'+''.join('<'+('th' if j==0 else 'td')+'>'+inline(c)+'</'+('th' if j==0 else 'td')+'>' for c in r)+'</tr>' for j,r in enumerate(rows))+'</table>')
        elif line in ('[[ARCHITECTURE]]','[[GEOMETRY]]'):
            key='architecture' if 'ARCHITECTURE' in line else 'geometry'
            d=architecture() if key=='architecture' else geometry()
            story.append(d);story.append(Spacer(1,6))
            web.append(html_image(save_drawing(d,key),key))
        elif line.startswith('[[EQ:'):
            eq+=1;formula=line[5:-2];path=equation(formula,eq)
            w,h=PILImage.open(path).size
            width=min(500,w/2.5);height=h*width/w
            story.extend([Image(str(path),width=width,height=height),Spacer(1,8)])
            web.append(html_image(path,formula))
        elif line.startswith('[[IMAGE:'):
            key=line[8:-2];path=image_paths[key]
            w,h=PILImage.open(path).size
            maxh=365 if key=='reserved' else 250
            scale=min(511/w,maxh/h)
            story.append(Image(str(path),width=w*scale,height=h*scale));story.append(Spacer(1,5))
            web.append(html_image(path,'Existing '+key+' analysis figure'))
        elif line.startswith('# '):
            story.append(Paragraph(inline(line[2:]),STYLES['title']));web.append('<h1>'+inline(line[2:])+'</h1>')
        elif line.startswith('## '):
            title=line[3:];story.append(Heading(title));web.append('<h2 id="'+str(HEADINGS.index(title))+'">'+inline(title)+'</h2>')
        elif line.startswith('### '):
            story.append(Paragraph(inline(line[4:]),STYLES['h3']));web.append('<h3>'+inline(line[4:])+'</h3>')
        elif line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].startswith('|'):
                cells=[c.strip() for c in lines[i].strip('|').split('|')]
                if not all(re.fullmatch(r'[-: ]+',c) for c in cells):rows.append(cells)
                i+=1
            n=len(rows[0])
            if n==4:widths=[119,125,134,133]
            elif n==2:widths=[130,381]
            elif rows[0][0]=='ID':widths=[63,350,98]
            elif rows[0][0]=='ID and quantity':widths=[73,177,261]
            else:widths=[110,185,216]
            header_style=ParagraphStyle('header-cell',parent=STYLES['cell'],fontName='ReportArialBold',textColor=colors.white)
            data=[[Paragraph(inline(c),header_style if j==0 else STYLES['cell']) for c in r] for j,r in enumerate(rows)]
            t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
            pad=4 if len(rows)>1 and rows[1][0].startswith('M16 ') else 6
            t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#246b42')),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f7f1')]),('GRID',(0,0),(-1,-1),.3,colors.HexColor('#cddfcf')),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),pad),('BOTTOMPADDING',(0,0),(-1,-1),pad)]))
            story.extend([t,Spacer(1,9)])
            web.append('<table>'+''.join('<tr>'+''.join('<'+('th' if j==0 else 'td')+'>'+inline(c)+'</'+('th' if j==0 else 'td')+'>' for c in r)+'</tr>' for j,r in enumerate(rows))+'</table>')
            continue
        else:
            text=line
            while i+1<len(lines) and lines[i+1].strip() and not lines[i+1].startswith(('#','|','---','[[')):
                i+=1;text+=' '+lines[i]
            style=STYLES['caption'] if text.startswith(('Figure ', '**Symbols:', '**Math notation:')) else STYLES['body']
            story.append(Paragraph(inline(text),style));web.append('<p>'+inline(text)+'</p>')
        i+=1
    return story,web

target=OUT/'technical-report.pdf'
for run in range(2):
    story,web=make_story(dict(PAGES))
    doc=ReportDoc(str(target),pagesize=(595.28,841.89),leftMargin=42,rightMargin=42,topMargin=56,bottomMargin=47,title='IntelQCopter Autonomous Quadcopter',author='IntelQCopter project - student attribution to confirm')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
css='body{margin:0;background:#eaf1e9;color:#293b32;font:17px/1.65 Segoe UI,Arial,sans-serif}main{max-width:1050px;margin:auto}section{background:white;padding:45px;margin:24px 0;border-top:4px solid #198548;box-shadow:0 3px 16px #18382c14}h1{font-size:42px;color:#18382c}h2{color:#18382c;line-height:1.3}h3{color:#198548}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border:1px solid #cddfcf;padding:10px;text-align:left}th{background:#246b42;color:white}tr:nth-child(even){background:#f2f7f1}img{display:block;max-width:100%;max-height:550px;margin:16px auto}.cover img{width:100%;max-height:none}.toc{font-size:14px;margin:3px;display:flex;justify-content:space-between;border-bottom:1px dotted #cddfcf}.toc a{text-decoration:none;color:#18382c}a{color:#198548}@media print{section{break-before:page;box-shadow:none;margin:0;padding:12px}section:first-child{break-before:auto}}'
(OUT/'technical-report.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>IntelQCopter Autonomous Quadcopter</title><style>'+css+'</style><main><section>'+''.join(web)+'</section></main></html>',encoding='utf-8')
(OUT/'section-pages.json').write_text(json.dumps(PAGES,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'sections':len(HEADINGS),'bom_rows':len(bom),'section_pages':PAGES},indent=2))


