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
from reportlab.graphics.shapes import Drawing, Rect, String, Line, Circle, Polygon
from reportlab.graphics import renderPM
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
pdfmetrics.registerFont(TTFont('ReportArial', r'C:\Windows\Fonts\arial.ttf'))
pdfmetrics.registerFont(TTFont('ReportArialBold', r'C:\Windows\Fonts\arialbd.ttf'))
pdfmetrics.registerFontFamily('ReportArial', normal='ReportArial', bold='ReportArialBold')
INK = colors.HexColor('#172f46')
TEAL = colors.HexColor('#116d78')
STYLES = {
    'title': ParagraphStyle('title', fontName='ReportArialBold', fontSize=29, leading=34, spaceAfter=17, textColor=colors.black),
    'h2': ParagraphStyle('h2', fontName='ReportArialBold', fontSize=18, leading=23, spaceAfter=13, textColor=INK),
    'h3': ParagraphStyle('h3', fontName='ReportArialBold', fontSize=11, leading=15, spaceBefore=8, spaceAfter=6, textColor=TEAL),
    'body': ParagraphStyle('body', fontName='ReportArial', fontSize=10.2, leading=14.4, spaceAfter=8, textColor=colors.HexColor('#263645')),
    'cell': ParagraphStyle('cell', fontName='ReportArial', fontSize=8.2, leading=11.2),
    'toc': ParagraphStyle('toc', fontName='ReportArial', fontSize=8.8, leading=11.3, spaceAfter=1),
    'caption': ParagraphStyle('caption', fontName='ReportArial', fontSize=8.7, leading=11.8, spaceAfter=9, textColor=colors.HexColor('#506274')),
}
HEADINGS = re.findall(r'^## (.+)$', TEXT, re.M)
PAGES = {}

def inline(text):
    text = html.escape(text)
    text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
    text = re.sub(r'`(.+?)`', r'<font color="#116d78">\1</font>', text)
    return text

class Heading(Paragraph):
    def __init__(self, title):
        super().__init__(inline(title), STYLES['h2'])
        self.section_title = title

class ReportDoc(SimpleDocTemplate):
    def afterFlowable(self, flowable):
        if hasattr(flowable, 'section_title'):
            PAGES[flowable.section_title] = self.page

def footer(c, doc):
    c.setStrokeColor(TEAL)
    c.line(42, 805, 553, 805)
    c.setFont('ReportArial', 8)
    c.setFillColor(colors.HexColor('#536575'))
    c.drawString(42, 816, 'IntelQCopter  |  Professional and Personal Project  |  Revision 2.0')
    c.drawString(42, 25, '28 October 2026  |  University submission draft  |  Evidence and targets distinguished')
    c.drawRightString(553, 25, str(doc.page))

def architecture():
    d = Drawing(511, 153)
    boxes = [(0,90,112,45,'Xbox input','BLE to verify'), (148,90,172,45,'ESP32-S3 FC','Sensors / estimate / PID'), (358,90,153,45,'ESCs and motors','Outputs disabled'), (0,9,112,46,'Ground station','Approved mission'), (148,9,172,46,'ROS 2 companion','Optional perception / planner')]
    for x,y,w,h,a,b in boxes:
        d.add(Rect(x,y,w,h,fillColor=colors.HexColor('#edf4f6'),strokeColor=TEAL,strokeWidth=.8))
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
    fig=plt.figure(figsize=(9,0.65), dpi=220)
    fig.text(.5,.5,'$'+latex+'$',ha='center',va='center',fontsize=14,color='#172f46')
    path=ASSETS/f'equation-{index:02d}.png'
    fig.savefig(path,bbox_inches='tight',pad_inches=.10,facecolor='white')
    plt.close(fig)
    return path

EVIDENCE = [
    ('P1','measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md'),
    ('P2','measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md'),
    ('P3','measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md'),
    ('P4','measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md'),
    ('P5','measurements/software/2026-10-27-input-guard/summary.json'),
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
bom=[]
for line in TEXT.splitlines():
    if re.match(r'\| (?:M|T|O|F)\d{2} x',line):
        c=[v.strip() for v in line.strip('|').split('|')]
        match=re.match(r'([MTOF]\d{2}) x(.+)',c[0])
        category={'M':'Aircraft','T':'Test equipment','O':'Optional companion','F':'Future Pico and radio'}[match[1][0]]
        bom.append({'id':match[1],'category':category,'quantity':match[2],'component':c[1],'purpose_and_checks':c[2],'exact_model':'TBD','owned_or_to_buy':'Confirm','supplier':'','currency':'','quote_date':'','unit_cost':'','shipping_allocation':'','verified_mass_g':''})
with (OUT/'bill-of-materials.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(bom[0]),lineterminator="\n");writer.writeheader();writer.writerows(bom)

def make_story(toc_pages):
    story=[]; web=[]; eq=0
    lines=TEXT.splitlines();i=0
    while i<len(lines):
        line=lines[i]
        if not line.strip():i+=1;continue
        if line=='---':story.append(PageBreak());web.append('</section><section>');i+=1;continue
        if line=='[[CONTENTS]]':
            for title in HEADINGS[2:]:
                label=title+'  '+str(toc_pages.get(title,''))
                story.append(Paragraph(inline(label),STYLES['toc']))
                web.append('<p class="toc"><a href="#'+str(HEADINGS.index(title))+'">'+inline(title)+'</a></p>')
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
            data=[[Paragraph(inline(c),STYLES['cell']) for c in r] for r in rows]
            t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
            pad=4 if len(rows)>1 and rows[1][0].startswith('M16 ') else 6
            t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dcebf0')),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f6f8fa')]),('GRID',(0,0),(-1,-1),.3,colors.HexColor('#cbd5de')),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),pad),('BOTTOMPADDING',(0,0),(-1,-1),pad)]))
            story.extend([t,Spacer(1,9)])
            web.append('<table>'+''.join('<tr>'+''.join('<'+('th' if j==0 else 'td')+'>'+inline(c)+'</'+('th' if j==0 else 'td')+'>' for c in r)+'</tr>' for j,r in enumerate(rows))+'</table>')
            continue
        else:
            text=line
            while i+1<len(lines) and lines[i+1].strip() and not lines[i+1].startswith(('#','|','---','[[')):
                i+=1;text+=' '+lines[i]
            style=STYLES['caption'] if text.startswith('Figure ') else STYLES['body']
            story.append(Paragraph(inline(text),style));web.append('<p>'+inline(text)+'</p>')
        i+=1
    return story,web

target=OUT/'technical-report.pdf'
for run in range(2):
    story,web=make_story(dict(PAGES))
    doc=ReportDoc(str(target),pagesize=(595.28,841.89),leftMargin=42,rightMargin=42,topMargin=56,bottomMargin=47,title='IntelQCopter Professional and Personal Project',author='IntelQCopter project - student attribution to confirm')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
css='body{margin:0;background:#edf2f6;color:#263645;font:17px/1.65 Segoe UI,Arial,sans-serif}main{max-width:1050px;margin:auto}section{background:white;padding:45px;margin:24px 0;box-shadow:0 3px 16px #1232}h1{font-size:42px;color:black}h2{color:#172f46;line-height:1.3}h3{color:#116d78}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border:1px solid #cbd5de;padding:10px;text-align:left}th{background:#dcebf0}tr:nth-child(even){background:#f6f8fa}img{display:block;max-width:100%;max-height:550px;margin:16px auto}.toc{font-size:14px;margin:3px}a{color:#116d78}@media print{section{break-before:page;box-shadow:none;margin:0;padding:12px}section:first-child{break-before:auto}}'
(OUT/'technical-report.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>IntelQCopter Professional and Personal Project</title><style>'+css+'</style><main><section>'+''.join(web)+'</section></main></html>',encoding='utf-8')
(OUT/'section-pages.json').write_text(json.dumps(PAGES,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'sections':len(HEADINGS),'bom_rows':len(bom),'section_pages':PAGES},indent=2))
