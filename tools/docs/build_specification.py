"""Build the reviewed French specification from its Markdown source."""
from pathlib import Path
import html
import re
import sys
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.enums import TA_CENTER

root = Path(sys.argv[1])
out = root / 'docs/specification'
out.mkdir(parents=True, exist_ok=True)
source = out / 'cahier-des-charges.md'
body = source.read_text(encoding='utf-8')
pdfmetrics.registerFont(TTFont('Arial', r'C:\Windows\Fonts\arial.ttf'))
pdfmetrics.registerFont(TTFont('Arial-Bold', r'C:\Windows\Fonts\arialbd.ttf'))
pdfmetrics.registerFontFamily('Arial', normal='Arial', bold='Arial-Bold')
styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='TextFR', fontName='Arial', fontSize=9.3, leading=14, spaceAfter=8, textColor=colors.HexColor('#243449')))
styles.add(ParagraphStyle(name='TitleFR', fontName='Arial-Bold', fontSize=27, leading=33, spaceAfter=20, textColor=colors.HexColor('#12344f')))
styles.add(ParagraphStyle(name='HeadFR', fontName='Arial-Bold', fontSize=19, leading=24, spaceAfter=15, textColor=colors.HexColor('#12344f')))
styles.add(ParagraphStyle(name='SubFR', fontName='Arial-Bold', fontSize=11, leading=16, spaceBefore=9, spaceAfter=7, textColor=colors.HexColor('#007f89')))
styles.add(ParagraphStyle(name='CellFR', fontName='Arial', fontSize=8, leading=11, spaceAfter=0))
def inline(s):
    s=html.escape(s)
    s=re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', s)
    s=re.sub(r'`(.+?)`', r'<font color="#007f89">\1</font>', s)
    return s
story=[]
web=[]
lines=body.splitlines()
i=0
while i<len(lines):
    line=lines[i]
    if line=='---':
        story.append(PageBreak()); web.append('</section><section>'); i+=1; continue
    if not line.strip(): i+=1; continue
    if line.startswith('# '):
        text=line[2:]; story.append(Paragraph(inline(text),styles['TitleFR']));web.append('<h1>'+inline(text)+'</h1>')
    elif line.startswith('## '):
        text=line[3:];story.append(Paragraph(inline(text),styles['HeadFR']));web.append('<h2>'+inline(text)+'</h2>')
    elif line.startswith('### '):
        text=line[4:];story.append(Paragraph(inline(text),styles['SubFR']));web.append('<h3>'+inline(text)+'</h3>')
    elif line.startswith('|'):
        rows=[]
        while i<len(lines) and lines[i].startswith('|'):
            cells=[c.strip() for c in lines[i].strip('|').split('|')]
            if not all(re.fullmatch(r'[-: ]+',c) for c in cells): rows.append(cells)
            i+=1
        n=len(rows[0]); widths=([72,190,249] if n==3 else [160,351])
        table=Table([[Paragraph(inline(c),styles['CellFR']) for c in row] for row in rows],colWidths=widths,repeatRows=1,hAlign='LEFT')
        table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceef0')),('VALIGN',(0,0),(-1,-1),'TOP'),('GRID',(0,0),(-1,-1),0.3,colors.HexColor('#c5d2dc')),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7)]))
        story.extend([table,Spacer(1,9)])
        web.append('<table>'+''.join('<tr>'+''.join('<'+('th' if j==0 else 'td')+'>'+inline(c)+'</'+('th' if j==0 else 'td')+'>' for c in row)+'</tr>' for j,row in enumerate(rows))+'</table>')
        continue
    else:
        text=line
        while i+1<len(lines) and lines[i+1].strip() and not lines[i+1].startswith(('#','|','---')):
            i+=1;text+=' '+lines[i]
        story.append(Paragraph(inline(text),styles['TextFR']));web.append('<p>'+inline(text)+'</p>')
    i+=1
def page(c,doc):
    c.setStrokeColor(colors.HexColor('#007f89'));c.line(42,805,553,805)
    c.setFont('Arial',8);c.setFillColor(colors.HexColor('#56677a'))
    c.drawString(42,816,'IntelQCopter  /  Spécification système  /  Révision 1.0')
    c.drawString(42,25,'28 octobre 2026 • Document de travail • Acceptation physique distincte')
    c.drawRightString(553,25,str(doc.page))
doc=SimpleDocTemplate(str(out/'cahier-des-charges.pdf'),pagesize=(595.28,841.89),rightMargin=42,leftMargin=42,topMargin=58,bottomMargin=47,title='IntelQCopter - Cahier des charges système',author='Projet IntelQCopter')
doc.build(story,onFirstPage=page,onLaterPages=page)
css='body{margin:0;background:#edf2f6;color:#243449;font:16px/1.65 Segoe UI,Arial,sans-serif}main{max-width:1000px;margin:auto}section{background:white;padding:48px;margin:24px 0;border-top:5px solid #007f89;box-shadow:0 4px 18px #1232}h1{font-size:42px;line-height:1.2}h1,h2{color:#12344f}h3{color:#007f89}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border:1px solid #c5d2dc;padding:12px;text-align:left;vertical-align:top}th{background:#dceef0}@media print{body{background:white}section{break-before:page;box-shadow:none;margin:0;padding:15px}section:first-child{break-before:auto}}'
(out/'cahier-des-charges.html').write_text('<!doctype html><html lang="fr"><meta charset="utf-8"><title>IntelQCopter — Cahier des charges</title><style>'+css+'</style><main><section>'+''.join(web)+'</section></main></html>',encoding='utf-8')
print('Built PDF and standalone HTML from',source)
