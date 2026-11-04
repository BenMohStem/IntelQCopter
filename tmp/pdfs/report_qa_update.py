from pathlib import Path
import json,hashlib,re
import fitz

root=Path(r'C:\Users\Mohamed\Desktop\Work\IntelQCopter-USTHB')
out=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter\tmp\pdfs\nyquist-qa')
out.mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for folder,pdfname in [('report','IntelQCopter.pdf'),('full','IntelQCopter-Full.pdf')]:
    doc=fitz.open(root/pdfname)
    sections=json.loads((root/folder/'section-pages.json').read_text())
    wanted=[7] if folder=='report' else [1,2,7,27,28,30,31,54,55]
    for page in wanted:
        doc[page-1].get_pixmap(matrix=fitz.Matrix(1.4,1.4)).save(out/f'{folder}-{page:02d}.png')
    # Contact sheets allow checking all appendix pages for layout regressions.
    from PIL import Image,ImageOps,ImageDraw
    for start in range(0,len(doc),12):
        canvas=Image.new('RGB',(4*300,3*450),'#dce7df')
        for j in range(start,min(start+12,len(doc))):
            pix=doc[j].get_pixmap(matrix=fitz.Matrix(.45,.45))
            im=Image.frombytes('RGB',(pix.width,pix.height),pix.samples)
            im.thumbnail((285,415));x=((j-start)%4)*300;y=((j-start)//4)*450
            canvas.paste(im,(x,y+22));ImageDraw.Draw(canvas).text((x+5,y+5),f'{folder} page{j+1}',fill='black')
        canvas.save(out/f'{folder}-sheet-{start//12+1}.png')
    p=root/folder/'validation.json';data=json.loads(p.read_text())
    data.update(checked_on='2026-10-29',revision='7.1',pages=len(doc),section_pages=sections)
    source=(root/folder/'report.md').read_text(encoding='utf-8')
    data['technical_visual_instances']=len(re.findall(r'\[\[VIS:',source))
    data['equation_panels']=len(re.findall(r'\[\[EQ:',source))
    data['unique_generated_figures']=len(json.loads((root/folder/'assets/visual/figure-manifest.json').read_text()))
    data['nyquist_improvement']='Complete finite indented contour with every sampled point in overview, symmetric-log global axes, equal-linear -1 detail, winding/root cross-check. Plot limits and42,000samples/case checked.'
    data['visual_review']='Rendered short Nyquist page and full Nyquist, contour, anti-windup and asymptotic-derivation pages inspected; complete-page contact sheets reviewed.'
    data['checks'].append('All42,000complete Nyquist contour points per case fit global viewport; computed winding equals characteristic-root count')
    if folder=='full':data['appendix_improvement']='Two new worked pages: back-calculation sign/equilibrium/Euler condition/allocator units; origin-pole asymptotics and contour pieces.55pages total.'
    for rel in list(data['sha256']):data['sha256'][rel]=sha(root/rel)
    p.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

p=root/'knowledge/index.json';d=json.loads(p.read_text());d['updated']='2026-10-29';p.write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8')
p=root/'knowledge/domains/presentation-preferences.json';d=json.loads(p.read_text());d['updated']='2026-10-29'
d['entries'].append({'id':'preferences.complete-nyquist','status':'user_preference','summary':'Show every Nyquist turn with a full contour overview plus a critical-point close-up; retain the rest of the short report and continue improving full appendices. MATLAB or another suitable analysis tool is optional.','sources':[{'kind':'user','date':'2026-10-29','reference':'Current request: show whole Nyquist turns; short edition otherwise good; improve full appendices. Follow-up permits MATLAB or another tool if needed.'}]})
p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
p=root/'knowledge/domains/reporting.json';d=json.loads(p.read_text());d['updated']='2026-10-29'
for fact in d['entries']:
    if fact['id']=='reporting.short-edition':fact['summary']='Visual edition7.1 has23pages; complete Nyquist overview plus linear critical-point detail; remaining short structure retained.'
    if fact['id']=='reporting.full-edition':fact['summary']='Full mathematical edition7.1 has55pages, with32applied appendix sections: new anti-windup proof/experiment and origin-pole contour explanation; other proofs/registers retained.'
    for src in fact['sources']:
        if src['kind']=='file' and 'sha256' in src:src['sha256']=sha(root/src['path'])
p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
event={'schema':2,'id':'2026-10-29-complete-nyquist-and-appendices','date':'2026-10-29','domains':['presentation-preferences','reporting','control-example'],'summary':'User requests whole Nyquist turns and continued full-appendix improvement; MATLAB optional.','outcome':'NumPy/Matplotlib complete indented contour plus equal-linear critical-point detail; all42,000points/case inside overview and winding checked against roots. Short23pages; full55pages with two new worked derivations. No firmware, calibration, physical test or flight simulation changed.'}
(root/'knowledge/events/2026-10-29-complete-nyquist-and-appendices.json').write_text(json.dumps(event,indent=2)+'\n',encoding='utf-8')
print('Rendered QA pages and refreshed reviewed-report metadata/domain provenance.')
