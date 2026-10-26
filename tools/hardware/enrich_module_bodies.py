"""Add sourced reference outlines; native reloading verifies pads and placement."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import pcbnew


def children(source):
    depth=0
    quoted=False
    escaped=False
    start=None
    result=[]
    for index,char in enumerate(source):
        if quoted:
            if escaped:
                escaped=False
            elif char=='\\':
                escaped=True
            elif char=='"':
                quoted=False
            continue
        if char=='"':
            quoted=True
        elif char=='(':
            depth+=1
            if depth==2:
                start=index
        elif char==')':
            if depth==2:
                result.append((start,index+1))
            depth-=1
            if depth<0:
                raise ValueError('Unbalanced CAD text')
    if quoted or depth:
        raise ValueError('Unbalanced CAD text')
    return result


def caption(value,x,y,layer,size=.7,angle=0):
    return f'(fp_text user {json.dumps(value)} (at {x:g} {y:g} {angle}) (layer "{layer}") (effects (font (size {size} {size}) (thickness 0.12))))'


def compact_blank_lines(source):
    result=[]
    quoted=False
    escaped=False
    for line in source.splitlines():
        inside_string=quoted
        for char in line:
            if escaped:
                escaped=False
            elif quoted and char=='\\':
                escaped=True
            elif char=='"':
                quoted=not quoted
        if inside_string or line.strip():
            result.append(line)
    return '\n'.join(result)


def geometry(spec,config):
    span=(spec['header_pins']-1)*2.54
    left=-config['header_edge_offset_mm']
    top=-(spec['along_header_mm']-span)/2
    return [left,top,left+spec['perpendicular_mm'],top+spec['along_header_mm']]


def enriched_text(source,spec,config):
    left,top,right,bottom=geometry(spec,config)
    margin=config['courtyard_margin_mm']
    edits=[]
    for start,end in children(source):
        part=source[start:end]
        key=re.match(r'\(([^\s()]+)',part).group(1)
        if key.startswith('fp_') and key!='fp_text' and re.search(r'\(layer "F\.(?:Fab|SilkS|CrtYd)"\)',part):
            edits.append((start,end,''))
        elif key=='fp_text' and re.search(r'"(?:BODY REF|COMPONENTS UP|1 VCC)',part):
            edits.append((start,end,''))
        elif key=='property' and re.match(r'\(property\s+"(?:Reference|Value)"',part):
            y=top-margin-1.5 if re.match(r'\(property\s+"Reference"',part) else bottom+margin+1.5
            replacement=re.sub(r'\(at\s+[-\d.]+\s+[-\d.]+',f'(at {(left+right)/2:g} {y:g}',part,count=1)
            edits.append((start,end,replacement))
        elif key=='descr':
            edits.append((start,end,'(descr "Sourced nominal body; components up, body RIGHT of VCC-first header; assumed edge offset; 2 mm placement margin; verify actual module")'))
    for start,end,replacement in reversed(edits):
        source=source[:start]+replacement+source[end:]
    added=[]
    for layer,width,bounds in [('F.Fab',.1,[left,top,right,bottom]),('F.SilkS',.12,[left,top,right,bottom]),
                               ('F.CrtYd',.05,[left-margin,top-margin,right+margin,bottom+margin])]:
        x1,y1,x2,y2=bounds
        added.append(f'(fp_rect (start {x1:g} {y1:g}) (end {x2:g} {y2:g}) (stroke (width {width}) (type default)) (fill none) (layer "{layer}"))')
    for start,end in [((-3.4,-.6),(-2.1,0)),((-2.1,0),(-3.4,.6)),((-3.4,.6),(-3.4,-.6))]:
        added.append(f'(fp_line (start {start[0]} {start[1]}) (end {end[0]} {end[1]}) (stroke (width 0.15) (type default)) (layer "F.SilkS"))')
    added.append(caption('1 VCC',0,top-1,'F.SilkS',.8))
    added.append(caption(f"BODY REF {spec['along_header_mm']:g} x {spec['perpendicular_mm']:g} mm",(left+right)/2,bottom+.9,'F.Fab'))
    added.append(caption('COMPONENTS UP',right-1.4,(top+bottom)/2,'F.Fab',.65,90))
    closing=source.rfind(')')
    return compact_blank_lines(source[:closing]+'\n'+'\n'.join(added)+'\n'+source[closing:])+'\n'


def pad_state(fp):
    return sorted((p.GetNumber(),p.GetPosition().x,p.GetPosition().y,p.GetSize().x,p.GetSize().y,
                   p.GetDrillSize().x,p.GetDrillSize().y,p.GetNetCode(),p.GetShape()) for p in fp.Pads())


def enrich_library(project,config_path):
    config=json.loads(Path(config_path).read_text(encoding='utf-8'))
    library=Path(project)/'QCopterModules.pretty'
    results={}
    for name,spec in config['modules'].items():
        path=library/(name+'.kicad_mod')
        original=path.read_text(encoding='utf-8')
        before=pad_state(pcbnew.FootprintLoad(str(library),name))
        path.write_text(enriched_text(original,spec,config),encoding='utf-8',newline='\n')
        checked=pcbnew.FootprintLoad(str(library),name)
        if checked is None or pad_state(checked)!=before:
            path.write_text(original,encoding='utf-8',newline='\n')
            raise ValueError('Native footprint rejected or pads changed: '+name)
        results[name]=geometry(spec,config)
    return results


def enrich_board(source,output,config_path):
    config=json.loads(Path(config_path).read_text(encoding='utf-8'))
    before=pcbnew.LoadBoard(str(source))
    pads={fp.GetReference():pad_state(fp) for fp in before.GetFootprints()}
    text=Path(source).read_text(encoding='utf-8')
    edits=[]
    updated=[]
    for start,end in children(text):
        part=text[start:end]
        match=re.match(r'\(footprint\s+"(?:QCopterModules:)?([^\"]+)"',part)
        if match and match.group(1) in config['modules']:
            layer=next((part[a:b] for a,b in children(part) if part[a:b].startswith('(layer ')),None)
            if layer!='(layer "F.Cu")':
                raise ValueError('Preview helper supports front modules; use native KiCad update for flipped footprints')
            edits.append((start,end,enriched_text(part,config['modules'][match.group(1)],config)))
            updated.append(match.group(1))
    for start,end,replacement in reversed(edits):
        text=text[:start]+replacement+text[end:]
    Path(output).write_text(text,encoding='utf-8',newline='\n')
    checked=pcbnew.LoadBoard(str(output))
    assert pads=={fp.GetReference():pad_state(fp) for fp in checked.GetFootprints()}
    assert len(list(before.GetTracks()))==len(list(checked.GetTracks()))
    assert len(list(before.Zones()))==len(list(checked.Zones()))
    assert len(updated)==3
    return {'updated_modules':updated,'pads_nets_placement_preserved':True,'source_board_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),
            'output_board_sha256':hashlib.sha256(Path(output).read_bytes()).hexdigest()}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project',type=Path)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--board-copy',type=Path)
    args=parser.parse_args()
    print(json.dumps({'library_bounds_mm':enrich_library(args.project,args.config)},indent=2))
    if args.board_copy:
        if args.board_copy.exists():
            raise SystemExit('Refusing to overwrite board copy')
        print(json.dumps(enrich_board(args.project/'QCopter.kicad_pcb',args.board_copy,args.config),indent=2))
