"""Native geometry check of nominal body rectangles and preservation contracts."""
import argparse
import json
from pathlib import Path
import shutil
import tempfile
import pcbnew
from enrich_module_bodies import geometry, pad_state, enrich_library

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('project',type=Path)
args=parser.parse_args()
path=args.project/'module-body-dimensions.json'
config=json.loads(path.read_text(encoding='utf-8'))
library=args.project/'QCopterModules.pretty'
counts={}
for name,spec in config['modules'].items():
    fp=pcbnew.FootprintLoad(str(library),name)
    rectangles={}
    for drawing in fp.GraphicalItems():
        if isinstance(drawing,pcbnew.PCB_SHAPE) and drawing.GetShape()==pcbnew.SHAPE_T_RECT:
            rectangles.setdefault(drawing.GetLayer(),[]).append(drawing)
    bounds=geometry(spec,config)
    for layer in [pcbnew.F_Fab,pcbnew.F_SilkS,pcbnew.F_CrtYd]:
        assert len(rectangles[layer])==1,name
        shape=rectangles[layer][0]
        expected=bounds[:]
        if layer==pcbnew.F_CrtYd:
            expected=[bounds[0]-2,bounds[1]-2,bounds[2]+2,bounds[3]+2]
        actual=[pcbnew.ToMM(shape.GetStart().x),pcbnew.ToMM(shape.GetStart().y),pcbnew.ToMM(shape.GetEnd().x),pcbnew.ToMM(shape.GetEnd().y)]
        assert max(abs(a-b) for a,b in zip(actual,expected))<1e-6,(name,actual,expected)
    assert len(list(fp.Pads()))==spec['header_pins']
    counts[name]=len(list(fp.GraphicalItems()))
# Reapplying must not add duplicate body drawings or alter pads.
before={name:pad_state(pcbnew.FootprintLoad(str(library),name)) for name in config['modules']}
with tempfile.TemporaryDirectory() as temp:
    copied=Path(temp)
    shutil.copytree(library,copied/'QCopterModules.pretty')
    enrich_library(copied,path)
    for name in config['modules']:
        fp=pcbnew.FootprintLoad(str(copied/'QCopterModules.pretty'),name)
        assert counts[name]==len(list(fp.GraphicalItems()))
        assert before[name]==pad_state(fp)
print('PASS: 3 native body rectangles, 3 silk outlines, 3 exact 2 mm courtyards; pad preservation and repeat-application checks passed.')
