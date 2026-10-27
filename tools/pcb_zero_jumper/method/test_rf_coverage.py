"""Test actual KiCad rule-polygon coverage of a nominal antenna region."""
from pathlib import Path
import json,sys
import pcbnew as p
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_zero import rf_rule_covers_antenna

board=p.BOARD()
antenna={'kind':'polygon','outer':[(0,0),(10,0),(10,5),(0,5)],'holes':[]}
def area(points,hole=None):
    z=p.ZONE(board);z.SetIsRuleArea(True);poly=z.Outline();poly.NewOutline()
    for x,y in points:poly.Append(p.FromMM(x),p.FromMM(y))
    if hole:
        i=poly.NewHole(0)
        for x,y in hole:poly.Append(p.FromMM(x),p.FromMM(y),0,i)
    board.Add(z)
    return z
full=area([(-1,-1),(11,-1),(11,6),(-1,6)])
unrelated=area([(20,20),(30,20),(30,30),(20,30)])
left=area([(-1,-1),(5,-1),(5,6),(-1,6)])
right=area([(5,-1),(11,-1),(11,6),(5,6)])
with_hole=area([(-1,-1),(11,-1),(11,6),(-1,6)],[(4,2),(6,2),(6,3),(4,3)])
tests=[('complete_rule_covers',antenna,[full],True),('wrong_location_rejected',antenna,[unrelated],False),
       ('partial_rule_rejected',antenna,[left],False),('union_of_two_rules_covers',antenna,[left,right],True),
       ('hole_in_rule_rejected',antenna,[with_hole],False),('missing_rule_rejected',antenna,[],False),
       ('missing_antenna_rejected',None,[full],False)]
out=[]
for name,shape,zones,want in tests:
    actual=rf_rule_covers_antenna(shape,zones)
    assert actual==want,(name,actual,want)
    out.append({'case':name,'observed':actual,'expected':want,'pass':True})
import argparse,datetime
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'hardware/zero-jumper-study/method/runs'/("rf-rule-coverage-controls-"+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json'))
path=parser.parse_args().output
if path.exists():raise SystemExit('Refusing to overwrite existing evidence')
path.write_text(json.dumps({'engine':'KiCad 10 native SHAPE_POLY_SET boolean subtraction, integer coordinates','cases':out,'all_controls_pass':True,'limits':'Nominal antenna shape and geometric rule coverage only; no RF performance or physical-fit claim'},indent=2)+'\n',encoding='utf-8')
print('PASS:',len(out),'native rule-area coverage controls, including an internal hole')
