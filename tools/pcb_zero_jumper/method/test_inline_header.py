"""Analytic controls for straight, ordered 2.54 mm header geometry."""
from pathlib import Path
import json,math,sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_zero import inline_header_geometry
p=2.54
axis=[(5+i*p,7) for i in range(8)]
angle=math.radians(37)
rotated=[(5+i*p*math.cos(angle),7+i*p*math.sin(angle)) for i in range(8)]
curved=[(0,0),(p,0),(0,2*p)]
misordered=[(0,0),(p,0),(3*p,0),(2*p,0)]
cases=[('horizontal_row',axis,True),('arbitrarily_rotated_row',rotated,True),('reversed_row',list(reversed(axis)),True),
       ('curved_but_correct_radial_distances',curved,False),('pin_order_swapped',misordered,False),
       ('wrong_pitch',[(0,0),(2.5,0),(5,0)],False),('one_pad_only',[(0,0)],False)]
out=[]
for name,points,want in cases:
    got=inline_header_geometry(points)
    assert got==want,(name,got,want)
    out.append({'case':name,'expected':want,'observed':got,'pass':True})
import argparse,datetime
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'hardware/zero-jumper-study/method/runs'/("inline-header-controls-"+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json'))
path=parser.parse_args().output
if path.exists():raise SystemExit('Choose a new results file; preserve evidence')
path.write_text(json.dumps({'engine':'Analytic straight-line geometry using PCB coordinates in mm','cases':out,'all_controls_pass':True,'limits':'Tests hole pattern only; actual module pin labels and physical body dimensions remain unverified'},indent=2)+'\n',encoding='utf-8')
print('PASS:',len(out),'inline-header geometry controls')
