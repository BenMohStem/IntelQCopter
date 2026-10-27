"""Synthetic native-KiCad copper graph controls, not a complete PCB test."""
from pathlib import Path
import json,shutil,sys
import pcbnew as p
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_zero import assess
BASE=Path(__file__).resolve().parent/'fixtures/power-supply-base'
import argparse,datetime
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'hardware/zero-jumper-study/method/runs'/("single-3v3-power-controls-"+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')))
OUT=parser.parse_args().output
if OUT.exists():raise SystemExit('Choose a new report directory to preserve evidence')
OUT.mkdir(parents=True)
cases=[]
def build(name,used,omit=None):
    dest=OUT/name;dest.mkdir()
    for filename in ['QCopter.kicad_pcb','QCopter.kicad_sch','QCopter.kicad_pro']:shutil.copy2(BASE/filename,dest/filename)
    b=p.LoadBoard(str(dest/'QCopter.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
    def pad(ref,num):return next(a for a in fps[ref].Pads() if a.GetNumber()==num)
    pad('U1','2' if used=='1' else '1').SetNetCode(0)
    def line(a,c):
        t=p.PCB_TRACK(b);t.SetLayer(p.B_Cu);t.SetNet(b.FindNet('3V3'));t.SetWidth(p.FromMM(1.2));t.SetStart(a);t.SetEnd(c);b.Add(t)
    def v(x,y):return p.VECTOR2I(p.FromMM(x),p.FromMM(y))
    origin=pad('U1',used).GetPosition();a=v(25,p.ToMM(origin.y));hub=v(25,75)
    line(origin,a);line(a,hub)
    for key in [('U2','1'),('U3','1'),('U3','5'),('U3','6'),('U4','1')]:
        if key!=omit:line(hub,pad(*key).GetPosition())
    p.SaveBoard(str(dest/'QCopter.kicad_pcb'),b)
    return dest
for name,used,omit,want in [('used-pad-1-powered','1',None,True),('used-pad-2-powered','2',None,True),('sensor-vcc-disconnected','1',('U4','1'),False)]:
    project=build(name,used,omit);result=assess(project,False,OUT/(name+'-report'),'0x77',used)
    checks={c['check']:c['pass'] for c in result['checks']}
    assert checks['all_3V3_endpoints_connected_in_bare_copper']==want,(name,checks['all_3V3_endpoints_connected_in_bare_copper'])
    assert checks['unused_duplicate_3v3_has_no_carrier_copper_connection']
    assert not result['cad_zero_jumper_acceptance']
    cases.append({'case':name,'power_graph_pass':want,'full_pcb_accepted':False})
(OUT/'controls.json').write_text(json.dumps({'engine':'Native KiCad synthetic connected/disconnected 3V3 copper geometry; native DRC intentionally not run','cases':cases,'all_controls_pass':True,'limits':'Synthetic tracks may intersect unrelated nets; only the targeted graph is tested; never manufacture these fixtures.'},indent=2)+'\n',encoding='utf-8')
print('PASS:',len(cases),'3V3 copper-connectivity controls for both options and disconnected VCC. Synthetic boards not accepted.')
