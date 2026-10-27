"""Native KiCad fixture controls for the optional single 3.3 V header policy.

These targeted controls use an incomplete candidate with synthetic changes;
they do not purport to validate a complete PCB or measured hardware.
"""
from pathlib import Path
import hashlib,json,shutil,sys
import pcbnew as p
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_zero import assess

BASE=Path(__file__).resolve().parent/'fixtures/power-supply-base'
import argparse,datetime
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'hardware/zero-jumper-study/method/runs'/("single-3v3-controls-"+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')))
OUT=parser.parse_args().output
if OUT.exists():
    raise SystemExit('Choose a new results directory; do not overwrite evidence')
OUT.mkdir(parents=True)
results=[]

def make(name,unused_pin=None,bridge=False):
    dest=OUT/name;dest.mkdir()
    for filename in ['QCopter.kicad_pcb','QCopter.kicad_sch','QCopter.kicad_pro']:
        shutil.copy2(BASE/filename,dest/filename)
    b=p.LoadBoard(str(dest/'QCopter.kicad_pcb'))
    u=next(f for f in b.GetFootprints() if f.GetReference()=='U1')
    pads={a.GetNumber():a for a in u.Pads()}
    assert pads['1'].GetNetname()=='3V3' and pads['2'].GetNetname()=='3V3'
    if unused_pin:
        pads[unused_pin].SetNetCode(0)
    if bridge:
        track=p.PCB_TRACK(b);track.SetLayer(p.B_Cu);track.SetWidth(p.FromMM(1.2))
        track.SetNet(b.FindNet('3V3'));track.SetStart(pads[unused_pin].GetPosition())
        track.SetEnd(pads['1' if unused_pin=='2' else '2'].GetPosition());b.Add(track)
    p.SaveBoard(str(dest/'QCopter.kicad_pcb'),b)
    return dest

def check(name,project,selected,expect):
    d=assess(project,False,OUT/(name+'-report'),'0x77',selected)
    checks={c['check']:c['pass'] for c in d['checks']}
    for key,want in expect.items():
        assert checks.get(key)==want,(name,key,want,checks.get(key))
    assert not d['cad_zero_jumper_acceptance'],'Synthetic fixture must never be accepted as PCB'
    results.append({'case':name,'selected_3v3_pad':selected,'target_checks':{k:checks[k] for k in expect},
                    'complete_PCB_accepted':False,'board_sha256':hashlib.sha256((project/'QCopter.kicad_pcb').read_bytes()).hexdigest()})

both=make('both-connected')
one=make('only-pad-1',unused_pin='2')
two=make('only-pad-2',unused_pin='1')
short=make('unused-pad-2-physically-shorted',unused_pin='2',bridge=True)
check('default-both-connected',both,None,{'esp_power_1':True,'esp_power_2':True})
check('default-rejects-pad-2-unused',one,None,{'esp_power_1':True,'esp_power_2':False})
check('option-pad-1-accepts-local-pin-contract',one,'1',{'esp_power_1':True,'unused_duplicate_3v3_2':True,'unused_duplicate_3v3_has_no_carrier_copper_connection':True})
check('option-pad-2-accepts-local-pin-contract',two,'2',{'esp_power_2':True,'unused_duplicate_3v3_1':True,'unused_duplicate_3v3_has_no_carrier_copper_connection':True})
check('option-rejects-both-wired',both,'1',{'unused_duplicate_3v3_2':False})
check('option-rejects-wrong-selected-pad',two,'1',{'esp_power_1':False,'unused_duplicate_3v3_2':False})
check('option-rejects-unused-pad-short',short,'1',{'unused_duplicate_3v3_2':True,'unused_duplicate_3v3_has_no_carrier_copper_connection':False})
(OUT/'controls.json').write_text(json.dumps({'engine':'KiCad 10 actual PCB pad/net/track fixtures plus independent validator; native DRC deliberately skipped in focused controls','cases':results,'all_controls_pass':True,'limit':'Incomplete intentionally altered fixtures; no successful zero-jumper PCB claim'},indent=2)+'\n',encoding='utf-8')
print('PASS:',len(results),'single-3V3 positive/negative KiCad fixture controls. No full-board acceptance asserted.')
