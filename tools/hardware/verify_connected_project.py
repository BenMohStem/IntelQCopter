"""Run native KiCad connectivity checks and preserve original outputs."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project',type=Path,default=ROOT/'hardware/connected-i2c')
parser.add_argument('--kicad-bin',type=Path,default=Path(os.environ['LOCALAPPDATA'])/'Programs/KiCad/10.0/bin')
args=parser.parse_args()
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
out=ROOT/'.local/connected-project-checks'/stamp
out.mkdir(parents=True)
commands=[
    [str(args.kicad_bin/'kicad-cli.exe'),'sch','export','netlist','--format','kicadxml','-o',str(out/'netlist.xml'),str(args.project/'QCopter.kicad_sch')],
    [str(args.kicad_bin/'python.exe'),str(ROOT/'tools/hardware/check_connected_netlist.py'),str(args.project),'--netlist',str(out/'netlist.xml')],
]
for number,command in enumerate(commands):
    result=subprocess.run(command,cwd=ROOT,capture_output=True,text=True)
    (out/f'{number}.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode:
        print(result.stdout+result.stderr)
        raise SystemExit(result.returncode)
    if number:
        print(result.stdout.strip())
(out/'summary.json').write_text(json.dumps({'success':True,'engine':'KiCad native schematic/netlist/board parser','physical_test':False,'uploaded':False,'routed':False,'project':str(args.project)},indent=2)+'\n',encoding='utf-8')
print('Original validation evidence:',out)
