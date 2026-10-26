"""Run native KiCad checks on the module palette, preserving stdout/stderr locally."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project',type=Path,default=ROOT/'hardware/module-library')
parser.add_argument('--kicad-bin',type=Path,default=Path(os.environ['LOCALAPPDATA'])/'Programs/KiCad/10.0/bin')
args=parser.parse_args()
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
out=ROOT/'.local/module-library-checks'/stamp
out.mkdir(parents=True)
commands=[
    [str(args.kicad_bin/'kicad-cli.exe'),'sch','export','netlist','--format','kicadxml','-o',str(out/'parts.xml'),str(args.project/'QCopter.kicad_sch')],
    [str(args.kicad_bin/'python.exe'),str(ROOT/'tools/hardware/check_module_palette.py'),str(args.project),'--netlist',str(out/'parts.xml')],
]
for index,command in enumerate(commands):
    result=subprocess.run(command,cwd=ROOT,capture_output=True,text=True)
    (out/f'{index}.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode:
        print(result.stdout+result.stderr)
        raise SystemExit(result.returncode)
    if index:
        print(result.stdout.strip())
(out/'summary.json').write_text(json.dumps({'engine':'Native KiCad schematic/netlist/PCB parser','success':True,'project':str(args.project),'hardware_test':False},indent=2)+'\n',encoding='utf-8')
print('Original CAD check output:',out)
