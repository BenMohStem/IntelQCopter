"""Run native KiCad DRC and XML-netlist checks; preserve every original result."""
import argparse,datetime,json,os,subprocess,sys
from pathlib import Path
R=Path(__file__).resolve().parents[2]
arg=argparse.ArgumentParser(description=__doc__)
arg.add_argument('--project',type=Path,default=R/'hardware/home-etched-v2')
args=arg.parse_args()
binpath=Path(os.environ['LOCALAPPDATA'])/'Programs/KiCad/10.0/bin'
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
out=R/'.local/home-etched-checks'/stamp;out.mkdir(parents=True)
commands=[
    [str(binpath/'kicad-cli.exe'),'pcb','drc','--format','json','--output',str(out/'drc.json'),str(args.project/'QCopter.kicad_pcb')],
    [str(binpath/'kicad-cli.exe'),'sch','export','netlist','--format','kicadxml','--output',str(out/'netlist.xml'),str(args.project/'QCopter.kicad_sch')],
    [str(binpath/'python.exe'),str(R/'tools/hardware/check_routed_pcb.py'),str(args.project),'--netlist',str(out/'netlist.xml'),'--drc',str(out/'drc.json'),'--report',str(out/'verification.json')]]
with (out/'original.log').open('w',encoding='utf-8') as log:
    for command in commands:
        result=subprocess.run(command,capture_output=True,text=True)
        log.write('$ '+subprocess.list2cmdline(command)+'\n'+result.stdout+result.stderr+'\n');log.flush()
        print(result.stdout,end='')
        if result.returncode:print(result.stderr);raise SystemExit(result.returncode)
print('Original check evidence:',out)
