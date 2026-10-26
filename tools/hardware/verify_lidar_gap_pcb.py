"""Native PCB parity/DRC and independent common-ground wire verification; no hardware."""
from pathlib import Path
import argparse,datetime,os,subprocess
R=Path(__file__).resolve().parents[2]
arg=argparse.ArgumentParser();arg.add_argument('--project',type=Path,default=R/'hardware/home-etched-v3/lidar-gap-two-ground-wires');args=arg.parse_args()
bin=Path(os.environ['LOCALAPPDATA'])/'Programs/KiCad/10.0/bin'
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');out=R/'.local/lidar-gap-checks'/stamp;out.mkdir(parents=True)
cmds=[[str(bin/'kicad-cli.exe'),'pcb','drc','--schematic-parity','--format','json','--output',str(out/'drc.json'),str(args.project/'QCopter.kicad_pcb')],
      [str(bin/'kicad-cli.exe'),'sch','export','netlist','--format','kicadxml','--output',str(out/'netlist.xml'),str(args.project/'QCopter.kicad_sch')],
      [str(bin/'python.exe'),str(R/'tools/hardware/check_lidar_gap_pcb.py'),str(args.project),'--netlist',str(out/'netlist.xml'),'--drc',str(out/'drc.json'),'--report',str(out/'verification.json')]]
with (out/'original.log').open('w',encoding='utf-8') as log:
    for cmd in cmds:
        result=subprocess.run(cmd,capture_output=True,text=True);log.write('$ '+subprocess.list2cmdline(cmd)+'\n'+result.stdout+result.stderr+'\n');log.flush();print(result.stdout,end='')
        if result.returncode:print(result.stderr);raise SystemExit(result.returncode)
print('Original verification evidence:',out)
