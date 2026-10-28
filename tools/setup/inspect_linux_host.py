"""Read-only Linux host inventory. Does not mount disks, capture images or start services."""
import json,platform,shutil,subprocess,sys
from pathlib import Path

def probe(argv):
    if not shutil.which(argv[0]):return {'available':False}
    try:
        p=subprocess.run(argv,text=True,capture_output=True,timeout=10)
        return {'available':True,'returncode':p.returncode,'stdout':p.stdout.strip(),'stderr':p.stderr.strip()}
    except (OSError,subprocess.TimeoutExpired) as e:return {'available':True,'error':str(e)}

def main():
    report={'scope':'Read-only OS/CPU/RAM/device inventory; no camera capture, mounts, service or flight operations.','platform':platform.system(),'machine':platform.machine()}
    if sys.platform!='linux':
        report['status']='Debian is not running in this process.'
        report['next']='Boot the existing Debian installation and rerun this script; a stopped dual-boot OS cannot accept SSH.'
    else:
        report['os_release']=Path('/etc/os-release').read_text() if Path('/etc/os-release').exists() else 'unknown'
        report['memory']=probe(['free','-h'])
        report['cpu']=probe(['lscpu'])
        report['disks']=probe(['lsblk','-o','NAME,SIZE,FSTYPE,MOUNTPOINTS'])
        report['usb']=probe(['lsusb'])
        report['pci']=probe(['lspci','-nn'])
        report['camera_devices']=[str(p) for p in sorted(Path('/dev').glob('video*'))]
        report['v4l2']=probe(['v4l2-ctl','--list-devices'])
        report['status']='Inventory complete; hardware functionality and ROS compatibility still require validation.'
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
