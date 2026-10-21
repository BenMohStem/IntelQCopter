"""Validate command targets, current source layout, and script syntax without running missions."""
from pathlib import Path
import ast
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
data=json.loads((ROOT/'project.json').read_text(encoding='utf8'))
checks=[]
for name,command in data['commands'].items():
    target=(ROOT/command['path']).resolve()
    assert target.is_relative_to(ROOT) and target.is_file(),f'Missing/unsafe command target: {name}'
    assert command['runtime'] in {'python','powershell','node','cmd','pio'},name
    checks.append('command:'+name)
for domain,path in data['domains'].items():
    target=(ROOT/path).resolve()
    assert target.is_relative_to(ROOT) and target.is_dir(),f'Missing domain: {domain}'
    checks.append('domain:'+domain)
for base in ('tools','sim' if data['branch']=='main' else 'simulation'):
    for folder,dirs,files in os.walk(ROOT/base):
        dirs[:]=[d for d in dirs if d not in {'__pycache__','.pixi','.pio','.cache','build','node_modules'} and not d.startswith('.venv')]
        for name in files:
            if not name.endswith('.py'):continue
            path=Path(folder)/name
            ast.parse(path.read_text(encoding='utf8'),filename=str(path))
            checks.append('python-syntax:'+path.relative_to(ROOT).as_posix())
for script in (ROOT/'tools').rglob('*.cjs'):
    subprocess.run(['node','--check',str(script)],cwd=ROOT,check=True)
    checks.append('javascript-syntax:'+script.relative_to(ROOT).as_posix())
assert not (ROOT/'build/esp32-xbox/.git').exists(),'A worktree still lives inside build/'
report={'branch':data['branch'],'passed':True,'checks':checks,
        'scope':'Layout and source syntax only. No ROS/Gazebo mission or hardware execution.'}
if len(sys.argv)>1:
    output=(ROOT/sys.argv[1]).resolve()
    assert output.is_relative_to(ROOT)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8',newline='\n')
print(f"Layout valid: {data['branch']}, {len(checks)} checks")
