"""Curated reading copies. Active builds keep their canonical source paths.
Never overwrites a hand-edited reading copy. --check validates hashes and freshness.
"""
from pathlib import Path
import argparse,hashlib,json,subprocess
R=Path(__file__).resolve().parents[2];O=R/'important_sources'
MAIN='bce665ad97adbfcb282f02970a1db599e67e6479'
def git(*args):return subprocess.check_output(['git',*args],cwd=R)
def sha(b):return hashlib.sha256(b).hexdigest()
def selection():
 rows=[]
 def add(src,dst,ref=None):rows.append((src,dst,ref))
 add('firmware/src/flight_controller.cpp','firmware/esp32/flight_controller.cpp')
 add('firmware/include/fc_api.h','firmware/esp32/fc_api.h')
 add('platformio.ini','firmware/esp32/platformio.ini')
 add('tools/platformio/patch_xbox.py','firmware/esp32/patch_xbox.py')
 add('firmware/flight_controller/flight_controller.c','firmware/rp2350/flight_controller.c',MAIN)
 add('firmware/handler_controller/handler_controller.c','firmware/rp2040/handler_controller.c',MAIN)
 prefix='companion/ros2_ws/src/qcopter_autonomy/'
 for p in sorted((R/prefix).rglob('*')):
  if p.is_file() and p.suffix in {'.cpp','.hpp','.msg','.srv','.py','.xml','.txt'}:
   src=p.relative_to(R).as_posix();add(src,'ros2/qcopter_autonomy/'+src[len(prefix):])
 for name in ['engine.py','fc_binding.py','build_world.py','environments.json','ros_bridge.py','live_session.py','live_standalone.py','session_controls.py','experiment_settings.py','station_manager.py']:
  add('simulation/gazebo/'+name,'simulation/gazebo/'+name)
 add('simulation/model/parameters.json','simulation/model/parameters.json')
 add('companion/pc_mission_map.py','ros2/pc_mission_map.py')
 prefix='companion/ros2_ws/src/qcopter_description/'
 paths=git('ls-tree','-r','--name-only',MAIN,'--',prefix).decode().splitlines()
 for src in paths:add(src,'drone_model/qcopter_description/'+src[len(prefix):],MAIN)
 return rows
def run():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');a=p.parse_args()
 manifest=O/'manifest.json';old=json.loads(manifest.read_text(encoding='utf8')) if manifest.exists() else {'files':[]}
 index={r['copy']:r for r in old['files']};rows=[];head=git('rev-parse','HEAD').decode().strip()
 for src,dst,ref in selection():
  data=git('show',ref+':'+src) if ref else (R/src).read_bytes()
  target=O/dst;previous=index.get(dst)
  if a.check:
   assert previous and target.is_file(),('missing reading copy',dst)
   assert sha(target.read_bytes())==previous['sha256']==sha(data),('changed/stale reading copy',dst)
   continue
  if target.exists() and previous and sha(target.read_bytes())!=previous['sha256']:
   raise RuntimeError('Hand-edited reference copy: preserve your edits before refresh: '+dst)
  if target.exists() and not previous:raise RuntimeError('Unmanaged file: '+dst)
  target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
  rows.append(dict(copy=dst,canonical_source=src,source_branch='main' if ref else 'esp32-xbox',
   source_kind='git-snapshot' if ref else 'working-tree',source_commit=ref or head,sha256=sha(data)))
 if a.check:
  assert len(index)==len(selection()),'Selection changed; refresh required'
  print('PASS:',len(index),'reference copies match canonical bytes / pinned main sources');return
 manifest.write_bytes((json.dumps(dict(schema=1,purpose='Reading copies; edit canonical sources, then refresh',main_snapshot=MAIN,files=rows),indent=2)+'\n').encode())
 print('Refreshed',len(rows),'important source files; no source code moved or deleted.')
if __name__=='__main__':run()
