"""Machine dependencies stay in ignored .local/config.json, not project source."""
import json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
config_path=ROOT/'.local/config.json'
CONFIG=json.loads(config_path.read_text(encoding='utf8')) if config_path.exists() else {}
def setting(name,default):
    value=CONFIG.get(name,default)
    if value is None:raise RuntimeError(f'Configure {name} in .local/config.json')
    return str(value)
def path(name,default):
    value=Path(setting(name,default)).expanduser()
    return str(value if value.is_absolute() else ROOT/value)
PIXI=setting('pixi',shutil.which('pixi') or 'C:/WinLibs/pixi/pixi.exe')
ROS_MANIFEST=path('ros_manifest','C:/dev/pixi.toml')
ROS_PYTHON=path('ros_python','C:/dev/.pixi/envs/default/python.exe')
GZ_MANIFEST=path('gazebo_manifest','simulation/gazebo/pixi.toml')
GZ_PYTHON=path('gazebo_python','simulation/gazebo/.pixi/envs/default/python.exe')
PANEL_MODEL=path('panel_model','build/models/panels.onnx')
def ffmpeg():return setting('ffmpeg',shutil.which('ffmpeg'))
