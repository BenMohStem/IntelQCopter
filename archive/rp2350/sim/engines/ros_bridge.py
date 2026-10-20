"""Reuse the proven DDS adapter while preserving the earlier proof files."""
import argparse,importlib.util,shutil,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim'))
ap=argparse.ArgumentParser();ap.add_argument('--engine',choices=['isaac','gazebo'],required=True);args=ap.parse_args()
spec=importlib.util.spec_from_file_location('native_dds_bridge',ROOT/'sim/ros2_garden_bridge.py')
bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)
# libraries() retains its real workspace root; only the adapter's two output
# paths are redirected. The frozen, earlier transport record is not overwritten.
bridge.ROOT=ROOT/'build/native-ros-evidence'/args.engine
directory=bridge.ROOT/'artifacts/simulation/2026-10-20-missions';directory.mkdir(parents=True,exist_ok=True)
bridge.main()
target=ROOT/'artifacts/simulation/2026-10-20-engines';target.mkdir(parents=True,exist_ok=True)
shutil.copy2(directory/'garden-ros2-transport.json',target/f'{args.engine}-ros2-transport.json')
