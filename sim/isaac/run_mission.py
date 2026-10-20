"""Drive the same C/C++ mission using the native Isaac engine bridge."""
import argparse,hashlib,importlib.util,json
from multiprocessing.connection import Client
from pathlib import Path
import numpy as np,cv2
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('gazebo_adapter',ROOT/'sim/gazebo/run_mission.py')
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
mission=adapter.mission
class IsaacAdapter(adapter.GazeboAdapter):
    def __init__(self):
        self.client=Client(('127.0.0.1',9251),authkey=b'intelqcopter-local-engine')
        self.force=np.zeros(3);self.torque=np.zeros(3);self.drag=np.zeros(3)
        self.state=self.exchange({'op':'state'});self.ground=0;self.panels=[1,2]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--name',default='isaac-garden');ap.add_argument('--video',action='store_true');ap.add_argument('--ros',action='store_true');args=ap.parse_args()
    world=IsaacAdapter();mission.p=world;mission.Garden=lambda *a,**kw:world
    mission.OUT=ROOT/'artifacts/simulation/2026-10-20-engines';mission.OUT.mkdir(parents=True,exist_ok=True)
    original_frame=mission.video_frame
    def frame(*values):
        canvas=original_frame(*values);canvas[363:400,:]=[22,28,40]
        cv2.putText(canvas,'IntelQcopter | Native Isaac Sim 6.2 PhysX + RGB',(15,385),cv2.FONT_HERSHEY_SIMPLEX,.6,(225,235,245),1,cv2.LINE_AA)
        canvas[283:313,640:]=[22,28,40]
        cv2.putText(canvas,f'Mapped tracks: {len(values[5])}',(650,300),cv2.FONT_HERSHEY_SIMPLEX,.5,(225,235,245),1,cv2.LINE_AA)
        return canvas
    mission.video_frame=frame
    paths=['sim/isaac/run_mission.py','sim/isaac/engine_server.py','sim/isaac/build_garden.py','sim/isaac/assets/garden_native.usda','sim/gazebo/run_mission.py',
           'sim/engines/ros_bridge.py','sim/garden/ros_transport.py',
           'companion/ros2_ws/src/qcopter_autonomy/include/qcopter_autonomy/mission.hpp',
           'companion/ros2_ws/src/qcopter_autonomy/include/qcopter_autonomy/navigation.hpp']
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
    model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    print('NATIVE ISAAC RUN: actual PhysX and RGB; inherited baseline result labels are corrected below.',flush=True)
    result=mission.run_case(args.name,1,2452,model,ROOT/'build/panel-training/rgb-panel/weights/best.pt',video=args.video,ros=args.ros,gain_scale=.7)
    result.update(method=f"Native Isaac Sim 6.2 / PhysX CPU + {world.state.get('render_mode','MinimalRendering')} RGB + production C + common C++ mission + YOLO/PnP",
        native_mass_kg=world.state.get('native_mass_kg'),native_dt_s=world.state.get('native_dt_s'),render_mode=world.state.get('render_mode'),
        virtual_ground_contact_for_landing=False,contact_source='Native PhysX filtered contact forces',native_RGB_cameras=True,
        downward_lidar_model='Analytic floor-only virtual ray, not an Isaac lidar sensor',
        clearance_metric='Evaluation-only conservative cylinder/box distance',
        scope='Assumed airframe, known-size blue panels, simulated local odometry; not real flight')
    result['source_sha256_at_start'].update(hashes)
    (mission.OUT/f'{args.name}-summary.json').write_text(json.dumps(result,indent=2));print('Isaac final result:',json.dumps(result),flush=True)
    raise SystemExit(0 if result['success'] else 2)
if __name__=='__main__':main()
