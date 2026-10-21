"""Load the generated URDF in an independent parser and save its real render."""
from pathlib import Path
import json
import numpy as np
import pybullet as p
from PIL import Image
root=Path(__file__).resolve().parents[2]
output=root/'artifacts/setup/jazzy-repair'
path=root/'companion/ros2_ws/src/qcopter_description/urdf/f450.urdf'
p.connect(p.DIRECT)
drone=p.loadURDF(str(path),useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE)
mass=sum(p.getDynamicsInfo(drone,j)[0] for j in range(p.getNumJoints(drone)))
expected=json.loads(path.with_name('mass-properties.json').read_text())['mass_kg']
assert abs(mass-expected)<1e-10
view=p.computeViewMatrixFromYawPitchRoll([0,0,.01],.9,40,-35,0,2)
projection=p.computeProjectionMatrixFOV(45,1.5,.01,10)
rgba=np.array(p.getCameraImage(900,600,view,projection,renderer=p.ER_TINY_RENDERER)[2])
Image.fromarray(rgba[:,:,:3]).save(output/'f450-urdf-preview.png')
report={'URDF_loaded':True,'fixed_joints':p.getNumJoints(drone),'component_mass_kg':mass,
        'mass_sum_matches_generator':True,'render':'PyBullet TinyRenderer; model preview only, not a flight',
        'all_masses_provisional':True}
(output/'f450-urdf-validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2));p.disconnect()
