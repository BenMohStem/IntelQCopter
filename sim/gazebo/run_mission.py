"""Reuse the recorded mission orchestration with native Gazebo physics/RGB.

The adapter exposes the narrow legacy engine interface; PyBullet is not stepped
or used for rendering. Analytic panel clearance is evaluation-only, never sensed.
"""
import argparse,hashlib,json,sys
from pathlib import Path
from multiprocessing.connection import Client
import numpy as np
from scipy.spatial.transform import Rotation
import cv2
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim/garden'))
import run_mission as mission

class GazeboAdapter:
    LINK_FRAME=1;WORLD_FRAME=2
    def __init__(self):
        self.client=Client(('127.0.0.1',9250),authkey=b'intelqcopter-local-engine')
        self.force=np.zeros(3);self.torque=np.zeros(3);self.drag=np.zeros(3)
        self.state=self.exchange({'op':'state'})
        self.ground=0;self.panels=[1,2]
    def exchange(self,message):self.client.send(message);return self.client.recv()
    def create_drone(self,start):
        assert np.linalg.norm(np.array(start[:2])-self.state['position'][:2])<.02
        return 3
    def getBasePositionAndOrientation(self,drone):return self.state['position'],self.state['quaternion']
    def getBaseVelocity(self,drone):return self.state['velocity'],self.state['omega']
    def getMatrixFromQuaternion(self,q):return Rotation.from_quat(q).as_matrix().ravel()
    def getEulerFromQuaternion(self,q):return Rotation.from_quat(q).as_euler('xyz')
    def applyExternalForce(self,drone,link,f,pos,frame):
        if frame==self.LINK_FRAME:self.force+=f
        else:self.drag+=f
    def applyExternalTorque(self,drone,link,t,frame):self.torque+=t
    def stepSimulation(self):
        self.state=self.exchange({'op':'step','force':self.force.tolist(),'torque':self.torque.tolist(),'drag':self.drag.tolist()})
        self.force[:]=0;self.torque[:]=0;self.drag[:]=0
    def camera(self,pos,r,overview=False):
        w,h,raw=self.exchange({'op':'camera','topic':'/overview/rgb' if overview else '/drone/rgb'})
        return np.frombuffer(raw,np.uint8).reshape(h,w,3).copy(),None
    def getContactPoints(self,bodyA=None,bodyB=None):
        result=[]
        for a,b in self.state['contacts']:
            for name,index in [('ground',0),('panel0',1),('panel1',2)]:
                if name in a or name in b:
                    if bodyB is None or bodyB==index:result.append((0,3,index))
        return result
    def getClosestPoints(self,drone,panel,distance):
        # Evaluation-only conservative cylinder/box horizontal distance, with
        # panel geometry hidden from the perception/planner path.
        x,y,yaw=[(3.4,.15,-.08),(6,.65,.12)][panel-1]
        p=np.array(self.state['position']);d=p[:2]-[x,y]
        local=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])@d
        horizontal=np.linalg.norm(np.maximum(np.abs(local)-[.04,.6],0))-.34
        clearance=max(horizontal, max(-p[2]-.08,p[2]-.08-1.8))
        return [(0,0,0,0,0,0,0,0,float(clearance))] if clearance<distance else []
    def downward_range(self,pos,r):
        # Analytic floor-only virtual ray: explicitly distinct from a native
        # lidar sensor. No obstacle range/depth is passed to navigation.
        origin=np.array(pos)+r@np.array([0,0,-.085])
        if origin[2]<.001 or r[2,2]<=0:return None
        distance=origin[2]/r[2,2]
        return (0,0,distance/8) if distance<8 else None
    def close(self):self.exchange({'op':'close'});self.client.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--name',default='gazebo-garden');ap.add_argument('--video',action='store_true');args=ap.parse_args()
    world=GazeboAdapter();mission.p=world;mission.Garden=lambda *a,**kw:world
    mission.OUT=ROOT/'artifacts/simulation/2026-10-20-engines';mission.OUT.mkdir(parents=True,exist_ok=True)
    original_frame=mission.video_frame
    def frame(*values):
        canvas=original_frame(*values);canvas[363:400,:]=[22,28,40]
        cv2.putText(canvas,'IntelQcopter | Native Gazebo 10.5 physics + OGRE2 RGB',(15,385),cv2.FONT_HERSHEY_SIMPLEX,.6,(225,235,245),1,cv2.LINE_AA)
        return canvas
    mission.video_frame=frame
    model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    native_provenance={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in
        ['sim/gazebo/run_mission.py','sim/gazebo/engine_server.py','sim/gazebo/build_world.py','sim/gazebo/garden.sdf']}
    print('NATIVE GAZEBO RUN: physics/OGRE2 RGB via local adapter; inherited baseline result labels are corrected below.',flush=True)
    result=mission.run_case(args.name,1,2452,model,ROOT/'build/panel-training/rgb-panel/weights/best.pt',video=args.video,gain_scale=.7)
    result.update(method='Native Gazebo 10.5 physics + OGRE2 RGB + production C + common C++ mission + actual YOLO/PnP',
        downward_lidar_model='Analytic floor-only ray, not a Gazebo lidar sensor',
        clearance_metric='Evaluation-only conservative cylinder/box distance, not Gazebo closest-points API',
        original_mission_orchestrator='sim/garden/run_mission.py; engine calls replaced by the documented Gazebo adapter')
    result.update(virtual_ground_contact_for_landing=False,contact_source='Native Gazebo contact sensor with collision entity IDs',native_RGB_cameras=True)
    result['source_sha256_at_start'].update(native_provenance)
    (mission.OUT/f'{args.name}-summary.json').write_text(json.dumps(result,indent=2))
    print('Gazebo final result:',json.dumps(result),flush=True)
    raise SystemExit(0 if result['success'] else 2)
if __name__=='__main__':main()
