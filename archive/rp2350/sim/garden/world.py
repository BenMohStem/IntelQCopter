"""Procedural campus-garden scene. No surveyed USTHB geometry or downloaded asset.

Scene truth is private to rendering/physics and evaluation. Navigation only gets
RGB-derived observations; camera depth/segmentation are NOT used at runtime.
"""
import math
import numpy as np
import pybullet as p

PANEL_WIDTH=1.2
PANEL_HEIGHT=1.8
WIDTH,HEIGHT=320,240
FOV=70.
FOCAL=HEIGHT/(2*math.tan(math.radians(FOV/2)))
K=np.array([[FOCAL,0,WIDTH/2],[0,FOCAL,HEIGHT/2],[0,0,1.]])

class Garden:
    def __init__(self,model,seed=2450,layout=0):
        self.client=p.connect(p.DIRECT);self.model=model;self.rng=np.random.default_rng(seed)
        p.setGravity(0,0,-model['gravity_mps2']);p.setTimeStep(.002)
        p.setPhysicsEngineParameter(numSolverIterations=50,deterministicOverlappingPairs=1)
        self.ground=self.box([5,0,-.1],[12,7,.1],[.28,.45,.2,1],collision=True)
        self.box([4,0,.002],[6,1.35,.003],[.72,.70,.64,1])
        for y in [-5.3,5.3]:
            self.box([4,y,1.8],[7,.6,1.8],[.69,.66,.56,1],collision=True)
            for x in np.arange(-1,11,1.2):self.box([x,y- np.sign(y)*.61,2.4],[.35,.025,.45],[.2,.28,.32,1])
        for x,y in [(-1,-3),(1,3),(4,3.3),(7,-3),(9,3.5)]:
            self.cylinder([x,y,1.1],.12,2.2,[.4,.25,.12,1])
            visual=p.createVisualShape(p.GEOM_SPHERE,radius=.8,rgbaColor=[.15,.4,.12,1])
            p.createMultiBody(0,-1,visual,[x,y,2.5])
        for x,y in [(1,-2.8),(6,2.9)]:
            self.box([x,y,.45],[.7,.25,.08],[.45,.25,.1,1]);self.box([x,y+.2,.8],[.7,.06,.35],[.45,.25,.1,1])
        self.panels=[]
        # Positions known only by this simulator; never sent to the planner.
        configurations=[[(3.2,0,0),(5.8,-.65,.10)],[(3.4,.15,-.08),(6.,.65,.12)],[(3.1,-.25,.1),(5.6,-.8,-.12)]]
        for x,y,yaw in configurations[layout%len(configurations)]:self.add_panel(x,y,yaw)
        self.drone=None
    def box(self,pos,half,color,collision=False,orientation=None):
        visual=p.createVisualShape(p.GEOM_BOX,halfExtents=half,rgbaColor=color)
        shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=half) if collision else -1
        return p.createMultiBody(0,shape,visual,pos,orientation or [0,0,0,1])
    def cylinder(self,pos,radius,height,color):
        visual=p.createVisualShape(p.GEOM_CYLINDER,radius=radius,length=height,rgbaColor=color)
        return p.createMultiBody(0,-1,visual,pos)
    def add_panel(self,x,y,yaw):
        q=p.getQuaternionFromEuler([0,0,yaw])
        body=self.box([x,y,PANEL_HEIGHT/2],[.035,PANEL_WIDTH/2,PANEL_HEIGHT/2],[.05,.30,.90,1],True,q)
        # Narrow white border makes an ordinary rectangular fixture, not a depth oracle.
        for dy,dz,hy,hz in [(0,-.87,.6,.025),(0,.87,.6,.025),(-.57,0,.025,.9),(.57,0,.025,.9)]:
            local=np.array([-.038,dy,dz]);r=np.array(p.getMatrixFromQuaternion(q)).reshape(3,3)
            self.box(np.array([x,y,.9])+r@local,[.004,hy,hz],[.95,.95,.90,1],False,q)
        self.panels.append(body)
        return body
    def create_drone(self,start=(0,0,.082)):
        # Conservative rotor-envelope collision cylinder; local inertia supplied explicitly.
        col=p.createCollisionShape(p.GEOM_CYLINDER,radius=.34,height=.16)
        visual=p.createVisualShape(p.GEOM_BOX,halfExtents=[.07,.07,.025],rgbaColor=[.12,.12,.13,1])
        a=.45/(2*math.sqrt(2));visuals=[];positions=[];rotations=[]
        for x,y,c in [(a,a,[.85,.02,.02,1]),(a,-a,[.85,.02,.02,1]),(-a,-a,[.95,.95,.90,1]),(-a,a,[.95,.95,.90,1])]:
            visuals.append(p.createVisualShape(p.GEOM_BOX,halfExtents=[.1125,.014,.012],rgbaColor=c))
            positions.append([x/2,y/2,0]);rotations.append(p.getQuaternionFromEuler([0,0,math.atan2(y,x)]))
            visuals.append(p.createVisualShape(p.GEOM_CYLINDER,radius=.1143,length=.003,rgbaColor=[.1,.1,.12,.8]))
            positions.append([x,y,.035]);rotations.append([0,0,0,1])
        n=len(visuals)
        self.drone=p.createMultiBody(self.model['mass_kg'],col,visual,list(start),
          linkMasses=[0]*n,linkCollisionShapeIndices=[-1]*n,linkVisualShapeIndices=visuals,
          linkPositions=positions,linkOrientations=rotations,linkInertialFramePositions=[[0,0,0]]*n,
          linkInertialFrameOrientations=[[0,0,0,1]]*n,linkParentIndices=[0]*n,
          linkJointTypes=[p.JOINT_FIXED]*n,linkJointAxis=[[0,0,1]]*n)
        p.changeDynamics(self.drone,-1,localInertiaDiagonal=self.model['inertia_kg_m2'],linearDamping=0,angularDamping=0,
                         lateralFriction=.8,restitution=0)
        return self.drone
    def camera(self,pos,rotation,overview=False):
        if overview:
            eye=[6,-4.5,7];target=[4,0,.5];up=[0,0,1];w,h=640,360
        else:
            eye=np.array(pos)+rotation@np.array([.18,0,.02]);target=eye+rotation[:,0];up=rotation[:,2];w,h=WIDTH,HEIGHT
        view=p.computeViewMatrix(eye,target,up);proj=p.computeProjectionMatrixFOV(FOV,w/h,.05,30)
        im=p.getCameraImage(w,h,view,proj,renderer=p.ER_TINY_RENDERER,flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
        # Depth/IDs returned solely for dataset labels or evaluator; runtime ignores both.
        return np.array(im[2],np.uint8).reshape(h,w,4)[:,:,:3],np.array(im[4]).reshape(h,w)
    def downward_range(self,pos,r):
        origin=np.array(pos)+r@np.array([0,0,-.085]);end=origin-8*r[:,2]
        if origin[2]<.001:return None
        result=p.rayTest(origin,end)[0]
        # Ray origin must be outside the conservative body collider.
        return result
    def close(self):p.disconnect(self.client)
