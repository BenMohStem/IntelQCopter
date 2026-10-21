"""Author/validate a portable USD model without needing Isaac's renderer.

This is a scene asset, not evidence of a running Isaac simulation. Dynamics
attributes describe the same assumed model as CPU SITL. No flight plugin yet.
"""
import json,math
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,Gf,Sdf

ROOT=Path(__file__).resolve().parents[2]

def main():
    model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    target=ROOT/'sim/isaac/assets/qcopter_test.usda'
    stage=Usd.Stage.CreateNew(str(target));UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage,1);stage.SetTimeCodesPerSecond(500)
    world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
    scene=UsdPhysics.Scene.Define(stage,'/World/Physics');scene.CreateGravityDirectionAttr((0,0,-1));scene.CreateGravityMagnitudeAttr(model['gravity_mps2'])
    UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(700)
    def cube(path,pos,scale,color,collision=False):
        obj=UsdGeom.Cube.Define(stage,path);obj.CreateSizeAttr(1)
        obj.AddTranslateOp().Set(Gf.Vec3d(*pos));obj.AddScaleOp().Set(Gf.Vec3f(*scale))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        return obj
    cube('/World/Ground',(4,0,-.1),(20,12,.2),(.2,.25,.3),True)
    drone=UsdGeom.Xform.Define(stage,'/World/Drone');drone.AddTranslateOp().Set(Gf.Vec3d(*model['start_m']))
    drone.AddOrientOp().Set(Gf.Quatf(1,Gf.Vec3f(0)))
    UsdPhysics.RigidBodyAPI.Apply(drone.GetPrim()).CreateRigidBodyEnabledAttr(True)
    mass=UsdPhysics.MassAPI.Apply(drone.GetPrim());mass.CreateMassAttr(model['mass_kg'])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*model['inertia_kg_m2']))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1,Gf.Vec3f(0)))
    cube('/World/Drone/Body',(0,0,0),(.12,.10,.035),(.1,.1,.12))
    a=model['motor_diagonal_m']/(2*math.sqrt(2))
    for name,x,y,color in [('FL',a,a,(.8,.03,.03)),('FR',a,-a,(.8,.03,.03)),
                            ('RR',-a,-a,(.9,.9,.85)),('RL',-a,a,(.9,.9,.85))]:
        arm=cube('/World/Drone/Arm'+name,(x/2,y/2,0),(math.hypot(x,y),.025,.02),color)
        # Reorder operations so scale occurs in arm-local axes before rotation.
        xf=UsdGeom.Xformable(arm);xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(x/2,y/2,0))
        xf.AddRotateZOp().Set(math.degrees(math.atan2(y,x)));xf.AddScaleOp().Set(Gf.Vec3f(math.hypot(x,y),.025,.02))
        rotor=UsdGeom.Cylinder.Define(stage,'/World/Drone/Rotor'+name);rotor.CreateRadiusAttr(.1143);rotor.CreateHeightAttr(.003)
        rotor.AddTranslateOp().Set(Gf.Vec3d(x,y,.035));rotor.CreateDisplayColorAttr([Gf.Vec3f(.05,.05,.05)])
    envelope=UsdGeom.Cylinder.Define(stage,'/World/Drone/CollisionEnvelope')
    envelope.CreateRadiusAttr(model['robot_collision_radius_m']);envelope.CreateHeightAttr(.08)
    envelope.CreatePurposeAttr(UsdGeom.Tokens.guide);UsdPhysics.CollisionAPI.Apply(envelope.GetPrim())
    for i,(x,y,r) in enumerate(model['obstacles_x_y_radius_m']):
        obj=UsdGeom.Cylinder.Define(stage,f'/World/Obstacle{i}');obj.CreateRadiusAttr(r);obj.CreateHeightAttr(3)
        obj.AddTranslateOp().Set(Gf.Vec3d(x,y,1.5));obj.CreateDisplayColorAttr([Gf.Vec3f(.3,.4,.5)])
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    camera=UsdGeom.Camera.Define(stage,'/World/Overview')
    matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(9,-10,9),Gf.Vec3d(4,0,1),Gf.Vec3d(0,0,1)).GetInverse()
    UsdGeom.Xformable(camera).AddTransformOp().Set(matrix)
    stage.GetRootLayer().customLayerData={'status':'ASSUMED model, USD parser validated; not run in Isaac',
        'controller':'External flight/ROS/force adapters not yet connected to this stage'}
    stage.GetRootLayer().Save()
    check=Usd.Stage.Open(str(target));assert check
    assert float(UsdPhysics.MassAPI(check.GetPrimAtPath('/World/Drone')).GetMassAttr().Get())>0
    prims=list(check.Traverse());assert len(prims)>=18
    report={'stage':str(target),'prims':len(prims),'mass_kg':model['mass_kg'],
       'inertia_kg_m2':model['inertia_kg_m2'],'collision_radius_m':model['robot_collision_radius_m'],
       'validation':'USD parser + required mass/scene attributes; NOT Isaac physics or render validation'}
    out=ROOT/'artifacts/test-runs/2026-10-20/usd-validation.json'
    out.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))

if __name__=='__main__':main()
