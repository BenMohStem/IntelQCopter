"""Translate the local Gazebo garden geometry into a matching native USD scene."""
import math,sys
from pathlib import Path
from xml.etree import ElementTree as ET
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,UsdShade,Gf,Sdf
ROOT=Path(__file__).resolve().parents[2]

def build():
    source=ET.parse(ROOT/'sim/gazebo/garden.sdf').getroot().find('world')
    target=ROOT/'sim/isaac/assets/garden_native.usda'
    stage=Usd.Stage.CreateNew(str(target));UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1)
    stage.SetTimeCodesPerSecond(500);stage.SetStartTimeCode(0);stage.SetEndTimeCode(60000)
    world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
    scene=UsdPhysics.Scene.Define(stage,'/World/Physics');scene.CreateGravityDirectionAttr((0,0,-1));scene.CreateGravityMagnitudeAttr(9.80665)
    # Author vendor schema tokens without importing Isaac into the standalone
    # USD authoring environment. Isaac resolves them when it loads the stage.
    def schema(prim,name):
        op=prim.GetMetadata('apiSchemas');items=[] if op is None else [*op.prependedItems,*op.explicitItems,*op.addedItems]
        prim.SetMetadata('apiSchemas',Sdf.TokenListOp.Create(prependedItems=list(dict.fromkeys([*items,name]))))
    schema(scene.GetPrim(),'PhysxSceneAPI');scene.GetPrim().CreateAttribute('physxScene:enableGPUDynamics',Sdf.ValueTypeNames.Bool).Set(False)
    UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(700)
    sun=UsdLux.DistantLight.Define(stage,'/World/Sun');sun.CreateIntensityAttr(1500)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-30,20,45))
    materials={}
    def material(color):
        key=tuple(color)
        if key not in materials:
            path='/World/Materials/Color'+str(len(materials))
            mat=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/Surface')
            shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');materials[key]=mat
        return materials[key]
    def numbers(node,path,default):return list(map(float,node.findtext(path,default).split()))
    def pose(xform,values):
        xform.AddTranslateOp().Set(Gf.Vec3d(*values[:3]));xform.AddRotateXYZOp().Set(Gf.Vec3f(*[math.degrees(v) for v in values[3:]]))
    for model in source.findall('model'):
        path='/World/'+model.get('name');parent=UsdGeom.Xform.Define(stage,path);pose(parent,numbers(model,'pose','0 0 0 0 0 0'))
        dynamic=model.findtext('static','false')!='true'
        if dynamic:
            UsdPhysics.RigidBodyAPI.Apply(parent.GetPrim());mass=UsdPhysics.MassAPI.Apply(parent.GetPrim())
            inertial=model.find('link/inertial');mass.CreateMassAttr(float(inertial.findtext('mass')))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(inertial.findtext('inertia/'+key)) for key in ['ixx','iyy','izz']]))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0));mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
            schema(parent.GetPrim(),'PhysxContactReportAPI');parent.GetPrim().CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0)
            schema(parent.GetPrim(),'PhysxRigidBodyAPI')
            parent.GetPrim().CreateAttribute('physxRigidBody:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(16)
            parent.GetPrim().CreateAttribute('physxRigidBody:solverVelocityIterationCount',Sdf.ValueTypeNames.Int).Set(4)
            parent.GetPrim().CreateAttribute('physxRigidBody:maxDepenetrationVelocity',Sdf.ValueTypeNames.Float).Set(.5)
        for link in model.findall('link'):
            link_path=path+'/'+link.get('name');UsdGeom.Xform.Define(stage,link_path)
            for item in [*link.findall('visual'),*link.findall('collision')]:
                item_path=link_path+'/'+item.get('name').replace('-','_');geometry=item.find('geometry')
                if geometry.find('box') is not None:
                    obj=UsdGeom.Cube.Define(stage,item_path);obj.CreateSizeAttr(1);size=numbers(geometry,'box/size','1 1 1')
                else:
                    obj=UsdGeom.Cylinder.Define(stage,item_path);obj.CreateRadiusAttr(float(geometry.findtext('cylinder/radius')));obj.CreateHeightAttr(float(geometry.findtext('cylinder/length')));size=None
                pose(UsdGeom.Xformable(obj),numbers(item,'pose','0 0 0 0 0 0'))
                if size:UsdGeom.Xformable(obj).AddScaleOp().Set(Gf.Vec3f(*size))
                if item.tag=='collision':
                    UsdPhysics.CollisionAPI.Apply(obj.GetPrim());UsdGeom.Imageable(obj).MakeInvisible()
                else:
                    color=numbers(item,'material/diffuse','.5 .5 .5 1')[:3];obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(material(color))
            for sensor in link.findall("sensor[@type='camera']"):
                cam=UsdGeom.Camera.Define(stage,link_path+'/'+sensor.get('name'))
                values=numbers(sensor,'pose','0 0 0 0 0 0')
                # Gazebo camera +X forward/+Z up -> USD camera -Z forward/+Y up.
                camera_to_body=Gf.Matrix4d(0,-1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1)
                rotation=Gf.Rotation(Gf.Vec3d(1,0,0),math.degrees(values[3]))*Gf.Rotation(Gf.Vec3d(0,1,0),math.degrees(values[4]))*Gf.Rotation(Gf.Vec3d(0,0,1),math.degrees(values[5]))
                local=Gf.Matrix4d().SetRotate(rotation);local.SetTranslateOnly(Gf.Vec3d(*values[:3]))
                cam.AddTransformOp().Set(camera_to_body*local)
                hfov=float(sensor.findtext('camera/horizontal_fov'));w=int(sensor.findtext('camera/image/width'));h=int(sensor.findtext('camera/image/height'))
                cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(20.955*h/w);cam.CreateFocalLengthAttr(20.955/(2*math.tan(hfov/2)))
                cam.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
    stage.SetMetadata('customLayerData',{'status':'Assumed matching Gazebo garden; native runtime evidence recorded separately'})
    stage.GetRootLayer().Save();return target
if __name__=='__main__':print(build())
