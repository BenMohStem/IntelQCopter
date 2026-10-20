"""Original primitive F450 model: component inertia and parallel-axis accounting."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT/'companion/ros2_ws/src/qcopter_description'
p = json.loads((ROOT/'sim/model/f450_components.json').read_text())
r = ET.Element('robot', name='intelqcopter_f450')
ET.SubElement(r, 'link', name='base_link')  # massless root for robot_state_publisher
components = []

def body(name, mass, xyz, size, color, yaw=0., cylinder=False):
    link = ET.SubElement(r, 'link', name=name)
    if cylinder:
        radius, height = size
        diag = [mass*(3*radius**2+height**2)/12]*2+[mass*radius**2/2]
    else:
        x,y,z = size
        diag = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'inertia', ixx=str(diag[0]), iyy=str(diag[1]),
                  izz=str(diag[2]), ixy='0', ixz='0', iyz='0')
    for kind in ['visual','collision']:
        element = ET.SubElement(link, kind)
        geometry = ET.SubElement(element, 'geometry')
        if cylinder and kind == 'visual' and name.startswith('propeller_'):
            # Two-blade visual; swept collision disk and disk inertia approximation.
            ET.SubElement(geometry,'box',size=f'{2*size[0]} 0.020 {size[1]}')
        elif cylinder:
            ET.SubElement(geometry,'cylinder',radius=str(size[0]),length=str(size[1]))
        else:
            ET.SubElement(geometry,'box',size=' '.join(map(str,size)))
        if kind == 'visual':
            ET.SubElement(ET.SubElement(element,'material',name=name+'_color'),
                          'color',rgba=' '.join(map(str,color)))
    joint=ET.SubElement(r,'joint',name=name+'_mount',type='fixed')
    ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link=name)
    ET.SubElement(joint,'origin',xyz=' '.join(map(str,xyz)),rpy=f'0 0 {yaw}')
    c,s=math.cos(yaw),math.sin(yaw)
    rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    components.append((name,mass,np.array(xyz),rotation@np.diag(diag)@rotation.T))

body('chassis',p['frame_kg']*.45,[0,0,0],[.13,.13,.025],[.12,.12,.12,1])
a=p['diagonal_m']/(2*math.sqrt(2))
for name,sx,sy in [('fl',1,1),('fr',1,-1),('rr',-1,-1),('rl',-1,1)]:
    color=[.85,.04,.03,1] if sx>0 else [.90,.90,.86,1]
    body('arm_'+name,p['frame_kg']*.55/4,[sx*a/2,sy*a/2,0],
         [p['diagonal_m']/2,.025,.018],color,math.atan2(sy,sx))
    body('motor_'+name,p['motor_each_kg'],[sx*a,sy*a,.018],[.014,.025],[1,.35,.03,1],cylinder=True)
    body('propeller_'+name,p['propeller_each_kg'],[sx*a,sy*a,.038],
         [p['propeller_diameter_m']/2,.002],[.08,.08,.08,.45],cylinder=True)
body('battery',p['battery_kg'],p['battery_position_m'],p['battery_size_m'],[.12,.25,.65,1])
mass=sum(v[1] for v in components)
com=sum(v[1]*v[2] for v in components)/mass
inertia=np.zeros((3,3))
for _,m,position,local in components:
    d=position-com
    inertia += local + m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
assert np.linalg.eigvalsh(inertia).min()>0
DEST.joinpath('urdf').mkdir(parents=True,exist_ok=True)
ET.indent(r)
ET.ElementTree(r).write(DEST/'urdf/f450.urdf',encoding='utf-8',xml_declaration=True)
report={'status':p['status'],'mass_kg':mass,'center_of_mass_m':com.tolist(),
        'inertia_about_com_kg_m2':inertia.tolist(),
        'components':[{'name':n,'mass_kg':m,'position_m':x.tolist()} for n,m,x,_ in components],
        'flight_baseline_unchanged':True}
DEST.joinpath('urdf/mass-properties.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
