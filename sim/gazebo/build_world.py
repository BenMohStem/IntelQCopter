"""Author a local SDF garden using the final experiment's assumed parameters."""
import json,math
from pathlib import Path
from xml.etree.ElementTree import Element,SubElement,tostring
ROOT=Path(__file__).resolve().parents[2]

def build(target=None,drop_test=False):
    model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    sdf=Element('sdf',version='1.10');world=SubElement(sdf,'world',name='intelqcopter')
    SubElement(world,'gravity').text='0 0 -9.80665'
    physics=SubElement(world,'physics',name='fixed500',type='ignored')
    SubElement(physics,'max_step_size').text='.002';SubElement(physics,'real_time_factor').text='0'
    for short,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('contact','Contact')]:
        p=SubElement(world,'plugin',filename=f'gz-sim-{short}-system',name=f'gz::sim::systems::{name}')
        if short=='sensors':SubElement(p,'render_engine').text='ogre2'
    scene=SubElement(world,'scene');SubElement(scene,'ambient').text='.65 .65 .65 1';SubElement(scene,'background').text='.6 .75 .85 1'
    light=SubElement(world,'light',name='sun',type='directional');SubElement(light,'pose').text='0 0 8 0 0 0';SubElement(light,'direction').text='-.3 .2 -1';SubElement(light,'diffuse').text='.9 .9 .9 1'
    def box(name,pos,size,color,collision=True,static=True,parent=world):
        body=SubElement(parent,'model',name=name);SubElement(body,'static').text=str(static).lower()
        SubElement(body,'pose').text=' '.join(map(str,[*pos,0,0,0]));link=SubElement(body,'link',name='body')
        def geom(element):SubElement(SubElement(element,'geometry'),'box').append(Element('size'));element.find('geometry/box/size').text=' '.join(map(str,size))
        visual=SubElement(link,'visual',name='visual');geom(visual)
        mat=SubElement(visual,'material');SubElement(mat,'ambient').text=' '.join(map(str,[*color,1]));SubElement(mat,'diffuse').text=' '.join(map(str,[*color,1]))
        if collision:geom(SubElement(link,'collision',name='collision'))
        return body,link
    box('ground',(4,0,-.1),(25,18,.2),(.22,.4,.15))
    box('path',(4,0,.002),(14,2.6,.004),(.63,.61,.53),False)
    for i,(x,y,yaw) in enumerate([(3.4,.15,-.08),(6,.65,.12)]):
        panel,link=box(f'panel{i}',(x,y,.9),(.08,1.2,1.8),(.93,.93,.9))
        panel.find('pose').text=f'{x} {y} .9 0 0 {yaw}'
        for side in [-1,1]:
            v=SubElement(link,'visual',name=f'blue{side}');SubElement(v,'pose').text=f'{side*.041} 0 0 0 0 0'
            g=SubElement(SubElement(v,'geometry'),'box');SubElement(g,'size').text='.004 1.09 1.69'
            m=SubElement(v,'material');SubElement(m,'diffuse').text='.025 .08 .68 1';SubElement(m,'ambient').text='.025 .08 .68 1'
    for i,(x,y) in enumerate([(-1,4),(2,4.5),(5,4.8),(9,4),(-1,-4),(4,-4),(9,-4)]):
        box(f'tree_trunk{i}',(x,y,.6),(.2,.2,1.2),(.3,.17,.06))
        box(f'tree_crown{i}',(x,y,1.7),(1.1,1.1,1.5),(.1,.32,.08))
    for i,(x,y) in enumerate([(0,3),(7,3)]):box(f'bench{i}',(x,y,.45),(1.5,.5,.3),(.4,.22,.08))
    box('building',(4,7,2),(12,2,4),(.7,.67,.54))
    drone=SubElement(world,'model',name='drone');SubElement(drone,'pose').text=f'0 0 {1.5 if drop_test else .082} 0 0 0'
    link=SubElement(drone,'link',name='body');inertial=SubElement(link,'inertial');SubElement(inertial,'mass').text=str(model['mass_kg'])
    inertia=SubElement(inertial,'inertia')
    for key,value in zip(['ixx','iyy','izz','ixy','ixz','iyz'],[*model['inertia_kg_m2'],0,0,0]):SubElement(inertia,key).text=str(value)
    for tag in ['collision','visual']:
        item=SubElement(link,tag,name=tag);shape=SubElement(SubElement(item,'geometry'),'cylinder');SubElement(shape,'radius').text='.34' if tag=='collision' else '.06';SubElement(shape,'length').text='.16' if tag=='collision' else '.035'
        if tag=='visual':
            mat=SubElement(item,'material');SubElement(mat,'diffuse').text='.1 .1 .12 1'
    sensor=SubElement(link,'sensor',name='contact',type='contact');SubElement(sensor,'always_on').text='true';SubElement(sensor,'update_rate').text='500';SubElement(sensor,'topic').text='/drone/contact'
    contact=SubElement(sensor,'contact');SubElement(contact,'collision').text='collision';SubElement(contact,'topic').text='/drone/contact'
    a=.45/(2*math.sqrt(2))
    for i,(x,y) in enumerate([(a,a),(a,-a),(-a,-a),(-a,a)]):
        arm=SubElement(link,'visual',name=f'arm{i}');SubElement(arm,'pose').text=f'{x/2} {y/2} 0 0 0 {math.atan2(y,x)}'
        geometry=SubElement(SubElement(arm,'geometry'),'box');SubElement(geometry,'size').text='.225 .025 .02'
        material=SubElement(arm,'material');SubElement(material,'diffuse').text='1 .05 .05 1' if i<2 else '.95 .95 .9 1'
        v=SubElement(link,'visual',name=f'rotor{i}');SubElement(v,'pose').text=f'{x} {y} .09 0 0 0'
        shape=SubElement(SubElement(v,'geometry'),'cylinder');SubElement(shape,'radius').text='.1143';SubElement(shape,'length').text='.006'
        mat=SubElement(v,'material');SubElement(mat,'diffuse').text='1 .05 .05 1' if i<2 else '.95 .95 .9 1'
    def camera(parent,name,pose,width,height,hfov,topic):
        sensor=SubElement(parent,'sensor',name=name,type='camera');SubElement(sensor,'pose').text=pose
        SubElement(sensor,'always_on').text='true';SubElement(sensor,'update_rate').text='10';SubElement(sensor,'topic').text=topic
        cam=SubElement(sensor,'camera');SubElement(cam,'horizontal_fov').text=str(hfov)
        im=SubElement(cam,'image');SubElement(im,'width').text=str(width);SubElement(im,'height').text=str(height);SubElement(im,'format').text='R8G8B8'
        clip=SubElement(cam,'clip');SubElement(clip,'near').text='.05';SubElement(clip,'far').text='100'
    camera(link,'rgb','.18 0 .02 0 0 0',320,240,2*math.atan(math.tan(math.radians(35))*4/3),'/drone/rgb')
    overview=SubElement(world,'model',name='overview');SubElement(overview,'static').text='true';ol=SubElement(overview,'link',name='camera')
    camera(ol,'camera',f'6 -4.5 7 0 {math.atan2(6.5,math.hypot(-2,4.5))} {math.atan2(4.5,-2)}',640,360,1.3,'/overview/rgb')
    target=Path(target or ROOT/'sim/gazebo/garden.sdf');target.parent.mkdir(parents=True,exist_ok=True)
    target.write_bytes(tostring(sdf,encoding='utf-8',xml_declaration=True));return target

if __name__=='__main__':print(build())
