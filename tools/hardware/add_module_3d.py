"""Original illustrative VRML models for current module carrier; no copper edits.
VRML dimensions use KiCad's legacy 0.1 inch model unit (mm / 2.54).
Footprint-local Y is negated for the 3D right-handed coordinate system.
"""
from pathlib import Path
import argparse,json,math,hashlib,sys
ROOT=Path(__file__).resolve().parents[2]
arg=argparse.ArgumentParser();arg.add_argument('project',type=Path);args=arg.parse_args();O=args.project
# Use the maintained S-expression reader, not a whole-board rewrite.
sys.path.insert(0,str(ROOT/'tools/hardware'))
from enrich_module_bodies import children
M=O/'models3d';M.mkdir(exist_ok=True)
materials={'pcb_black':(.05,.08,.10),'pcb_blue':(.06,.20,.55),'pcb_purple':(.40,.10,.53),'pcb_green':(.05,.35,.18),
           'black':(.025,.03,.035),'silver':(.66,.69,.72),'gold':(.80,.57,.18),'ceramic':(.65,.43,.23),'white':(.86,.86,.82),
           'red':(.72,.04,.035),'yellow':(.88,.66,.04),'pink':(.65,.07,.36),'blue':(.04,.19,.64)}
class Model:
    def __init__(self,name):
        self.name=name;self.parts=['#VRML V2.0 utf8\n# Original illustrative model; units = 0.1 inch; NOT a certified mechanical model.']
        for n,c in materials.items():self.parts.append(f'DEF {n} Appearance {{ material Material {{ diffuseColor {c[0]} {c[1]} {c[2]} specularColor .25 .25 .25 shininess .35 }} }}')
        self.bounds=[]
    def mesh(self,vertices,faces,mat):
        self.bounds+=vertices
        coords=', '.join(f'{x/2.54:.6f} {-y/2.54:.6f} {z/2.54:.6f}' for x,y,z in vertices)
        inds=', '.join(' '.join(map(str,reversed(f)))+' -1' for f in faces)
        self.parts.append(f'Shape {{ appearance USE {mat} geometry IndexedFaceSet {{ solid TRUE ccw TRUE coord Coordinate {{ point [ {coords} ] }} coordIndex [ {inds} ] }} }}')
    def box(self,x,y,z,w,l,h,mat):
        v=[(x,y,z),(x+w,y,z),(x+w,y+l,z),(x,y+l,z),(x,y,z+h),(x+w,y,z+h),(x+w,y+l,z+h),(x,y+l,z+h)]
        self.mesh(v,[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]],mat)
    def tube(self,a,b,r,mat,sides=16):
        def norm(v):
            n=math.sqrt(sum(x*x for x in v));return [x/n for x in v]
        def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
        t=norm([b[k]-a[k] for k in range(3)]);u=norm(cross(t,[0,0,1] if abs(t[2])<.9 else [1,0,0]));v=cross(t,u)
        verts=[]
        for p in [a,b]:
            for i in range(sides):
                q=2*math.pi*i/sides;verts.append(tuple(p[k]+r*(math.cos(q)*u[k]+math.sin(q)*v[k]) for k in range(3)))
        faces=[list(reversed(range(sides))),list(range(sides,2*sides))]
        faces += [[i,(i+1)%sides,(i+1)%sides+sides,i+sides] for i in range(sides)]
        self.mesh(verts,faces,mat)
    def save(self):
        path=M/(self.name+'.wrl');path.write_text('\n'.join(self.parts)+'\n',encoding='utf-8',newline='\n')
        return {'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'bounds_mm':[[round(min(v[k] for v in self.bounds),3),round(max(v[k] for v in self.bounds),3)] for k in range(3)]}
def socket(m,x,n):
    m.box(x-1.27,-1.27,0,2.54,n*2.54,8.5,'black')
    for i in range(n):
        y=i*2.54;m.box(x-.30,y-.30,-3.0,.60,.60,12.4,'gold')
        m.box(x-.65,y-.65,8.50,1.30,1.30,.12,'silver')
        m.box(x-.30,y-.30,8.61,.60,.60,.8,'gold')
def small_parts(m,z,locations):
    for x,y in locations:
        m.box(x-.6,y-1,z,1.2,2,.65,'ceramic');m.box(x-.6,y-1,z,1.2,.3,.70,'silver');m.box(x-.6,y+.7,z,1.2,.3,.70,'silver')
generated=[]
esp=Model('ESP32_S3_DevKitC1_visual');socket(esp,0,22);socket(esp,22.86,22)
esp.box(-1.27,-1.4,9,25.4,62.74,1.6,'pcb_black')
esp.box(2.54,-8,10.6,17.78,25.5,.8,'pcb_green')
esp.box(2.85,-.7,11.4,17.15,17.8,2.2,'silver')
for i in range(4):
    y=-7.1+i*1.3;esp.box(4,y,11.42,14,.22,.08,'gold')
    if i<3:esp.box(4 if i%2 else 17.8,y,11.42,.22,1.5,.08,'gold')
for x in [3.5,16.0]:
    esp.box(x,55.3,10.6,7.5,6.7,.35,'silver');esp.box(x,55.3,13.15,7.5,6.7,.35,'silver')
    esp.box(x,55.3,10.95,.35,6.7,2.2,'silver');esp.box(x+7.15,55.3,10.95,.35,6.7,2.2,'silver')
    esp.box(x+.6,55.5,11.15,6.3,5.8,1.6,'black')
for x in [4.2,17.5]:
    esp.box(x,46.2,10.6,3.4,3.4,.8,'silver');esp.box(x+.6,46.8,11.4,2.2,2.2,.9,'black')
esp.box(3.5,35.5,10.6,5,5,.9,'black');esp.box(14.7,33.5,10.6,4.5,4.5,.8,'black')
small_parts(esp,10.6,[(4,26),(9,27),(17,25),(12,43),(20,41)])
generated.append(esp.save())
for name,n,along,depth,color in [('MPU6050_GY521_visual',8,21.2,16.4,'pcb_blue'),('BMP280_GY_visual',6,15.5,11.8,'pcb_purple'),('GY271_visual',5,13.5,14.8,'pcb_blue')]:
    m=Model(name);socket(m,0,n);top=-(along-(n-1)*2.54)/2;m.box(-1.27,top,9,depth,along,1,'pcb_blue' if name.startswith('MPU') else color)
    if name.startswith('MPU'):
        m.box(5.0,6.6,10,4,4,1,'black');m.box(9.2,13.5,10,2.1,3,.8,'black');small_parts(m,10,[(4.5,3.3),(10.8,5.3),(10,11.5)])
    elif name.startswith('BMP'):
        m.box(3.8,5.1,10,2.5,2.1,.95,'silver');m.tube((5,6.2,10.96),(5,6.2,10.98),.23,'black');m.box(6.2,10,10,2.2,2.7,.7,'black');small_parts(m,10,[(3.5,1.6),(7.6,4)])
    else:
        m.box(5,3.8,10,3,3,.9,'black');m.box(7.5,8,10,2.3,2.5,.8,'black');small_parts(m,10,[(3.8,1),(10.5,2.8)])
    generated.append(m.save())
header=Model('ESC_3pin_header_visual');header.box(-1.27,-1.27,0,2.54,7.62,2.54,'black')
for i in range(3):header.box(-.32,i*2.54-.32,-3,.64,.64,11,'gold')
generated.append(header.save())
lidar=Model('Lidar_solder_wire_stubs_visual')
for i,(x,color) in enumerate([(0,'red'),(8,'black'),(12,'yellow'),(17.08,'white')]):
    lidar.tube((x,0,-2.5),(x,0,1.5),.28,'silver');lidar.tube((x,0,1.5),(x,0,5.5),.60,color)
    lidar.tube((x,0,5.5),(x+(i-1.5)*.4,5,10),.60,color)
generated.append(lidar.save())
# Assembly-only ground overlay: attached to the project-specific J1 anchor.
g=Model('Ground_wires_assembly_visual')
for points,color in [([(113.96,71.81),(113.96,61.65)],'pink'),([(98.36,106.34),(75.5,106.34),(58,107)],'blue')]:
    rel=[(x-47,y-95.5,-2.35) for x,y in points]
    for a,b in zip(rel,rel[1:]):g.tube(a,b,.40,color)
    for x,y,z in rel:g.tube((x,y,-2.7),(x,y,-1.2),.32,'silver')
generated.append(g.save())
bindings={'U1':['ESP32_S3_DevKitC1_visual'],'U2':['MPU6050_GY521_visual'],'U3':['BMP280_GY_visual'],'U4':['GY271_visual'],
          'J2':['ESC_3pin_header_visual'],'J3':['ESC_3pin_header_visual'],'J4':['ESC_3pin_header_visual'],'J5':['ESC_3pin_header_visual'],
          'J1':['Lidar_solder_wire_stubs_visual','Ground_wires_assembly_visual']}
def attach(text,models):
    cuts=[(a,b) for a,b in children(text) if text[a:b].startswith('(model ')]
    for a,b in reversed(cuts):text=text[:a]+text[b:]
    blocks='\n'.join(f'(model "${{KIPRJMOD}}/models3d/{n}.wrl" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))' for n in models)
    return '\n'.join(line.rstrip() for line in (text.rstrip()[:-1]+'\n'+blocks+'\n)').splitlines())
board=O/'QCopter.kicad_pcb';source=board.read_text(encoding='utf-8');out=source
found={}
for a,b in reversed(children(source)):
    part=source[a:b]
    if not part.startswith('(footprint '):continue
    ref=__import__('re').search(r'\(property "Reference" "([^"]+)"',part)[1]
    if ref not in bindings:continue
    found[ref]=True;out=out[:a]+attach(part,bindings[ref])+out[b:]
assert set(found)==set(bindings)
board.write_text(out,encoding='utf-8',newline='\n')
manifest=json.loads((O/'connections.json').read_text())
for c in manifest['components']:
    if c['reference'] not in bindings:continue
    file=O/'QCopterModules.pretty'/(c['name']+'.kicad_mod')
    file.write_text(attach(file.read_text(encoding='utf-8'),bindings[c['reference']])+'\n',encoding='utf-8',newline='\n')
report={'engine':'Original generated VRML meshes, native KiCad 3D renderer verification pending','model_units_mm':2.54,
        'model_vertex_conversion':'x/2.54, -y/2.54, z/2.54; native model scale 1',
        'nominal_module_pcb_bottom_z_mm':9,'assumed_socket_height_mm':8.5,'bindings':bindings,'models':generated,
        'not_mechanically_certified':True,'assumptions':['Board envelopes and pin pitch follow current reference footprints','Sockets 8.5mm and module PCB/part heights are visual assumptions','Small component positions, USB shells and antenna artwork are illustrative','Ground wire overlay is specific to this board placement; regenerate after moving parts','Lidar is remote; only soldered wire stubs are shown; colours are chosen, not a factory cable pinout']}
(M/'models.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
print('Attached',sum(len(v) for v in bindings.values()),'portable model references to',len(bindings),'footprints;',len(generated),'original VRML files')


