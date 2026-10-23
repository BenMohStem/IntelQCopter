"""Single-bottom-copper ESP32-S3 prototype carrier. Run with KiCad's Python.
Reference geometry: Espressif DevKitC-1, 2x22 pins, 2.54mm pitch/22.86mm rows.
This is an assembly/review prototype, not a validated flight-board release.
"""
from pathlib import Path
import csv,json,math,uuid,pcbnew as p
D=Path(__file__).resolve().parent
MECH=json.loads((D/'reference-dimensions.json').read_text(encoding='utf8'))
(D/'HomeEtch.pretty').mkdir(exist_ok=True)
def uid(s):return str(uuid.uuid5(uuid.NAMESPACE_URL,'intelqcopter/home-etch/'+s))
board=p.BOARD();board.SetCopperLayerCount(2) # KiCad format; ONLY B.Cu is routed/etched.
def vec(x,y):return p.VECTOR2I(p.FromMM(x),p.FromMM(y))
nets={};pads={};components=[];links=[];segments=[]
def net(name):
 if name not in nets:
  n=p.NETINFO_ITEM(board,name);board.Add(n);nets[name]=n
 return nets[name]
def text(s,x,y,size=1,layer=p.F_SilkS):
 t=p.PCB_TEXT(board);t.SetText(s);t.SetPosition(vec(x,y));t.SetTextSize(vec(size,size));t.SetTextThickness(p.FromMM(.15));t.SetLayer(layer);board.Add(t)
REFS={'U1L': 'J1', 'U1R': 'J2', 'J_PWR': 'J3', 'J_IMU': 'J4', 'J_BARO': 'J5', 'J_MAG': 'J6', 'J_BAT': 'J7', 'J_FL': 'J8', 'J_FR': 'J9', 'J_RR': 'J10', 'J_RL': 'J11', 'J_ROS': 'J12', 'J_LIDAR': 'J13', 'J_CURRENT': 'J14', 'R_VTOP': 'R1', 'R_VBOTTOM': 'R2', 'R_VFILTER': 'R3', 'R_FL': 'R4', 'R_FR': 'R5', 'R_RR': 'R6', 'R_RL': 'R7', 'R_IFILTER': 'R8', 'C_VFILTER': 'C1', 'C_IFILTER': 'C2'}
def fp(ref,value,items,diameter=3.2,drill=1.2):
 role=ref;ref=REFS.get(ref,ref)
 f=p.FOOTPRINT(board);f.SetReference(ref);f.SetValue(value);f.SetPosition(vec(items[0][1],items[0][2]));f.Reference().SetVisible(False);f.Value().SetVisible(False);board.Add(f)
 # Actual passive bodies on F.Fab, independent from enlarged copper pads.
 if ref.startswith(('R','C')):
  cx=sum(a[1] for a in items)/len(items);cy=sum(a[2] for a in items)/len(items)
  w,h=(6.5,2.5) if ref.startswith('R') else (4.0,2.6)
  for a,b in [((cx-w/2,cy-h/2),(cx+w/2,cy-h/2)),((cx+w/2,cy-h/2),(cx+w/2,cy+h/2)),((cx+w/2,cy+h/2),(cx-w/2,cy+h/2)),((cx-w/2,cy+h/2),(cx-w/2,cy-h/2))]:
   shape=p.PCB_SHAPE(f);shape.SetShape(p.SHAPE_T_SEGMENT);shape.SetStart(vec(*a));shape.SetEnd(vec(*b));shape.SetLayer(p.F_Fab);shape.SetWidth(p.FromMM(.1));f.Add(shape)
 for num,x,y,name in items:
  pad=p.PAD(f);pad.SetNumber(str(num));pad.SetShape(p.PAD_SHAPE_CIRCLE);pad.SetAttribute(p.PAD_ATTRIB_PTH)
  pad.SetSize(vec(diameter,diameter));pad.SetDrillSize(vec(drill,drill));pad.SetLayerSet(p.PAD.PTHMask());pad.SetPosition(vec(x,y))
  pad.SetNet(net(name if name else 'unconnected-('+ref+'-Pad'+str(num)+')'))
  if name=='GND':pad.SetLocalZoneConnection(p.ZONE_CONNECTION_FULL)
  f.Add(pad);pads[(ref,str(num))]=(x,y,name)
 f.SetFPIDAsString('HomeEtch:'+ref);f.SetPath(p.KIID_PATH('/'+uid('root')+'/'+uid(ref)))
 clone=p.FOOTPRINT(f);clone.SetPosition(vec(0,0));clone.Reference().SetText('REF**')
 for a in clone.Pads():a.SetNetCode(0)
 p.PCB_IO_MGR.FindPlugin(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(D/'HomeEtch.pretty'),clone)
 components.append(dict(ref=ref,role=role,value=value,pad_diameter_mm=diameter,drill_mm=drill,pads=[dict(number=str(i),x=x,y=y,net=n) for i,x,y,n in items]))
 return f
def route(name,points,width=1.):
 for a,b in zip(points,points[1:]):
  if a==b:continue
  t=p.PCB_TRACK(board);t.SetStart(vec(*a));t.SetEnd(vec(*b));t.SetWidth(p.FromMM(width));t.SetLayer(p.B_Cu);t.SetNet(net(name));board.Add(t);segments.append(dict(net=name,start=a,end=b,width=width))
def wire(ref,a,na,b,nb,description):
 fp(ref,'INSULATED_WIRE',[(1,*a,na),(2,*b,nb)],2.8,1.2)
 links.append(dict(ref=ref,from_xy_mm=a,from_net=na,to_xy_mm=b,to_net=nb,minimum_straight_length_mm=round(math.dist(a,b),1),instruction=description))
def line(a,b,layer=p.Edge_Cuts,width=.1):
 s=p.PCB_SHAPE();s.SetShape(p.SHAPE_T_SEGMENT);s.SetStart(vec(*a));s.SetEnd(vec(*b));s.SetLayer(layer);s.SetWidth(p.FromMM(width));board.Add(s)

for a,b in [((0,0),(100,0)),((100,0),(100,100)),((100,100),(0,100)),((0,100),(0,0))]:line(a,b)
# Socket numbering is the actual two header rows, viewed FROM COMPONENT SIDE.
L=['3V3','3V3','','GPIO4','GPIO5','GPIO6','GPIO7','','','GPIO17','GPIO18','GPIO8','','','GPIO9','','','GPIO12','','GPIO14','5V_MCU','GND']
R=['GND','','','GPIO1','GPIO2','','','','','','','','','','','','','','','','','']
R[20]=R[21]='GND'
fp('U1L','DevKitC-1 LEFT 1x22',[(i+1,40,14+i*2.54,n) for i,n in enumerate(L)],1.94,1.1)
fp('U1R','DevKitC-1 RIGHT 1x22',[(i+1,62.86,14+i*2.54,n) for i,n in enumerate(R)],1.94,1.1)
# Published body, separate from the larger RF/USB service envelopes.
x0,y0,x1,y1=MECH['devkit']['body_xy_mm']
for a,b in [((x0,y0),(x1,y0)),((x1,y0),(x1,y1)),((x1,y1),(x0,y1)),((x0,y1),(x0,y0))]:line(a,b,p.F_Fab,.15)
ux0,uy0,ux1,uy1=MECH['usb_access']['rect_xy_mm']
for a,b in [((ux0,uy0),(ux1,uy0)),((ux1,uy0),(ux1,uy1)),((ux1,uy1),(ux0,uy1)),((ux0,uy1),(ux0,uy0))]:line(a,b,p.Dwgs_User,.1)
text('ESP32-S3 DevKitC-1',51.4,9,1.0);text('USB / buttons access',51.4,74.2,.9)
text('ANTENNA: NO COPPER / METAL',51,3,1.)
text('TOP / COMPONENT VIEW',50,97.2,1.0)
# Photo-matched header order: pin1 VCC, pin2 GND, pin3 SCL, pin4 SDA.
# These are solder/header landings for modules on wires. Body/hole outlines of
# unknown clone boards are deliberately not asserted by a chip datasheet.
fp('J_PWR','REGULATED 5V ONLY',[(1,8,89,'GND'),(2,13.08,89,'5V_EXT')])
text('GND   +5V IN',12,94.2,1.)
sensor_headers=[
 ('J_IMU',45,'GY-521 MPU6050',['SENS_3V3','GND','I2C_SCL','I2C_SDA','','','GND','']),
 ('J_BARO',55,'GY-BM280 I2C',['SENS_3V3','GND','I2C_SCL','I2C_SDA','BARO_CSB','GND']),
 ('J_MAG',75,'GY-271 MAG',['SENS_3V3','GND','I2C_SCL','I2C_SDA',''])]
for ref,y,label,names in sensor_headers:
 fp(ref,label,[(i+1,round(8+2.54*i,2),y,n) for i,n in enumerate(names)],1.94,1.1)
 text(label,16,y-3.0,.9,p.Dwgs_User)
 text('1:VCC 2:G 3:SCL 4:SDA',16,y+3.1,.8,p.Dwgs_User)
# Solid common ground trunk kept clear of the new VCC column at x=8.
route('GND',[(8,19.08),(4,19.08),(4,93),(94.08,93),(94.08,20)],2.)
route('GND',[(10.54,45),(10.54,77.5),(30.32,77.5)],.8)
route('SENS_3V3',[(8,39),(8,75)],1.)
route('I2C_SCL',[(13.08,38),(13.08,75)],1.)
route('I2C_SDA',[(15.62,41),(15.62,75)],1.)
# I2C requires CSB high; SDO low chooses address0x76. AD0 low chooses MPU0x68.
# The removable top wire W24 crosses the I2C columns without top copper.
wire('W24',(8,62),'SENS_3V3',(18.16,62),'BARO_CSB','BMP280 CSB HIGH for I2C: fit this insulated wire')
route('BARO_CSB',[(18.16,55),(18.16,62)],1.)
# Battery voltage low-current input, divider and RC. NOT an ESC power feed.
fp('J_BAT','SENSE ONLY',[(1,8,14,'VBAT_SENSE'),(2,8,19.08,'GND')]);text('PACK+ sense',10,10,1.)
fp('R_VTOP','47k 1%',[(1,14,14,'VBAT_SENSE'),(2,24.16,14,'VBAT_DIV')])
fp('R_VBOTTOM','10k 1%',[(1,14,24,'GND'),(2,24.16,24,'VBAT_DIV')])
fp('R_VFILTER','1k',[(1,24.16,29,'VBAT_DIV'),(2,14,29,'VBAT_ADC')])
fp('C_VFILTER','100nF X7R 5mm',[(1,14,34,'VBAT_ADC'),(2,9.0,34,'GND')])
route('VBAT_SENSE',[(8,14),(14,14)])
route('VBAT_DIV',[(24.16,14),(24.16,29)])
route('GND',[(14,24),(8,24)]);route('GND',[(9.0,34),(4,34)])
route('VBAT_ADC',[(14,29),(14,34),(18.16,34)])
text('47k',19,11,.9);text('10k',19,21,.9);text('1k',19,27,.9)
# ESC signal resistors and signal-ground returns; BEC positive omitted.
for ref,y,sig,gpio in [('FL',60,'ESC_FL','GPIO12'),('FR',20,'ESC_FR','GPIO1'),('RR',30,'ESC_RR','GPIO2'),('RL',70,'ESC_RL','GPIO14')]:
 fp('R_'+ref,'220R',[(1,73,y,gpio if ref in ('FR','RR') else gpio+'_ESC'),(2,83.16,y,sig)])
 fp('J_'+ref,sig,[(1,89,y,sig),(2,94.08,y,'GND')]);route(sig,[(83.16,y),(89,y)])
 text(ref+' SIG GND',86,y-3.5,.9)
route('GPIO1',[(62.86,21.62),(67,21.62),(69,20)])
route('GPIO1',[(69,20),(73,20)])
route('GPIO2',[(62.86,24.16),(68,24.16),(68,30),(69,30)])
route('GPIO2',[(69,30),(73,30)])
# Optional ROS2: signal only, no companion power carried by PCB.
fp('J_ROS','OPTIONAL 3V3 UART',[(1,89,40,'COMP_TX'),(2,89,45.08,'COMP_RX'),(3,89,50.16,'GND')])
route('COMP_TX',[(78,40),(89,40)]);route('COMP_RX',[(78,45.08),(89,45.08)]);route('GND',[(89,50.16),(94.08,50.16)])
text('ROS TX / RX / G',86,36,1.)
# Exact TOFSense-F2 Mini: 4.3-5.2V supply, 3.3V UART (official V1.0 datasheet).
fp('J_LIDAR','F2 MINI WIRE PADS',[(1,73,40,'GND'),(2,73,45.08,'LIDAR_5V'),(3,73,50.16,'LIDAR_RX_FC'),(4,73,55.24,'LIDAR_TX_FC')])
route('GND',[(73,40),(73,36.8),(94.08,36.8)])
route('LIDAR_RX_FC',[(68,50.16),(73,50.16)]);route('LIDAR_TX_FC',[(68,55.24),(73,55.24)])
text('LIDAR REG 5V',75,64,1.)
# Current-sense SIGNAL connector; external current sensor/PDB not designed here.
fp('J_CURRENT','0-2.5V current input',[(1,15,83,'GND'),(2,20.08,83,'SENS_3V3'),(3,25.16,83,'CURRENT_IN')])
route('GND',[(15,83),(4,83)])
route('SENS_3V3',[(8,75),(8,79.5),(20.08,79.5),(20.08,83)],1.)
fp('R_IFILTER','1k',[(1,25.16,88,'CURRENT_IN'),(2,35.32,88,'CURRENT_ADC')])
fp('C_IFILTER','100nF X7R 5mm',[(1,35.32,82,'CURRENT_ADC'),(2,30.32,82,'GND')])
route('CURRENT_IN',[(25.16,83),(25.16,88)])
route('CURRENT_ADC',[(35.32,88),(35.32,77)])
route('GND',[(30.32,82),(30.32,74)])
text('CURRENT: G V3 OUT',60,83.5,.9)
# Top insulated links isolate crossings and enlarge their soldering points.
ends={1:((8,39),'SENS_3V3','3V3 sensor supply'),4:((18.16,34),'VBAT_ADC','Voltage RC output to GPIO4; disconnect sense when MCU unpowered'),5:((35.32,77),'CURRENT_ADC','Filtered external current signal to GPIO5'),6:((68,50.16),'LIDAR_RX_FC','Future lidar TX -> FC RX; driver pending'),7:((68,55.24),'LIDAR_TX_FC','Future lidar RX <- FC TX; driver pending'),10:((78,40),'COMP_TX','FC TX -> companion RX'),11:((78,45.08),'COMP_RX','FC RX <- companion TX'),12:((13.08,38),'I2C_SCL','Shared I2C clock'),15:((15.62,41),'I2C_SDA','Shared I2C data'),18:((69,60),'GPIO12_ESC','FL signal via 220R'),20:((69,70),'GPIO14_ESC','RL signal via 220R'),21:((20,89),'5V_EXT','REMOVABLE POWER LINK: remove for USB power')}
for idx,(end,external,desc) in ends.items():
 y=14+(idx-1)*2.54;x=34 if idx%2==0 else 30;gpio=L[idx-1]
 route(gpio,[(40,y),(x,y)],1.)
 wire('W'+str(idx),(x,y),gpio,end,external,desc)
route('5V_EXT',[(13.08,89),(20,89)],1.5)
route('GPIO12_ESC',[(69,60),(73,60)])
route('GPIO14_ESC',[(69,70),(73,70)])
# Four documented DevKit GND pins connect directly to B.Cu GND, without W22.
# Left J1.22 and right J2.1/J2.21/J2.22 use same-net copper plus the large fill.
route('GND',[(40,67.34),(44,67.34)],1.5)
route('GND',[(44,67.34),(56,67.34)],2.5)
route('GND',[(62.86,14),(94.08,14),(94.08,20)],1.5)
route('GND',[(62.86,64.8),(66,64.8),(66,67.34),(62.86,67.34)],1.5)
route('GND',[(56,67.34),(66,67.34)],2.5)

# Both reference DevKit 3V3 header pins share copper.
route('3V3',[(40,14),(40,16.54)],1.)
# Insulated top link supplies lidar from external regulated5V, never 3V3.
wire('W23',(19,87),'5V_EXT',(81.5,53.5),'LIDAR_5V','TOFSense-F2 Mini regulated 5V supply; UART signals remain 3.3V')
route('5V_EXT',[(20,89),(19,87)],1.5)
route('LIDAR_5V',[(73,45.08),(75.5,47.8),(81.5,47.8),(81.5,53.5)],1.)

# Only mounting holes; physical F450 matching is not assumed.
for i,(x,y) in enumerate([(5,5),(95,5),(5,97),(95,97)]):
 f=p.FOOTPRINT(board);f.SetAttributes(p.FP_BOARD_ONLY);f.SetReference('H'+str(i+1));f.SetPosition(vec(x,y));f.Reference().SetVisible(False);f.Value().SetVisible(False);board.Add(f)
 a=p.PAD(f);a.SetAttribute(p.PAD_ATTRIB_NPTH);a.SetShape(p.PAD_SHAPE_CIRCLE);a.SetSize(vec(3.2,3.2));a.SetDrillSize(vec(3.2,3.2));a.SetPosition(vec(x,y));a.SetLayerSet(p.PAD.PTHMask());f.Add(a)
text('PROTOTYPE - NO MOTOR POWER',53,87,1.)
text('100 x 100 mm / bottom copper only',53,90,1.)
(D/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "HomeEtch") (type "KiCad") (uri "${KIPRJMOD}/HomeEtch.pretty") (options "") (descr "Project-local enlarged THT pads and insulated links")))',encoding='utf8')
# Bottom GND plane: keep a wide connected copper area, never floating islands.
# Notch clears the reference antenna region; actual board/frame RF fit remains unvalidated.
plane=p.ZONE(board);plane.SetLayer(p.B_Cu);plane.SetNet(net('GND'))
plane.SetLocalClearance(p.FromMM(.8));plane.SetMinThickness(p.FromMM(.8))
plane.SetPadConnection(p.ZONE_CONNECTION_FULL) # Solid GND: no cross-shaped thermal reliefs.
plane.SetThermalReliefGap(p.FromMM(.8));plane.SetThermalReliefSpokeWidth(p.FromMM(1.0))
plane.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS)
poly=plane.Outline();poly.NewOutline()
for x,y in [(2.5,2.5),(36,2.5),(36,17),(67,17),(67,2.5),(97.5,2.5),(97.5,97.5),(2.5,97.5)]:
 poly.Append(int(p.FromMM(x)),int(p.FromMM(y)))
board.Add(plane);board.BuildConnectivity();p.ZONE_FILLER(board).Fill(board.Zones())
output=D/'esp32_home_etch.kicad_pcb';p.SaveBoard(str(output),board)
(D/'components-and-nets.json').write_text(json.dumps(dict(status='DRAFT prototype; electrical/mechanical/physical review pending',components=components,wire_links=links,bottom_tracks=segments,mechanical=MECH),indent=2)+'\n',encoding='utf8')
with (D/'wire-links.csv').open('w',newline='',encoding='utf8') as f:
 w=csv.writer(f);w.writerow(['Link','From XY mm','From net','To XY mm','To net','Straight distance mm (add routing/slack)','Instruction'])
 for a in links:w.writerow([a['ref'],a['from_xy_mm'],a['from_net'],a['to_xy_mm'],a['to_net'],a['minimum_straight_length_mm'],a['instruction']])
print('Saved',output,'components',len(components),'wire links',len(links))
