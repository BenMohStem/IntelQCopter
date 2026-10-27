"""Independent zero-jumper CAD gate. Run with KiCad's bundled Python.
This proves scoped CAD properties, not manufactured hardware or RF performance.
"""
from pathlib import Path
import argparse,datetime,hashlib,json,math,os,re,subprocess,sys,xml.etree.ElementTree as ET
import pcbnew as p
from geometry import component_labels,touches

EXPECTED_COUNTS={'U1':44,'U2':8,'U3':6,'U4':5,'J1':4,'J2':3,'J3':3,'J4':3,'J5':3}
EXPECTED={
'U2':{'1':'3V3','2':'GND','3':'MPU_SCL','4':'MPU_SDA','7':'GND','8':'MPU_INT'},
'U3':{'1':'3V3','2':'GND','3':'SLOW_SCL','4':'SLOW_SDA','5':'3V3','6':'GND'},
'U4':{'1':'3V3','2':'GND','3':'SLOW_SCL','4':'SLOW_SDA','5':'MAG_DRDY'},
'J1':{'1':'FC_5V','2':'GND','3':'LIDAR_TO_FC_RX','4':'FC_TX_TO_LIDAR'}}
SIGNALS={'MPU_SCL','MPU_SDA','MPU_INT','SLOW_SCL','SLOW_SDA','MAG_DRDY','LIDAR_TO_FC_RX','FC_TX_TO_LIDAR','ESC1_SIGNAL','ESC2_SIGNAL','ESC3_SIGNAL','ESC4_SIGNAL'}
GPIO_BY_PAD={4:4,5:5,6:6,7:7,8:15,9:16,10:17,11:18,12:8,13:3,14:46,15:9,16:10,17:11,18:12,19:13,20:14,
41:1,40:2,39:42,38:41,37:40,36:39,35:38,34:37,33:36,32:35,31:0,30:45,29:48,28:47,27:21,26:20,25:19,43:43,42:44}
SAFE_GPIO={1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,39,40,41,42,47}
POWER={'1':'3V3','2':'3V3','21':'FC_5V','22':'GND','23':'GND','24':'GND','44':'GND'}

def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def xy(q):return (p.ToMM(q.x),p.ToMM(q.y))
def ring(chain):return [xy(chain.CPoint(i)) for i in range(chain.PointCount())]
def unused(name):return not name or name.startswith('unconnected-')



def inline_header_geometry(points, pitch=2.54, tolerance=1e-5):
    """Require all pads on the same straight line, in increasing pin order."""
    if len(points)<2:
        return False
    dx=points[1][0]-points[0][0]
    dy=points[1][1]-points[0][1]
    if abs(math.hypot(dx,dy)-pitch)>tolerance:
        return False
    return all(math.dist(q,(points[0][0]+i*dx,points[0][1]+i*dy))<=tolerance for i,q in enumerate(points))


def rf_rule_covers_antenna(antenna, keepouts):
    """Subtract eligible KiCad rule polygons from the complete nominal antenna.

    This uses the actual rule polygon geometry, including holes, rather than
    merely checking that an unrelated rule exists somewhere on the board.
    A tiny 0.001 mm square residual is tolerated for integer-coordinate rounding.
    """
    if antenna is None or not keepouts:
        return False
    remaining=p.SHAPE_POLY_SET()
    remaining.NewOutline()
    for x,y in antenna['outer']:
        remaining.Append(p.FromMM(x),p.FromMM(y))
    for zone in keepouts:
        remaining.BooleanSubtract(zone.Outline())
    return remaining.IsEmpty() or abs(remaining.Area()) <= p.FromMM(.001)**2


def assess(project,run_native=True,report_dir=None,bmp_address='0x76',single_3v3_pad=None):
    project=Path(project).resolve();report_dir=Path(report_dir).resolve();report_dir.mkdir(parents=True,exist_ok=True)
    files=list(project.glob('*.kicad_pcb'));assert len(files)==1,'Exactly one PCB in project root'
    board_file=files[0];sch=board_file.with_suffix('.kicad_sch');pro=board_file.with_suffix('.kicad_pro')
    b=p.LoadBoard(str(board_file));fails=[];facts={};checks=[]
    expected={ref:dict(q) for ref,q in EXPECTED.items()}
    expected['U3']['6']='GND' if bmp_address=='0x76' else '3V3'
    expected_ground_count=14 if bmp_address=='0x76' else 13
    unused_3v3_pin=str(3-int(single_3v3_pad)) if single_3v3_pad else None
    def check(ok,name,detail=None):
        checks.append({'check':name,'pass':bool(ok),'detail':detail})
        if not ok:fails.append(name)
    fps={f.GetReference():f for f in b.GetFootprints()}
    check(not re.search(r'\(jumper_pad_groups\b',board_file.read_text()),'no_explicit_internal_pad_jumper_groups')
    check(set(fps)==set(EXPECTED_COUNTS),'exact_nine_components_no_jumpers',sorted(fps))
    pads={}
    shapes=[];gnd_shapes=[];gnd_refs=[]
    for ref,f in fps.items():
        ps=list(f.Pads());numbers=[a.GetNumber() for a in ps]
        check(len(ps)==EXPECTED_COUNTS.get(ref,-1) and len(set(numbers))==len(numbers),'pad_count_unique_'+ref, len(ps))
        check(not f.IsNetTie() and not f.GetDuplicatePadNumbersAreJumpers(),'no_virtual_internal_connections_'+ref)
        check(not any(g.IsOnLayer(p.F_Cu) or g.IsOnLayer(p.B_Cu) for g in f.GraphicalItems()),'no_hidden_copper_graphics_'+ref)
        for a in ps:
            key=(ref,a.GetNumber());pads[key]=a
            circular=a.GetShape()==p.PAD_SHAPE_CIRCLE and a.GetSize().x==a.GetSize().y
            check(circular,'round_pad_'+ref+'.'+a.GetNumber())
            check(a.GetAttribute()==p.PAD_ATTRIB_PTH and p.ToMM(a.GetDrillSize().x)>=.9,'drilled_PTH_'+ref+'.'+a.GetNumber())
            check(p.ToMM(a.GetSize().x)>=2-1e-6 and p.ToMM(a.GetSize().x-a.GetDrillSize().x)/2>=.45-1e-6,'solder_land_'+ref+'.'+a.GetNumber())
            if circular:
                s={'kind':'circle','point':xy(a.GetPosition()),'radius':p.ToMM(a.GetSize().x)/2,'name':ref+'.'+a.GetNumber(),'net':a.GetNetname()};shapes.append(s)
                if a.GetNetname()=='GND':gnd_refs.append((s['name'],len(gnd_shapes)));gnd_shapes.append(s)
    check(len(pads)==79,'exact_79_real_pads_holes',len(pads))
    for ref,pinmap in expected.items():
        if ref not in fps:continue
        for a in fps[ref].Pads():
            net=a.GetNetname();pin=a.GetNumber()
            check(net==pinmap[pin] if pin in pinmap else unused(net),'sensor_lidar_pin_'+ref+'.'+pin,net)
    selected=[]
    for ref,number in [('J5',1),('J2',2),('J4',3),('J3',4)]:
        if ref not in fps:continue
        for pin,net in [('1','GND'),('3',f'ESC{number}_SIGNAL')]:check(pads.get((ref,pin)) is not None and pads[(ref,pin)].GetNetname()==net,'esc_pin_'+ref+'.'+pin)
        net=pads[(ref,'2')].GetNetname()
        if net=='FC_5V':selected.append(ref)
        else:check(unused(net) or net==f'ESC{number}_BEC_ISOLATED','isolated_BEC_'+ref,net)
    check(len(selected)==1,'exactly_one_BEC_positive_feeds_5V',selected)
    assignment={}
    if 'U1' in fps:
        for a in fps['U1'].Pads():
            pin=a.GetNumber();net=a.GetNetname()
            if pin==unused_3v3_pin:check(unused(net),'unused_duplicate_3v3_'+pin,net)
            elif pin in POWER:check(net==POWER[pin],'esp_power_'+pin,net)
            elif net in SIGNALS:
                gpio=GPIO_BY_PAD.get(int(pin));assignment.setdefault(net,[]).append(gpio)
                check(gpio in SAFE_GPIO,'eligible_GPIO_'+net, gpio)
            else:check(unused(net),'unused_esp_pin_'+pin,net)
        check(set(assignment)==SIGNALS and all(len(v)==1 for v in assignment.values()),'one_eligible_GPIO_per_signal',assignment)
        # Reference footprint geometry is independent of board rotation/placement.
        if all(('U1',q) in pads for q in ['1','2','44']):
            origin=xy(pads[('U1','1')].GetPosition());px=xy(pads[('U1','44')].GetPosition());py=xy(pads[('U1','2')].GetPosition())
            ex=((px[0]-origin[0])/22.86,(px[1]-origin[1])/22.86);ey=((py[0]-origin[0])/2.54,(py[1]-origin[1])/2.54)
            errors=[]
            for (ref,pin),a in pads.items():
                if ref!='U1':continue
                i=int(pin);x=0 if i<=22 else 22.86;y=(i-1)*2.54 if i<=22 else (44-i)*2.54
                q=(origin[0]+ex[0]*x+ey[0]*y,origin[1]+ex[1]*x+ey[1]*y);errors.append(math.dist(q,xy(a.GetPosition())))
            check(abs(math.hypot(*ex)-1)<1e-5 and abs(math.hypot(*ey)-1)<1e-5 and abs(ex[0]*ey[0]+ex[1]*ey[1])<1e-5 and max(errors)<1e-5,'ESP_reference_header_geometry')
            antenna={'kind':'polygon','outer':[(origin[0]+ex[0]*x+ey[0]*y,origin[1]+ex[1]*x+ey[1]*y) for x,y in [(2.54,-8),(20.32,-8),(20.32,-1.4),(2.54,-1.4)]],'holes':[]}
        else:antenna=None
    else:antenna=None
    for ref in ['U2','U3','U4']:
        if ref not in fps:continue
        keys=[(ref,str(i)) for i in range(1,EXPECTED_COUNTS[ref]+1)]
        if not all(key in pads for key in keys):
            check(False,'inline_2.54mm_order_'+ref,'Missing header pad')
            continue
        q=[xy(pads[key].GetPosition()) for key in keys]
        check(inline_header_geometry(q),'inline_2.54mm_order_'+ref)
    tracks=list(b.GetTracks());check(bool(tracks),'routes_exist')
    check(all(type(t)==p.PCB_TRACK and t.GetLayer()==p.B_Cu and p.ToMM(t.GetWidth())>=1.2-1e-6 for t in tracks),'single_BCu_no_vias_minimum_track_1.2mm')
    for t in tracks:
        if type(t)!=p.PCB_TRACK:continue
        s={'kind':'segment','a':xy(t.GetStart()),'b':xy(t.GetEnd()),'radius':p.ToMM(t.GetWidth())/2,'net':t.GetNetname()};shapes.append(s)
        if t.GetNetname()=='GND':gnd_shapes.append(s)
    check(not any(g.IsOnLayer(p.F_Cu) or g.IsOnLayer(p.B_Cu) for g in b.GetDrawings()),'no_board_copper_graphic_shortcuts')
    for z in b.Zones():
        if z.GetIsRuleArea():continue
        check(z.GetLayer()==p.B_Cu,'only_BCu_pours',z.GetNetname())
        if not z.HasFilledPolysForLayer(p.B_Cu):check(False,'zones_filled');continue
        fill=z.GetFilledPolysList(p.B_Cu)
        for i in range(fill.OutlineCount()):
            s={'kind':'polygon','outer':ring(fill.COutline(i)),'holes':[ring(fill.CHole(i,j)) for j in range(fill.HoleCount(i))],'net':z.GetNetname()};shapes.append(s)
            if z.GetNetname()=='GND':gnd_shapes.append(s)
    labels=component_labels(gnd_shapes)
    groups={}
    for ref,i in gnd_refs:groups.setdefault(str(labels[i]),[]).append(ref)
    check(len(gnd_refs)==expected_ground_count and len(groups)==1,'all_GND_points_connected_in_bare_copper',{'expected_count':expected_ground_count,'components':list(groups.values())})
    power_groups={}
    for net,expected_count in [('3V3',(4 if bmp_address=='0x76' else 5)+(1 if single_3v3_pad else 2)),('FC_5V',3)]:
        copper=[s for s in shapes if s.get('net')==net];components=component_labels(copper);groups_for_net={}
        for i,s in enumerate(copper):
            if s.get('name'):groups_for_net.setdefault(str(components[i]),[]).append(s['name'])
        check(sum(len(q) for q in groups_for_net.values())==expected_count and len(groups_for_net)==1,'all_'+net+'_endpoints_connected_in_bare_copper',groups_for_net)
        power_groups[net]=groups_for_net
    if unused_3v3_pin:
        q=next((s for s in shapes if s.get('name')=='U1.'+unused_3v3_pin),None)
        check(q is not None and not any(s is not q and touches(q,s) for s in shapes),'unused_duplicate_3v3_has_no_carrier_copper_connection')
    collisions=[s.get('name',s.get('net','zone')) for s in shapes if antenna and touches(s,antenna)]
    check(antenna is not None and not collisions,'nominal_antenna_projection_no_carrier_copper',collisions)
    keepouts=[z for z in b.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowTracks() and z.GetDoNotAllowVias() and z.GetDoNotAllowPads() and z.GetDoNotAllowZoneFills() and z.GetLayerSet().Contains(p.F_Cu) and z.GetLayerSet().Contains(p.B_Cu)]
    check(bool(keepouts),'permanent_two_copper_layer_RF_rule_area')
    check(rf_rule_covers_antenna(antenna,keepouts),'nominal_antenna_projection_covered_by_RF_rule_area')
    if pro.exists():
        d=json.loads(pro.read_text());rules=d.get('board',{}).get('design_settings',{})
        check(rules.get('rules',{}).get('min_clearance',0)>=.5 and all(c.get('clearance',0)>=.5 for c in d.get('net_settings',{}).get('classes',[])),'explicit_0.5mm_project_clearance')
        check(not rules.get('drc_exclusions'),'no_DRC_exclusion_waivers')
        sevs=rules.get('rule_severities',{})
        check(all(sevs.get(k)!='ignore' for k in ['shorting_items','clearance','tracks_crossing','unconnected_items','items_not_allowed','track_width','invalid_outline']),'critical_DRC_checks_not_disabled')
    else:check(False,'project_rules_file_present')
    native={}
    if run_native and sch.exists():
        cli=Path(os.environ['LOCALAPPDATA'])/'Programs/KiCad/10.0/bin/kicad-cli.exe'
        commands=[[str(cli),'pcb','drc','--schematic-parity','--format','json','--output',str(report_dir/'drc.json'),str(board_file)],
                  [str(cli),'sch','export','netlist','--format','kicadxml','--output',str(report_dir/'netlist.xml'),str(sch)]]
        with (report_dir/'native-original.log').open('w',encoding='utf-8') as log:
            for cmd in commands:
                r=subprocess.run(cmd,capture_output=True,text=True);log.write('$ '+subprocess.list2cmdline(cmd)+'\n'+r.stdout+r.stderr+'\n');log.flush();check(r.returncode==0,'native_command_'+cmd[1],r.returncode)
        if (report_dir/'drc.json').exists():
            native=json.loads((report_dir/'drc.json').read_text());check(not native.get('violations') and not native.get('unconnected_items') and not native.get('schematic_parity'),'native_DRC_unconnected_parity_all_zero', {k:len(native.get(k,[])) for k in ['violations','unconnected_items','schematic_parity']})
        if (report_dir/'netlist.xml').exists():
            by_pin={}
            for net in ET.parse(report_dir/'netlist.xml').getroot().findall('./nets/net'):
                for node in net.findall('node'):by_pin[(node.attrib['ref'],node.attrib['pin'])]=net.attrib['name']
            check(all(unused(a.GetNetname()) or by_pin.get(k)==a.GetNetname() for k,a in pads.items()),'native_XML_matches_all_connected_pad_nets')
    else:check(False,'native_schematic_DRC_parity_gate_performed', 'Schematic missing or --skip-native requested')
    result={'engine':'Native KiCad parser/DRC/XML plus independent continuous-geometry bare-GND/power graph','board':board_file.name,'board_sha256':sha(board_file),'cad_zero_jumper_acceptance':not fails,'failed_checks':fails,'checks':checks,'facts':{'components':len(fps),'pads':len(pads),'tracks':len(tracks),'BMP280_address':bmp_address,'selected_3v3_carrier_pads':[single_3v3_pad] if single_3v3_pad else ['1','2'],'expected_GND_endpoint_count':expected_ground_count,'GND_copper_components':list(groups.values()),'power_copper_components':power_groups,'GPIO_assignment':assignment},'limits':['Nominal module dimensions/antenna projection; exact clone measurements remain unverified','No current-carrying capacity, solder/etch/drill quality, power integrity, I2C waveforms, ESC input thresholds or actual RF range verified','No module internal connections, jumper pads, net ties or 3D models are counted as carrier copper','BMP280 address option is a candidate hardware choice; its future driver configuration must match','A single duplicate DevKit 3.3V output may feed all sensor copper; the other header remains physically unconnected on this carrier','This is a scoped CAD acceptance test, not a proof of flight safety']}
    (report_dir/'report.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('project',type=Path);parser.add_argument('--report-dir',type=Path,required=True);parser.add_argument('--skip-native',action='store_true');parser.add_argument('--bmp-address',choices=['0x76','0x77'],default='0x76');parser.add_argument('--single-3v3-pad',choices=['1','2']);args=parser.parse_args()
    result=assess(args.project,not args.skip_native,args.report_dir,args.bmp_address,args.single_3v3_pad)
    print(json.dumps({'cad_zero_jumper_acceptance':result['cad_zero_jumper_acceptance'],'failed_checks':result['failed_checks'],'report':str(args.report_dir/'report.json')},indent=2))
    raise SystemExit(0 if result['cad_zero_jumper_acceptance'] else 1)
