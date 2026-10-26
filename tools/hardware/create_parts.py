"""Create an unwired, unrouted KiCad module palette for the user to design."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import uuid

import pcbnew

LIB = 'QCopterModules'


def uid():
    return str(uuid.uuid4())


def q(value):
    return json.dumps(str(value))


def prop(name, value, x, y, hidden=False):
    return f'(property {q(name)} {q(value)} (at {x} {y} 0) (effects (font (size 1.27 1.27)) {"hide" if hidden else ""}))'


def symbol(name, labels):
    two_sided = len(labels) == 44
    left = list(labels.items())[:22] if two_sided else list(labels.items())
    right = list(labels.items())[22:] if two_sided else []
    top = 27.94 if two_sided else (len(left)-1)*1.27
    half_width = 15.24
    height = top + 2.54
    body = [f'(symbol {q(name)} (pin_names (offset 1.016)) (in_bom yes) (on_board yes)',
            prop('Reference', 'U' if not name.startswith(('ESC','Lidar')) else 'J', 0, height+5.08),
            prop('Value', name, 0, height+2.54),
            prop('Footprint', LIB+':'+name, 0, 0, True),
            f'(symbol {q(name+"_0_1")} (rectangle (start {-half_width} {height}) (end {half_width} {-height}) (stroke (width 0.254) (type default)) (fill (type background))))',
            f'(symbol {q(name+"_1_1")}']
    for group, sign in [(left, -1), (right, 1)]:
        for i, (number, label) in enumerate(group):
            y = top - i*2.54
            body.append(f'(pin passive line (at {sign*(half_width+5.08)} {y} {0 if sign<0 else 180}) (length 5.08) (name {q(label)} (effects (font (size 1.016 1.016)))) (number {q(number)} (effects (font (size 1.016 1.016)))))')
    return '\n'.join(body) + '))'


def footprint(name, labels, positions):
    xs = [p[0] for p in positions.values()]
    ys = [p[1] for p in positions.values()]
    left, right, top, bottom = min(xs)-1.5, max(xs)+1.5, min(ys)-1.5, max(ys)+1.5
    result = [f'(footprint {q(name)} (version 20240108) (generator "pcbnew") (layer "F.Cu")',
              '(attr through_hole)',
              f'(descr {q("Module/header interface; 2.54 mm pitch. Header courtyard only; module body clearance is user-defined.")})',
              f'(property "Reference" "REF**" (at {(left+right)/2} {top-2} 0) (layer "F.SilkS") (effects (font (size 1.0 1.0) (thickness 0.15))))',
              f'(property "Value" {q(name)} (at {(left+right)/2} {bottom+2} 0) (layer "F.Fab") (effects (font (size 1.0 1.0) (thickness 0.15))))']
    # Courtyard is the header interface, not a guessed sensor-module body.
    for layer in ['F.CrtYd', 'F.SilkS']:
        result.append(f'(fp_rect (start {left} {top}) (end {right} {bottom}) (stroke (width {0.05 if layer=="F.CrtYd" else 0.12}) (type default)) (fill none) (layer {q(layer)}))')
    for number, (x, y) in positions.items():
        diameter = 2.0 if name != 'Lidar_Wires_1x04' else 3.2
        drill = 1.1 if name != 'Lidar_Wires_1x04' else 1.2
        shape = 'rect' if number == '1' else 'circle'
        result.append(f'(pad {q(number)} thru_hole {shape} (at {x} {y}) (size {diameter} {diameter}) (drill {drill}) (layers "*.Cu" "*.Mask"))')
        if len(labels) == 44:
            tx = x + (2.0 if x == 0 else -2.0)
            justification = 'left' if x == 0 else 'right'
        else:
            tx = x + 2.2
            justification = 'left'
        result.append(f'(fp_text user {q(number+":"+labels[number])} (at {tx} {y}) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12)) (justify {justification})))')
    return '\n'.join(result)+')\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New empty staging directory, never overwrite a design')
    parser.add_argument('--reference', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Use a new staging directory to preserve existing designs')
    args.output.mkdir(parents=True)
    pretty = args.output/(LIB+'.pretty')
    pretty.mkdir()
    left = ['3V3','3V3','EN','GPIO4','GPIO5','GPIO6','GPIO7','GPIO15','GPIO16','GPIO17','GPIO18','GPIO8','GPIO3','GPIO46','GPIO9','GPIO10','GPIO11','GPIO12','GPIO13','GPIO14','5V','GND']
    right = ['GND','GPIO43_TX','GPIO44_RX','GPIO1','GPIO2','GPIO42','GPIO41','GPIO40','GPIO39','GPIO38','GPIO37','GPIO36','GPIO35','GPIO0','GPIO45','GPIO48','GPIO47','GPIO21','GPIO20','GPIO19','GND','GND']
    esp = {str(i+1): label for i,label in enumerate(left)}
    esp.update({str(44-i):label for i,label in enumerate(right)})
    definitions = {
        'ESP32_S3_DevKitC_1_44pin':esp,
        'MPU6050_GY521_1x08':dict(zip(map(str,range(1,9)),['VCC','GND','SCL','SDA','XDA','XCL','AD0','INT'])),
        'BMP280_GY_1x06':dict(zip(map(str,range(1,7)),['VCC','GND','SCL','SDA','CSB','SDO'])),
        'Magnetometer_GY271_1x05':dict(zip(map(str,range(1,6)),['VCC','GND','SCL','SDA','DRDY'])),
        'Lidar_Wires_1x04':dict(zip(map(str,range(1,5)),['VCC_5V','GND','TX_FROM_LIDAR','RX_TO_LIDAR'])),
        'ESC_Servo_1x03':dict(zip(map(str,range(1,4)),['GND','BEC_5V_OPTIONAL','SIGNAL'])),
    }
    positions = {}
    for name,labels in definitions.items():
        if len(labels)==44:
            pads={str(i+1):(0,i*2.54) for i in range(22)}
            pads.update({str(44-i):(22.86,i*2.54) for i in range(22)})
        else:
            pads={number:(0,(int(number)-1)*(5.08 if name=='Lidar_Wires_1x04' else 2.54)) for number in labels}
        positions[name]=pads
        (pretty/(name+'.kicad_mod')).write_text(footprint(name,labels,pads),encoding='utf-8')
    bodies=Path(__file__).resolve().parents[2]/'hardware/module-library/module-body-dimensions.json'
    if bodies.exists():
        from enrich_module_bodies import enrich_library
        enrich_library(args.output,bodies)
        shutil.copyfile(bodies,args.output/'module-body-dimensions.json')
    reference=args.reference.read_text(encoding='utf-8')
    matched=re.findall(r'\(pad "(\d+)" thru_hole \w+ \(at ([\d.\-]+) ([\d.\-]+)',reference)
    assert len(matched)==44
    maximum_error=max(max(abs(float(x)-positions['ESP32_S3_DevKitC_1_44pin'][number][0]),abs(float(y)-positions['ESP32_S3_DevKitC_1_44pin'][number][1])) for number,x,y in matched)
    assert maximum_error<0.01
    library='(kicad_symbol_lib (version 20231120) (generator "kicad_symbol_editor")\n'+'\n'.join(symbol(name,labels) for name,labels in definitions.items())+')\n'
    (args.output/(LIB+'.kicad_sym')).write_text(library,encoding='utf-8')
    for filename,kind in [('sym-lib-table','sym_lib_table'),('fp-lib-table','fp_lib_table')]:
        target=LIB+'.kicad_sym' if filename.startswith('sym') else LIB+'.pretty'
        (args.output/filename).write_text(f'({kind} (version 7) (lib (name "{LIB}") (type "KiCad") (uri "${{KIPRJMOD}}/{target}") (options "") (descr "Project module interfaces and wire headers")))\n',encoding='utf-8')
    components=[
        ('U1','ESP32_S3_DevKitC_1_44pin',(75,80),(70,65)),
        ('U2','MPU6050_GY521_1x08',(145,45),(115,65)),
        ('U3','BMP280_GY_1x06',(220,45),(145,65)),
        ('U4','Magnetometer_GY271_1x05',(145,95),(175,65)),
        ('J1','Lidar_Wires_1x04',(220,95),(115,110)),
        ('J2','ESC_Servo_1x03',(60,160),(145,110)),
        ('J3','ESC_Servo_1x03',(120,160),(175,110)),
        ('J4','ESC_Servo_1x03',(180,160),(145,135)),
        ('J5','ESC_Servo_1x03',(240,160),(175,135)),
    ]
    sheet_uuid=uid()
    schematic=['(kicad_sch (version 20250114) (generator "eeschema")',f'(uuid {sheet_uuid}) (paper "A4")',
               '(lib_symbols '+ '\n'.join(symbol(name,labels).replace('(symbol '+q(name), '(symbol '+q(LIB+':'+name),1) for name,labels in definitions.items())+')']
    board=pcbnew.BOARD()
    manifest=[]
    for ref,name,(sx,sy),(px,py) in components:
        instance=uid()
        height=(27.94 if len(definitions[name])==44 else (len(definitions[name])-1)*1.27)+2.54
        schematic.extend([f'(symbol (lib_id {q(LIB+":"+name)}) (at {sx} {sy} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {instance})',
                          prop('Reference',ref,sx,sy-height-5.08),prop('Value',name,sx,sy-height-2.54),
                          prop('Footprint',LIB+':'+name,sx,sy,True),
                          f'(instances (project "QCopter" (path "/{sheet_uuid}" (reference {q(ref)}) (unit 1)))))'])
        fp=pcbnew.FootprintLoad(str(pretty),name)
        if fp is None:
            raise ValueError('KiCad rejected footprint '+name)
        fp.SetReference(ref)
        fp.SetValue(name)
        fp.SetFPID(pcbnew.LIB_ID(LIB,name))
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(px),pcbnew.FromMM(py)))
        fp.SetPath(pcbnew.KIID_PATH('/'+sheet_uuid+'/'+instance))
        board.Add(fp)
        manifest.append({'reference':ref,'library_id':LIB+':'+name,'pins':definitions[name],
                         'pad_positions_mm':positions[name],'schematic_uuid':instance})
    schematic.append('(text "MODULE PALETTE - connect the schematic, then update PCB (F8). Unwired/unrouted parts only." (at 20 20 0) (effects (font (size 1.27 1.27)) (justify left)) (uuid '+uid()+'))')
    schematic.append('(embedded_fonts no))')
    (args.output/'QCopter.kicad_sch').write_text('\n'.join(schematic)+'\n',encoding='utf-8')
    pcbnew.SaveBoard(str(args.output/'QCopter.kicad_pcb'),board)
    (args.output/'QCopter.kicad_pro').write_text('{}\n',encoding='utf-8')
    reloaded=pcbnew.LoadBoard(str(args.output/'QCopter.kicad_pcb'))
    assert len(list(reloaded.GetFootprints()))==9
    assert sum(len(list(fp.Pads())) for fp in reloaded.GetFootprints())==79
    assert len(list(reloaded.GetTracks()))==0
    assert len(list(reloaded.Zones()))==0
    for fp in reloaded.GetFootprints():
        assert fp.GetPath().AsString()=='/'+sheet_uuid+'/'+next(c['schematic_uuid'] for c in manifest if c['reference']==fp.GetReference())
    (args.output/'parts.json').write_text(json.dumps({'status':'Unwired palette for user PCB design; no fabricated fit or electrical acceptance',
        'engine':'KiCad '+pcbnew.Version()+' native footprint/board parser',
        'esp_reference_url':'https://raw.githubusercontent.com/espressif/kicad-libraries/main/footprints/Espressif.pretty/ESP32-S3-DevKitC.kicad_mod',
        'esp_reference_sha256':hashlib.sha256(args.reference.read_bytes()).hexdigest(),
        'esp_pad_geometry_max_difference_mm':maximum_error,'footprints':9,'pads':79,'tracks':0,'zones':0,
        'sensor_body_geometry':'Sourced nominal reference rectangles; exact clone and header offsets unverified' if bodies.exists() else 'Header interface only',
        'components':manifest},indent=2)+'\n',encoding='utf-8')
    print('Created and natively reloaded: 9 footprints / 79 pads; ESP44 pad positions within',maximum_error,'mm of manufacturer reference.')


if __name__=='__main__':
    main()
