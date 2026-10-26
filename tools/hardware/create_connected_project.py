"""Connected schematic and unrouted net-assigned PCB; no physical validation."""
import argparse
import json
from pathlib import Path
import re
import shutil
import sys
import uuid
import pcbnew

ROOT=Path(__file__).resolve().parents[2]
if not (ROOT/'hardware/module-library').exists():
    raise SystemExit('Run this generator from the IntelQCopter checkout')
sys.path.insert(0,str(ROOT/'tools/hardware'))
from create_parts import symbol,footprint,prop


def uid():
    return str(uuid.uuid4())


def make(output):
    if output.exists():
        raise ValueError('Use a new directory; existing designs are never replaced')
    output.mkdir(parents=True)
    palette=ROOT/'hardware/module-library'
    library=output/'QCopterModules.pretty'
    shutil.copytree(palette/'QCopterModules.pretty',library)
    lidar=library/'Lidar_Wires_1x04.kicad_mod'
    lidar.write_text(re.sub(r'\(start -1\.5 -1\.5\)\s+\(end 1\.5 16\.74[0-9]*\)',
                     '(start -2.1 -2.1) (end 2.1 17.34)',lidar.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
    for name in ['fp-lib-table','sym-lib-table','module-body-dimensions.json','module-dimensions.md','module-orientation.png','module-orientation.svg']:
        shutil.copyfile(palette/name,output/name)
    previous=json.loads((palette/'parts.json').read_text(encoding='utf-8'))['components']
    definitions={c['library_id'].split(':')[1]:c['pins'] for c in previous}
    definitions.update({
        'BEC_Link_1x02':{'1':'ESC_BEC_IN','2':'BOARD_5V_OUT'},
        'MPU_Auxiliary_1x04':{'1':'3V3','2':'GND','3':'AUX_SCL','4':'AUX_SDA'},
        'Companion_UART_1x03':{'1':'GND','2':'TX_FROM_FC','3':'RX_TO_FC'},
        'Optional_Pullup_Axial_P10_16':{'1':'3V3','2':'BUS_SIGNAL'},
    })
    for name in ['BEC_Link_1x02','MPU_Auxiliary_1x04','Companion_UART_1x03','Optional_Pullup_Axial_P10_16']:
        positions={n:([10.16*(int(n)-1),0] if name.startswith('Optional') else [0,2.54*(int(n)-1)]) for n in definitions[name]}
        text=footprint(name,definitions[name],positions)
        if name.startswith('Optional'):
            text=text.replace('Module/header interface; 2.54 mm pitch. Header courtyard only; module body clearance is user-defined.','Optional axial resistor, 10.16 mm lead spacing; DNP until bus pullups assessed.')
        (library/(name+'.kicad_mod')).write_text(text,encoding='utf-8',newline='\n')
    names={c['reference']:c['library_id'].split(':')[1] for c in previous}
    components=[]
    def add(ref,name,xy,board_xy,nets,value=None,dnp=False):
        assert set(nets)<=set(definitions[name]),ref
        components.append({'reference':ref,'name':name,'symbol_xy':xy,'board_xy':board_xy,'pin_nets':nets,
                           'value':value or name,'dnp':dnp,'uuid':uid()})
    esp={str(i):'GND' for i in [22,23,24,44]}
    esp.update({'1':'3V3','2':'3V3','21':'FC_5V','12':'I2C_SCL','15':'I2C_SDA',
                '10':'MAG_DRDY','9':'MPU_INT','4':'FC_UART_RX','5':'FC_UART_TX',
                '6':'LIDAR_TO_FC_RX','7':'FC_TX_TO_LIDAR','18':'ESC1_SIGNAL',
                '41':'ESC2_SIGNAL','40':'ESC3_SIGNAL','20':'ESC4_SIGNAL'})
    # U1 pad numbers above follow the manufacturer DevKit numbering, not GPIO numbers.
    add('U1',names['U1'],[85,92],[70,65],esp)
    add('U2',names['U2'],[200,55],[115,65],{'1':'3V3','2':'GND','3':'I2C_SCL','4':'I2C_SDA','5':'MPU_AUX_SDA','6':'MPU_AUX_SCL','7':'GND','8':'MPU_INT'})
    add('U3',names['U3'],[310,55],[145,65],{'1':'3V3','2':'GND','3':'I2C_SCL','4':'I2C_SDA','5':'3V3','6':'GND'})
    add('U4',names['U4'],[200,112],[175,65],{'1':'3V3','2':'GND','3':'I2C_SCL','4':'I2C_SDA','5':'MAG_DRDY'})
    add('J1',names['J1'],[310,112],[115,110],{'1':'FC_5V','2':'GND','3':'LIDAR_TO_FC_RX','4':'FC_TX_TO_LIDAR'})
    for index,ref in enumerate(['J2','J3','J4','J5'],1):
        add(ref,names[ref],[60+90*(index-1),175],[145+30*((index-1)%2),110+25*((index-1)//2)],
            {'1':'GND','2':f'ESC{index}_BEC','3':f'ESC{index}_SIGNAL'},f'ESC{index}_CONTROL')
        add('JP'+str(index),'BEC_Link_1x02',[60+90*(index-1),222],[145+30*((index-1)%2),157+15*((index-1)//2)],
            {'1':f'ESC{index}_BEC','2':'FC_5V'},'BEC_LINK_FIT_ONE_ONLY')
    add('J6','MPU_Auxiliary_1x04',[200,265],[115,157],{'1':'3V3','2':'GND','3':'MPU_AUX_SCL','4':'MPU_AUX_SDA'})
    add('J7','Companion_UART_1x03',[85,265],[70,145],{'1':'GND','2':'FC_UART_TX','3':'FC_UART_RX'})
    add('R1','Optional_Pullup_Axial_P10_16',[310,145],[70,170],{'1':'3V3','2':'I2C_SDA'},'4k7_DNP',True)
    add('R2','Optional_Pullup_Axial_P10_16',[370,145],[90,170],{'1':'3V3','2':'I2C_SCL'},'4k7_DNP',True)
    assert len(components)==17
    # Validate GPIO allocations independently of schematic-header numbering.
    gpio={'i2c_sda':9,'i2c_scl':8,'mpu_interrupt':16,'mag_drdy':17,'lidar_rx':6,'lidar_tx':7,
          'companion_rx':4,'companion_tx':5,'esc1':12,'esc2':1,'esc3':2,'esc4':14}
    assert len(set(gpio.values()))==len(gpio)
    all_definitions='\n'.join(symbol(name,pins) for name,pins in definitions.items())
    (output/'QCopterModules.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator "kicad_symbol_editor")\n'+all_definitions+')\n',encoding='utf-8')
    cached='\n'.join(symbol(name,pins).replace('(symbol '+json.dumps(name),'(symbol '+json.dumps('QCopterModules:'+name),1) for name,pins in definitions.items())
    sheet=uid()
    sch=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet}) (paper "A3") (lib_symbols {cached})']
    board=pcbnew.BOARD()
    net_names=sorted({name for c in components for name in c['pin_nets'].values()})
    nets={}
    for name in net_names:
        item=pcbnew.NETINFO_ITEM(board,name)
        board.Add(item)
        nets[name]=item
    for c in components:
        ref,name=c['reference'],c['name']
        sx,sy=c['symbol_xy']
        pin_names=definitions[name]
        top=27.94 if len(pin_names)==44 else (len(pin_names)-1)*1.27
        height=top+2.54
        sch.extend([f'(symbol (lib_id "QCopterModules:{name}") (at {sx} {sy} 0) (unit 1) (in_bom yes) (on_board yes) (dnp {"yes" if c["dnp"] else "no"}) (uuid {c["uuid"]})',
            prop('Reference',ref,sx,sy-height-5.08),prop('Value',c['value'],sx,sy-height-2.54),
            prop('Footprint','QCopterModules:'+name,sx,sy,True),
            f'(instances (project "QCopter" (path "/{sheet}" (reference "{ref}") (unit 1)))))'])
        for index,pin in enumerate(pin_names):
            side=1 if len(pin_names)==44 and index>=22 else -1
            row=index-22 if side==1 else index
            x=sx+side*20.32
            y=sy-top+row*2.54
            if pin not in c['pin_nets']:
                sch.append(f'(no_connect (at {x:g} {y:g}) (uuid {uid()}))')
                continue
            endpoint=x+side*5.08
            sch.append(f'(wire (pts (xy {x:g} {y:g}) (xy {endpoint:g} {y:g})) (stroke (width 0) (type default)) (uuid {uid()}))')
            sch.append(f'(global_label "{c["pin_nets"][pin]}" (shape bidirectional) (at {endpoint:g} {y:g} 0) (effects (font (size 1.016 1.016)) (justify {"left" if side==1 else "right"})) (uuid {uid()}))')
        fp=pcbnew.FootprintLoad(str(library),name)
        fp.SetReference(ref)
        fp.SetValue(c['value'])
        fp.SetFPID(pcbnew.LIB_ID('QCopterModules',name))
        if c['dnp']:
            fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_DNP)
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(c['board_xy'][0]),pcbnew.FromMM(c['board_xy'][1])))
        fp.SetPath(pcbnew.KIID_PATH('/'+sheet+'/'+c['uuid']))
        for pad in fp.Pads():
            if pad.GetNumber() in c['pin_nets']:
                pad.SetNet(nets[c['pin_nets'][pad.GetNumber()]])
        board.Add(fp)
    notes=['CONNECTED I2C DESIGN - labels with matching names connect electrically. PCB has airwires; routing remains to be designed.',
           'Fit at most ONE verified 5 V BEC link JP1-JP4. Remove all links before USB powering. Motor current stays on separate power distribution.',
           'R1/R2 optional I2C pullups: DNP initially; assess existing module pullups. Companion UART is 3.3 V logic, separate computer supply.']
    for index,note in enumerate(notes):
        sch.append(f'(text {json.dumps(note)} (at 15 {15+5*index} 0) (effects (font (size 1.27 1.27)) (justify left)) (uuid {uid()}))')
    sch.append('(embedded_fonts no))')
    (output/'QCopter.kicad_sch').write_text('\n'.join(sch)+'\n',encoding='utf-8',newline='\n')
    (output/'QCopter.kicad_pro').write_text('{}\n',encoding='utf-8')
    pcbnew.SaveBoard(str(output/'QCopter.kicad_pcb'),board)
    assert sum(len(list(fp.Pads())) for fp in board.GetFootprints())==98
    manifest={'schema':1,'engine':'Native KiCad schematic/netlist and net-assigned PCB generation',
              'status':'Connected reference design; PCB unrouted; no physical or firmware interface acceptance',
              'bus_choice':'I2C for MPU6050/BMP280/GY-271; UART for lidar/companion',
              'gpio':gpio,'components':components,'expected_components':17,'expected_pads':98,
              'all_bec_links_initially_open':True,'physical_motor_outputs_enabled':False}
    (output/'connections.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8',newline='\n')
    print('Created 17 connected schematic components and 98 net-assigned PCB pads; no copper routing.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    make(parser.parse_args().output)
