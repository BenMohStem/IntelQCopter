"""Labelled KiCad carrier schematic from the same component/net specification.
External MCU/sensor modules are connectors, not models of their internal circuitry.
"""
from pathlib import Path
import json,uuid
D=Path(__file__).resolve().parent
def uid(s):return str(uuid.uuid5(uuid.NAMESPACE_URL,'intelqcopter/home-etch/'+s))
def q(s):return json.dumps(str(s))
data=json.loads((D/'components-and-nets.json').read_text(encoding='utf8'));root=uid('root')
out=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid "{root}") (paper "A2") (title_block (title "ESP32-S3 single-sided prototype carrier") (comment 1 "Module connectors only; physical arming disabled") (comment 2 "Lidar power open; external motor PDB; no LoRa/CC1101")) (lib_symbols']
for n in sorted({len(c['pads']) for c in data['components']}):
 out.append(f'(symbol "HomeEtch:Block_{n}" (pin_names (offset 0.5) hide) (in_bom yes) (on_board yes) (property "Reference" "J" (at 8 3 0) (effects (font (size 1.27 1.27)))) (property "Value" "Block{n}" (at 8 1 0) (effects (font (size 1 1)))) (symbol "Block_{n}_0_1" (rectangle (start 0 0) (end 18 {-2.54*(n+1)}) (stroke (width .254) (type default)) (fill (type background)))) (symbol "Block_{n}_1_1"')
 for i in range(1,n+1):out.append(f'(pin passive line (at -5.08 {-2.54*i:.2f} 0) (length 5.08) (name "{i}" (effects (font (size 1 1)))) (number "{i}" (effects (font (size 1 1)))) )')
 out.append('))')
out.append(')')
library='\n'.join(out[1:-1])
for n in sorted({len(c['pads']) for c in data['components']}):library=library.replace('"HomeEtch:Block_'+str(n)+'"','"Block_'+str(n)+'"')
(D/'HomeEtch.kicad_sym').write_text('(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor")\n'+library+'\n)',encoding='utf8')
(D/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "HomeEtch") (type "KiCad") (uri "${KIPRJMOD}/HomeEtch.kicad_sym") (options "") (descr "Project-local passive carrier connector blocks")))',encoding='utf8')
for j,c in enumerate(data['components']):
 ref=c['ref'];n=len(c['pads']);x,y=(70+170*j,22) if j<2 else (45+80*((j-2)%7),115+55*((j-2)//7));x=round(x/1.27)*1.27;y=round(y/1.27)*1.27;symbol=uid(ref)
 out.append(f'(symbol (lib_id "HomeEtch:Block_{n}") (at {x} {y} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid "{symbol}") (property "Reference" {q(ref)} (at {x+9} {y-5} 0) (effects (font (size 1.27 1.27)))) (property "Value" {q(c["value"])} (at {x+9} {y-2.5} 0) (effects (font (size 1 1))))')
 out.append(f'(property "Footprint" "HomeEtch:{ref}" (at {x} {y} 0) (effects (font (size 1 1)) hide))')
 for a in c['pads']:out.append(f'(pin {q(a["number"])} (uuid "{uid(ref+"pin"+a["number"])}"))')
 out.append(f'(instances (project "esp32_home_etch" (path "/{root}" (reference {q(ref)}) (unit 1)))))')
 for i,a in enumerate(c['pads'],1):
  px=x-5.08;py=y+2.54*i
  if a['net']:
   out.append(f'(wire (pts (xy {px:.2f} {py:.2f}) (xy {px-7.62:.2f} {py:.2f})) (stroke (width 0) (type default)) (uuid "{uid(ref+"wire"+str(i))}"))')
   out.append(f'(global_label {q(a["net"])} (shape bidirectional) (at {px-7.62:.2f} {py:.2f} 0) (effects (font (size 1 1)) (justify left bottom)) (uuid "{uid(ref+"label"+str(i))}"))')
  else:out.append(f'(no_connect (at {px:.2f} {py:.2f}) (uuid "{uid(ref+"NC"+str(i))}"))')
out.append('(embedded_fonts no))');(D/'esp32_home_etch.kicad_sch').write_text('\n'.join(out),encoding='utf8')
print('Saved labelled KiCad schematic for',len(data['components']),'carrier components')
