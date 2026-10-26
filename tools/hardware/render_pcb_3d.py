"""Render the actual portable KiCad project, preserving native output as evidence."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project', type=Path, default=ROOT / 'hardware/home-etched-v3/lidar-gap-two-ground-wires')
args = parser.parse_args()
project = args.project.resolve()
output = project / 'output'
output.mkdir(exist_ok=True)
cli = Path(os.environ['LOCALAPPDATA']) / 'Programs/KiCad/10.0/bin/kicad-cli.exe'
report = {'engine': 'Native KiCad pcb render, basic ray tracing',
          'board_sha256': hashlib.sha256((project / 'QCopter.kicad_pcb').read_bytes()).hexdigest(),
          'models_are_illustrative': True, 'renders': []}
with (output / '3d-native-render.log').open('w', encoding='utf-8') as log:
    version = subprocess.run([str(cli), 'version'], capture_output=True, text=True, check=True)
    report['kicad_version'] = version.stdout.strip()
    log.write(version.stdout)
    for side, rotation in [('top', '-35,0,25'), ('bottom', '25,0,20')]:
        destination = output / ('3d-' + side + '.png')
        command = [str(cli), 'pcb', 'render', '--output', str(destination),
                   '--width', '1600', '--height', '1100', '--side', side,
                   '--background', 'opaque', '--quality', 'basic',
                   '--rotate', rotation, '--zoom', '0.9', str(project / 'QCopter.kicad_pcb')]
        result = subprocess.run(command, capture_output=True, text=True)
        # Use portable relative arguments in the recorded command; preserve native output verbatim.
        log.write('$ kicad-cli pcb render: ' + side + '\n' + result.stdout + result.stderr + '\n')
        log.flush()
        if result.returncode:
            raise RuntimeError('Native render failed; see ' + str(log.name))
        report['renders'].append({'file': destination.name, 'side': side, 'rotation': rotation,
                                  'width': 1600, 'height': 1100, 'zoom': .9,
                                  'sha256': hashlib.sha256(destination.read_bytes()).hexdigest()})
        print('Native KiCad render:', destination)
(output / '3d-render-verification.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
