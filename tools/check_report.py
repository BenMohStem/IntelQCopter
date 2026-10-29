"""Read-only document/model integrity checks, not flight acceptance."""
from pathlib import Path
import hashlib
import json
import re
import fitz

ROOT=Path(__file__).resolve().parents[1]
config=json.loads((ROOT/'report/editions.json').read_text())
expected_url='https://github.com/BenMohStem/IntelQCopter/tree/codex/usthb-university-report'
for name in ['short','full']:
    record=config[name];folder=ROOT/record['folder'];source=(folder/'report.md').read_text(encoding='utf-8')
    pdf=fitz.open(ROOT/record['pdf'])
    assert len(pdf)==record['expected_pages'], name+' page count'
    if name=='short':
        assert len(pdf)<=record['page_budget_maximum']
        assert '[[VIS:bode-example]]' not in source and '[[VIS:nichols-example]]' not in source
    else:
        assert '[[VIS:bode-example]]' in source and '[[VIS:nichols-example]]' in source
    sections=json.loads((folder/'section-pages.json').read_text())
    assert list(sections.values())==list(range(1,len(pdf)+1)), name+' section overflow'
    assert 'RSI7.8' in pdf[0].get_text() and '27 OCTOBER 2026' in pdf[0].get_text()
    for p in pdf:
        assert '\ufffd' not in p.get_text(), name+' missing glyph'
        for b in p.get_text('blocks'):
            assert 0<=b[0]<=b[2]<=p.rect.width+1 and 0<=b[1]<=b[3]<=p.rect.height+1
    assert any(l.get('uri')==expected_url for p in pdf for l in p.get_links())
    figures=json.loads((folder/'assets/visual/figure-manifest.json').read_text())
    for figure in re.findall(r'\[\[VIS:([^]]+)\]\]',source):
        entry=figures[figure]
        assert hashlib.sha256((folder/entry['file']).read_bytes()).hexdigest()==entry['sha256']
    for case in figures['nyquist-example']['coverage_checks']:
        assert case['every_point_inside_overview'] and case['sample_count']==42000
        assert abs(case['clockwise_turns']-case['expected_from_roots'])<1e-6
    assert len(figures['nyquist-example']['coverage_checks'])==2
    for entry in json.loads((folder/'evidence-manifest.json').read_text()):
        assert hashlib.sha256((ROOT/entry['path']).read_bytes()).hexdigest()==entry['sha256']
    original=ROOT/'evidence/mpu6050/2026-10-26-offline-attitude-preparation/reserved-replay.png'
    assert original.read_bytes()==(folder/'assets/reserved-replay.png').read_bytes()
    print('PASS:',name,len(pdf),'pages; sections, bounds/glyphs, figure/evidence hashes, replay and links')
archive=config['archived_full']
assert len(fitz.open(ROOT/archive['pdf']))==archive['expected_pages']
for entry in json.loads((ROOT/'reference/manifest.json').read_text())['artifacts']:
    assert hashlib.sha256((ROOT/entry['path']).read_bytes()).hexdigest()==entry['sha256']
model=json.loads((ROOT/'analysis/control-design-example.json').read_text())
assert model['source_sha256']==hashlib.sha256((ROOT/'tools/control_design_example.py').read_bytes()).hexdigest()
assert model['status']=='ILLUSTRATIVE_ANALYSIS_ONLY'
for case in model['cases']:
    assert case['clockwise_encirclements_N']==case['closed_RHP_poles_Z']-case['open_RHP_poles_P']
    assert case['maximum_normalized_root_residual']<1e-12
assert [c['closed_RHP_poles_Z'] for c in model['cases']]==[0,2]
assert len(json.loads((ROOT/'report/requirements-register.json').read_text()))==50
print('PASS: original38-page reference unchanged; model source/winding/root checks;50requirements')
print('Visual review and technical limitations remain separately recorded in edition validation files.')
