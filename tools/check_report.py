"""Read-only document checks; does not certify hardware or scientific validity."""
from pathlib import Path
import hashlib
import json
import re
import sys
import fitz

ROOT = Path(__file__).resolve().parents[1]
report = ROOT / 'report'
source = (report / 'report.md').read_text(encoding='utf-8')
pdf = fitz.open(ROOT / 'IntelQCopter.pdf')
assert len(pdf) == 20, 'Report must contain exactly 20 pages'
sections = json.loads((report / 'section-pages.json').read_text())
assert list(sections.values()) == list(range(1, 21)), 'Each section must fit its page'
assert 'RSI7.8' in pdf[0].get_text()
assert '27 OCTOBER 2026' in pdf[0].get_text()
for forbidden in ['esp32-xbox', 'codex/stepwise', 'A_d=', 'e^{A']:
    assert forbidden not in source, forbidden
for page in pdf:
    assert '\ufffd' not in page.get_text(), 'Missing text glyph'
    for block in page.get_text('blocks'):
        assert 0 <= block[0] <= block[2] <= page.rect.width + 1
        assert 0 <= block[1] <= block[3] <= page.rect.height + 1
for item in json.loads((report / 'evidence-manifest.json').read_text()):
    data = (ROOT / item['path']).read_bytes()
    assert hashlib.sha256(data).hexdigest() == item['sha256']
original = ROOT / 'evidence/mpu6050/2026-10-26-offline-attitude-preparation/reserved-replay.png'
assert original.read_bytes() == (report / 'assets/reserved-replay.png').read_bytes()
figures = json.loads((report / 'assets/visual/figure-manifest.json').read_text())
for name in re.findall(r'\[\[VIS:([^]]+)\]\]', source):
    figure = figures[name]
    assert hashlib.sha256((report / figure['file']).read_bytes()).hexdigest() == figure['sha256']
expected = 'https://github.com/BenMohStem/IntelQCopter/tree/codex/usthb-university-report'
assert any(link.get('uri') == expected for page in pdf for link in page.get_links())
assert len(json.loads((report / 'requirements-register.json').read_text())) == 50
print('PASS: 20 pages; section pagination; bounds/glyphs; source and figure hashes; original replay; repository link; 50 requirements')
print('Visual review and technical limitations remain separately recorded in report/validation.json.')
