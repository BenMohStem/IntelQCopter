from pathlib import Path
import html
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
t=Path(r'C:\Users\Mohamed\Desktop\Work\IntelQCopter-USTHB')
code="os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = str(Path(PyQt5.__file__).parent / 'Qt5/plugins/platforms')\nos.environ['QT_API'] = 'pyqt5'\nmatplotlib.use('QtAgg', force=True)\nimport matplotlib.pyplot as plt"
explanation="os is Python's operating-system module. .environ is a mapping of this process's environment variables; ['name'] chooses a key and = sets its string value for this process and its children, not permanent system settings. Path(...) constructs a filesystem path from PyQt5.__file__, the installed module filename; .parent selects its directory; / appends a relative path (Path overloads this operator, so here it is not numeric division). str converts the Path object to text. QT_API chooses PyQt5 as the Qt binding. matplotlib.use calls the backend selector; 'QtAgg' combines a Qt interactive window with Agg rendering. force=True requests selection explicitly; True is Python's Boolean literal. This must happen before pyplot creates figures. import loads a module; as plt gives pyplot a short local name. Without an installed interactive binding, Agg alone can render files but cannot show a window. This backend correction was the earlier plot-opening fix, independent of the angle mathematics."
block='<details><summary>Earlier window fix: choose an interactive Qt backend</summary><pre><code>'+html.escape(code)+'</code></pre><p>'+html.escape(explanation)+'</p></details>'
for p in [r/'docs/learning/gravity-angle-preview/report.html',t/'learning/gravity-angle-preview/report.html']:
    s=p.read_text(encoding='utf-8');s=s.replace('<h2>4 · Why',block+'<h2>4 · Why');p.write_text(s,encoding='utf-8')
(r/'docs/learning/README.md').write_text('''# Selected illustrated lessons

User preference: important completed engineering topics get a visual HTML note and reviewed publication to codex/usthb-university-report. Do not produce a teacher note for every minor edit.

Each note: concrete question/result, real graph where evidence exists, labelled model/assumptions, one intuitive diagram, essential equations with symbols defined, exact source/evidence identity, limitation and next milestone. Keep syntax behind expandable sections. Include a symbol-by-symbol explanation of new code when requested. Never present a display correction as an estimator fix or replay/model as physical acceptance.

Publish reviewed lesson files and a short index to learning/ in the teacher worktree. Preserve report PDFs, history and original evidence. Commit/push each branch independently; don't merge the whole development tree into the teacher branch. Validate local links/assets, figures, domain catalog and required teacher checks. No automatic full report rebuild for a supplementary HTML lesson.
''',encoding='utf-8')
# Local asset/link parity and encoding checks for the new publication only.
for rel in ['report.html','actual-trace.png','geometry.png','evidence.json']:
    assert (r/'docs/learning/gravity-angle-preview'/rel).read_bytes()==(t/'learning/gravity-angle-preview'/rel).read_bytes()
assert 'â€' not in (r/'docs/learning/gravity-angle-preview/report.html').read_text(encoding='utf-8')
print('PASS lesson asset parity, UTF-8 title and expandable explanations')
