"""Exercise failure containment and provenance checks on an isolated ledger copy."""
from contextlib import redirect_stdout, closing
import copy
import io
import json
from pathlib import Path
import tempfile
import knowledge_db as k

baseline=json.loads(k.LEDGER.read_text(encoding='utf8'))
parent=k.ROOT/'build/knowledge-checks'; parent.mkdir(parents=True,exist_ok=True)
passed=0
with tempfile.TemporaryDirectory(dir=parent) as folder:
    folder=Path(folder);k.LEDGER=folder/'ledger.json';k.CACHE=folder/'cache.sqlite'
    def write(data): k.LEDGER.write_text(json.dumps(data),encoding='utf8')
    def reject(label, data):
        global passed
        write(data);before=k.digest(k.CACHE)
        try: k.build()
        except ValueError: pass
        else: raise AssertionError(label+' unexpectedly accepted')
        assert k.digest(k.CACHE)==before,label+' changed cache'
        passed+=1;print('PASS',label,'old cache retained')
    write(baseline)
    with redirect_stdout(io.StringIO()): k.build()
    with closing(k.connect()) as db:
        assert db.execute('PRAGMA integrity_check').fetchone()[0]=='ok'
        assert db.execute("SELECT count(*) FROM search WHERE search MATCH ?",('"calibration"',)).fetchone()[0]>0
    passed+=1;print('PASS valid build and full-text lookup')
    bad=copy.deepcopy(baseline);bad['facts'].append(copy.deepcopy(bad['facts'][0]));reject('duplicate fact',bad)
    bad=copy.deepcopy(baseline);bad['facts'][0]['revisions'][0]['sources']=['absent-source'];reject('dangling evidence',bad)
    bad=copy.deepcopy(baseline);bad['relations'] += [dict(zip(('from','type','to'),x)) for x in [('calibration','depends_on','nvs')]];reject('dependency cycle',bad)
    bad=copy.deepcopy(baseline);bad['facts'][0]['revisions'][0]['statement']+=' edited';reject('immutable previous revision',bad)
    bad=copy.deepcopy(baseline);bad['sources'].append({'id':'escape','kind':'file','locator':'../../../../outside.txt'});reject('path escape',bad)
    bad=copy.deepcopy(baseline);bad['sources'].append({'id':'bad-hash','kind':'file','locator':'README.md','sha256':'0'*64});reject('incorrect pinned hash',bad)
    write(baseline);before=k.digest(k.CACHE);lock=k.CACHE.with_suffix('.lock');lock.write_text('owned by fixture')
    try:
        try: k.build()
        except ValueError: pass
        else: raise AssertionError('competing build accepted')
        assert lock.exists() and k.digest(k.CACHE)==before
        passed+=1;print('PASS competing builder preserves lock and cache')
    finally: lock.unlink()
    bad=copy.deepcopy(baseline);bad['updated']='2026-10-22';write(bad)
    try: k.connect()
    except ValueError: passed+=1;print('PASS stale ledger query rejected')
    else: raise AssertionError('stale cache queried')
print('Knowledge checks passed:',passed)
