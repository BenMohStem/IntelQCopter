"""Versioned factual ledger and disposable SQLite FTS5 cache. Standard library only."""
import argparse
from collections import deque
from contextlib import closing
from datetime import date
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import tempfile

ROOT = Path(__file__).resolve().parents[1]
LEDGER = ROOT/'knowledge/project.json'
CACHE = ROOT/'build/knowledge/project.sqlite'
STATUSES = {'confirmed','implemented','assumed','measured','historical','pending'}
LEVELS = {'requirement','source','build','emulated','simulated','physical','none'}

def require(ok, message):
    if not ok: raise ValueError(message)

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1048576), b''): h.update(chunk)
    return h.hexdigest()

def local_path(value):
    require(isinstance(value,str) and value, 'Missing evidence path')
    path=(ROOT/value).resolve()
    require(path.is_relative_to(ROOT), f'Evidence escapes checkout: {value}')
    return path

def identifier(value):
    return isinstance(value,str) and re.fullmatch(r'[a-z][a-z0-9_-]{0,95}',value)

def load():
    data=json.loads(LEDGER.read_text(encoding='utf8'))
    require(isinstance(data,dict),'Ledger must be an object')
    require(data['schema']==2 and data['branch']=='esp32-xbox','Wrong ledger schema/scope')
    date.fromisoformat(data['updated'])
    require(all(isinstance(data[k],list) for k in ('sources','facts','relations')),'sources, facts and relations must be arrays')
    sources,facts={},{}
    for s in data['sources']:
        require(isinstance(s,dict),'Source must be an object')
        sid=s['id']
        require(identifier(sid) and sid not in sources,f'Duplicate/invalid source: {sid}')
        require(s['kind'] in {'file','directory','url','decision','note'},f'Unknown source kind: {sid}')
        require(isinstance(s['locator'],str) and s['locator'].strip(),f'Empty source: {sid}')
        if s['kind'] in {'file','directory'}:
            p=local_path(s['locator'])
            require(p.is_file() if s['kind']=='file' else p.is_dir(),f'Missing evidence: {p}')
            if s.get('sha256'): require(s['kind']=='file' and digest(p)==s['sha256'],f'Pinned evidence changed: {sid}')
        if s['kind']=='url': require(s['locator'].startswith('https://'),f'Non-HTTPS URL: {sid}')
        sources[sid]=s
    for f in data['facts']:
        require(isinstance(f,dict),'Fact must be an object')
        fid=f['id']
        require(identifier(fid) and fid not in facts,f'Duplicate/invalid fact: {fid}')
        require(identifier(f['topic']),f'Invalid topic: {fid}')
        require(f['scope'] in {'esp32-xbox','airframe','historical'},f'Invalid scope: {fid}')
        require(isinstance(f['revisions'],list) and f['revisions'],f'No revision array: {fid}')
        previous=''
        for n,r in enumerate(f['revisions'],1):
            require(isinstance(r,dict),f'Revision must be an object: {fid}')
            require(r['revision']==n,f'Non-contiguous revisions: {fid}')
            require(r['status'] in STATUSES and r['evidence_level'] in LEVELS,f'Invalid status/evidence: {fid}')
            date.fromisoformat(r['date'])
            require(r['date']>=previous,f'Revision date order: {fid}')
            previous=r['date']
            require(all(isinstance(r[k],str) and r[k].strip() for k in ('statement','reason')),f'Missing statement/reason: {fid}')
            require(isinstance(r['sources'],list) and r['sources'] and all(isinstance(s,str) for s in r['sources']),f'Missing source array: {fid}')
            require(len(set(r['sources']))==len(r['sources']),f'Duplicate sources: {fid}')
            require(all(s in sources for s in r['sources']),f'Dangling source: {fid}')
            require(r['status']!='measured' or r['evidence_level'] in {'build','emulated','simulated','physical'},f'Measurement lacks evidence class: {fid}')
        facts[fid]=f
    edges=set()
    for e in data['relations']:
        require(isinstance(e,dict),'Relation must be an object')
        t=(e['from'],e['type'],e['to'])
        require(t not in edges and e['type'] in {'depends_on','supersedes','related_to','limited_by'},f'Invalid/duplicate edge: {t}')
        require(e['from'] in facts and e['to'] in facts and e['from']!=e['to'],f'Dangling/self edge: {t}')
        edges.add(t)
    for kind in ('depends_on','supersedes'):
        # Linear-time cycle detection, without Python recursion-depth limits.
        adjacent={fid:[] for fid in facts};degree=dict.fromkeys(facts,0)
        for a,t,b in edges:
            if t==kind: adjacent[a].append(b);degree[b]+=1
        ready=deque(fid for fid in facts if degree[fid]==0);visited=0
        while ready:
            node=ready.popleft();visited+=1
            for target in adjacent[node]:
                degree[target]-=1
                if degree[target]==0:ready.append(target)
        require(visited==len(facts),f'Cycle in {kind}')
    return data,sources,facts

SCHEMA='''
PRAGMA foreign_keys=ON;
CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL);
CREATE TABLE sources(id TEXT PRIMARY KEY,kind TEXT NOT NULL,locator TEXT NOT NULL,sha256 TEXT);
CREATE TABLE facts(id TEXT PRIMARY KEY,topic TEXT NOT NULL,scope TEXT NOT NULL);
CREATE TABLE revisions(fact_id TEXT REFERENCES facts(id),revision INTEGER CHECK(revision>0),
 date TEXT NOT NULL,status TEXT NOT NULL,statement TEXT NOT NULL,reason TEXT NOT NULL,
 evidence_level TEXT NOT NULL,PRIMARY KEY(fact_id,revision));
CREATE TABLE evidence(fact_id TEXT,revision INTEGER,source_id TEXT REFERENCES sources(id),
 PRIMARY KEY(fact_id,revision,source_id),FOREIGN KEY(fact_id,revision) REFERENCES revisions(fact_id,revision));
CREATE TABLE relations(from_id TEXT REFERENCES facts(id),kind TEXT,to_id TEXT REFERENCES facts(id),
 PRIMARY KEY(from_id,kind,to_id),CHECK(from_id<>to_id));
CREATE INDEX fact_topic ON facts(topic);
CREATE INDEX revision_status ON revisions(status);
CREATE INDEX evidence_source ON evidence(source_id);
CREATE INDEX relation_target ON relations(to_id);
CREATE VIEW current_facts AS SELECT f.id,f.topic,f.scope,r.revision,r.date,r.status,r.statement,r.evidence_level
 FROM facts f JOIN revisions r ON r.fact_id=f.id
 WHERE r.revision=(SELECT MAX(rr.revision) FROM revisions rr WHERE rr.fact_id=f.id);
CREATE VIRTUAL TABLE search USING fts5(fact_id UNINDEXED,topic,statement,tokenize='unicode61');
'''

def build():
    before_hash=digest(LEDGER)
    data,sources,facts=load()
    require(before_hash==digest(LEDGER),'Ledger changed while reading; retry')
    CACHE.parent.mkdir(parents=True,exist_ok=True)
    lock=CACHE.with_suffix('.lock')
    try: fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
    except FileExistsError: raise ValueError(f'Build lock exists: {lock}. Remove only after confirming no builder is running.')
    os.close(fd)
    temp=None
    try:
        if CACHE.exists():
            # Preserve published revision content when a previous cache is available.
            # Git remains the durable history when this disposable cache is absent.
            with closing(sqlite3.connect(CACHE.as_uri()+'?mode=ro',uri=True)) as old:
                old.row_factory=sqlite3.Row
                for prior in old.execute('SELECT * FROM revisions'):
                    fid=prior['fact_id']; n=prior['revision']
                    require(fid in facts and len(facts[fid]['revisions'])>=n,f'History removed: {fid} revision {n}')
                    current=facts[fid]['revisions'][n-1]
                    require(all(current[k]==prior[k] for k in ('revision','date','status','statement','reason','evidence_level')),f'History edited: {fid} revision {n}; append a revision instead')
                    previous_sources={r[0] for r in old.execute('SELECT source_id FROM evidence WHERE fact_id=? AND revision=?',(fid,n))}
                    require(previous_sources==set(current['sources']),f'Historical evidence links edited: {fid} revision {n}')
        with tempfile.NamedTemporaryFile(dir=CACHE.parent,suffix='.sqlite',delete=False) as f: temp=Path(f.name)
        with closing(sqlite3.connect(temp)) as db, db:
            db.executescript(SCHEMA)
            db.executemany('INSERT INTO metadata VALUES(?,?)',[('schema','2'),('ledger_sha256',before_hash),('branch',data['branch'])])
            for s in sources.values():
                h=digest(local_path(s['locator'])) if s['kind']=='file' else None
                db.execute('INSERT INTO sources VALUES(?,?,?,?)',(s['id'],s['kind'],s['locator'],h))
            for f in facts.values():
                db.execute('INSERT INTO facts VALUES(?,?,?)',(f['id'],f['topic'],f['scope']))
                for r in f['revisions']:
                    db.execute('INSERT INTO revisions VALUES(?,?,?,?,?,?,?)',(f['id'],r['revision'],r['date'],r['status'],r['statement'],r['reason'],r['evidence_level']))
                    db.executemany('INSERT INTO evidence VALUES(?,?,?)',[(f['id'],r['revision'],s) for s in r['sources']])
                db.execute('INSERT INTO search VALUES(?,?,?)',(f['id'],f['topic'],f['revisions'][-1]['statement']))
            db.executemany('INSERT INTO relations VALUES(?,?,?)',[(e['from'],e['type'],e['to']) for e in data['relations']])
            require(not db.execute('PRAGMA foreign_key_check').fetchall(),'Foreign key check failed')
            require(db.execute('PRAGMA integrity_check').fetchone()[0]=='ok','Database integrity failed')
        require(digest(LEDGER)==before_hash,'Ledger changed during build; old cache retained')
        os.replace(temp,CACHE)
        print(f'Built {CACHE}: {len(facts)} facts, {sum(len(f["revisions"]) for f in facts.values())} revisions, {len(sources)} sources.')
    finally:
        if temp and temp.exists(): temp.unlink()
        lock.unlink()

def connect():
    require(CACHE.exists(),'Run python tools/update_knowledge.py build first')
    db=sqlite3.connect(CACHE.as_uri()+'?mode=ro',uri=True)
    db.row_factory=sqlite3.Row
    try:
        require(db.execute("SELECT value FROM metadata WHERE key='ledger_sha256'").fetchone()[0]==digest(LEDGER),'Ledger changed; rebuild cache')
        for s in db.execute("SELECT locator,sha256 FROM sources WHERE kind='file'"):
            p=local_path(s['locator'])
            require(p.is_file() and digest(p)==s['sha256'],f'Evidence changed: {p}; rebuild cache')
        return db
    except Exception: db.close(); raise

def cli():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['build','validate','query','show'],nargs='?',default='build')
    p.add_argument('text',nargs='?')
    p.add_argument('--status',choices=sorted(STATUSES))
    p.add_argument('--scope',choices=['esp32-xbox','airframe','historical'])
    p.add_argument('--limit',type=int,default=12)
    a=p.parse_args()
    require(1<=a.limit<=100,'Limit must be 1..100')
    if a.command=='build': build(); return
    if a.command=='validate':
        data,sources,facts=load()
        print(f'VALID schema=2 facts={len(facts)} sources={len(sources)} relations={len(data["relations"])}'); return
    with closing(connect()) as db:
        if a.command=='show':
            require(a.text,'show requires an ID')
            row=db.execute('SELECT * FROM current_facts WHERE id=?',(a.text,)).fetchone()
            require(row is not None,f'Unknown fact: {a.text}')
            result=dict(row)
            result['history']=[dict(r) for r in db.execute('SELECT * FROM revisions WHERE fact_id=? ORDER BY revision',(a.text,))]
            result['sources']=[dict(r) for r in db.execute('SELECT e.revision,s.* FROM evidence e JOIN sources s ON s.id=e.source_id WHERE e.fact_id=? ORDER BY e.revision,s.id',(a.text,))]
            result['relations']=[dict(r) for r in db.execute('SELECT * FROM relations WHERE from_id=? OR to_id=?',(a.text,a.text))]
        else:
            words=re.findall(r'\w+',a.text or '',re.UNICODE)
            require(words or a.status or a.scope,'query needs words, --status or --scope')
            sql='SELECT c.* FROM current_facts c'
            where,params=[],[]
            if words:
                sql+=' JOIN search ON search.fact_id=c.id'
                where.append('search MATCH ?'); params.append(' AND '.join('"'+w+'"' for w in words))
            if a.status: where.append('c.status=?'); params.append(a.status)
            if a.scope: where.append('c.scope=?'); params.append(a.scope)
            sql+=' WHERE '+' AND '.join(where)+(' ORDER BY bm25(search),c.id' if words else ' ORDER BY c.id')+' LIMIT ?'
            result=[dict(r) for r in db.execute(sql,params+[a.limit])]
        print(json.dumps(result,indent=2,ensure_ascii=False))

def main():
    try: cli()
    except (ValueError,KeyError,TypeError,OSError,sqlite3.Error) as e: raise SystemExit(f'Knowledge error: {e}')

if __name__=='__main__': main()
