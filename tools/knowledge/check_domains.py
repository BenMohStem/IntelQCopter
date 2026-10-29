"""Validate the additive domain knowledge catalog; no hardware acceptance implied."""
from pathlib import Path
import hashlib
import json
import re
import sys
from urllib.parse import urlparse

if not __debug__:
    raise SystemExit('Run integrity checks without Python -O optimization')

ROOT = Path(sys.argv[1] if len(sys.argv)>1 else '.').resolve()
KB = ROOT/'knowledge'
allowed={'user_preference','verified_artifact','verified_source','assumed','pending','historical_evidence'}

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def local_path(rel):
    path=(ROOT/rel).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Reference escapes project: '+rel)
    if not path.is_file():
        raise ValueError('Missing local reference: '+rel)
    return path

index=read(KB/'index.json')
assert index['schema']==2
seen_domains=set();seen_facts=set();warnings=[]
for item in index['domains']:
    assert item['id'] not in seen_domains, 'Duplicate domain ID'
    seen_domains.add(item['id'])
    path=local_path('knowledge/'+item['path'])
    domain=read(path)
    assert domain['schema']==2 and domain['id']==item['id']
    if path.stat().st_size>index['maintenance']['split_above_bytes'] or len(domain['entries'])>index['maintenance']['split_above_entries']:
        warnings.append('Review domain split: '+item['id'])
    for fact in domain['entries']:
        assert re.fullmatch(r'[a-z][a-z0-9_.-]+',fact['id'])
        assert fact['id'] not in seen_facts, 'Duplicate fact ID: '+fact['id']
        seen_facts.add(fact['id'])
        assert fact['status'] in allowed and fact['summary'].strip()
        assert fact.get('sources'), 'Fact needs provenance: '+fact['id']
        for source in fact['sources']:
            if source['kind']=='file':
                file=local_path(source['path'])
                if 'sha256' in source:
                    assert hashlib.sha256(file.read_bytes()).hexdigest()==source['sha256'], 'Stale fact source: '+fact['id']
            elif source['kind']=='url':
                u=urlparse(source['url']);assert u.scheme=='https' and u.hostname and not u.username and not u.password
            elif source['kind']=='user':
                assert re.fullmatch(r'\d{4}-\d{2}-\d{2}',source['date']) and source['reference'].strip()
            else:
                raise ValueError('Unknown provenance type')
events=list((KB/'events').glob('*.json'))
seen_events=set()
for path in events:
    event=read(path)
    assert event['id'] not in seen_events
    seen_events.add(event['id'])
    assert set(event['domains'])<=seen_domains
    assert event['summary'].strip() and event['outcome'].strip()
print(f'PASS: {len(seen_domains)} domains, {len(seen_facts)} facts, {len(events)} events; local hashes/paths and provenance valid.')
print('Scope: curated catalog integrity only; no remote retrieval, runtime fingerprint refresh or flight acceptance.')
for warning in warnings:print('REVIEW:',warning)
