"""Render current facts from the validated versioned ledger; standard library only."""
from collections import Counter
import hashlib, os, tempfile
from knowledge_db import load, ROOT

def source_link(sid, source):
    kind,loc=source['kind'],source['locator']
    if kind in ('file','directory'): return '['+sid+'](../'+loc+')'
    if kind=='url': return '['+sid+']('+loc+')'
    if kind=='git':
        commit,path=loc.split(':',1)
        return '['+sid+'](https://github.com/BenMohStem/IntelQCopter/blob/'+commit+'/'+path+')'
    return '`'+sid+'` ('+loc+')'

def main():
    data, sources, facts = load()
    status=Counter(f['revisions'][-1]['status'] for f in facts.values())
    lines=['# Current project knowledge', '', 'Generated from the validated append-only [ledger](project.json). Edit the ledger through a new revision, then regenerate this view. No transcript or private reasoning is stored.', '',
           'Ledger SHA-256: `'+hashlib.sha256((ROOT/'knowledge/project.json').read_bytes()).hexdigest()+'`.', '',
           f"{len(facts)} facts · {sum(len(f['revisions']) for f in facts.values())} revisions · {len(sources)} sources.", '',
           'Status counts: '+', '.join(f'{k}: {v}' for k,v in sorted(status.items()))+'.', '',
           '[Documentation map](../docs/README.md) · [Math](../docs/current-model-and-fc-integration.html) · [PCB](../hardware/home-etched-fc/README.md) · [Search and history commands](README.md)', '']
    for topic in sorted({f['topic'] for f in facts.values()}):
        lines += ['## '+topic.replace('_',' ').title(), '']
        for f in sorted((f for f in facts.values() if f['topic']==topic),key=lambda f:f['id']):
            r=f['revisions'][-1]
            lines += [f"### {f['id']}", '', f"**{r['status']} · evidence: {r['evidence_level']} · {f['scope']} · revision {r['revision']}**", '', r['statement'], '', 'Sources: '+', '.join(source_link(sid,sources[sid]) for sid in r['sources'])+'.', '']
    # Replace only after the complete validated view is written.
    destination=ROOT/'knowledge/current.md'
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf8',dir=destination.parent,suffix='.tmp',delete=False) as output:
            temporary=output.name
            output.write('\n'.join(lines))
        os.replace(temporary,destination)
    finally:
        if temporary and os.path.exists(temporary): os.unlink(temporary)
    print(f"Rendered {len(facts)} latest facts; historical revisions remain in ledger.")

if __name__=='__main__': main()
