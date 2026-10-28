"""Fault-injection tests for the small domain catalog; no project state is edited."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile

checker=Path(__file__).with_name('check_knowledge.py')

def put(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data),encoding='utf-8')

with tempfile.TemporaryDirectory(prefix='iqc-knowledge-') as temp:
    root=Path(temp)/'fixture'
    proof=root/'proof.txt';proof.parent.mkdir();proof.write_text('original evidence',encoding='utf-8')
    index={'schema':2,'domains':[{'id':'fixture','path':'domains/fixture.json'}], 'events':'events/', 'maintenance':{'split_above_entries':25,'split_above_bytes':15000}}
    base={'schema':2,'id':'fixture','entries':[{'id':'fixture.one','status':'verified_artifact','summary':'Fixture proof','sources':[{'kind':'file','path':'proof.txt','sha256':hashlib.sha256(proof.read_bytes()).hexdigest()}]}]}
    event={'id':'fixture-event','domains':['fixture'],'summary':'Fixture','outcome':'No real project mutation'}
    put(root/'knowledge/index.json',index)
    def run(domain,expected_ok,label,opt=False,ev=None):
        put(root/'knowledge/domains/fixture.json',domain);put(root/'knowledge/events/fixture.json',event if ev is None else ev)
        cmd=[sys.executable]+(['-O'] if opt else [])+[str(checker),str(root)]
        done=subprocess.run(cmd,capture_output=True,text=True)
        if (done.returncode==0)!=expected_ok:raise RuntimeError(label+': unexpected acceptance/rejection\n'+done.stdout+done.stderr)
    clone=lambda:json.loads(json.dumps(base))
    run(clone(),True,'valid')
    d=clone();d['entries'].append(d['entries'][0].copy());run(d,False,'duplicate ID')
    d=clone();d['entries'][0]['sources'][0]['path']='../outside.txt';run(d,False,'path escape')
    d=clone();d['entries'][0]['sources'][0]['sha256']='0'*64;run(d,False,'stale hash')
    d=clone();d['entries'][0]['status']='magically_accepted';run(d,False,'unknown status')
    ev=event.copy();ev['domains']=['missing'];run(clone(),False,'unknown event domain',ev=ev)
    run(clone(),False,'optimized assertions guard',opt=True)
print('PASS: 7 catalog cases - valid fixture plus 6 integrity failures rejected; project state untouched.')
