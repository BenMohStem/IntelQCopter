"""Acceptance ledger: builds are observations; real acceptance binds exact inputs."""
from pathlib import Path
import argparse, datetime, hashlib, json, os, subprocess

ROOT=Path(__file__).resolve().parents[2]

def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':')).encode()
def digest(data):return hashlib.sha256(data).hexdigest()

class Gates:
    def __init__(self,root):
        self.root=Path(root).resolve()
        self.catalog=json.loads((self.root/'validation/stages.json').read_text())
        self.stages={s['id']:s for s in self.catalog['stages']}
        assert len(self.stages)==len(self.catalog['stages']),'Duplicate stage'
        def visit(name,stack):
            assert name in self.stages,('Unknown dependency',name)
            assert name not in stack,('Dependency cycle',name)
            for dep in self.stages[name]['requires']:visit(dep,stack+[name])
        for name in self.stages:visit(name,[])
        self.events=[];previous='ROOT'
        for path in sorted((self.root/'validation/events').glob('*.json')):
            e=json.loads(path.read_text());claimed=e.pop('event_hash')
            assert e['previous_hash']==previous,'Broken ledger chain'
            assert digest(canonical(e))==claimed,'Changed ledger event'
            e['event_hash']=claimed;self.events.append(e);previous=claimed

    def fingerprint(self,name):
        stage=self.stages[name]
        inputs={}
        for rel in self.catalog['global_inputs']+stage['sources']:
            p=(self.root/rel).resolve()
            assert p.is_relative_to(self.root) and p.is_file(),('Missing/unsafe source',rel)
            inputs[rel]=digest(p.read_bytes())
        return digest(canonical({'stage':stage,'inputs':inputs}))

    def evidence_ok(self,event):
        p=(self.root/event['evidence']).resolve()
        return p.is_relative_to(self.root/'artifacts') and p.is_file() and digest(p.read_bytes())==event['evidence_sha256']

    def state(self,name):
        stage=self.stages[name]
        if stage['locked']:return {'state':'locked'}
        if not stage['implemented']:return {'state':'planned'}
        relevant=[e for e in self.events if e['stage']==name and e['kind']==stage['acceptance_kind']]
        if not relevant:return {'state':'pending-'+stage['acceptance_kind']}
        e=relevant[-1]
        if e['result']=='fail':return {'state':'failed','receipt':e['event_hash']}
        if e['source_fingerprint']!=self.fingerprint(name):return {'state':'stale-source'}
        if not self.evidence_ok(e):return {'state':'changed-or-missing-evidence'}
        for dep in stage['requires']:
            status=self.state(dep)
            if status['state']!='passed' or e['dependencies'].get(dep)!=status.get('receipt'):
                return {'state':'blocked','by':dep}
        return {'state':'passed','receipt':e['event_hash']}

    def record(self,name,kind,result,evidence,note,operator='',hardware_id='',attest=False,engine=''):
        stage=self.stages[name]
        assert stage['implemented'] and not stage['locked'],'Step not implemented or locked'
        p=Path(evidence)
        if not p.is_absolute():p=self.root/p
        p=p.resolve()
        assert p.is_relative_to(self.root/'artifacts') and p.is_file(),'Evidence must be a file inside artifacts/'
        assert kind in ['physical','build','simulation'] and result in ['pass','fail']
        assert note.strip(),'Explain the observation and acceptance criteria'
        if kind=='physical':
            assert attest and operator.strip() and hardware_id.strip(),'Physical evidence requires explicit real-hardware attestation, operator and bench identity'
            hardware=json.loads((self.root/'config/hardware.json').read_text())
            assert hardware.get('bench_id')==hardware_id and hardware_id!='UNMEASURED','Confirm actual bench identity in config/hardware.json first'
            assert hardware.get('module_identity_confirmed') is True,'Record actual board/module identity before physical acceptance'
        if kind=='simulation':assert engine.strip(),'Name the real simulation engine'
        dependencies={}
        if kind==stage['acceptance_kind'] and result=='pass':
            for dep in stage['requires']:
                status=self.state(dep)
                assert status['state']=='passed',('Prerequisite not accepted',dep,status)
                dependencies[dep]=status['receipt']
        event={'stage':name,'kind':kind,'result':result,'note':note,'operator':operator,
               'hardware_id':hardware_id,'real_hardware_attested':bool(attest),'engine':engine,
               'time_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
               'source_fingerprint':self.fingerprint(name),'dependencies':dependencies,
               'evidence':p.relative_to(self.root).as_posix(),'evidence_sha256':digest(p.read_bytes()),
               'previous_hash':self.events[-1]['event_hash'] if self.events else 'ROOT'}
        event['event_hash']=digest(canonical(event))
        folder=self.root/'validation/events';folder.mkdir(parents=True,exist_ok=True)
        lock=folder/'.lock';fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
        try:
            # Refuse a concurrent writer instead of silently forking the chain.
            assert len(list(folder.glob('*.json')))==len(self.events),'Ledger changed; retry'
            target=folder/f'{len(self.events)+1:06d}-{event["event_hash"][:12]}.json'
            with target.open('x',encoding='utf8') as f:f.write(json.dumps(event,indent=2)+'\n')
        finally:os.close(fd);lock.unlink()
        self.events.append(event);return event

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('status')
    r=sub.add_parser('record');r.add_argument('stage');r.add_argument('--kind',choices=['physical','build','simulation'],required=True)
    r.add_argument('--result',choices=['pass','fail'],required=True);r.add_argument('--evidence',required=True);r.add_argument('--note',required=True)
    r.add_argument('--operator',default='');r.add_argument('--hardware-id',default='');r.add_argument('--attest-real-hardware',action='store_true');r.add_argument('--engine',default='')
    args=p.parse_args();g=Gates(ROOT)
    if args.command=='status':
        for name in g.stages:print(f'{name:24} {json.dumps(g.state(name))}')
    else:
        e=g.record(args.stage,args.kind,args.result,args.evidence,args.note,args.operator,args.hardware_id,args.attest_real_hardware,args.engine)
        print('Recorded',e['event_hash']);print('Acceptance:',g.state(args.stage)['state'])
if __name__=='__main__':
    try:main()
    except (AssertionError,KeyError,ValueError,OSError) as e:raise SystemExit('Gate rejected: '+str(e))
