"""Synthetic temporary-fixture tests only; never records real acceptance."""
from pathlib import Path
import json,tempfile,unittest
from gates import Gates

class GateTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.r=Path(self.tmp.name)
        for name in ['validation','config','artifacts']:(self.r/name).mkdir()
        (self.r/'config/hardware.json').write_text(json.dumps({'bench_id':'SYNTHETIC','module_identity_confirmed':True}))
        (self.r/'a.cpp').write_text('a');(self.r/'b.cpp').write_text('b');(self.r/'artifacts/fake.log').write_text('SYNTHETIC FIXTURE, NOT HARDWARE')
        stages=[dict(id=n,requires=d,sources=[n+'.cpp'],acceptance_kind='physical',implemented=True,locked=False) for n,d in [('a',[]),('b',['a'])]]
        (self.r/'validation/stages.json').write_text(json.dumps(dict(global_inputs=['config/hardware.json'],stages=stages)))
    def tearDown(self):self.tmp.cleanup()
    def record(self,g,stage,result='pass',kind='physical'):
        return g.record(stage,kind,result,'artifacts/fake.log','Synthetic unit fixture','TEST','SYNTHETIC',True)
    def test_blocked_order(self):
        with self.assertRaises(AssertionError):self.record(Gates(self.r),'b')
    def test_fail_then_parent_repass_keeps_child_blocked(self):
        g=Gates(self.r);self.record(g,'a');self.record(g,'b');self.assertEqual(g.state('b')['state'],'passed')
        self.record(g,'a','fail');self.assertEqual(g.state('b')['state'],'blocked')
        self.record(g,'a');self.assertEqual(g.state('b')['state'],'blocked')
        self.record(g,'b');self.assertEqual(Gates(self.r).state('b')['state'],'passed')
    def test_build_is_not_hardware_acceptance(self):
        g=Gates(self.r);self.record(g,'a',kind='build');self.assertEqual(g.state('a')['state'],'pending-physical')
    def test_source_change_invalidates_descendant(self):
        g=Gates(self.r);self.record(g,'a');self.record(g,'b');(self.r/'a.cpp').write_text('fixed a')
        self.assertEqual(g.state('a')['state'],'stale-source');self.assertEqual(g.state('b')['state'],'blocked')
    def test_config_change_invalidates(self):
        g=Gates(self.r);self.record(g,'a');(self.r/'config/hardware.json').write_text('{}')
        self.assertEqual(g.state('a')['state'],'stale-source')
    def test_changed_evidence_invalidates(self):
        g=Gates(self.r);self.record(g,'a');(self.r/'artifacts/fake.log').write_text('changed')
        self.assertEqual(g.state('a')['state'],'changed-or-missing-evidence')
    def test_no_attestation(self):
        with self.assertRaises(AssertionError):Gates(self.r).record('a','physical','pass','artifacts/fake.log','synthetic')
    def test_ledger_tamper_rejected(self):
        g=Gates(self.r);self.record(g,'a');p=next((self.r/'validation/events').glob('*.json'))
        e=json.loads(p.read_text());e['result']='fail';p.write_text(json.dumps(e))
        with self.assertRaises(AssertionError):Gates(self.r)
if __name__=='__main__':unittest.main(verbosity=2)
