"""Start repair at an implementation checkpoint; later prepared code is excluded."""
from pathlib import Path
import argparse,json,re,subprocess
R=Path(__file__).resolve().parents[1]
def git(*args):return subprocess.check_output(['git',*args],cwd=R,text=True).strip()
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--dry-run',action='store_true');p.add_argument('--step',required=True);p.add_argument('--checkpoint',required=True)
p.add_argument('--branch',required=True);p.add_argument('--destination',type=Path,required=True)
a=p.parse_args()
assert re.fullmatch(r'codex/[A-Za-z0-9][A-Za-z0-9_/-]*',a.branch),'Use a new codex/ repair branch'
assert re.fullmatch(r'[0-9a-fA-F]{7,40}',a.checkpoint),'Use an explicit implementation commit SHA'
sha=git('rev-parse',a.checkpoint+'^{commit}')
message=git('show','-s','--format=%B',sha)
assert ('Step-ID: '+a.step in message.splitlines()) or (a.step=='00-inspection' and message.startswith('step 00:')),'Commit is not the named implementation checkpoint'
catalog=json.loads(git('show',sha+':validation/stages.json'))
stages={s['id']:s for s in catalog['stages']};assert stages[a.step]['implemented']
def depends(name):return a.step in stages[name]['requires'] or any(depends(x) for x in stages[name]['requires'])
assert not any(s['implemented'] and depends(s['id']) for s in stages.values()),'Checkpoint contains later prepared implementation; select the earlier step commit'
destination=a.destination.resolve();assert not destination.exists(),'Destination must be new'
assert not destination.is_relative_to(R),'Put the new worktree outside this checkout/build/cache'
assert not any(x in ['build','.cache'] for x in destination.parts),'No source checkout in build/cache'
if a.dry_run:
    print('CHECKPOINT_OK:',a.step,sha,'; descendants excluded; no worktree or branch created')
    raise SystemExit(0)
subprocess.run(['git','worktree','add','-b',a.branch,str(destination),sha],cwd=R,check=True)
context=destination/'artifacts/repair-context';context.mkdir(parents=True,exist_ok=True)
(context/'lineage.json').write_text(json.dumps(dict(failed_step=a.step,implementation_checkpoint=sha,
    rejected_prepared_branch=git('branch','--show-current'),rejected_prepared_head=git('rev-parse','HEAD'),
    policy='Fix this step first, then implement its descendants anew. Old later code is excluded, preserved in original history and available for selective reference. No new physical acceptance granted.'),indent=2)+'\n')
print('Repair checkout:',destination)
print('Fix and physically retest',a.step,'before implementing the next step again. Original branch preserved.')
