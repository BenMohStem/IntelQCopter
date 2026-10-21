"""Inspect, verify or restore a byte-checked archive from immutable Git history."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

ROOT=Path(__file__).resolve().parents[2]
MANIFEST=ROOT/'archive/rp2350.snapshot.json'
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()
def verify(folder,manifest):
    expected={f['path'] for f in manifest['files']}
    actual={p.relative_to(folder).as_posix() for p in folder.rglob('*') if p.is_file()} if folder.is_dir() else set()
    if actual!=expected:raise ValueError(f'Archive file set differs: {len(expected-actual)} missing, {len(actual-expected)} extra')
    for f in manifest['files']:
        p=(folder/f['path']).resolve()
        if not p.is_relative_to(folder.resolve()) or p.stat().st_size!=f['bytes'] or digest(p)!=f['sha256']:
            raise ValueError('Archive hash mismatch: '+f['path'])
    print(f"VERIFIED {len(expected)} files, {manifest['bytes']} bytes")
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['status','verify','restore'],default='status',nargs='?')
    args=p.parse_args();m=json.loads(MANIFEST.read_text(encoding='utf8'))
    cache=(ROOT/m['cache']).resolve()
    if not cache.is_relative_to((ROOT/'.cache/archive').resolve()):raise ValueError('Cache target escapes archive cache')
    if args.action=='status':
        print(json.dumps({k:v for k,v in m.items() if k!='files'},indent=2));print('Present:',cache.exists());return
    if args.action=='verify':verify(cache,m);return
    if cache.exists():verify(cache,m);print('Existing verified archive retained');return
    cache.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='restore-',dir=cache.parent) as tmp:
        staging=Path(tmp);archive=staging/'snapshot.tar'
        subprocess.run(['git','-c','safe.directory='+ROOT.as_posix(),'archive','--format=tar','--output='+str(archive),m['source_commit'],m['source_tree']],cwd=ROOT,check=True)
        with tarfile.open(archive) as tar:
            for entry in tar.getmembers():
                target=(staging/entry.name).resolve()
                if not target.is_relative_to(staging) or not (entry.isfile() or entry.isdir()):raise ValueError('Unsafe archive entry')
            tar.extractall(staging)
        restored=staging/m['source_tree'];verify(restored,m)
        restored.rename(cache)
    print('Restored',cache)
if __name__=='__main__':
    try:main()
    except (ValueError,OSError,subprocess.CalledProcessError) as e:raise SystemExit(str(e))
