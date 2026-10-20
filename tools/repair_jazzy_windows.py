"""Repair CRLF patch files without deleting a ROS workspace or changing Git globals."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path('C:/dev/jazzy')
evidence = Path(__file__).resolve().parents[1] / 'artifacts/setup/jazzy-repair'
backup = Path(__file__).resolve().parents[1] / 'build/jazzy-patch-backup'
evidence.mkdir(parents=True, exist_ok=True)
changes = []
for source in root.joinpath('src').rglob('*.patch'):
    before = source.read_bytes()
    after = before.replace(b'\r\n', b'\n')
    if before == after:
        continue
    dest = backup / source.relative_to(root)
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        shutil.copy2(source, dest)
    source.write_bytes(after)
    changes.append(dict(path=str(source), crlf=before.count(b'\r\n'),
                        before_sha256=hashlib.sha256(before).hexdigest(),
                        after_sha256=hashlib.sha256(after).hexdigest()))
if changes:
    (evidence / 'patch-normalization.json').write_text(json.dumps(changes, indent=2))
print(json.dumps({'normalized_patch_files': len(changes), 'backup': str(backup)}))
