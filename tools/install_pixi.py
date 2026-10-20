"""Download the official portable Pixi release; no shell profile changes."""
from pathlib import Path
import hashlib,json,zipfile,requests
ROOT=Path(__file__).resolve().parents[1]
target=Path('C:/WinLibs/pixi');target.mkdir(parents=True,exist_ok=True)
release=requests.get('https://api.github.com/repos/prefix-dev/pixi/releases/latest',timeout=30)
release.raise_for_status();release=release.json()
asset=next(a for a in release['assets'] if a['name']=='pixi-x86_64-pc-windows-msvc.zip')
response=requests.get(asset['browser_download_url'],timeout=120);response.raise_for_status()
digest=hashlib.sha256(response.content).hexdigest()
expected=asset.get('digest')
if expected and expected.startswith('sha256:'):
    assert digest==expected.split(':',1)[1],'Download checksum mismatch'
archive=target/asset['name'];archive.write_bytes(response.content)
with zipfile.ZipFile(archive) as z:
    member=next(n for n in z.namelist() if n.endswith('pixi.exe'))
    (target/'pixi.exe').write_bytes(z.read(member))
out=ROOT/'artifacts/test-runs/2026-10-20-engines';out.mkdir(parents=True,exist_ok=True)
(out/'pixi-install.json').write_text(json.dumps({'release':release['tag_name'],
    'source':asset['browser_download_url'],'sha256':digest,'release_digest':expected,
    'executable':str(target/'pixi.exe')},indent=2))
print('Installed official Pixi',release['tag_name'],'at',target/'pixi.exe')
