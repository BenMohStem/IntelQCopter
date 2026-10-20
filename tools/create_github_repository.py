"""Create the user-requested public repo with existing Git credentials, if present.

Credentials stay in memory and are never printed, logged, or added to URLs.
This does not invent a login or require a token to be pasted into chat.
"""
import json
import os
from pathlib import Path
import subprocess
import requests

ROOT=Path(__file__).resolve().parents[1]
env=dict(os.environ,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='Never')
try:
    result=subprocess.run(['git','credential','fill'],input='protocol=https\nhost=github.com\n\n',
                          capture_output=True,text=True,env=env,timeout=30,cwd=ROOT)
except subprocess.TimeoutExpired:
    raise SystemExit('No noninteractive GitHub credential was available within 30 seconds.')
credential=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
if result.returncode or not credential.get('password'):
    raise SystemExit('GitHub sign-in is required: no usable credential in the configured Git credential manager.')
session=requests.Session()
session.auth=(credential.get('username','git'),credential['password'])
session.headers.update({'Accept':'application/vnd.github+json','X-GitHub-Api-Version':'2022-11-28'})
profile=session.get('https://api.github.com/user',timeout=30)
if profile.status_code!=200:
    raise SystemExit(f'GitHub credential could not authenticate (HTTP {profile.status_code}); no token was logged.')
owner=profile.json()['login']
if owner.casefold()!='benmohstem':
    raise SystemExit('Stored Git login differs from the connected project owner; select the intended account first.')
url=f'https://api.github.com/repos/{owner}/IntelQcopter'
repo=session.get(url,timeout=30)
if repo.status_code==404:
    repo=session.post('https://api.github.com/user/repos',json={
        'name':'IntelQcopter','private':False,'auto_init':False,
        'description':'C Pico SDK quadcopter control, ROS 2 autonomy, mathematical modelling and original simulation evidence.'},timeout=30)
if repo.status_code not in (200,201):
    raise SystemExit(f'Repository creation/access failed (HTTP {repo.status_code}); browser creation may be required.')
data=repo.json()
if data['private']:
    raise SystemExit('An existing private repository was found; it was not changed to public.')
print(json.dumps({'repository':data['html_url'],'public':True,'created':repo.status_code==201},indent=2))
