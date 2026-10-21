import json,urllib.request,urllib.error,sys
from pathlib import Path
import cv2
R=Path(__file__).resolve().parents[2];out=R/'artifacts/maintenance/workbench-authority-20261021';checks={}
def fetch(path,headers=None):
 with urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:8088'+path,headers=headers or {}),timeout=10) as r:return r.status,r.headers,r.read()
settings=json.loads(fetch('/settings')[2]);checks['default_profile_restored']=settings['settings']['ros']['max_speed_mps']==.8
checks['offline_mathml_guide']=b'<math' in fetch('/guide')[2]
checks['model_guide_link_works']=b'<math' in fetch('/current-model-and-fc-integration.html')[2]
checks['authority_page_works']=b'Who actually flew the route' in fetch('/proof/')[2]
status,headers,body=fetch('/recordings/authority-seven-20261021-a/gazebo.mp4',{'Range':'bytes=0-1023'});checks['video_range_streaming']=status==206 and len(body)==1024 and headers['Content-Type']=='video/mp4'
bad=settings['settings'];bad['fc']['hover_collective']=-2
try:
 request=urllib.request.Request('http://127.0.0.1:8088/settings',data=json.dumps(bad).encode(),headers={'Content-Type':'application/json'});urllib.request.urlopen(request);checks['invalid_profile_rejected']=False
except urllib.error.HTTPError as e:checks['invalid_profile_rejected']=e.code==409
checks['invalid_request_does_not_modify_saved_profile']=json.loads(fetch('/settings')[2])['settings']['fc']['hover_collective']==.5
videos=[]
for name in ['authority-seven-20261021-a','authority-uart-loss-20261021-a','managed-20261021-144908-env01','managed-20261021-145223-env01']:
 d=R/'artifacts/gazebo-suite'/name;cap=cv2.VideoCapture(str(d/'gazebo.mp4'));fps=cap.get(cv2.CAP_PROP_FPS);ok,frame=cap.read();videos.append(dict(run=name,fps=fps,first_frame_decodes=bool(ok),shape=list(frame.shape) if ok else None));cap.release();checks['video_'+name]=ok and abs(fps-15)<.01
result=dict(passed=all(checks.values()),checks=checks,videos=videos,scope='Local workstation UI/API and original native video decoding; synthetic physics/model assumptions unchanged')
(out/'backend-checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['passed']
