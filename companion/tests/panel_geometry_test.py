"""Test known-size single-RGB geometry, including scale ambiguity and rejection."""
import sys,json
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'companion/perception'))
from panel_pose import panel_observations
k=np.array([[171.37776,0,160],[0,171.37776,120],[0,0,1.]])
object_points=np.array([[-.545,-.845,0],[.545,-.845,0],[.545,.845,0],[-.545,.845,0]],float)
errors=[]
for distance in [2.5,3,4,5]:
    corners,_=cv2.projectPoints(object_points,np.array([.03,.15,.01]),np.array([.1,-.1,distance]),k,np.zeros(5))
    q=corners.reshape(-1,2);image=np.zeros((240,320,3),np.uint8)
    cv2.fillConvexPoly(image,np.rint(q).astype(np.int32),(230,77,13))
    bounds=[*q.min(0),*q.max(0)]
    obs=panel_observations(image,[(bounds,.99)],k);assert obs
    error=abs(obs[0]['position_optical_m'][2]-distance);errors.append(float(error));assert error<.12
    wrong=panel_observations(image,[(bounds,.99)],k,width=1.09*1.2,height=1.69*1.2)
    assert abs(wrong[0]['position_optical_m'][2]/obs[0]['position_optical_m'][2]-1.2)<1e-5
assert not panel_observations(np.zeros((240,320,3),np.uint8),[([1,1,319,239],.99)],k)
report={'projection_tests':'passed','range_errors_m':errors,
    'scale_ambiguity':'20% assumed fixture size error produces20% inferred scale error',
    'operating_domain':'known-size blue rectangular fixture; not arbitrary unknown obstacles'}
out=ROOT/'artifacts/simulation/2026-10-20-missions/panel-geometry-tests.json';out.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))
