#!/usr/bin/env python3
"""Synthetic positive/negative controls for magnetometer validation."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from magnetometer_quality import fit_spd,normalized_residual,analyze

rng=np.random.default_rng(73)
n=2200
angles=rng.normal(size=(n,3)); angles/=np.linalg.norm(angles,axis=1)[:,None]
B=47.0
M=np.array([[1.2,.03,.08],[.02,.85,-.04],[.04,.01,1.5]])
b=np.array([12.,-8.,21.])
raw=(angles*B)@M.T+b+rng.normal(0,.10,(n,3))
fb,fS,uS=fit_spd(raw)
assert np.linalg.norm(fb-b)<0.7,(fb,b)
assert np.sqrt(np.mean(normalized_residual(raw,fb,uS)**2))<.012
stages=['YAW','PITCH','ROLL','TUMBLE']
records=[(stages[i//550],raw[i].tolist(),'QMC5883P') for i in range(n)]
report=analyze({'session_1':records},bootstrap=3)
assert report['heldout_radial_error'] is not None
assert report['fresh_magnetic_rows']==n
# A second independent session with a shifted constant hard-iron field, as a
# crude model of cable repositioning; this should degrade cross-session fit.
new=raw+np.array([7.,0.,0.])
records2=[(stages[i//550],new[i].tolist(),'QMC5883P') for i in range(n)]
changed=analyze({'session_1':records,'session_2':records2},bootstrap=0)
assert any(k['rms_fraction']>.04 for k in changed['leave_one_session_out'].values())
print('PASS: known-center recovery, residual and distribution holdout, independent magnetic-field shift warning')
