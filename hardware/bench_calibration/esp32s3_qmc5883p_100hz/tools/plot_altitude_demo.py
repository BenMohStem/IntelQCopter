#!/usr/bin/env python3
"""Generate explicitly SYNTHETIC altitude-fusion demonstration / research figure."""
from __future__ import annotations
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from altitude_fusion import VerticalKF

rng=np.random.default_rng(20261102)
kf=VerticalKF(sigma_acc=.15,baro_sigma_m=.35,lidar_sigma_m=.065,
             q_baro_bias=.004,q_acc_bias=1e-5)
rate=500;T=12;t=np.arange(1,int(T*rate)+1)/rate
true=.8+.35*np.sin(.7*t)+.13*np.sin(1.6*t)
acc=-.35*.7**2*np.sin(.7*t)-.13*1.6**2*np.sin(1.6*t)
baro=np.full(len(t),np.nan);lidar=np.full(len(t),np.nan)
fused=np.zeros(len(t));v=np.zeros(len(t));accepted=[]
for i,ti in enumerate(t):
    kf.predict(1/rate,float(acc[i]+.025+rng.normal(0,.09)))
    if i%50==0:
        baro[i]=true[i]+.35+.025*ti+rng.normal(0,.30)
        kf.update('baro',float(baro[i]))
    if i%10==0:
        lidar[i]=true[i]+rng.normal(0,.045)
        if 6.6<ti<7.0:lidar[i]+=1.8  # invalid obstacle return
        accepted.append((ti,kf.update('lidar',float(lidar[i]))))
    fused[i]=kf.x[0];v[i]=kf.x[1]
fig,axs=plt.subplots(2,1,figsize=(10.2,6.6),sharex=True,gridspec_kw={'height_ratios':[2.3,1]})
axs[0].plot(t,true,lw=2,label='Synthetic true height')
axs[0].scatter(t[::50],baro[::50],s=9,alpha=.6,label='Synthetic BMP280-like pressure altitude')
axs[0].plot(t,fused,lw=1.5,label='Linear multi-rate Kalman estimate')
axs[0].scatter(t[::10],lidar[::10],s=5,alpha=.25,label='Synthetic 50 Hz range')
axs[0].set_ylabel('Height above flat floor (m)');axs[0].grid(alpha=.2);axs[0].legend(fontsize=8,ncol=2)
axs[0].set_title('SYNTHETIC EXAMPLE ONLY — no measured IntelQCopter altitude performance')
axs[1].plot(t,v,label='Estimated vertical speed',lw=1)
axs[1].plot(t,np.gradient(true,t),label='True vertical speed',lw=1)
axs[1].set_xlabel('Time (s)');axs[1].set_ylabel('Vertical speed (m/s)');axs[1].grid(alpha=.2);axs[1].legend(fontsize=8)
fig.tight_layout()
out=Path(__file__).resolve().parent.parent/'analysis'/'synthetic_multirate_altitude.png'
fig.savefig(out,dpi=180)
plt.close(fig)
print(out,'nIS stats',kf.stats)
