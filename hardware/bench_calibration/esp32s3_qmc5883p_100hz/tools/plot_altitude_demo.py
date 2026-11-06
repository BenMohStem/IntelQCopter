#!/usr/bin/env python3
"""Generate an explicitly SYNTHETIC terrain-separated altitude example."""
from __future__ import annotations
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from altitude_fusion import VerticalKF, lidar_com_height_above_hit

rng=np.random.default_rng(20261106)
kf=VerticalKF(sigma_acc=.12,baro_sigma_m=.15,lidar_sigma_m=.065,
             q_baro_bias=.001,q_acc_bias=1e-5,q_terrain=2e-4)
kf.initialize_confirmed_ground_reference(stationary_confirmed=True,ground_confirmed=True)
rate=500;T=12;t=np.arange(1,int(T*rate)+1)/rate

# Synthetic takeoff/climb, with a low slope in the terrain under the beam.
true_z=.8*(1.-(1.+t/.6)*np.exp(-t/.6))+.12*(1.-np.cos(.7*t))
terrain_z=.15*np.clip((t-6.)/3.,0.,1.)
true_clearance=true_z-terrain_z
acc=np.gradient(np.gradient(true_z,t),t)
baro=np.full(len(t),np.nan);lidar=np.full(len(t),np.nan)
local_z_est=np.zeros(len(t));terrain_est=np.zeros(len(t));clearance_est=np.zeros(len(t))
vertical_speed=np.zeros(len(t));accepted=[]

for i,ti in enumerate(t):
    kf.predict(1/rate,float(acc[i]+.015+rng.normal(0,.04)))
    if i%50==0:
        baro[i]=true_z[i]+.015*ti+rng.normal(0,.10)
        kf.update('baro',float(baro[i]))
    if i%10==0:
        measured_range=true_clearance[i]-.026658+rng.normal(0,.045)
        if 6.6<ti<7.0:measured_range+=1.8  # synthetic invalid return
        if measured_range>=.02:
            lidar[i]=lidar_com_height_above_hit(measured_range,0.,0.)
            accepted.append((ti,kf.update('lidar',float(lidar[i]))))
    local_z_est[i]=kf.x[0]
    terrain_est[i]=kf.x[4]
    clearance_est[i]=kf.x[0]-kf.x[4]
    vertical_speed[i]=kf.x[1]

fig,axs=plt.subplots(3,1,figsize=(10.2,8.0),sharex=True,
                      gridspec_kw={'height_ratios':[1.5,1.4,1.]})
axs[0].plot(t,true_z,lw=2,label='Synthetic local vertical position')
axs[0].scatter(t[::50],baro[::50],s=9,alpha=.55,label='Synthetic pressure altitude')
axs[0].plot(t,local_z_est,lw=1.4,label='Estimated local position')
axs[0].plot(t,terrain_est,lw=1.2,label='Estimated terrain height')
axs[0].set_ylabel('Local height (m)');axs[0].grid(alpha=.2);axs[0].legend(fontsize=8,ncol=2)
axs[1].plot(t,true_clearance,lw=2,label='True beam-hit terrain clearance')
axs[1].scatter(t[::10],lidar[::10],s=5,alpha=.3,label='Synthetic corrected LiDAR')
axs[1].plot(t,clearance_est,lw=1.3,label='Estimated terrain clearance')
axs[1].set_ylabel('Clearance (m)');axs[1].grid(alpha=.2);axs[1].legend(fontsize=8,ncol=2)
axs[2].plot(t,vertical_speed,label='Estimated vertical speed',lw=1)
axs[2].plot(t,np.gradient(true_z,t),label='Synthetic true vertical speed',lw=1)
axs[2].set_xlabel('Time (s)');axs[2].set_ylabel('Speed (m/s)');axs[2].grid(alpha=.2);axs[2].legend(fontsize=8)
fig.suptitle('SYNTHETIC EXAMPLE ONLY — no measured IntelQCopter altitude performance')
fig.tight_layout()
out=Path(__file__).resolve().parent.parent/'analysis'/'synthetic_terrain_separated_altitude.png'
out.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(out,dpi=180)
plt.close(fig)
print(out,'NIS stats',kf.stats)
