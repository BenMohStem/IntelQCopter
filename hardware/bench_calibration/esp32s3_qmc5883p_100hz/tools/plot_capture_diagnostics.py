#!/usr/bin/env python3
"""Render distinct, reproducible figures from the saved v9 audit and raw recordings."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap=argparse.ArgumentParser();ap.add_argument('--analysis',required=True,type=Path);ap.add_argument('csv',nargs='+',type=Path);args=ap.parse_args()
a=json.loads((args.analysis/'SENSOR_STATISTICS_AND_MATRICES.json').read_text())
files=[p for p in args.csv if p.is_file()]
labels=[];gyr=[];std=[]
for name,r in a['runs'].items():
 if 'rest' not in r:continue
 labels.append(name.rsplit('_',1)[-1].replace('.csv',''))
 gyr.append(r['rest']['gyro_bias_dps']);std.append(r['rest']['gyro_std_dps'])
if labels:
 idx=np.arange(len(labels));gyr=np.asarray(gyr);std=np.asarray(std)
 fig,ax=plt.subplots(figsize=(9,4.8))
 for i,n in enumerate(['X','Y','Z']):ax.errorbar(idx,gyr[:,i],yerr=std[:,i],marker='o',capsize=4,label=f'Gyro {n} (within-run σ)')
 ax.set_xticks(idx,labels);ax.set_ylabel('Stationary rate mean ± within-run σ (°/s)')
 ax.set_xlabel('Recording time HHMMSS');ax.grid(alpha=.25);ax.legend(loc='best');fig.tight_layout()
 fig.savefig(args.analysis/'GYRO_BIAS_AND_NOISE.png',dpi=150);plt.close(fig)

# Plot normalized radial distributions: raw ellipsoid, corrected sphere.
try:
 src=next(p for p in files if '230644' in p.name)
 df=pd.read_csv(src,low_memory=False)
 df=df[(df.pose=='MAG')&(df.mag_ok==1)&(df.mag_new==1)&(df.phase_accepted==1)]
 x=df[['mx_raw','my_raw','mz_raw']].to_numpy(float)
 m=a['runs'][src.name]['magnetometer'];b=np.asarray(m['hard_iron_offset_raw_counts']);C=np.asarray(m['dimensionless_symmetric_soft_iron_matrix'])
 raw=np.linalg.norm(x-b,axis=1);corrected=np.linalg.norm((x-b)@C,axis=1)
 raw=raw/np.median(raw)-1;corrected=corrected/np.median(corrected)-1
 fig,ax=plt.subplots(figsize=(9,4.8))
 ax.hist(raw,bins=90,alpha=.5,density=True,label='Before soft-iron compensation')
 ax.hist(corrected,bins=90,alpha=.5,density=True,label='After soft-iron compensation')
 ax.set_xlabel('Relative magnetic-field norm error (fraction)')
 ax.set_ylabel('Density');ax.set_title('Historical 23:06 MAG rotation — radial fit only')
 ax.legend();ax.grid(alpha=.15);fig.tight_layout()
 fig.savefig(args.analysis/'MAGNETOMETER_RADIAL_RESIDUALS.png',dpi=150);plt.close(fig)
except StopIteration:pass

s=a['magnetometer_cross_session_generalization']
if s:
 names=list(s)
 M=np.array([[s[n][t]['radial_rms_fraction']*100 for t in names] for n in names])
 fig,ax=plt.subplots(figsize=(7.3,6))
 im=ax.imshow(M)
 ax.set_xticks(np.arange(len(names)),[n[-10:-4] for n in names]);ax.set_yticks(np.arange(len(names)),[n[-10:-4] for n in names]);
 ax.set_xlabel('Evaluated recording');ax.set_ylabel('Calibration fitted from recording');ax.set_title('Cross-session magnetic radial RMS (%)')
 for i in range(len(names)):
  for j in range(len(names)):ax.text(j,i,f'{M[i,j]:.2f}%',ha='center',va='center')
 fig.colorbar(im,ax=ax,label='Normalized radial RMS %');fig.tight_layout()
 fig.savefig(args.analysis/'MAGNETOMETER_CROSS_SESSION.png',dpi=150);plt.close(fig)
print('Saved three diagnostic figures to',args.analysis)
