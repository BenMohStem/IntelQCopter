"""Analyze a real stationary, independently level log; after-curves are replay predictions."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('log', type=Path)
parser.add_argument('--fit', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--deps', type=Path)
parser.add_argument('--level-attested', action='store_true', required=True)
args = parser.parse_args()
if args.output.exists():
    raise SystemExit('Output exists; choose a new directory to preserve every attempt.')
if args.deps:
    sys.path.insert(0,str(args.deps))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

lines = args.log.read_text(encoding='utf-8',errors='replace').splitlines()
rows = []
for line in lines:
    if line.startswith('IMU,'):
        row = [float(x) for x in line.split(',')[1:]]
        if len(row)!=7 or not np.isfinite(row).all():
            raise SystemExit('Malformed original IMU row; no silent trimming.')
        rows.append(row)
data = np.array(rows)
if len(data)<100:
    raise SystemExit('Need at least 100 real samples for this split analysis.')
dt = np.array([(int(b)-int(a)) & 0xffffffff for a,b in zip(data[:-1,0],data[1:,0])])/1e6
if np.any(dt<=0) or np.any(dt>5):
    raise SystemExit('Reset/duplicate/gap: select a continuous recording.')
t = np.r_[0,np.cumsum(dt)]
acc = data[:,1:4]
gyro = data[:,4:7]
split = int(len(data)*0.6)

def huber_location(values):
    centre = np.median(values,axis=0)
    scale = np.maximum(1.4826*np.median(abs(values-centre),axis=0),1e-8)
    for _ in range(30):
        r = np.sqrt(np.mean(((values-centre)/scale)**2,axis=1))
        weights = np.minimum(1,1.5/np.maximum(r,1e-12))
        updated = np.average(values,axis=0,weights=weights)
        if np.max(abs(updated-centre))<1e-10:
            break
        centre = updated
    return updated, weights

gravity, weights = huber_location(acc[:split])
gyro_bias, gyro_weights = huber_location(gyro[:split])
u = gravity / np.linalg.norm(gravity)
z = np.array([0.,0.,1.])
v = np.cross(u,z)
c = float(u@z)
if c<0.5:
    raise SystemExit('Not sufficiently upright for a level-reference refinement.')
K = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
L = np.eye(3)+K+(K@K)/(1+c)
assert np.allclose(L@L.T,np.eye(3),atol=1e-12) and np.linalg.det(L)>0.999999
after = acc@L.T
oldfit = json.loads(args.fit.read_text())
C0 = np.array(oldfit['transform_sensor_to_drone'])
pending_raw = None
paired_errors = []
for line in lines:
    if line.startswith('MPU_BIAS_CORRECTED '):
        match = re.search(r'X=([\d.eE+-]+) Y=([\d.eE+-]+) Z=([\d.eE+-]+)',line)
        pending_raw = np.array([float(x) for x in match.groups()]) if match else None
    elif line.startswith('IMU,') and pending_raw is not None:
        actual = np.array([float(x) for x in line.split(',')[2:5]])
        paired_errors.append(abs(C0@pending_raw-actual))
        pending_raw = None
rounding_bound = np.sum(abs(C0),axis=1)*0.00005+0.000003
paired_errors = np.array(paired_errors)
if len(paired_errors) == 0 or np.any(paired_errors > rounding_bound):
    raise SystemExit('Logged raw/corrected pairs do not match the expected current matrix within print rounding.')
Cnew = L@np.array(oldfit['transform_sensor_to_drone'])
# Offset-register change at 32.75 register counts per degree/s.
register_delta = -np.where(gyro_bias>=0,np.floor(gyro_bias*180/np.pi*32.75+0.5),np.ceil(gyro_bias*180/np.pi*32.75-0.5)).astype(int)
gyro_after = gyro + register_delta*np.pi/(180*32.75)

def angles(a):
    return np.column_stack((np.arctan2(a[:,1],a[:,2]),np.arctan2(-a[:,0],np.hypot(a[:,1],a[:,2]))))
before_angles=angles(acc)
after_angles=angles(after)
norm=np.linalg.norm(acc,axis=1)
assert np.max(abs(norm-np.linalg.norm(after,axis=1)))<1e-10

def stats(a):
    return dict(mean=np.mean(a,axis=0).tolist(),stddev=np.std(a,axis=0,ddof=1).tolist(),
                median=np.median(a,axis=0).tolist(),robust_sigma=(1.4826*np.median(abs(a-np.median(a,axis=0)),axis=0)).tolist())

def block_ci(values,seed):
    rng=np.random.default_rng(seed)
    n=len(values); block=min(10,n//4)
    nblocks=int(np.ceil(n/block))
    boot=[]
    for _ in range(2000):
        starts=rng.integers(0,n,size=nblocks)
        indices=((starts[:,None]+np.arange(block))%n).ravel()[:n]
        boot.append(np.mean(values[indices],axis=0))
    return dict(method='circular block bootstrap; 2000 resamples; block length '+str(block),
                lower=np.quantile(boot,.025,axis=0).tolist(),upper=np.quantile(boot,.975,axis=0).tolist())

def slopes(a,tt):
    return [float(np.polyfit(tt, a[:,i], 1)[0]) for i in range(a.shape[1])]

report=dict(source_sha256=hashlib.sha256(args.log.read_bytes()).hexdigest(),
    user_attestation='Motionless, independently level',samples=len(data),duration_s=float(t[-1]),
    observed_rate_hz=float(1/np.mean(dt)),min_interval_s=float(dt.min()),max_interval_s=float(dt.max()),
    engine='NumPy statistics/Huber fit + block bootstrap + Matplotlib replay, not simulator or device execution',
    train_samples=split,holdout_samples=len(data)-split,
    gravity_train_robust_m_s2=gravity.tolist(),level_rotation=L.tolist(),
    level_rotation_degrees=float(np.arccos(np.clip(c,-1,1))*180/np.pi),
    transform_sensor_to_drone=Cnew.tolist(),gyro_bias_sensor_rad_s=gyro_bias.tolist(),
    gyro_register_delta=register_delta.tolist(),
    all_acceleration=stats(acc),all_gyro_sensor=stats(gyro),norm=stats(norm),
    all_angles_degrees=stats(before_angles*180/np.pi),
    holdout_angles_before_degrees=stats(before_angles[split:]*180/np.pi),
    holdout_angles_after_replay_degrees=stats(after_angles[split:]*180/np.pi),
    holdout_angles_after_mean_ci95_degrees=block_ci(after_angles[split:]*180/np.pi,20261025),
    holdout_gyro_before_rad_s=stats(gyro[split:]),holdout_gyro_after_replay_rad_s=stats(gyro_after[split:]),
    gyro_train_mean_ci95_rad_s=block_ci(gyro[:split],20261026),
    angle_covariance_after_replay_rad2=np.cov(after_angles[split:].T,ddof=1).tolist(),
    gyro_covariance_sensor_rad2_s2=np.cov(gyro[split:].T,ddof=1).tolist(),
    angle_trend_degrees_s=slopes(before_angles*180/np.pi,t),gyro_trend_rad_s2=slopes(gyro,t),
    downweighted_accel_samples=int(np.sum(weights<1)),downweighted_gyro_samples=int(np.sum(gyro_weights<1)),
    observed_transform_check=dict(pairs=len(paired_errors),max_abs_difference_m_s2=np.max(paired_errors,axis=0).tolist(),
                                  print_rounding_bounds_m_s2=rounding_bound.tolist(),passed=True,
                                  limit='Confirms logged linear transform consistency, not register bias correctness or complete firmware identity'),
    raw_samples_discarded=0,
    limits=['After values are offline replay predictions, not new hardware measurements',
            'Temporal holdout only, same pose and mounting; no independent multi-pose validation',
            'No embedded running-source fingerprint or startup configuration readback captured',
            'No temperature time series, no spectral/Allan characterization, no true 500Hz acquisition',
            'Level fixture uncertainty not quantified; mean confidence interval is not absolute attitude accuracy',
            'A single known pose cannot identify new 3D bias/scale errors; applied level trim is a constrained rigid rotation',
            'Gyro results in sensor axes; no drone-frame gyro rotation established',
            'Filter correlation, long-term drift, vibration and future flight behavior remain unvalidated'])
args.output.mkdir(parents=True)
(args.output/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
with (args.output/'samples.csv').open('w') as out:
    out.write('time_s,ax,ay,az,gx_sensor,gy_sensor,gz_sensor\n')
    for tt,row in zip(t,data):
        out.write(','.join(str(x) for x in [tt,*row[1:]])+'\n')

plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25})
fig,axs=plt.subplots(3,2,figsize=(13,11))
for i,name in enumerate(('Roll','Pitch')):
    ax=axs[0,i]
    ax.plot(t,before_angles[:,i]*180/np.pi,label='Real measured output',alpha=.7)
    ax.plot(t,after_angles[:,i]*180/np.pi,label='After trim: OFFLINE replay',alpha=.7)
    ax.axvline(t[split],color='k',ls='--',label='Fit / temporal holdout')
    ax.set(title=name+' on user-confirmed level fixture',ylabel='degrees',xlabel='seconds')
    ax.legend(fontsize=8)
axs[1,0].plot(t,norm,label='Real acceleration magnitude')
axs[1,0].axhline(9.80665,color='k',ls='--',label='g reference; not fitted')
axs[1,0].set(title='Rotation preserves magnitude',xlabel='seconds',ylabel='m/s²')
axs[1,0].legend(fontsize=8)
for i,name in enumerate('XYZ'):
    axs[1,1].plot(t,gyro[:,i]*180/np.pi,label='Sensor '+name,alpha=.7)
axs[1,1].set(title='Real stationary gyro output',xlabel='seconds',ylabel='degrees/s')
axs[1,1].legend(fontsize=8)
for i,name in enumerate('XYZ'):
    axs[2,0].plot(t,gyro_after[:,i]*180/np.pi,label='Sensor '+name,alpha=.7)
axs[2,0].axvline(t[split],color='k',ls='--')
axs[2,0].set(title='Gyro hardware-trim prediction: OFFLINE replay',xlabel='seconds',ylabel='degrees/s')
axs[2,0].legend(fontsize=8)
for i,name in enumerate(('Roll','Pitch')):
    axs[2,1].hist(after_angles[split:,i]*180/np.pi,bins=22,alpha=.5,label=name)
axs[2,1].set(title='Temporal holdout after-trim prediction',xlabel='degrees',ylabel='sample count')
axs[2,1].legend(fontsize=8)
fig.suptitle('Actual ESP32 serial capture — predicted improvements require upload + new physical verification',fontsize=12)
fig.tight_layout(rect=(0,0,1,.97))
fig.savefig(args.output/'precision-analysis.png',dpi=160)
plt.close(fig)
print(json.dumps({key:report[key] for key in ('samples','duration_s','observed_rate_hz','level_rotation_degrees','gyro_bias_sensor_rad_s','gyro_register_delta','all_angles_degrees','holdout_angles_after_replay_degrees','all_gyro_sensor')},indent=2))
print('Original-data analysis and graphs:',args.output)
