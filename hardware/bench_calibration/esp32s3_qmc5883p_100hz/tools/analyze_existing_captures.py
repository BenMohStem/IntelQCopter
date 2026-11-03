#!/usr/bin/env python3
"""IntelQCopter historical-capture audit (offline; never modifies source CSVs).

Accepted latest attempts, raw gyroscope & accelerometer statistics, six-face
bias/scale, stationary pressure scatter, magnetometer ellipsoid fitting in raw
ADC counts and cross-session holdouts. Requires numpy, scipy, pandas, matplotlib.

The matrices describe old datasets. They are NOT certified for the QMC device
when its I2C address is currently not acknowledged.
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

G=9.80665
FACES=['+X','-X','+Y','-Y','+Z','-Z']

def vec(x,n=7):return [round(float(v),n) for v in np.asarray(x).ravel()]
def mat(x,n=7):return [vec(r,n) for r in np.asarray(x)]
def finite(x):return np.asarray(x,dtype=float)[np.all(np.isfinite(x),axis=1)]
def unit(x):return x/np.linalg.norm(x,axis=1)[:,None]

def ci_boot(face_samples, repeats=400):
    rng=np.random.default_rng(1125)
    groups=[]
    for face in FACES:
        a=face_samples[face]
        blocks=[a[i:i+500] for i in range(0,len(a),500) if len(a[i:i+500])>=200]
        groups.append(np.array([b.mean(axis=0) for b in blocks]))
    biases=[];scales=[]
    if min(len(g) for g in groups)<4:return None
    for _ in range(repeats):
        m=np.array([g[rng.integers(len(g),size=len(g))].mean(axis=0) for g in groups]); b=[];s=[]
        for j in range(3):
            plus,minus=m[2*j,j],m[2*j+1,j]
            b.append((plus+minus)/2)
            s.append(2*G/(plus-minus))
        biases.append(b);scales.append(s)
    return {'block_seconds_approx':1,'replicates':repeats,
        'bias_mps2_95pct_interval':mat(np.percentile(biases,[2.5,97.5],axis=0)),
        'scale_factor_95pct_interval':mat(np.percentile(scales,[2.5,97.5],axis=0)),
        'limitation':'Does not account for systematic errors from hand-tilted face poses.'}

def sixface(d):
    samples={p:d.loc[(d.pose==p)&(d.mpu_ok==1),['ax_mps2','ay_mps2','az_mps2']].to_numpy(float) for p in FACES}
    if min(len(a) for a in samples.values())<500:return {'status':'insufficient_faces','counts':{p:len(a) for p,a in samples.items()}}
    means={p:a.mean(axis=0) for p,a in samples.items()}
    bias=[];scale=[]
    for i in range(3):
        p,n=means[FACES[2*i]][i],means[FACES[2*i+1]][i]
        bias.append((p+n)/2)
        scale.append(2*G/(p-n))
    bias=np.array(bias);D=np.diag(scale)
    X=np.array([np.r_[means[p],1] for p in FACES])
    Y=np.zeros((6,3))
    for i in range(3):Y[2*i,i]=G;Y[2*i+1,i]=-G
    coefs,_,rank,_=np.linalg.lstsq(X,Y,rcond=None)
    M=coefs[:3].T
    bfull=-np.linalg.solve(M,coefs[3])
    diagerr=(np.array([D@(means[p]-bias) for p in FACES])-Y)
    fullerr=(X@coefs-Y)
    return {'status':'estimated','pose_counts':{p:len(a) for p,a in samples.items()},
      'pose_means_mps2':{p:vec(v) for p,v in means.items()},
      'pose_std_mps2':{p:vec(samples[p].std(axis=0,ddof=1)) for p in FACES},
      'recommended_diagonal_bias_mps2':vec(bias),'recommended_diagonal_matrix':mat(D),
      'full3x3_experimental_bias_mps2':vec(bfull),'full3x3_experimental_matrix':mat(M),
      'diagonal_face_rms_residual_mps2':float(np.sqrt(np.mean(np.sum(diagerr**2,axis=1)))),
      'full3x3_face_rms_residual_mps2':float(np.sqrt(np.mean(np.sum(fullerr**2,axis=1)))),
      'block_bootstrap_95pct':ci_boot(samples),
      'important':'Full 3x3 solution can absorb pose tilt; prefer diagonal unless using precise face fixture.'}

def spd_params(p):
    L=np.array([[np.exp(p[3]),0,0],[p[4],np.exp(p[5]),0],[p[6],p[7],np.exp(p[8])]])
    return L@L.T

def fit_mag(x, limit=1800):
    """Ellipsoid shape normalized to mean raw radius, no absolute µT claim."""
    a=np.asarray(x,dtype=float)
    if len(a)>limit:a=a[np.linspace(0,len(a)-1,limit).astype(int)]
    mu=a.mean(axis=0); sc=float(np.mean(np.ptp(a,axis=0)))
    if sc<10:raise ValueError('insufficient magnetic 3D span')
    v=(a-mu)/sc
    b0=(v.max(axis=0)+v.min(axis=0))*.5
    vals,V=np.linalg.eigh(np.cov(v.T))
    if min(vals)<1e-7:raise ValueError('rank deficient cloud')
    S0=V@np.diag(1/np.sqrt(vals))@V.T
    S0/=np.median(np.linalg.norm((v-b0)@S0.T,axis=1))
    L=np.linalg.cholesky(S0)
    p0=np.r_[b0,np.log(L[0,0]),L[1,0],np.log(L[1,1]),L[2,0],L[2,1],np.log(L[2,2])]
    def fun(p):return np.linalg.norm((v-p[:3])@spd_params(p),axis=1)-1
    sol=least_squares(fun,p0,loss='soft_l1',f_scale=.03,max_nfev=180,
        bounds=([-5,-5,-5,-6,-15,-6,-15,-15,-6],[5,5,5,6,15,6,15,15,6]))
    if not sol.success:raise ValueError('ellipsoid nonconvergence')
    b=mu+sc*sol.x[:3]
    U=spd_params(sol.x)/sc  # 1/count
    radius=float(np.median(np.linalg.norm(a-b,axis=1)))
    return b,U,U*radius,radius

def mag_r(x,b,U):return np.linalg.norm((x-b)@U,axis=1)-1

def magnetic_coverage(x,b,U):
    w=(x-b)@U
    w=w[np.linalg.norm(w,axis=1)>.001]
    u=unit(w);n=128;j=np.arange(n)+.5
    z=1-2*j/n;theta=j*np.pi*(3-np.sqrt(5));q=np.stack([np.sqrt(1-z*z)*np.cos(theta),np.sqrt(1-z*z)*np.sin(theta),z],axis=1)
    dist,_=cKDTree(u).query(q)
    return round(float(np.mean(dist<=np.sqrt(2-2*np.cos(np.deg2rad(18))))),5)

def summarize_mag(x,b,U,C,radius):
    r=mag_r(x,b,U)
    return {'sample_count':len(x),'hard_iron_offset_raw_counts':vec(b),
      'dimensionless_symmetric_soft_iron_matrix':mat(C),
      'unit_sphere_matrix_inverse_counts':mat(U,9),
      'uncorrected_median_radius_counts':round(radius,3),
      'corrected_norm_rms_fraction':round(float(np.sqrt(np.mean(r*r))),6),
      'corrected_norm_std_fraction':round(float(np.std(r,ddof=1)),6),
      'corrected_norm_p95_absolute_fraction':round(float(np.percentile(np.abs(r),95)),6),
      'coverage_128_directions_at_18deg':magnetic_coverage(x,b,U),
      'shape_eigenvalues':vec(np.linalg.eigvalsh(C)),
      'shape_condition_number':round(float(np.linalg.cond(C)),5),
      'formula':'m_corrected_counts = dimensionless_matrix @ (m_raw_counts - offset_counts)',
      'caution':'Ellipsoid correction is ambiguous by an arbitrary orthogonal rotation; no absolute field or heading truth.'}

def sample_period_stats(df):
    r=pd.to_numeric(df['observed_packet_dt_us'],errors='coerce')
    r=r[(r>0)&(r<1e6)]
    return {'median_us':round(float(r.median()),2),'p95_us':round(float(r.quantile(.95)),2),
       'p99_us':round(float(r.quantile(.99)),2),'mean_us':round(float(r.mean()),2),
       'gt_2500us_count':int((r>2500).sum()),'gt_5000us_count':int((r>5000).sum()),
       'gt_10000us_count':int((r>10000).sum())}

def run_analysis(files,outdir):
    runs={}; mag_sets={}; rest_bias=[]; acc_models=[]
    from scipy.signal import detrend
    for file in files:
        df=pd.read_csv(file,low_memory=False)
        if len(df)==0:runs[file.name]={'samples':0,'status':'empty'};continue
        df=df[df.kind=='D'].copy()
        timing_all=sample_period_stats(df)
        # Retain latest accepted redo for each (pose, mag_stage).
        latest=df.groupby(['pose','mag_stage'],dropna=False)['phase_attempt'].transform('max')
        df=df[(df.phase_attempt==latest)&(df.phase_accepted==1)]
        item={'samples':len(df),'mpu_temperature_range_c':vec([df.mpu_temp_c.min(),df.mpu_temp_c.max()]),
              'bmp_temperature_range_c':vec([df.bmp_temp_c.min(),df.bmp_temp_c.max()]),
              'timing_all_packets_before_attempt_filter':timing_all}
        rest=df[(df.pose=='REST')&(df.mpu_ok==1)]
        if len(rest)>=300:
            g=rest[['gx_dps','gy_dps','gz_dps']].to_numpy(float)
            a=rest[['ax_mps2','ay_mps2','az_mps2']].to_numpy(float)
            gc=np.cov(g,rowvar=False);ac=np.cov(a,rowvar=False)
            gr=detrend(g,axis=0,type='linear');ar=detrend(a,axis=0,type='linear')
            item['rest']={'n':len(rest),'gyro_bias_dps':vec(g.mean(axis=0)),
                          'gyro_std_dps':vec(g.std(axis=0,ddof=1)),
                          'gyro_covariance_dps2':mat(gc,10),
                          'gyro_correlation':mat(np.corrcoef(g,rowvar=False),5),
                          'gyro_std_detrended_dps':vec(gr.std(axis=0,ddof=1)),
                          'accel_mean_mps2':vec(a.mean(axis=0)),
                          'accel_std_mps2':vec(a.std(axis=0,ddof=1)),
                          'accel_covariance_mps4':mat(ac,10),
                          'accel_std_detrended_mps2':vec(ar.std(axis=0,ddof=1)),
                          'mpu_temp_mean_c':round(float(rest.mpu_temp_c.mean()),5)}
            rest_bias.append((file.name,item['rest']))
        rest_mag=df[(df.pose=='REST')&(df.mag_ok==1)&(df.mag_new==1)]
        if len(rest_mag)>100:
            mrest=rest_mag[['mx_raw','my_raw','mz_raw']].to_numpy(float)
            item['rest_magnetic_variation']={
                'n_fresh':len(mrest),'mean_raw_counts':vec(mrest.mean(axis=0)),
                'std_raw_counts':vec(mrest.std(axis=0,ddof=1)),
                'fresh_observed_rate_hz':round(float((len(rest_mag)-1)*1e6 /
                     (int(rest_mag.t_us.iloc[-1])-int(rest_mag.t_us.iloc[0]))),4)
                     if int(rest_mag.t_us.iloc[-1])>int(rest_mag.t_us.iloc[0]) else None,
                'covariance_raw_counts_squared':mat(np.cov(mrest,rowvar=False),8),
                'std_after_linear_detrend_counts':vec(detrend(mrest,axis=0).std(axis=0,ddof=1)),
                'note':'REST magnetic variations include hand motion, cable motion and changing local magnetic field; not pure sensor noise.'}
        six=sixface(df)
        item['accelerometer_six_face']=six
        if six.get('status')=='estimated':acc_models.append((file.name,six))
        bmp=df[(df.pose.isin(['BMP','REST']))&(df.bmp_new==1)&(df.bmp_ok==1)]
        if len(bmp)>40:
            p=bmp.pressure_pa.to_numpy(float)
            item['barometer']={'n_fresh':len(p),'pressure_mean_pa':round(float(p.mean()),5),
                    'pressure_std_pa':round(float(p.std(ddof=1)),5),
                    'pressure_median_pa':round(float(np.median(p)),5),
                    'pressure_mad_pa':round(float(np.median(np.abs(p-np.median(p)))),5),
                    'pressure_range_pa':vec([p.min(),p.max()]),
                    'altitude_equivalent_std_m':round(float((287.05*(273.15+bmp.bmp_temp_c.median())/(9.80665*np.median(p)))*p.std(ddof=1)),6),
                    'note':'Altitude-equivalent std only under local ideal-gas hydrostatic linearization, not flight accuracy.'}
        mag=df[(df.pose=='MAG')&(df.mag_new==1)&(df.mag_ok==1)]
        if len(mag)>100:
            x=mag[['mx_raw','my_raw','mz_raw']].to_numpy(float)
            mag_sets[file.name]=x
            # Chronological holdout per substage; no leakage from samples in train to test.
            tr=[];te=[]
            for _,g in mag.groupby('mag_stage',sort=False):
                a=g[['mx_raw','my_raw','mz_raw']].to_numpy(float);q=int(len(a)*.8)
                tr.append(a[:q]);te.append(a[q:])
            train=np.vstack(tr);test=np.vstack(te)
            try:
                b,U,C,rad=fit_mag(x)
                bt,Ut,Ct,rt=fit_mag(train)
                diag=summarize_mag(x,b,U,C,rad)
                diag['training_radial_rms_fraction']=round(float(np.sqrt(np.mean(mag_r(train,bt,Ut)**2))),6)
                diag['heldout_radial_rms_fraction']=round(float(np.sqrt(np.mean(mag_r(test,bt,Ut)**2))),6)
                diag['stage_fresh_counts']={str(k):len(g) for k,g in mag.groupby('mag_stage')}
                item['magnetometer']=diag
            except Exception as e:item['magnetometer']={'error':str(e)}
        runs[file.name]=item
        print('Processed',file.name,'records',len(df),'mag',len(mag),flush=True)
    mag_gen={}
    magnetic_session_models=[]
    for name, item in runs.items():
        m=item.get('magnetometer',{})
        if 'hard_iron_offset_raw_counts' in m:
            magnetic_session_models.append((name,m))
    if len(magnetic_session_models)>=2:
        centers=np.array([m['hard_iron_offset_raw_counts'] for _,m in magnetic_session_models])
        mats=np.array([m['dimensionless_symmetric_soft_iron_matrix'] for _,m in magnetic_session_models])
        mag_between={'session_count':len(centers),
            'mean_bias_counts':vec(centers.mean(axis=0)),
            'between_session_std_bias_counts':vec(centers.std(axis=0,ddof=1)),
            'mean_dimensionless_matrix':mat(mats.mean(axis=0)),
            'between_session_std_matrix_entries':mat(mats.std(axis=0,ddof=1)),
            'note':'This is observed run-to-run variability, not a metrological confidence interval; field/mounting/cable changes confound it.'}
    else:mag_between={}
    if len(mag_sets)>=2:
        for src,x in mag_sets.items():
            try:
                b,U,C,r=fit_mag(x)
                gen={}
                for target,y in mag_sets.items():
                    rr=mag_r(y,b,U)
                    gen[target]={'radial_rms_fraction':round(float(np.sqrt(np.mean(rr*rr))),5),
                                 'radial_median_fraction':round(float(np.median(rr)),5)}
                mag_gen[src]=gen
            except Exception as e:mag_gen[src]={'error':str(e)}
        x=np.vstack(list(mag_sets.values()))
        try:
            b,U,C,r=fit_mag(x)
            pooled=summarize_mag(x,b,U,C,r)
        except Exception as e:pooled={'error':str(e)}
    else:pooled={}
    # Pooled rest bias only across independent runs; not inflated sample count.
    if rest_bias:
        means=np.asarray([r['gyro_bias_dps'] for _,r in rest_bias])
        temps=np.asarray([r['mpu_temp_mean_c'] for _,r in rest_bias])
        rest_summary={'sessions':len(means),'mean_gyro_bias_dps':vec(means.mean(axis=0)),
          'between_session_std_bias_dps':vec(means.std(axis=0,ddof=1)) if len(means)>1 else None,
          'temperature_span_rest_c':round(float(np.ptp(temps)),5),
          'gyro_temperature_model_status':'INSUFFICIENT_TEMPERATURE_RANGE' if np.ptp(temps)<3 else 'NEEDS_VALIDATION',
          'between_session_covariance_bias_dps2':mat(np.cov(means.T),10) if len(means)>1 else None}
    else:rest_summary={}
    if acc_models:
        b=np.asarray([a['recommended_diagonal_bias_mps2'] for _,a in acc_models])
        scale=np.asarray([np.diag(a['recommended_diagonal_matrix']) for _,a in acc_models])
        accel_summary={'runs':len(acc_models),'session_mean_bias_mps2':vec(b.mean(axis=0)),
            'between_session_std_bias_mps2':vec(b.std(axis=0,ddof=1)) if len(b)>1 else None,
            'session_mean_diagonal_scale':vec(scale.mean(axis=0)),
            'between_session_std_diagonal_scale':vec(scale.std(axis=0,ddof=1)) if len(scale)>1 else None,
            'note':'These between-run deviations reflect setup/pose mismatch and potentially sensor drift; not sensor-only uncertainty.'}
    else:accel_summary={}
    result={'schema':'IQ_HISTORICAL_DATA_AUDIT_V2','capture_source':'User uploaded captures(1).7z on Nov 2, 2026',
        'overall_warning':'Historical runs do not demonstrate current QMC ACK/connection. No live v8 magnetic samples supplied.',
        'unit_warning':'Raw magnetometer counts fitted without asserting µT calibration; QMC5883P full-scale is controlled by 0x0B, not 0x0A.',
        'runs':runs,'gyro_cross_session':rest_summary,'accelerometer_cross_session':accel_summary,
        'magnetometer_pooled_raw_counts':pooled,'magnetometer_between_session':mag_between,
        'magnetometer_cross_session_generalization':mag_gen}
    outdir.mkdir(parents=True,exist_ok=True)
    (outdir/'SENSOR_STATISTICS_AND_MATRICES.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('csv',nargs='+',type=Path);ap.add_argument('--outdir',type=Path,required=True);a=ap.parse_args()
    r=run_analysis(a.csv,a.outdir)
    print(json.dumps({'gyro_cross_session':r['gyro_cross_session'],'accelerometer_cross_session':r['accelerometer_cross_session'],
      'magnetometer_pooled':r['magnetometer_pooled_raw_counts']},indent=2))
if __name__=='__main__':main()
