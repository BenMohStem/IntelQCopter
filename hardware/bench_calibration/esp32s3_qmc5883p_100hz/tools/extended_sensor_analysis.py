#!/usr/bin/env python3
"""Extended, read-only sensor uncertainty analysis for IntelQCopter CSV sessions.

Adds Allan deviation (from resampled stationary readings), block-bootstrap gyro
bias uncertainty, acceleration-pose alignment, magnetometer calibration matrix
spectra, barometric short-term variation, and independent-session consistency.

This is an OFFLINE historical study. It cannot fix a QMC I2C address NACK,
identify unknown chips, or establish absolute heading accuracy.

Run: py -3 tools/extended_sensor_analysis.py captures/*.csv --output analysis
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import pandas as pd

AXES=('x','y','z')
FACES=('+X','-X','+Y','-Y','+Z','-Z')
G=9.80665


def rounded(value, precision=8):
    if isinstance(value,np.ndarray): return rounded(value.tolist(),precision)
    if isinstance(value, (list,tuple)): return [rounded(x,precision) for x in value]
    if isinstance(value, dict): return {str(k):rounded(v,precision) for k,v in value.items()}
    if value is None:return None
    if isinstance(value, (int,np.integer)):return int(value)
    if isinstance(value, (float,np.floating)):
        x=float(value)
        return round(x,precision) if np.isfinite(x) else None
    return value


def independent_block_mean_ci(values, block_length=250, trials=800, seed=283):
    """Nonparametric block-bootstrap estimate; does not cover orientation bias."""
    a=np.asarray(values, dtype=float)
    a=a[np.all(np.isfinite(a),axis=1)]
    blocks=np.array([a[k:k+block_length].mean(axis=0) for k in
                     range(0,len(a)-block_length+1,block_length)])
    if len(blocks)<5:return {'status':'insufficient independent time blocks'}
    rng=np.random.default_rng(seed)
    draws=rng.integers(len(blocks),size=(trials,len(blocks)))
    means=blocks[draws].mean(axis=1)
    return {'block_samples':block_length,'nonoverlapping_blocks':len(blocks),
            'mean_95pct_ci':rounded(np.percentile(means,[2.5,97.5],axis=0)),
            'method':'nonparametric bootstrap of consecutive block means',
            'warning':'Correlated blocks, heating, motion and time drift can widen true uncertainty.'}


def allan_deviation_rates(t_us, values, taus_s=(0.01,0.02,0.05,0.1,0.2,0.5,1,2)):
    """Non-overlapping Allan deviation on linearly interpolated uniform 2ms grid.

    Repeated sampled values can look smoother under interpolation; restrict
    physical conclusions to experiments with near-uniform acquisition timing.
    """
    t=np.asarray(t_us,dtype=float)
    values=np.asarray(values,dtype=float)
    good=np.isfinite(t)&np.isfinite(values).all(axis=1)
    t=t[good];values=values[good]
    if len(t)<300 or len(np.unique(t))<300:return []
    t=(t-t[0])*1e-6
    order=np.argsort(t)
    t=t[order];values=values[order]
    valid=np.r_[True,np.diff(t)>0]
    t=t[valid];values=values[valid]
    # Choose a uniform grid at the observed median acquisition interval;
    # do not imply that the raw sensor actually updated at that cadence.
    spacing=float(np.median(np.diff(t)))
    if spacing<.0008 or spacing>.010:return []
    uniform=np.arange(t[0],t[-1],spacing)
    if len(uniform)>250000:return []
    x=np.column_stack([np.interp(uniform,t,values[:,k]) for k in range(3)])
    items=[]
    for tau in taus_s:
        m=max(1,int(round(tau/spacing)))
        n=len(x)//m
        if n<8: continue
        avg=x[:n*m].reshape(n,m,3).mean(axis=1)
        sigma=np.sqrt(.5*np.mean(np.diff(avg,axis=0)**2,axis=0))
        items.append({'tau_s':round(m*spacing,7),'allan_std':rounded(sigma),
                      'averaged_blocks':n})
    return items


def normalize_attempts(df):
    if 'kind' in df.columns:df=df[df.kind=='D'].copy()
    if all(k in df.columns for k in ('phase_attempt','pose','mag_stage','phase_accepted')):
        latest=df.groupby(['pose','mag_stage'],dropna=False)['phase_attempt'].transform('max')
        df=df[(df.phase_attempt==latest)&(df.phase_accepted==1)].copy()
    return df


def analyze_one_csv(file):
    cols=['kind','pose','mag_stage','phase_attempt','phase_accepted','mpu_ok',
          'gx_dps','gy_dps','gz_dps','ax_mps2','ay_mps2','az_mps2','t_us',
          'mpu_temp_c','bmp_temp_c','pressure_pa','bmp_new','bmp_ok']
    df=pd.read_csv(file,usecols=lambda x:x in cols,low_memory=False)
    df=normalize_attempts(df)
    result={'filename':file.name,'accepted_rows':int(len(df))}
    if len(df)==0:return result
    gyro_cols=['gx_dps','gy_dps','gz_dps']
    accel_cols=['ax_mps2','ay_mps2','az_mps2']
    rest=df[(df.pose=='REST')&(df.mpu_ok==1)].copy()
    if len(rest)>=300:
        g=rest[gyro_cols].to_numpy(float)
        a=rest[accel_cols].to_numpy(float)
        cc=np.corrcoef(g,rowvar=False)
        eigenvalues=np.linalg.eigvalsh(np.cov(g,rowvar=False))
        result['gyro_rest']={
            'sample_count':len(g),'bias_dps':rounded(g.mean(axis=0)),
            'std_dps':rounded(g.std(axis=0,ddof=1)),
            'covariance_dps2':rounded(np.cov(g,rowvar=False)),
            'correlation_matrix':rounded(cc),
            'covariance_eigenvalues_dps2':rounded(eigenvalues),
            'bias_block_bootstrap_ci':independent_block_mean_ci(g),
            'allan_deviation_dps':allan_deviation_rates(rest.t_us.to_numpy(float),g),
            'temperature_minmax_c':rounded([rest.mpu_temp_c.min(),rest.mpu_temp_c.max()]),
            'temperature_delta_c':rounded(rest.mpu_temp_c.max()-rest.mpu_temp_c.min()),
            'no_bias_yaw_drift_60s_degrees':rounded(g.mean(axis=0)[2]*60),
            'note':'Bias-drift angle assumes fixed z gyro bias and no other corrections; not a measured heading error.'}
        result['accel_rest']={
            'mean_mps2':rounded(a.mean(axis=0)),
            'std_mps2':rounded(a.std(axis=0,ddof=1)),
            'covariance_m4_s4':rounded(np.cov(a,rowvar=False)),
            'mean_vector_norm_mps2':rounded(np.linalg.norm(a.mean(axis=0))),
            'allan_deviation_mps2':allan_deviation_rates(rest.t_us.to_numpy(float),a)}
    face_means={}
    for pose in FACES:
        sub=df[(df.pose==pose)&(df.mpu_ok==1)]
        if len(sub)>=200:face_means[pose]=sub[accel_cols].to_numpy(float).mean(axis=0)
    if len(face_means)==6:
        b=np.array([(face_means[FACES[2*i]][i]+face_means[FACES[2*i+1]][i])/2 for i in range(3)])
        sc=np.array([2*G/(face_means[FACES[2*i]][i]-face_means[FACES[2*i+1]][i]) for i in range(3)])
        D=np.diag(sc)
        face_quality={}
        for ix,pose in enumerate(FACES):
            ideal=np.zeros(3);ideal[ix//2]=G*(1 if ix%2==0 else -1)
            actual=D@(face_means[pose]-b)
            cosine=np.clip(actual@ideal/max(1e-10,np.linalg.norm(actual)*G),-1,1)
            face_quality[pose]={
                'corrected_norm_mps2':rounded(np.linalg.norm(actual)),
                'angle_from_ideal_degrees':rounded(np.rad2deg(np.arccos(cosine)),4),
                'full_3d_residual_mps2':rounded(np.linalg.norm(actual-ideal))}
        result['six_face_model']={'bias_mps2':rounded(b),'scale_diagonal':rounded(sc),
                                  'matrix':rounded(D),'face_quality':face_quality,
                                  'caution':'Angles and residuals depend on freehand pose alignment; not rigorous cross-axis metrology.'}
    baro=df[(df.pose.isin(['REST','BMP']))&(df.bmp_new==1)&(df.bmp_ok==1)]
    if len(baro)>=30:
        p=baro.pressure_pa.to_numpy(float)
        temp=baro.bmp_temp_c.to_numpy(float)
        t=baro.t_us.to_numpy(float)
        dp=np.diff(p)
        med=float(np.median(p))
        temp_c=float(np.nanmedian(temp))
        height_per_pa=287.05*(temp_c+273.15)/(9.80665*med)
        duration_s=float((t[-1]-t[0])*1e-6)
        drift=float(np.polyfit((t-t[0])*1e-6,p,1)[0]) if duration_s>1 else None
        white_proxy=float(np.std(dp,ddof=1)/np.sqrt(2))
        result['barometer']={'fresh_samples':len(p),
            'pressure_std_pa':rounded(np.std(p,ddof=1)),
            'first_difference_std_pa':rounded(np.std(dp,ddof=1)),
            'white_noise_proxy_pa':rounded(white_proxy),
            'equivalent_height_std_m':rounded(np.std(p,ddof=1)*height_per_pa),
            'pressure_drift_pa_per_s':rounded(drift),
            'height_equivalent_drift_m_per_s':rounded(drift*height_per_pa if drift is not None else None),
            'note':'First-difference std/sqrt(2) assumes successive white-noise samples. Temperature/weather/drift and sensor filtering may violate that model.'}
    return result


def analyze_reference_matrices(json_file):
    data=json.load(open(json_file,encoding='utf-8'))
    derived={'source':str(json_file),'magnetometer':{},'accelerometer':{},'between_sessions':{}}
    for name,run in data.get('runs',{}).items():
        m=run.get('magnetometer',{})
        if 'dimensionless_symmetric_soft_iron_matrix' not in m:continue
        A=np.array(m['dimensionless_symmetric_soft_iron_matrix'],dtype=float)
        eig,Q=np.linalg.eigh(A)
        derived['magnetometer'][name]={
            'softiron_eigenvalues':rounded(eig),
            'condition_number':rounded(eig[-1]/eig[0]),
            'determinant':rounded(np.linalg.det(A)),
            'symmetric_residual':rounded(np.linalg.norm(A-A.T)),
            'offset_raw_counts':m['hard_iron_offset_raw_counts'],
            'matrix':m['dimensionless_symmetric_soft_iron_matrix'],
            'rotation_ambiguity':'No absolute compass orientation can be inferred from magnetic magnitude-only ellipsoid fitting.',
            'principal_axis_vectors_columns':rounded(Q)}
        a=run.get('accelerometer_six_face',{})
        if 'recommended_diagonal_matrix' in a:
            derived['accelerometer'][name]={
                'scale_matrix':a['recommended_diagonal_matrix'],
                'bias_mps2':a['recommended_diagonal_bias_mps2'],
                'scale_condition_number':rounded(np.linalg.cond(np.array(a['recommended_diagonal_matrix']))),
                'face_rms_residual_mps2':a['diagonal_face_rms_residual_mps2']}
    b=data.get('magnetometer_between_session',{})
    if b.get('session_count',0)>=2:
        centers=np.array([d['offset_raw_counts'] for d in derived['magnetometer'].values()],dtype=float)
        mats=np.array([d['matrix'] for d in derived['magnetometer'].values()],dtype=float)
        derived['between_sessions']={
            'offset_covariance_counts_squared':rounded(np.cov(centers,rowvar=False)),
            'relative_matrix_spread_frobenius':rounded([float(np.linalg.norm(A-mats.mean(axis=0))/np.linalg.norm(mats.mean(axis=0))) for A in mats]),
            'warning':'Only three full magnetic sessions; uncertainty includes mounting, cable, and environmental changes.'}
    return derived


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('csv',nargs='*',type=Path)
    ap.add_argument('--reference',type=Path,help='Existing SENSOR_STATISTICS_AND_MATRICES.json')
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    runs={}
    for p in args.csv:
        try:
            r=analyze_one_csv(p)
        except Exception as e:
            r={'error':str(e),'filename':p.name}
        runs[p.name]=r
        print('Audited',p.name,r.get('accepted_rows'),'rows',flush=True)
    result={'schema':'IQ_EXTENDED_SENSOR_AUDIT_V1',
            'limitations':['Historical CSVs do not show current MAG address 0x2C acknowledging.',
                           'Allan deviation estimated from irregularly timed resampled records; experimental stability is not guaranteed.',
                           'Temperature span does not support independent thermal calibration.',
                           'Six-face hand alignment limits absolute accuracy.',
                           'Magnetic ellipsoid magnitude fit has rotational ambiguity.'],
            'runs':runs}
    if args.reference:result['reference_models']=analyze_reference_matrices(args.reference)
    (args.output/'EXTENDED_SENSOR_STATISTICS.json').write_text(
        json.dumps(rounded(result),indent=2,ensure_ascii=False),encoding='utf-8')
    summary=[]
    models=result.get('reference_models',{})
    for name,run in runs.items():
        gr=run.get('gyro_rest',{})
        br=run.get('barometer',{})
        ar=run.get('six_face_model',{})
        mr=models.get('magnetometer',{}).get(name,{})
        row={'session':name,'accepted_rows':run.get('accepted_rows',0),
             'gyro_bias_x_dps':gr.get('bias_dps',[None]*3)[0],
             'gyro_bias_y_dps':gr.get('bias_dps',[None]*3)[1],
             'gyro_bias_z_dps':gr.get('bias_dps',[None]*3)[2],
             'gyro_std_x_dps':gr.get('std_dps',[None]*3)[0],
             'gyro_std_y_dps':gr.get('std_dps',[None]*3)[1],
             'gyro_std_z_dps':gr.get('std_dps',[None]*3)[2],
             'bmp_pressure_std_pa':br.get('pressure_std_pa'),
             'bmp_white_noise_proxy_pa':br.get('white_noise_proxy_pa'),
             'accel_scale_x':ar.get('scale_diagonal',[None]*3)[0],
             'accel_scale_y':ar.get('scale_diagonal',[None]*3)[1],
             'accel_scale_z':ar.get('scale_diagonal',[None]*3)[2],
             'mag_shape_condition_number':mr.get('condition_number')}
        summary.append(row)
    if summary:
        with (args.output/'EXTENDED_UNCERTAINTY_SUMMARY.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(summary[0]))
            writer.writeheader();writer.writerows(summary)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(8.5,5.5))
        for name,run in runs.items():
            arr=run.get('gyro_rest',{}).get('allan_deviation_dps',[])
            if arr:
                ax.loglog([x['tau_s'] for x in arr],
                          [x['allan_std'][2] for x in arr],marker='o',label=name.replace('iq_500hz_','')[-18:])
        ax.grid(True,which='both',alpha=.3)
        ax.set_xlabel('Averaging interval τ [s]')
        ax.set_ylabel('Allan deviation of gyro Z [°/s]')
        ax.set_title('Historical stationary gyro stability (time-resampled, exploratory)')
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(args.output/'GYRO_Z_ALLAN_DEVIATION.png',dpi=160)
        plt.close(fig)
    except Exception as e:
        print('Plot unavailable:',e,flush=True)
    print('Saved',args.output/'EXTENDED_SENSOR_STATISTICS.json')

if __name__=='__main__':main()
