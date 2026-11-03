#!/usr/bin/env python3
"""Thermal calibration audit for independent IntelQCopter sessions.

Provides session-level gyro bias/variance vs MPU DIE temperature, accelerometer
bias/scale vs temperature if full six-face sessions, and reference-based BMP280
pressure residual vs BMP DIE temperature. No reference => NO pressure correction.

Usage:
 py tools/thermal_characterization.py captures/cold.csv captures/room.csv captures/warm.csv \
    --output-prefix captures/thermal_audit
 py tools/thermal_characterization.py captures/*.csv \
    --reference-file thermal_pressure_references.json
"""
from __future__ import annotations
import argparse, csv, json, math, re
from collections import defaultdict
from pathlib import Path
import numpy as np

GYRO_COLS=('gx_dps','gy_dps','gz_dps')
ACC_COLS=('ax_mps2','ay_mps2','az_mps2')


def session_name(path:Path):
    return re.sub(r'_part\d+$','',Path(path).stem)


def _float(r,k):
    try:
        v=float(r[k]);return v if math.isfinite(v) else math.nan
    except (KeyError,ValueError,TypeError):return math.nan


def _extract(paths):
    from attempt_filter import only_latest
    groups=defaultdict(lambda:{'rest':[], 'bmp':[], 'faces':[], 'files':[]})
    full=[]
    for file in paths:
        file=Path(file)
        if not file.exists():raise FileNotFoundError(file)
        name=session_name(file)
        groups[name]['files'].append(str(file))
        with file.open(newline='',encoding='utf-8-sig') as fh:
            for r in csv.DictReader(fh):
                r['_source_file']=str(file.resolve())
                full.append(r)
    for r in only_latest(full):
        if r.get('kind')!='D' or r.get('phase_accepted','1')!='1':continue
        name=session_name(Path(r['_source_file']))
        g=groups[name]
        pose=r.get('pose')
        if r.get('mpu_ok')=='1' and pose=='REST':
            vals=[_float(r,v) for v in (*GYRO_COLS,*ACC_COLS,'mpu_temp_c')]
            if all(math.isfinite(x) for x in vals):g['rest'].append(vals)
        if r.get('bmp_ok')=='1' and r.get('bmp_new')=='1' and pose in ('REST','BMP'):
            vals=(_float(r,'bmp_temp_c'),_float(r,'pressure_pa'))
            if all(math.isfinite(x) for x in vals):g['bmp'].append(vals)
        if r.get('mpu_ok')=='1' and pose in ('+X','-X','+Y','-Y','+Z','-Z'):
            g['faces'].append((pose,[_float(r,'mpu_temp_c'),*[_float(r,c) for c in ACC_COLS]]))
    return groups


def linear_fit(points, min_session=3, min_span=3., allow_uncertainty=True):
    """Equal weight per independent session; normal-equation least squares.

    A fitted slope is not an independently validated thermal correction.
    """
    if len(points)<min_session:return {'status':'INSUFFICIENT_INDEPENDENT_SESSIONS','count':len(points),'required':min_session}
    t=np.asarray([p[0] for p in points]);y=np.asarray([p[1] for p in points],float)
    spread=float(np.ptp(t))
    if spread<min_span:return {'status':'INSUFFICIENT_TEMPERATURE_SPAN','span_c':spread,'required_span_c':min_span}
    ref=float(np.median(t));X=np.c_[np.ones(len(t)),t-ref]
    beta=np.linalg.lstsq(X,y,rcond=None)[0]
    err=y-X@beta
    sigma=np.sqrt(np.sum(err**2,axis=0)/max(1,len(t)-2))
    # Session-level leave-one-session-out prediction, meaningful when n >= 4.
    cv=[]
    if len(t)>=4:
        for i in range(len(t)):
            keep=np.arange(len(t))!=i
            b=np.linalg.lstsq(X[keep],y[keep],rcond=None)[0]
            cv.append(y[i]-X[i]@b)
    info={'status':'EXPLORATORY_FIT', 'session_count':len(t),
          'temperature_min_c':float(np.min(t)),'temperature_max_c':float(np.max(t)),
          'temperature_ref_c':ref,'intercept':np.asarray(beta[0]).tolist(),
          'slope_per_c':np.asarray(beta[1]).tolist(),
          'session_residual_std':np.asarray(sigma).tolist(),
          'leave_one_session_out_rmse':(np.sqrt(np.mean(np.asarray(cv)**2,axis=0)).tolist() if cv else None),
          'limitations':'Not flight validated. No extrapolation beyond temperature range. Sessions must be independent and thermally stable.'}
    return info


def analyze(paths, reference_file=None):
    groups=_extract(paths)
    ref_map={}
    if reference_file:
        raw=json.loads(Path(reference_file).read_text(encoding='utf-8'))
        for s in raw.get('sessions',[]):
            if not math.isfinite(float(s['reference_pressure_pa'])):continue
            ref_map[session_name(Path(s['csv']))]=float(s['reference_pressure_pa'])
    sessions=[]
    for name,entry in sorted(groups.items()):
        rest=np.asarray(entry['rest'],dtype=float).reshape((-1,7))
        bmp=np.asarray(entry['bmp'],dtype=float).reshape((-1,2))
        obj={'name':name,'files':entry['files'],'rest_samples':len(rest),'bmp_fresh_samples':len(bmp)}
        if len(rest)>=100:
            x=rest[:,:3]; accel=rest[:,3:6];tmp=rest[:,6]
            # Angular motion and warm-up temperature drift both invalidate a
            # stationary bias estimate. Large noise => reject from thermal fit.
            gstd=np.std(x,axis=0,ddof=1); astd=np.std(accel,axis=0,ddof=1)
            obj.update({'mpu_die_temp_median_c':float(np.median(tmp)),
                'mpu_die_temp_span_c':float(np.ptp(tmp)),
                'gyro_mean_dps':np.mean(x,axis=0).tolist(),
                'gyro_std_dps':gstd.tolist(),
                'accel_std_mps2':astd.tolist(),
                'gyro_noise_cov_dps2':np.cov(x,rowvar=False).tolist(),
                'stationary_quality':('PASS' if max(gstd)<0.35 and max(astd)<0.5
                                      and np.ptp(tmp)<1.5 else 'REJECT_MOVEMENT_OR_THERMAL_TRANSIENT')})
        if len(bmp)>=30:
            obj.update({'bmp_die_temp_median_c':float(np.median(bmp[:,0])),
                'bmp_die_temp_span_c':float(np.ptp(bmp[:,0])),
                'pressure_median_pa':float(np.median(bmp[:,1])),
                'pressure_std_pa':float(np.std(bmp[:,1],ddof=1)),
                'pressure_reference_pa':ref_map.get(name)})
            if name in ref_map:obj['pressure_residual_pa']=float(np.median(bmp[:,1])-ref_map[name])
        sessions.append(obj)
    gyro=[(s['mpu_die_temp_median_c'],s['gyro_mean_dps']) for s in sessions if s.get('stationary_quality')=='PASS']
    gyro_noise=[(s['mpu_die_temp_median_c'],np.log(np.maximum(np.diag(s['gyro_noise_cov_dps2']),1e-12)).tolist())
          for s in sessions if s.get('stationary_quality')=='PASS']
    bmp=[(s['bmp_die_temp_median_c'],s['pressure_residual_pa']) for s in sessions if 'pressure_residual_pa' in s]
    gyro_model=linear_fit(gyro)
    noise_model=linear_fit(gyro_noise,min_session=4,min_span=4.)
    bp_model=linear_fit(bmp,min_session=3,min_span=4.)
    if bp_model['status']=='EXPLORATORY_FIT':
        bp_model['status']='REFERENCE_BASED_PROVISIONAL'
        bp_model['units']='Pa and Pa/°C'
        bp_model['note']='Reference pressure must be traceable/known at each session. Validate on withheld thermal runs before use.'
    report={'schema':'IntelQCopter thermal characterization v1', 'sessions':sessions,
        'gyro_bias_model':gyro_model,'gyro_log_variance_model':noise_model,
        'bmp_pressure_residual_model':bp_model,
        'warnings':['Temperature is the sensors DIE temperature, not calibrated ambient air temperature.',
        'MPU gyro bias regression uses independent REST sessions only; warm-up transient invalidates slope.',
        'Do not use gyro noise slope for filter tuning until cross-validated and confirmed on the board.',
        'BMP280 already uses Bosch factory temperature compensation. NO correction without independent pressure reference.',
        'Hysteresis: heating and cooling may produce different bias at the same temperature.',
        'The calibration software does not drive motors or control the aircraft.']}
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv',nargs='+',type=Path)
    ap.add_argument('--reference-file',type=Path,help='JSON file of independently measured true pressures')
    ap.add_argument('--output-prefix',type=Path,default=Path('captures/thermal_characterization'))
    ns=ap.parse_args()
    r=analyze(ns.csv,ns.reference_file)
    dest=Path(str(ns.output_prefix)+'_thermal.json');dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    print('Saved',dest)
    print('Gyro:',r['gyro_bias_model']['status'],' BMP:',r['bmp_pressure_residual_model']['status'])
    print('Never automatically apply an exploratory temperature regression as a flight calibration.')

if __name__=='__main__':main()
