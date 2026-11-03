#!/usr/bin/env python3
"""Combine independent IntelQCopter calibration sessions with auditability.

Usage: python tools/combine_calibrations.py run1.csv run2.csv previous_calibration.json
       --output-prefix captures/ensemble

If raw CSVs exist, refit from raw (via calibrate.py) and use JSONs ONLY as
independent cross-checks, never as duplicate numerical observations.
If only JSON reports exist, equal-weight independent-session gyro/diagonal
accelerometer parameters, and select an existing magnetic candidate rather
than blindly averaging inconsistent 3x3 calibration matrices.
Do not combine hardware variants, coordinate mappings or magnetic environments.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent

def sha256(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()

def session_key(path):
    p=Path(path)
    return str(p.with_name(re.sub(r'_part\d+$','',p.stem)+p.suffix))

def as_vector(report, section, field):
    v=report.get(section,{}).get(field)
    if v is None:return None
    a=np.asarray(v,dtype=float)
    return a if a.shape==(3,) and np.all(np.isfinite(a)) else None

def aggregate_json_only(reports):
    out={'schema':'IntelQCopter calibration ensemble v1', 'method':'Equal weight per independent calibration session; NOT per repeated 500Hz sample',
         'warnings':[], 'contributing_reports':list(reports)}
    sessions={}
    for name,r in reports.items():
        inputs=r.get('input_files') or [r.get('input_file',name)]
        key=session_key(inputs[0]) if inputs else name
        if key in sessions:
            out['warnings'].append(f'{name}: same session as {sessions[key][0]}; skipped duplicated report')
        else:sessions[key]=(name,r)
    rs=[r for _,r in sessions.values()]
    def mean_by_sess(section,field):
        valid=[a for r in rs if (a:=as_vector(r,section,field)) is not None]
        if not valid:return None
        arr=np.asarray(valid)
        return arr.mean(axis=0).tolist(),np.std(arr,axis=0,ddof=1).tolist() if len(arr)>1 else None,len(arr)
    g=mean_by_sess('gyro','bias_dps')
    if g:
        out['gyro']={'bias_dps':g[0], 'std_between_independent_sessions_dps':g[1], 'independent_sessions':g[2],
                     'scale_matrix_3x3':np.eye(3).tolist(),
                     'note':'Unweighted mean of session bias estimates; not a gyro sensitivity calibration.'}
        temps=[r.get('gyro',{}).get('mpu_temperature_c_at_calibration') for r in rs]
        points=[(float(t),np.asarray(r['gyro']['bias_dps'],dtype=float)) for t,r in zip(temps,rs)
                if t is not None and as_vector(r,'gyro','bias_dps') is not None]
        if len(points)>=3 and max(t for t,_ in points)-min(t for t,_ in points)>=3:
            t=np.asarray([x[0] for x in points]); vals=np.asarray([x[1] for x in points]); tref=float(np.median(t))
            B=np.column_stack((np.ones(len(t)),t-tref)); beta=np.linalg.lstsq(B,vals,rcond=None)[0]
            out['gyro']['experimental_thermal_model']={'temperature_ref_c':tref,
                'bias_at_reference_dps':beta[0].tolist(),'slope_dps_per_c':beta[1].tolist(),
                'warning':'Temperature model inferred from saved summary data only; verify each REST segment was stationary.'}
        elif len(points)>1 and max(t for t,_ in points)-min(t for t,_ in points)>3:
            out['warnings'].append('Gyro sessions differ in temperature, but insufficient independent sessions for thermal regression; pooled bias may be misleading.')
    b=mean_by_sess('accelerometer','bias_mps2')
    if b:
        scale=[]
        for r in rs:
            M=np.asarray(r.get('accelerometer',{}).get('scale_matrix_3x3',[]),dtype=float)
            if M.shape==(3,3) and np.all(np.isfinite(M)):
                scale.append(np.diag(M))
        if scale:
            out['accelerometer']={'bias_mps2':b[0], 'scale_matrix_3x3':np.diag(np.mean(scale,axis=0)).tolist(),
                'session_count':len(scale),'std_between_sessions_mps2':b[1],
                'note':'Conservative per-axis only; never average experimental freehand full 3x3 fits as independent truth.'}
    magnetic=[(name,r['magnetometer']) for name,r in sessions.values() if 'magnetometer' in r]
    if magnetic:
        types={m.get('sensor') for _,m in magnetic}
        if len(types)>1:
            out['warnings'].append('Magnetometer sensor types differ: refusing automatic selection.')
        else:
            # Choose best calibration with measured coverage and residual, rather than
            # arithmetic-average rotations/ellipsoid matrices in incompatible gauge.
            def score(item):
                _,m=item
                quality=m.get('post_fit_radius_cv',math.inf)
                cov=m.get('orientation_coverage',{}).get('coverage_fraction_within_18_degrees',0)
                return (float(quality) if quality is not None else math.inf)+0.2*(1-float(cov))
            winner=min(magnetic,key=score)
            out['magnetometer']=winner[1]
            out['magnetometer_selected_report']=winner[0]
            out['warnings'].append('MAG matrix selected from best reported fit/coverage, not averaged; choose independent raw CSV refit for a better 3D ellipsoid and validate heading against a known reference.')
    bmps=[r.get('bmp280',{}) for r in rs if 'bmp280' in r]
    if bmps:
        out['bmp280']={'reference_pressure_pa':None,'session_references_pa':
                       [b.get('p0_pa_at_reference_height') for b in bmps],
                       'note':'BMP reference pressure depends on weather and height; recompute P0 on every power-up. No pooled absolute pressure reference.'}
    if len(rs)<2:out['warnings'].append('Only one independent session: no between-session reproducibility evidence.')
    return out

def combine(inputs,prefix):
    csvs=[];reports={};seen_hash=set(); warnings=[]
    for p in map(Path,inputs):
        if not p.exists():raise ValueError(f'File not found: {p}')
        if p.suffix.lower()=='.csv':
            digest=sha256(p)
            if digest in seen_hash:warnings.append(f'Duplicate identical raw CSV ignored: {p}')
            else:csvs.append(p.resolve());seen_hash.add(digest)
        elif p.suffix.lower()=='.json':
            obj=json.loads(p.read_text(encoding='utf-8-sig'))
            if not isinstance(obj,dict) or not any(k in obj for k in ('gyro','accelerometer','magnetometer','bmp280')):
                warnings.append(f'JSON does not contain recognizable sensor-calibration results; ignored: {p}')
            else:reports[str(p.resolve())]=obj
        else:raise ValueError(f'Unsupported file type {p.suffix}: {p}')
    if not csvs and not reports:raise ValueError('No recognized CSV or calibration JSON provided')
    prefix=Path(prefix).resolve();prefix.parent.mkdir(parents=True,exist_ok=True)
    if csvs:
        cmd=[sys.executable,str(HERE/'calibrate.py'),*map(str,csvs),'-o',str(prefix)]
        subprocess.run(cmd,check=True)
        j=Path(str(prefix)+'_calibration.json')
        out=json.loads(j.read_text())
        out['combination_provenance']={
            'raw_csvs_refit':[{'file':str(p),'sha256':sha256(p)} for p in csvs],
            'prior_reports_audit_only':list(reports),
            'method':'Refitted raw data once with calibrate.py; saved JSON reports did not double-count observations.',
            'caution':'Covariances and environment/temperature change between sessions; external reference validation still required.'}
        out['warnings'].extend(warnings)
        # Compare saved results as an independent consistency audit; do not count twice.
        audit={}
        for name,r in reports.items():
            audit[name]={}
            for sec,key in [('gyro','bias_dps'),('accelerometer','bias_mps2'),('magnetometer','hard_iron_offset_uT')]:
                old=as_vector(r,sec,key);new=as_vector(out,sec,key)
                if old is not None and new is not None:
                    audit[name][sec+'_offset_difference']=float(np.linalg.norm(old-new))
        out['prior_report_parameter_differences']=audit
        j.write_text(json.dumps(out,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        return j
    out=aggregate_json_only(reports)
    out['warnings'].extend(warnings)
    out['combination_provenance']={'summary_reports':list(reports),'method':'Summary-only conservative ensemble. Raw CSV refit strongly preferred.'}
    target=Path(str(prefix)+'_ensemble.json')
    target.write_text(json.dumps(out,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return target

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('inputs',nargs='+',type=Path,help='Completed .csv and/or *_calibration.json files')
    p.add_argument('-o','--output-prefix',default='captures/combined_best')
    a=p.parse_args()
    try:result=combine(a.inputs,a.output_prefix)
    except (ValueError,subprocess.CalledProcessError,OSError) as err:p.error(str(err))
    print('Combined report:',result)
    if result.name.endswith('_calibration.json'):
        print('C++ constants:',str(result).replace('_calibration.json','_calibration.h'))
    else: print('Summary-only report: do NOT use directly as flight calibration constants without verification.')

if __name__=='__main__':main()
