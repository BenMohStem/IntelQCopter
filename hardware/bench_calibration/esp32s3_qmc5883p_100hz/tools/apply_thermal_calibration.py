#!/usr/bin/env python3
"""Offline temperature-aware IMU correction; PRESERVE raw CSV, no flight outputs.

Uses the calibration JSON created by calibrate.py. The thermal model is
EXPERIMENTAL and must be explicitly enabled with --allow-experimental.
Out-of-range temperatures do not extrapolate: rows are tagged and uncorrected.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import numpy as np


def correct(r,cal,allow=False):
    """Return corrected gyro/accel and honest model status for one CSV row."""
    out={k:math.nan for k in ('gx_cal_dps','gy_cal_dps','gz_cal_dps',
                              'ax_cal_mps2','ay_cal_mps2','az_cal_mps2')}
    out['thermal_model_status']='NOT_APPLICABLE'
    if r.get('mpu_ok')!='1':return out
    try:
        T=float(r['mpu_temp_c']);g=np.array([float(r[k]) for k in ('gx_dps','gy_dps','gz_dps')]);
        a=np.array([float(r[k]) for k in ('ax_mps2','ay_mps2','az_mps2')]);
        if not (math.isfinite(T) and np.all(np.isfinite(g)) and np.all(np.isfinite(a))):return out
    except (KeyError,TypeError,ValueError):return out
    gyro=cal.get('gyro',{})
    accel=cal.get('accelerometer',{})
    base=np.asarray(gyro.get('bias_dps',[0,0,0]),dtype=float)
    tdata=cal.get('temperature_analysis',{})
    gt=tdata.get('gyro_bias_versus_mpu_temperature',{})
    status='CONSTANT_BIAS_ONLY'
    if gt.get('status')=='ESTIMATED_NOT_VALIDATED' and allow:
        tmin=min(x['temperature_c'] for x in gt['sessions'])
        tmax=max(x['temperature_c'] for x in gt['sessions'])
        if tmin<=T<=tmax:
            base=np.asarray(gt['intercept_at_reference'])+np.asarray(gt['slope_per_c'])*(T-gt['reference_temp_c'])
            status='EXPERIMENTAL_GYRO_TEMP_APPLIED'
        else:
            status='TEMP_OUT_OF_TRAINING_RANGE__CONSTANT_GYRO_USED'
    elif gt.get('status')=='ESTIMATED_NOT_VALIDATED':
        status='EXPERIMENTAL_GYRO_AVAILABLE_BUT_NOT_ENABLED'
    gg=g-base
    b=np.asarray(accel.get('bias_mps2',[0,0,0]),dtype=float)
    S=np.asarray(accel.get('scale_matrix_3x3',np.eye(3)),dtype=float)
    at=tdata.get('accelerometer_bias_versus_mpu_temperature',{})
    ast=tdata.get('accelerometer_scale_versus_mpu_temperature',{})
    if (allow and at.get('status')=='ESTIMATED_NOT_VALIDATED'
       and ast.get('status')=='ESTIMATED_NOT_VALIDATED'):
        amin=min(x['temperature_c'] for x in at['sessions'])
        amax=max(x['temperature_c'] for x in at['sessions'])
        if amin<=T<=amax:
            b=np.asarray(at['intercept_at_reference'])+np.asarray(at['slope_per_c'])*(T-at['reference_temp_c'])
            ss=np.asarray(ast['intercept_at_reference'])+np.asarray(ast['slope_per_c'])*(T-ast['reference_temp_c'])
            if np.all((ss>0.5)&(ss<1.5)):
                S=np.diag(ss)
                status+='|EXPERIMENTAL_ACCEL_TEMP_APPLIED'
            else:status+='|ACCEL_TEMP_SCALE_UNPHYSICAL_BASE_USED'
        else:status+='|ACCEL_TEMP_OUT_OF_RANGE_BASE_USED'
    aa=S@(a-b)
    out.update(zip(('gx_cal_dps','gy_cal_dps','gz_cal_dps'),[float(x) for x in gg]))
    out.update(zip(('ax_cal_mps2','ay_cal_mps2','az_cal_mps2'),[float(x) for x in aa]))
    out['thermal_model_status']=status
    return out


def apply(path,report,output,allow=False):
    cal=json.loads(Path(report).read_text(encoding='utf-8'))
    path=Path(path);out=Path(output)
    if path.resolve()==out.resolve():raise ValueError('Refusing to overwrite raw CSV')
    out.parent.mkdir(parents=True,exist_ok=True)
    fields=['gx_cal_dps','gy_cal_dps','gz_cal_dps','ax_cal_mps2','ay_cal_mps2','az_cal_mps2','thermal_model_status']
    with path.open(newline='',encoding='utf-8-sig') as fin,out.open('w',newline='',encoding='utf-8') as fout:
        rdr=csv.DictReader(fin); cols=list(rdr.fieldnames or [])
        if not cols:raise ValueError('Missing header')
        writer=csv.DictWriter(fout,fieldnames=cols+[x for x in fields if x not in cols]);writer.writeheader()
        count=0
        for row in rdr:
            row.update(correct(row,cal,allow));writer.writerow(row);count+=1
    return count


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv',type=Path);p.add_argument('calibration_json',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--allow-experimental',action='store_true',help='Explicitly opt into unvalidated temperature slope; no extrapolation')
    args=p.parse_args()
    dest=args.output or args.csv.with_name(args.csv.stem+'_thermal_corrected.csv')
    n=apply(args.csv,args.calibration_json,dest,args.allow_experimental)
    print('Saved',n,'rows to',dest)
    print('This output is research/offline only. Compare with independent reference before flight use.')

if __name__=='__main__':main()
