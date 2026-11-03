#!/usr/bin/env python3
"""IntelQCopter offline temperature-aware sensor error propagation (NO flight output).

Loads an independently characterised thermal report plus optional standard
calibration, then annotates a raw CSV. A report marked EXPLORATORY_FIT is not
used to change measurements unless --apply-provisional is given explicitly.

The thermal *sample* covariance is not IMU process-noise density: it must not
be dropped unchanged into a Kalman Q matrix. This is a research diagnostic.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import numpy as np

GYRO_COLS=('gx_dps','gy_dps','gz_dps')
OUTPUTS=('gx_temp_corrected_dps','gy_temp_corrected_dps','gz_temp_corrected_dps',
         'gx_meas_variance_dps2','gy_meas_variance_dps2','gz_meas_variance_dps2',
         'pressure_temp_reference_corrected_pa','gyro_thermal_status',
         'bmp_thermal_status','temperature_outside_validated_span')


def finite(value):
    try:
        x=float(value)
    except (ValueError,TypeError):return math.nan
    return x if math.isfinite(x) else math.nan


def scalar_temperature_fit(model, temperature, allow_provisional=False):
    """Return value of beta0+beta1(T-Tref), or None if use prohibited.

    Both in-range operation and independent-session status are essential;
    a fitted curve is not automatically an approved flight correction.
    """
    if not allow_provisional or model.get('status') not in ('EXPLORATORY_FIT','REFERENCE_BASED_PROVISIONAL'):
        return None
    lo=finite(model.get('temperature_min_c'));hi=finite(model.get('temperature_max_c'))
    ref=finite(model.get('temperature_ref_c'))
    if not all(map(math.isfinite,(lo,hi,ref,temperature))) or not lo<=temperature<=hi:
        return None
    b=np.asarray(model['intercept'],float);k=np.asarray(model['slope_per_c'],float)
    if b.shape!=k.shape or not np.all(np.isfinite(b)) or not np.all(np.isfinite(k)):
        return None
    return b+k*(temperature-ref)


def normalised_innovation_squared(z,z_pred,pred_variance,meas_variance):
    """One-dimensional NIS; Chi-square(1) 99% threshold ~= 6.635.

    Inputs: actual observed scalar z, predicted value z_pred, predicted
    measurement variance pred_variance and sensor variance meas_variance.
    Must not use this helper to suppress sensor data in the calibration CSV.
    """
    vals=(z,z_pred,pred_variance,meas_variance)
    if not all(math.isfinite(v) for v in vals) or pred_variance<0 or meas_variance<=0:
        raise ValueError('invalid NIS inputs')
    return (z-z_pred)**2/(pred_variance+meas_variance)


def correct_one(row,thermal,calibration=None,apply_provisional=False):
    """Preserve invalid sensor data as NAN and return explicit reason flags."""
    ans={k:math.nan for k in OUTPUTS[:7]}
    ans.update(gyro_thermal_status='NO_VALID_MPU',bmp_thermal_status='NO_VALID_BMP',
               temperature_outside_validated_span='0')
    calibration=calibration or {}
    if row.get('mpu_ok')=='1':
        t=finite(row.get('mpu_temp_c'))
        g=np.asarray([finite(row.get(k)) for k in GYRO_COLS])
        if math.isfinite(t) and np.all(np.isfinite(g)):
            baseline=np.asarray(calibration.get('gyro',{}).get('bias_dps',[0.,0.,0.]),float)
            if baseline.shape!=(3,) or not np.all(np.isfinite(baseline)):
                baseline=np.zeros(3)
            bias_model=thermal.get('gyro_bias_model',{})
            model_bias=scalar_temperature_fit(bias_model,t,apply_provisional)
            if model_bias is not None and model_bias.shape==(3,):
                bias=model_bias;ans['gyro_thermal_status']='EXPLORATORY_CORRECTION_APPLIED'
            else:
                bias=baseline;ans['gyro_thermal_status']='CONSTANT_BIAS_ONLY'
                if apply_provisional and bias_model.get('status')=='EXPLORATORY_FIT':
                    ans['gyro_thermal_status']='MODEL_OUT_OF_RANGE_OR_MALFORMED'
                    ans['temperature_outside_validated_span']='1'
            corrected=g-bias
            ans.update(zip(OUTPUTS[:3],map(float,corrected)))
            logvariance=scalar_temperature_fit(thermal.get('gyro_log_variance_model',{}),t,apply_provisional)
            if logvariance is not None and logvariance.shape==(3,):
                # Clamp to avoid pathological exp overflow and overconfidence;
                # still only per-sample variance, not a process-noise density.
                variance=np.exp(np.clip(logvariance,math.log(1e-9),math.log(1e5)))
                ans.update(zip(OUTPUTS[3:6],map(float,variance)))
    if row.get('bmp_ok')=='1':
        tp=finite(row.get('bmp_temp_c'));pressure=finite(row.get('pressure_pa'))
        if math.isfinite(tp) and math.isfinite(pressure) and pressure>0:
            ans['pressure_temp_reference_corrected_pa']=pressure
            ans['bmp_thermal_status']='FACTORY_COMPENSATED_ONLY'
            pb=thermal.get('bmp_pressure_residual_model',{})
            # Never derive a pressure correction merely from pressure/temperature
            # correlation: a true pressure reference is mandatory.
            if pb.get('status')=='REFERENCE_BASED_PROVISIONAL':
                corr=scalar_temperature_fit(pb,tp,apply_provisional)
                if corr is not None and np.asarray(corr).ndim==0:
                    adjusted=pressure-float(corr)
                    if 30000<adjusted<115000:
                        ans['pressure_temp_reference_corrected_pa']=adjusted
                        ans['bmp_thermal_status']='EXTERNAL_REFERENCE_PROVISIONAL_CORRECTION'
                elif apply_provisional:
                    ans['bmp_thermal_status']='REFERENCE_MODEL_OUT_OF_RANGE'
                    ans['temperature_outside_validated_span']='1'
    return ans


def process_file(input_csv,thermal_json,output_csv,calibration_json=None,apply_provisional=False):
    thermal=json.loads(Path(thermal_json).read_text(encoding='utf-8'))
    calibration=json.loads(Path(calibration_json).read_text(encoding='utf-8')) if calibration_json else None
    source=Path(input_csv);dest=Path(output_csv)
    if source.resolve()==dest.resolve():raise ValueError('Never overwrite raw sensor captures')
    dest.parent.mkdir(parents=True,exist_ok=True)
    counts={};n=0
    with source.open(newline='',encoding='utf-8-sig') as fin,dest.open('w',newline='',encoding='utf-8') as fout:
        rd=csv.DictReader(fin)
        if not rd.fieldnames:raise ValueError('Input CSV header missing')
        fields=[*rd.fieldnames,*[x for x in OUTPUTS if x not in rd.fieldnames]]
        out=csv.DictWriter(fout,fieldnames=fields);out.writeheader()
        for r in rd:
            val=correct_one(r,thermal,calibration,apply_provisional)
            r.update(val);out.writerow(r)
            n+=1
            for key in ('gyro_thermal_status','bmp_thermal_status'):
                code=f'{key}:{val[key]}';counts[code]=counts.get(code,0)+1
    return {'schema':'IntelQCopter thermal uncertainty annotated capture v1',
            'rows':n,'source_csv':str(source),'output_csv':str(dest),
            'applied_provisional_calibration':bool(apply_provisional),
            'status_counts':counts,
            'warnings':[
                'Research-only annotation. Validate against independent held-out thermal sessions.',
                'Gyro variance is the measured discrete-sample variance: not Allan-derived continuous-time Q.',
                'No BMP temperature correction without external pressure references.',
                'Raw measurements remain unchanged in their original columns.',
                'Magnetometer thermal dependence cannot be calibrated from MPU/BMP die temperatures alone.']}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv',type=Path)
    ap.add_argument('--thermal-report',type=Path,required=True)
    ap.add_argument('--calibration-json',type=Path)
    ap.add_argument('--output-prefix',type=Path)
    ap.add_argument('--apply-provisional',action='store_true',help='Opt-in to unvalidated in-range models for OFFLINE comparison only')
    a=ap.parse_args()
    prefix=a.output_prefix or a.csv.with_name(a.csv.stem+'_thermal_qc')
    dest=Path(str(prefix)+'.csv')
    report=process_file(a.csv,a.thermal_report,dest,a.calibration_json,a.apply_provisional)
    reportpath=Path(str(prefix)+'.json')
    reportpath.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Annotated',report['rows'],'rows:',dest)
    print('Report:',reportpath)
    print('STATUS: offline diagnostic only; no firmware/FC calibration auto-applied.')

if __name__=='__main__':main()
