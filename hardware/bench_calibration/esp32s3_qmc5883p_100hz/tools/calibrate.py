#!/usr/bin/env python3
"""Estimate gyro offset, accelerometer bias/scale and 3x3 affine fit,
HMC5883L hard/soft iron and BMP reference from the 500Hz CSV recorder.

All 500Hz records remain in the CSV; analysis retains downsampled stationary
IMU rows and every fresh magnetometer/barometer sample to limit RAM usage.

Dependencies: pip install numpy scipy
Usage: python tools/calibrate.py captures/intel_cal_YYYYMMDD_HHMMSS.csv
Output: <capture>_calibration.json and <capture>_calibration.h

This is an offline calibration estimator, NOT proof of flight readiness.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import re
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

G = 9.80665
POSES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
AXES = ['x', 'y', 'z']


def arr(rows, name, pose=None, predicate=None):
    result = []
    for r in rows:
        if pose is not None and r['pose'] != pose:
            continue
        if predicate is not None and not predicate(r):
            continue
        try:
            value = float(r[name])
            if math.isfinite(value):
                result.append(value)
        except (KeyError, TypeError, ValueError):
            pass
    return np.array(result, dtype=float)


def triplets(rows, columns, pose=None, predicate=None):
    out=[]
    for r in rows:
        if pose is not None and r['pose'] != pose:
            continue
        if predicate is not None and not predicate(r):
            continue
        try:
            v=[float(r[k]) for k in columns]
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(x) for x in v):
            out.append(v)
    return np.asarray(out, dtype=float).reshape((-1, 3))


def accepted(r): return r.get('phase_accepted', '1') == '1'
def has_mpu(r): return r['mpu_ok'] == '1' and accepted(r)
def has_bmp_new(r): return r['bmp_ok'] == '1' and r['bmp_new'] == '1' and accepted(r)
def has_mag_new(r): return r['mag_ok'] == '1' and r['mag_new'] == '1' and accepted(r)


def vec(x): return [float(v) for v in np.asarray(x).ravel()]
def mat(x): return [vec(row) for row in np.asarray(x)]


def fit_accel(rows):
    raw_cols=['ax_mps2', 'ay_mps2', 'az_mps2']
    means={}; errors={}; counts={}; means_norm={}; face_stds={}
    for p in POSES:
        data=triplets(rows,raw_cols,pose=p,predicate=has_mpu)
        if len(data)<100:
            raise ValueError(f'Pose {p}: only {len(data)} valid samples; need >=100')
        mean=np.mean(data,axis=0)
        means[p]=mean
        counts[p]=len(data)
        face_stds[p]=vec(np.std(data,axis=0))
        means_norm[p]=float(np.linalg.norm(mean))
        target_index=AXES.index(p[1].lower())
        if mean[target_index] * (1 if p[0]=='+' else -1) < 0.5 * G:
            errors[p]=f'Wrong face/sign or wiring: mean {vec(mean)}'
        if abs(np.linalg.norm(mean)-G)>2.0:
            errors[p]=f'Measured norm far from g: {np.linalg.norm(mean):.3f}'
        if float(np.max(np.std(data,axis=0)))>0.4:
            errors[p]=f'Board moving during pose (std >0.4 m/s2)'
    if errors:
        raise ValueError('Six-pose validation failed: '+str(errors))

    diag_bias=[];diag_scale=[]
    for axis_i,ax in enumerate('XYZ'):
        plus=means['+'+ax][axis_i]
        minus=means['-'+ax][axis_i]
        diag_bias.append(0.5*(plus+minus))
        if plus-minus <= 0.5*G:
            raise ValueError(f'Insufficient +/-{ax} separation')
        diag_scale.append(2*G/(plus-minus))
    diag_bias=np.asarray(diag_bias)
    diag_matrix=np.diag(diag_scale)

    # a_target = M @ a_measured + intercept; a_bias = -inv(M) @ intercept.
    X=np.vstack([np.r_[means[p],1.0] for p in POSES]); Y=[]
    for p in POSES:
        goal=np.zeros(3)
        goal[AXES.index(p[1].lower())]=G*(1 if p[0]=='+' else -1)
        Y.append(goal)
    Y=np.asarray(Y)
    coefs,_,rank,_=np.linalg.lstsq(X,Y,rcond=None)
    if rank<4: raise ValueError('Face means are rank-deficient')
    full_matrix=coefs[:3,:].T
    full_intercept=coefs[3,:]
    full_bias=-np.linalg.solve(full_matrix,full_intercept)
    res_full=np.linalg.norm((X@coefs)-Y,axis=1)
    res_diag=np.linalg.norm(np.asarray([diag_matrix@(means[p]-diag_bias) for p in POSES])-Y,axis=1)

    return {
        'units':'raw acceleration in m/s^2; corrected acceleration in m/s^2',
        'recommended': 'diagonal_6face (robust without precision orientation fixture)',
        'bias_mps2':vec(diag_bias),
        'scale_matrix_3x3':mat(diag_matrix),
        'full3x3_experimental_bias_mps2':vec(full_bias),
        'full3x3_experimental_matrix':mat(full_matrix),
        'full3x3_note':'Six freehand poses cannot distinguish tilt/position error from true cross-axis misalignment. Use diagonal result unless faces are precisely aligned to a fixture.',
        'pose_counts':counts,
        'pose_means_mps2':{k:vec(v) for k,v in means.items()},
        'pose_std_mps2':face_stds,
        'pose_means_norm_mps2':means_norm,
        'face_rms_residual_diag_mps2':float(np.sqrt(np.mean(res_diag**2))),
        'face_rms_residual_full3x3_mps2':float(np.sqrt(np.mean(res_full**2))),
    }


def fit_gyro(rows):
    rest=triplets(rows,['gx_dps','gy_dps','gz_dps'],pose='REST',predicate=has_mpu)
    if len(rest)<300:raise ValueError(f'REST only has {len(rest)} valid gyro samples; need >=300')
    avg=rest.mean(axis=0); std=rest.std(axis=0)
    acc=triplets(rows,['ax_mps2','ay_mps2','az_mps2'],pose='REST',predicate=has_mpu)
    acc_std=np.std(acc,axis=0)
    if np.max(std)>0.25 or np.max(acc_std)>0.4:
        raise ValueError(f'REST not stationary: gyro std {vec(std)} deg/s, accel std {vec(acc_std)} m/s2')
    temps=arr(rows,'mpu_temp_c',pose='REST',predicate=has_mpu)
    return {
        'bias_dps':vec(avg),'std_dps':vec(std),
        'sample_count':len(rest),
        'mpu_temperature_c_at_calibration':float(temps.mean()) if len(temps) else None,
        'scale_matrix_3x3':mat(np.eye(3)),
        'scale_note':'Gyro scale and axis cross-coupling CANNOT be determined from stationary six-face measurements. Requires a known angular-rate source/rotation profile.',
        'formula':'omega_corrected_dps = omega_measured_dps - bias_dps',
    }


def shape_from_params(p):
    l=np.array([[math.exp(p[3]),0.0,0.0],
                [p[4],math.exp(p[5]),0.0],
                [p[6],p[7],math.exp(p[8])]])
    return l@l.T


def magnetic_orientation_coverage(corrected_unit_vectors):
    """Approximate solid-angle coverage from a reproducible 128-point sphere grid.

    This reports coverage and sampling imbalance, not heading/accuracy truth.
    The corrected vectors are in MAGNETOMETER chip axes, not verified drone axes.
    """
    from scipy.spatial import cKDTree
    u=np.asarray(corrected_unit_vectors,dtype=float)
    u=u[np.all(np.isfinite(u),axis=1)]
    norm=np.linalg.norm(u,axis=1)
    u=u[norm>1e-8]
    if len(u)==0: raise ValueError('No valid magnetic vectors for coverage assessment')
    u=u/np.linalg.norm(u,axis=1)[:,None]
    n=128
    j=np.arange(n,dtype=float)+.5
    zz=1.-2.*j/n
    angle=j*(math.pi*(3-math.sqrt(5)))
    grid=np.column_stack([np.cos(angle)*np.sqrt(1-zz**2),
                          np.sin(angle)*np.sqrt(1-zz**2),zz])
    dist,_=cKDTree(u).query(grid,k=1)
    cos18=math.cos(math.radians(18.0))
    coverage=float(np.mean(dist <= math.sqrt(2-2*cos18)))
    # Octants indicate imbalance, not mandatory equal occupancy.
    octant={}
    for x in ('-','+'):
        for y in ('-','+'):
            for z in ('-','+'):
                mask=(u[:,0]>=0 if x=='+' else u[:,0]<0)&(u[:,1]>=0 if y=='+' else u[:,1]<0)&(u[:,2]>=0 if z=='+' else u[:,2]<0)
                octant[x+y+z]=int(np.count_nonzero(mask))
    return {
        'fibonacci_directions_total':n,
        'coverage_fraction_within_18_degrees':coverage,
        'positive_hemisphere_fractions_xyz':[float(np.mean(u[:,i]>=0)) for i in range(3)],
        'octant_counts_sensor_frame':octant,
        'least_sampled_octants_sensor_frame':sorted(octant,key=octant.get)[:3],
        'interpretation':'Orientation coverage is sampled on 128 quasi-uniform directions; a high value is helpful but does not certify heading accuracy. Octants use corrected sensor axes, not the aircraft frame.'}


def fit_mag(rows):
    samples=triplets(rows,['mx_uT','my_uT','mz_uT'],pose='MAG',predicate=has_mag_new)
    if len(samples)<200:
        raise ValueError(f'MAG only has {len(samples)} FRESH samples; need >=200 (HMC samples at 15Hz)')
    span=np.ptp(samples,axis=0)
    if np.min(span)<15.0:
        raise ValueError(f'Magnetometer was not rotated through sufficient 3D range: span={vec(span)} uT')
    center=(samples.max(axis=0)+samples.min(axis=0))/2
    cov=np.cov((samples-center).T)
    eig,V=np.linalg.eigh(cov)
    if eig[0]<1e-5:raise ValueError('Degenerate magnetometer orientation coverage')
    s0=V @ np.diag(1/np.sqrt(3*eig)) @ V.T
    norms=np.linalg.norm((samples-center)@s0.T,axis=1)
    s0 /= np.median(norms)
    l=np.linalg.cholesky(s0)
    params=np.r_[center, math.log(l[0,0]),l[1,0],math.log(l[1,1]),l[2,0],l[2,1],math.log(l[2,2])]

    def residual(p):
        b=p[:3]
        S=shape_from_params(p)
        return np.linalg.norm((samples-b)@S.T,axis=1)-1.0

    # Fit a uniform subset for bounded CPU cost on large QMC5883P datasets.
    # Validate on ALL original points below.
    fit_samples=samples[::max(1,len(samples)//6000)]
    def residual_subset(p):
        b=p[:3]
        S=shape_from_params(p)
        return np.linalg.norm((fit_samples-b)@S.T,axis=1)-1.0
    sol=least_squares(residual_subset,params,loss='soft_l1',f_scale=0.04,max_nfev=750)
    if not sol.success:raise ValueError('Mag soft-iron fit did not converge: '+sol.message)
    b=sol.x[:3]
    unitS=shape_from_params(sol.x)
    mag_u=np.linalg.norm((samples-b)@unitS.T,axis=1)
    if np.median(mag_u)<0.8 or np.median(mag_u)>1.2:
        raise ValueError('Unstable ellipsoid solution')
    before=np.linalg.norm(samples-b,axis=1)
    reference_radius=float(np.median(before))
    scale=unitS*reference_radius
    after=np.linalg.norm((samples-b)@scale.T,axis=1)
    std_after=float(np.std(mag_u))
    warnings=[]
    if std_after>0.10:warnings.append('Post-fit radial standard deviation >0.10: magnetic interference, insufficient rotations, or dynamic contamination')
    corrected=(samples-b)@unitS.T
    for i,axis in enumerate('XYZ'):
        if corrected[:,i].min()>-0.75 or corrected[:,i].max()<0.75:
            warnings.append(f'Corrected magnetometer {axis} did not cover nearly both poles; rotate the assembly further')
    coverage=magnetic_orientation_coverage(corrected)
    if coverage['coverage_fraction_within_18_degrees'] < 0.90:
        warnings.append('Magnetic 3D orientation coverage is uneven (<90% of test sphere directions within 18 degrees); add more slow diagonal/inverted tumbling rotations and validate on another session')
    if min(coverage['octant_counts_sensor_frame'].values()) / len(samples) < .015:
        warnings.append('One or more corrected magnetometer-frame octants have <1.5% of samples; rotations are strongly unbalanced')
    stage_counts = {stage:sum(r.get('mag_stage') == stage and has_mag_new(r) for r in rows)
                    for stage in ('YAW','PITCH','ROLL','TUMBLE')}
    for stage, n in stage_counts.items():
        if n < 40:
            warnings.append(f'Magnetometer stage {stage}: only {n} fresh points; check capture steps')
    types={r.get('mag_sensor_type','HMC5883L') for r in rows if r.get('pose')=='MAG' and has_mag_new(r)}
    if len(types)!=1:
        raise ValueError(f'Mixed magnetic sensor types in recording: {sorted(types)}')
    return {
        'sensor':next(iter(types))+' (firmware ID-checked)',
        'calibration_stage_fresh_samples':stage_counts,
        'hard_iron_offset_uT':vec(b),
        'soft_iron_matrix_3x3_dimensionless':mat(scale),
        'unit_sphere_matrix_3x3_inverse_uT':mat(unitS),
        'reference_radius_uT_approx_from_data':reference_radius,
        'formula':'m_corrected_uT = soft_iron_matrix_3x3_dimensionless @ (m_measured_uT - hard_iron_offset_uT)',
        'note':'Absolute magnetic field magnitude/gain cannot be established by rotations alone; scale keeps approximate measured radius. Calibrate mounted on the vehicle, with representative nearby electronics, away from steel and moving motors.',
        'samples':len(samples),'samples_used_in_fit':len(fit_samples),'axis_span_uT':vec(span),
        'pre_fit_radius_cv':float(np.std(before)/np.mean(before)),
        'post_fit_radius_cv':float(np.std(after)/np.mean(after)),
        'unit_sphere_rms_error':float(np.sqrt(np.mean((mag_u-1)**2))),
        'orientation_coverage':coverage,
        'warnings':warnings,
    }


def fit_bmp(rows):
    data=[r for r in rows if r['pose'] in ('BMP','REST') and has_bmp_new(r)]
    pp=arr(data,'pressure_pa'); tc=arr(data,'bmp_temp_c')
    if len(pp)<30: raise ValueError(f'BMP needs >=30 fresh REST/BMP readings; got {len(pp)}')
    return {
        'p0_pa_at_reference_height':float(np.median(pp)),
        'pressure_std_pa':float(np.std(pp)),
        'sample_count':len(pp),
        'temperature_mean_c':float(np.mean(tc)) if len(tc) else None,
        'altitude_formula':'h_relative_m = 44330*(1-(P/P0)**0.19029496) [ISA approximation]',
        'note':'P0 is local pressure at the reference elevation, NOT a sensor bias or sea-level pressure. Absolute pressure offset/temperature calibration requires reference instruments. BMP280 library already applies factory trimming coefficients.',
    }


def session_temperature_models(rows):
    """Estimate optional thermal behavior using SESSION MEANS, not pseudo-independent 500-Hz samples.

    A single temperature / uncalibrated pressure sensor cannot identify
    thermal sensitivity. Strong quality gates prevent silently inventing slopes.
    """
    by_source={}
    for r in rows:
        source=r.get('_source_file','unspecified')
        # Split CSV checkpoints (_part02, _part03...) belong to the same
        # thermal experiment, not independent temperature observations.
        sp=Path(source)
        stem=re.sub(r'_part[0-9]{2,}$','',sp.stem)
        session=str(sp.with_name(stem+sp.suffix))
        by_source.setdefault(session,[]).append(r)
    gyro_means=[]
    acc_fits=[]
    bmp_summaries=[]
    notes=[]
    for source,samples in by_source.items():
        rest=triplets(samples,['gx_dps','gy_dps','gz_dps'],pose='REST',predicate=has_mpu)
        rest_acc=triplets(samples,['ax_mps2','ay_mps2','az_mps2'],pose='REST',predicate=has_mpu)
        temps=arr(samples,'mpu_temp_c',pose='REST',predicate=has_mpu)
        if len(rest)>=100 and len(temps)>=100 and len(rest)==len(rest_acc)==len(temps):
            if np.max(np.std(rest,axis=0))<0.35 and np.max(np.std(rest_acc,axis=0))<0.5:
                gyro_means.append({'file':source,'temperature_c':float(np.median(temps)),
                                   'bias_dps':vec(np.mean(rest,axis=0)),'n':len(rest)})
            else:
                notes.append(f'{Path(source).name}: REST motion too large for thermal gyro regression')
        # Accelerometer thermal response needs a complete six-face campaign per
        # temperature; do NOT infer it from a single oriented static sample.
        try:
            a=fit_accel(samples)
            all_t=arr(samples,'mpu_temp_c',predicate=lambda r: r.get('pose') in POSES and has_mpu(r))
            if len(all_t):
                acc_fits.append({'file':source,'temperature_c':float(np.median(all_t)),
                    'bias_mps2':a['bias_mps2'],
                    'scale_diag':vec(np.diag(np.asarray(a['scale_matrix_3x3']))),
                    'samples':sum(a['pose_counts'].values())})
        except (ValueError,np.linalg.LinAlgError):
            pass
        bmp_rows=[r for r in samples if r.get('pose') in ('REST','BMP') and has_bmp_new(r)]
        bt=arr(bmp_rows,'bmp_temp_c'); bp=arr(bmp_rows,'pressure_pa')
        if len(bt)>=30 and len(bp)==len(bt):
            bmp_summaries.append({'file':source,'temperature_c':float(np.mean(bt)),
                'pressure_pa':float(np.median(bp)),'pressure_std_pa':float(np.std(bp)),
                'n_fresh':len(bp)})

    def regression(points, value_key, units):
        if len(points)<3:
            return {'status':'INSUFFICIENT_SESSION_COUNT','required':'3 complete valid sessions at different temperatures',
                    'session_count':len(points),'sessions':points,'units':units}
        temp=np.array([p['temperature_c'] for p in points])
        span=float(np.ptp(temp))
        if span<3.0:
            return {'status':'INSUFFICIENT_TEMPERATURE_SPAN','minimum_span_c':3.0,
                    'observed_span_c':span,'session_count':len(points),'sessions':points,'units':units}
        val=np.asarray([p[value_key] for p in points],dtype=float)
        ref=float(np.median(temp))
        X=np.column_stack((np.ones(len(temp)),temp-ref))
        beta,_,rank,_=np.linalg.lstsq(X,val,rcond=None)
        if rank<2:raise ValueError('Temperature regression rank deficient')
        fitted=X@beta
        rms=np.sqrt(np.mean((val-fitted)**2,axis=0))
        return {'status':'ESTIMATED_NOT_VALIDATED','reference_temp_c':ref,
                'intercept_at_reference':vec(beta[0]),
                'slope_per_c':vec(beta[1]),'rms_session_residual':vec(rms),
                'span_c':span,'session_count':len(points),'sessions':points,'units':units,
                'basis':'linear regression of independently captured session means, equal weight per session'}

    gyro=regression(gyro_means,'bias_dps','degrees/s and degrees/s/°C')
    accel_bias=regression(acc_fits,'bias_mps2','m/s² and m/s²/°C')
    accel_scale=regression(acc_fits,'scale_diag','dimensionless and 1/°C')
    bmp_diag=regression(bmp_summaries,'pressure_pa','Pa and Pa/°C (diagnostic only)')
    if bmp_diag['status']=='ESTIMATED_NOT_VALIDATED':
        bmp_diag['warning']='Do NOT use this slope as a pressure correction: actual air pressure changes with weather and elevation, and BMP280 already uses its factory temperature compensation. Obtain a calibrated pressure reference at known height to isolate residual sensor temperature error.'
    return {'gyro_bias_versus_mpu_temperature':gyro,
            'accelerometer_bias_versus_mpu_temperature':accel_bias,
            'accelerometer_scale_versus_mpu_temperature':accel_scale,
            'bmp280_pressure_versus_temperature_diagnostic':bmp_diag,
            'bmp280_sessions':bmp_summaries,
            'notes':notes+[
              'A single capture cannot identify a temperature slope.',
              'At least three stationary REST sessions with ≥3 °C span are required to estimate a gyro thermal slope.',
              'Acceleration temperature coefficients require repeated FULL six-face captures at different temperatures.',
              'BMP280 pressure/temperature relation without a known stable physical reference is descriptive only.',
              'Temperature coefficient estimates require independent validation; do not deploy automatically.'
            ]}


def fit_lidar(rows):
    """Fit the optional TOFSense F2 Mini without assuming a new MCU sample rate.

    The true reference distance is measured independently from the LiDAR optical
    reference plane to the target. Median-per-distance reduces pseudoreplication
    of serially correlated samples. Report withheld-distance errors too.
    """
    grouped={}
    all_count=0
    for r in rows:
        if r.get('kind')!='L':continue
        all_count+=1
        if not accepted(r):continue
        try:
            ref=float(r['lidar_reference_m'])
            measured=float(r['lidar_distance_m'])
        except (KeyError,ValueError,TypeError):continue
        if math.isfinite(ref) and math.isfinite(measured) and 0.02<=measured<=7.8:
            grouped.setdefault(ref,[]).append(measured)
    stats=[]
    for ref, samples in sorted(grouped.items()):
        if len(samples)<20:continue
        z=np.array(samples,dtype=float)
        med=float(np.median(z))
        stats.append({'reference_m':ref,'n':len(z),'median_m':med,
                      'mean_m':float(np.mean(z)),
                      'std_m':float(np.std(z,ddof=1)),
                      'mad_sigma_m':float(1.4826*np.median(np.abs(z-med))),
                      'median_bias_m':med-ref})
    if len(stats)<3:
        raise ValueError(f'LiDAR requires >=3 distinct measured references with >=20 fresh readings each; got {len(stats)}')
    xx=np.array([v['reference_m'] for v in stats])
    yy=np.array([v['median_m'] for v in stats])
    slope,intercept=np.polyfit(xx,yy,1)
    if slope<0.5 or slope>1.5:
        raise ValueError(f'Implausible range scale {slope:.4f}; verify reference distances and sensor orientation')
    residual=yy-(slope*xx+intercept)
    holdout=[]
    if len(xx)>=4:
        for i in range(len(xx)):
            train=np.arange(len(xx))!=i
            m,b=np.polyfit(xx[train],yy[train],1)
            holdout.append(float(yy[i]-(m*xx[i]+b)))
    return {'sensor':'TOFSense F2 Mini','nominal_max_hz':50,
      'source_rows':all_count,'per_reference':stats,
      'measured_equals_a_times_reference_plus_b':{'a':float(slope),'b_m':float(intercept)},
      'corrected_range_equation':'distance_m_corrected = (measured_distance_m - b_m) / a',
      'fit_rms_m':float(np.sqrt(np.mean(residual**2))),
      'leave_one_distance_out_errors_m':holdout,
      'leave_one_distance_out_rms_m':float(np.sqrt(np.mean(np.square(holdout)))) if holdout else None,
      'temperature_note':'MPU/BMP die temperature logged asynchronously for context; no LiDAR die temperature is exposed by NLink frame0.',
      'warnings':['Recheck on other surface colors, tilts, ambient lighting and independently known holdout distances before deployment.']}


def format_float(v):
    if not math.isfinite(float(v)):return 'NAN'
    t=f'{float(v):.9g}'
    if '.' not in t and 'e' not in t and 'E' not in t:t+='.0'
    return t+'f'


def cpp_array(name, arr_values):
    v=np.asarray(arr_values)
    if v.ndim==1:
        return f'inline constexpr float {name}[3] = {{{", ".join(map(format_float,v))}}};\n'
    return f'inline constexpr float {name}[3][3] = {{\n' + ',\n'.join('    {'+', '.join(map(format_float,row))+'}' for row in v)+'\n};\n'


def header_string(cal):
    parts=['#pragma once', '// Auto-generated sensor calibration constants for ESP32-S3.',
           '// Sensor frame only. Mounting orientation/frame transforms are separate.',
           '// v_corrected = MATRIX * (v_raw_in_physical_units - BIAS)',
           'namespace intelqcopter_cal {']
    if 'gyro' in cal:
        parts.append(cpp_array('GYRO_BIAS_DPS',cal['gyro']['bias_dps']))
        parts.append(cpp_array('GYRO_SCALE_MATRIX',cal['gyro']['scale_matrix_3x3']))
    if 'accelerometer' in cal:
        parts.append(cpp_array('ACCEL_BIAS_MPS2',cal['accelerometer']['bias_mps2']))
        parts.append(cpp_array('ACCEL_MATRIX',cal['accelerometer']['scale_matrix_3x3']))
        parts.append('// The following full 3x3 estimate is experimental from six hand poses:')
        parts.append(cpp_array('ACCEL_FULL3X3_BIAS_MPS2',cal['accelerometer']['full3x3_experimental_bias_mps2']))
        parts.append(cpp_array('ACCEL_FULL3X3_MATRIX',cal['accelerometer']['full3x3_experimental_matrix']))
    if 'magnetometer' in cal:
        parts.append(cpp_array('MAG_BIAS_UT',cal['magnetometer']['hard_iron_offset_uT']))
        parts.append(cpp_array('MAG_MATRIX',cal['magnetometer']['soft_iron_matrix_3x3_dimensionless']))
    if 'bmp280' in cal:
        parts.append('inline constexpr float BMP_REFERENCE_PRESSURE_PA = '+format_float(cal['bmp280']['p0_pa_at_reference_height'])+';')
    if 'lidar' in cal:
        params=cal['lidar']['measured_equals_a_times_reference_plus_b']
        parts.append('// OPTIONAL F2 Mini bench model; validate target material and range before use.')
        parts.append('inline constexpr float LIDAR_MEASURED_SCALE = '+format_float(params['a'])+';')
        parts.append('inline constexpr float LIDAR_MEASURED_OFFSET_M = '+format_float(params['b_m'])+';')
        parts.append('// corrected_distance_m = (measured_m - LIDAR_MEASURED_OFFSET_M) / LIDAR_MEASURED_SCALE')
    t=cal.get('temperature_analysis',{})
    gt=t.get('gyro_bias_versus_mpu_temperature',{})
    if gt.get('status')=='ESTIMATED_NOT_VALIDATED':
        parts.append('// EXPERIMENTAL thermal gyro bias model; validate with independent sessions.')
        parts.append('inline constexpr float GYRO_TEMP_REFERENCE_C = '+format_float(gt['reference_temp_c'])+';')
        parts.append(cpp_array('GYRO_BIAS_AT_TEMP_REF_DPS',gt['intercept_at_reference']))
        parts.append(cpp_array('GYRO_BIAS_TEMP_SLOPE_DPS_PER_C',gt['slope_per_c']))
        parts.append('// b(T) = GYRO_BIAS_AT_TEMP_REF_DPS + GYRO_BIAS_TEMP_SLOPE_DPS_PER_C * (T - GYRO_TEMP_REFERENCE_C)')
    at=t.get('accelerometer_bias_versus_mpu_temperature',{})
    st=t.get('accelerometer_scale_versus_mpu_temperature',{})
    if at.get('status')=='ESTIMATED_NOT_VALIDATED' and st.get('status')=='ESTIMATED_NOT_VALIDATED':
        parts.append('// EXPERIMENTAL accelerometer thermal model; repeated six-face sessions required.')
        parts.append('inline constexpr float ACCEL_TEMP_REFERENCE_C = '+format_float(at['reference_temp_c'])+';')
        parts.append(cpp_array('ACCEL_BIAS_AT_TEMP_REF_MPS2',at['intercept_at_reference']))
        parts.append(cpp_array('ACCEL_BIAS_TEMP_SLOPE_MPS2_PER_C',at['slope_per_c']))
        parts.append(cpp_array('ACCEL_SCALE_AT_TEMP_REF',st['intercept_at_reference']))
        parts.append(cpp_array('ACCEL_SCALE_TEMP_SLOPE_PER_C',st['slope_per_c']))
    parts.append('} // namespace intelqcopter_cal\n')
    return '\n'.join(parts)


def main():
    ap=argparse.ArgumentParser(description='Calibrate using one or MULTIPLE selected CSV segments/sessions.')
    ap.add_argument('csv',nargs='+',type=Path,help='One or more calibration CSV files')
    ap.add_argument('-o','--output-prefix',type=Path,default=None)
    ns=ap.parse_args()
    rows=[]
    lidar_rows=[]
    total_rows=0
    late_rows=0
    total_mpu=0
    stage_stats={}
    input_stats=[]
    duration_total_s=0.0
    all_mag_types=set()
    required={'pose','mpu_ok','bmp_ok','mag_ok','ax_mps2','gx_dps','mx_uT','bmp_new','mag_new'}

    # Each file is a complete or checkpointed segment. A new file can reset
    # MCU time/sequence, and prompts produce intentional time gaps.
    for path in ns.csv:
        path=path.resolve()
        file_rows=0;first_t=None;last_t=None;wraps=0
        with path.open(newline='',encoding='utf-8-sig') as f:
            reader=csv.DictReader(f)
            absent=required-set(reader.fieldnames or [])
            if absent:raise SystemExit(f'{path}: missing CSV columns: {sorted(absent)}')
            for r in reader:
                if r.get('kind')=='L':
                    r['_source_file']=str(path)
                    lidar_rows.append(r)
                    continue
                if r.get('kind')!='D':continue
                total_rows+=1;file_rows+=1
                r['_source_file']=str(path)
                if r.get('mpu_ok')=='1':total_mpu+=1
                if r.get('mag_new')=='1' and r.get('mag_ok')=='1':
                    all_mag_types.add(r.get('mag_sensor_type','HMC5883L'))
                try: late_rows+=int(int(r.get('late_us','0'))>300)
                except (TypeError,ValueError):pass
                key=(r.get('pose','')+'/'+(r.get('mag_stage','STILL') or 'STILL'))
                q=stage_stats.setdefault(key,{'rows':0,'accepted':0,'rejected':0,'mpu_valid':0,
                     'mag_fresh':0,'bmp_fresh':0,'seq_gaps':0,'late_over_300us':0,
                     '_previous_seq':None,'_previous_timestamp':None,'_last_source':None,
                     '_chunks':0,'_duration_us':0, '_temperature_mpu':[], '_temperature_bmp':[]})
                if q['_last_source']!=str(path):
                    q['_last_source']=str(path)
                    q['_previous_seq']=None;q['_previous_timestamp']=None
                    q['_chunks']+=1
                q['rows']+=1;q['accepted']+=int(accepted(r));q['rejected']+=int(not accepted(r))
                if r.get('phase_accepted','1')=='1':
                    for temp_key,enabled in (('_temperature_mpu',r.get('mpu_ok')=='1'),
                                              ('_temperature_bmp',r.get('bmp_ok')=='1' and r.get('bmp_new')=='1')):
                        if enabled:
                            col='mpu_temp_c' if temp_key=='_temperature_mpu' else 'bmp_temp_c'
                            try:
                                T=float(r[col])
                                if math.isfinite(T):q[temp_key].append(T)
                            except (KeyError,ValueError,TypeError):pass
                q['mpu_valid']+=int(r.get('mpu_ok')=='1')
                q['mag_fresh']+=int(has_mag_new(r));q['bmp_fresh']+=int(has_bmp_new(r))
                try:q['late_over_300us']+=int(int(r.get('late_us','0'))>300)
                except (ValueError,TypeError):pass
                try:
                    seq=int(r['seq']);stamp=int(r['t_us'])
                    prev_seq=q['_previous_seq'];prev_t=q['_previous_timestamp']
                    if prev_seq is not None:
                        gap=(seq-prev_seq)%(2**32)
                        if 1<gap<1000000:q['seq_gaps']+=gap-1
                    if prev_t is not None:
                        dt=(stamp-prev_t)%(2**32)
                        if 0<dt<2_000_000:q['_duration_us']+=dt
                    q['_previous_seq']=seq;q['_previous_timestamp']=stamp
                    if first_t is None:first_t=stamp
                    if last_t is not None and stamp<last_t and (last_t-stamp)>2**31:wraps+=1
                    last_t=stamp
                except (ValueError,KeyError,TypeError):pass
                # Every fifth IMU sample; every fresh magnetic or BMP sample.
                if ((r.get('pose') in (*POSES,'REST') and file_rows%5==0)
                    or (r.get('mag_new')=='1' and r.get('pose')=='MAG')
                    or (r.get('bmp_new')=='1' and r.get('pose') in ('BMP','REST'))):
                    rows.append(r)
        file_duration=((last_t-first_t+wraps*2**32)/1e6) if first_t is not None and last_t is not None else 0.0
        duration_total_s+=max(0,file_duration)
        input_stats.append({'file':str(path),'rows':file_rows,'time_spanned_s':file_duration})
    from attempt_filter import only_latest
    rows_before_attempt_filter=len(rows)
    rows=only_latest(rows)
    lidar_rows=only_latest(lidar_rows)
    if rows_before_attempt_filter != len(rows):
        print(f'Superseded attempts excluded: {rows_before_attempt_filter-len(rows)} analysis rows',flush=True)
    if not rows and not lidar_rows:raise SystemExit('CSV files empty or contain no recognized calibration poses')
    phase_report={}
    for key,q in stage_stats.items():
        n_intervals=max(0,q['rows']-q['_chunks'])
        hz=(n_intervals*1e6/q['_duration_us']) if q['_duration_us']>0 else None
        phase_report[key]={k:v for k,v in q.items() if not k.startswith('_')}
        phase_report[key]['measured_sample_rate_hz']=hz
        phase_report[key]['duration_seconds']=q['_duration_us']/1e6
        phase_report[key]['file_chunks']=q['_chunks']
        # Automatic thermal annotations for every labeled phase, not an
        # optional GUI action. Distinguish each silicon DIE temperature.
        for short,col in (('mpu_die',q['_temperature_mpu']),('bmp_die',q['_temperature_bmp'])):
            if col:
                phase_report[key][short+'_temp_c']={
                  'median':float(np.median(col)),'minimum':float(min(col)),
                  'maximum':float(max(col)),'span':float(max(col)-min(col)),
                  'samples':len(col)}
    skipped_transport=sum(q['seq_gaps'] for q in stage_stats.values())
    result={'schema':'IntelQCopter calibration v3 multiCSV thermal 500Hz',
            'input_file':str(ns.csv[0]), 'input_files':[str(p.resolve()) for p in ns.csv],
            'input_file_stats':input_stats,'input_rows':total_rows,
            'retained_analysis_rows':len(rows),
            'superseded_analysis_rows':rows_before_attempt_filter-len(rows),
            'logging_quality':{'mean_rate_hz_during_selected_file_spans':
                (total_rows-len(input_stats))/duration_total_s if duration_total_s>0 else None,
                'mpu_valid_rows':total_mpu,'device_sequence_gaps_within_phases':skipped_transport,
                'rows_late_over_300us':late_rows,'phase_metrics':phase_report},
            'g_mps2':G,'units':'accel m/s2; gyro degrees/s; magnetic microtesla; pressure pascal',
            'warnings':[],'hardware_validation':'Sensor logs do not demonstrate flight readiness.'}
    for file in ns.csv:
        manifest=file.with_suffix('.skipped.json')
        if manifest.exists():
            try:
                skipped=json.loads(manifest.read_text(encoding='utf-8')).get('skipped_stages',[])
                if skipped:result['warnings'].append(f'{file.name}: skipped phases {skipped}')
            except (ValueError,OSError):
                result['warnings'].append(f'Cannot read skipped-phase manifest {manifest}')
    if len(all_mag_types)>1:
        result['warnings'].append('Different magnetometer types detected across selected files; MAG fit will refuse mixed types.')
    if len(ns.csv)>1:
        result['warnings'].append('Multi-file analysis assumes the SAME physical board, axis mapping, full-scale ranges and magnetic configuration. Do not merge captures made after hardware changes.')
    if skipped_transport:result['warnings'].append(f'{skipped_transport} missing serial frames within captured phases')
    for key,q in phase_report.items():
        hz=q['measured_sample_rate_hz']
        if hz is not None and (hz<470 or hz>530) and not key.startswith(('MAG/','BMP/')):
            result['warnings'].append(f'{key}: measured host capture rate {hz:.1f} Hz outside IMU 500±30 Hz')
    jobs=[('gyro',fit_gyro),('accelerometer',fit_accel),('magnetometer',fit_mag),('bmp280',fit_bmp)]
    if lidar_rows:jobs.append(('lidar',lambda _rows:fit_lidar(lidar_rows)))
    for name,fn in jobs:
        try:
            result[name]=fn(rows)
            print(f'[OK] {name}')
        except (ValueError,np.linalg.LinAlgError,OverflowError) as exc:
            result['warnings'].append(f'{name}: {exc}')
            print(f'[WARN] {name}: {exc}',file=sys.stderr)
    # Surface per-sensor fit-quality warnings at the report top level as well.
    # A numerically successful ellipsoid fit does NOT mean orientation coverage
    # or out-of-sample heading accuracy has been verified.
    for sensor in ('gyro','accelerometer','magnetometer','bmp280'):
        for message in result.get(sensor,{}).get('warnings',[]):
            result['warnings'].append(f'{sensor}: {message}')
    # Temperature is ALWAYS part of normal calibration analysis, never an
    # unrelated GUI button. A single ambient-temperature run yields only bias
    # at that temperature; several independently stabilized sessions may
    # identify drift. Never infer BMP residual correction from pressure alone.
    result['temperature_analysis']=session_temperature_models(rows)
    try:
        from thermal_characterization import analyze as thermal_analyze
        reference_candidates=[Path('thermal_pressure_references.json'),
                              ns.csv[0].parent/'thermal_pressure_references.json']
        ref=next((v for v in reference_candidates if v.is_file()),None)
        result['temperature_uncertainty_analysis']=thermal_analyze(ns.csv,ref)
        result['temperature_uncertainty_analysis']['reference_source']=str(ref) if ref else None
        result['temperature_uncertainty_analysis']['model_deployment']='DIAGNOSTIC_UNTIL_VALIDATED'
        if ref is None:
            result['warnings'].append('BMP280 temperature residual correction NOT identified (no independent reference-pressure file); Bosch factory compensation retained.')
    except (ValueError,OSError,KeyError) as exc:
        result['warnings'].append(f'Automatic temperature/uncertainty analysis: {exc}')
    result['temperature_dependent_measurement_equations']={
        'gyro':'omega_corrected(T) = gyro_measured(T) - [b_g(T_ref)+k_g*(T_MPU-T_ref)], only where multi-temperature model is valid',
        'accel':'f_corrected(T) = S_a(T) @ [a_measured - b_a(T)], only for repeated six-face thermal datasets',
        'bmp':'p_compensated = Bosch_factory_compensation(raw_adc,T_BMP); residual correction ONLY with independent pressure reference',
        'thermal_covariance':'R_g(T) = diag(exp(alpha_g + beta_g*(T_MPU-T_ref))) when independently identified; else constant conservative R_g',
        'time_alignment':'Record MPU and BMP temperatures separately; do not treat their die temperatures as ambient or synchronous LiDAR temperature.',
    }

    temp=result['temperature_analysis']
    if 'gyro' in result:
        gt=temp['gyro_bias_versus_mpu_temperature']
        result['gyro']['thermal_reference_model']=gt['status']
        if gt['status']=='ESTIMATED_NOT_VALIDATED':
            result['gyro']['temperature_reference_c']=gt['reference_temp_c']
            result['gyro']['bias_at_reference_dps']=gt['intercept_at_reference']
            result['gyro']['bias_temperature_slope_dps_per_c']=gt['slope_per_c']
            result['gyro']['formula_with_temperature']='omega_calibrated = omega_raw - (bias_Tref + slope*(mpu_temp_c - Tref))'
            result['gyro']['recommended_offline_temperature_model']={
               'reference_temp_c':gt['reference_temp_c'],
               'bias_dps_at_reference':gt['intercept_at_reference'],
               'slope_dps_per_c':gt['slope_per_c'],
               'valid_temp_span_c':gt['span_c'],
               'status':'ESTIMATED_NOT_INDEPENDENTLY_VALIDATED; USE ONLY INSIDE MEASURED TEMPERATURE RANGE'}
    if len(temp['bmp280_sessions'])>1:
        result['warnings'].append('BMP280 reference pressure varies by session due to altitude/weather. Re-zero P0 in each operating session.')
    prefix=ns.output_prefix or ns.csv[0].with_suffix('')
    jpath=Path(str(prefix)+'_calibration.json')
    hpath=Path(str(prefix)+'_calibration.h')
    jpath.parent.mkdir(parents=True,exist_ok=True)
    jpath.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    hpath.write_text(header_string(result),encoding='utf-8')
    print('Saved',jpath,'and',hpath)
    if result['warnings']:print('Review warnings BEFORE using calibration constants.')
    if not any(k in result for k in ('gyro','accelerometer','magnetometer','bmp280','lidar')):
        raise SystemExit('No calibrations succeeded; inspect CSV sample stages and hardware.')

if __name__ == '__main__':main()
