"""Analyze verified actual stationary IMU data; retain pose and calibration without refitting."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from analyze_gyro_runs import ROOT, verify_session


def angles(acceleration):
    x,y,z=np.asarray(acceleration).T
    return np.column_stack([np.arctan2(y,z),np.arctan2(-x,np.hypot(y,z))])


def tilt_jacobian(mean):
    x,y,z=mean
    h=np.hypot(y,z)
    d=x*x+h*h
    if h<1e-3:
        raise ValueError('Cannot propagate tilt uncertainty near singularity')
    return np.array([[0,z/(h*h),-y/(h*h)],[-h/d,x*y/(h*d),x*z/(h*d)]])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--seconds',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output directory')
    fit=json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    summary,header,profile,samples,rates,temp,times=verify_session(args.session,fit,args.seconds)
    accel=np.array([sample['drone_m_s2'] for sample in samples])
    tilt=angles(accel)
    mean=accel.mean(axis=0)
    covariance=np.cov(accel,rowvar=False,ddof=1)
    jac=tilt_jacobian(mean)
    propagated=jac@covariance@jac.T
    step=1e-5
    numeric=np.column_stack([(angles([mean+np.eye(3)[i]*step])[0]-angles([mean-np.eye(3)[i]*step])[0])/(2*step) for i in range(3)])
    jac_error=float(np.max(np.abs(jac-numeric)))
    if jac_error>1e-7:
        raise ValueError('Jacobian failed independent finite-difference check')
    blocks=np.array([tilt[(times>=i)&(times<i+1)].mean(axis=0) for i in range(int(times[-1]))])
    observed_cov=np.cov(tilt,rowvar=False,ddof=1)
    std=np.sqrt(np.diag(observed_cov))
    predicted=np.sqrt(np.diag(propagated))
    result={'engine':'Real ESP32-S3/MPU6050 serial capture performed by assistant, verified and analyzed offline; no simulator',
            'source_sha256':summary['source_sha256'],'firmware_commit':summary.get('firmware_commit'),
            'session':args.session.name,'serial_sha256':summary['serial_sha256'],'samples':len(samples),'duration_s':float(times[-1]),
            'report_rate_hz':summary['observed_report_rate_hz'],
            'stationarity':'Assumed from explicit delegation; support not independently observed',
            'independently_level':False,'calibration_changed':False,'uploaded':False,
            'angle_computation':'PC replay from actual calibrated MCU acceleration; V2 has no emitted angle columns',
            'mean_acceleration_body_m_s2':mean.tolist(),'acceleration_std_m_s2':accel.std(axis=0,ddof=1).tolist(),
            'acceleration_covariance_m2_s4':covariance.tolist(),
            'mean_angle_deg':np.degrees(tilt.mean(axis=0)).tolist(),'sample_angle_std_deg':np.degrees(std).tolist(),
            'angle_of_mean_acceleration_deg':np.degrees(angles([mean])[0]).tolist(),
            'angle_covariance_deg2':(observed_cov*(180/np.pi)**2).tolist(),
            'jacobian_rad_per_m_s2':jac.tolist(),'jacobian_finite_difference_max_error':jac_error,
            'first_order_predicted_angle_std_deg':np.degrees(predicted).tolist(),
            'one_second_block_angle_mean_std_deg':np.degrees(blocks.std(axis=0,ddof=1)).tolist(),
            'gyro_mean_sensor_deg_s':np.degrees(rates.mean(axis=0)).tolist(),
            'gyro_sample_std_deg_s':np.degrees(rates.std(axis=0,ddof=1)).tolist(),
            'gyro_covariance_rad2_s2':np.cov(rates,rowvar=False,ddof=1).tolist(),
            'temperature_range_C':[float(temp.min()),float(temp.max())],
            'mean_acceleration_norm_m_s2':float(np.linalg.norm(accel,axis=1).mean()),
            'mean_acceleration_vector_norm_m_s2':float(np.linalg.norm(mean)),
            'limits':['Actual pose tilt is not an absolute calibration error without a level reference','Sample noise/std is not bias uncertainty or certified accuracy','No independence/Gaussian assumption; no sigma/sqrt(N) precision claim','Gravity-only angle unreliable under translation','~83 Hz wide-band telemetry is not final 1000 Hz Kalman noise','Not a test of V3 MCU angle output']}
    args.output.mkdir(parents=True)
    fig,axes=plt.subplots(4,1,figsize=(11,9),constrained_layout=True)
    for i,label in enumerate(['Roll','Pitch']):
        axes[i].plot(times,np.degrees(tilt[:,i]),alpha=.55,lw=.5,label='Actual acceleration -> PC angle')
        axes[i].plot(np.arange(len(blocks))+.5,np.degrees(blocks[:,i]),lw=1.5,label='1 s means')
        axes[i].set_ylabel(label+' (deg)')
    for i,label in enumerate(['Sensor X','Sensor Y','Sensor Z']):
        means=[rates[(times>=k)&(times<k+1),i].mean() for k in range(int(times[-1]))]
        axes[2].plot(np.arange(len(means))+.5,np.degrees(means),label=label)
    axes[2].set_ylabel('Gyro 1 s means (deg/s)')
    axes[3].plot(times[::10],temp[::10],lw=.8)
    axes[3].set_ylabel('Chip temperature (C)')
    axes[3].set_xlabel('Seconds in selected capture')
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].legend()
    axes[2].legend(ncol=3)
    fig.suptitle('Assistant-run stationary capture: actual V2 data, no calibration changes\nTilt computed on PC; level and motionlessness not independently instrumented')
    fig.savefig(args.output/'stationary-results.png',dpi=150)
    plt.close(fig)
    (args.output/'analysis.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    lines=['# Assistant-run stationary IMU test: math and results','',
           f'Actual capture: **{len(samples):,} selected samples**, **{times[-1]:.3f} seconds**, **{summary["observed_report_rate_hz"]:.2f} Hz**. COM3 was opened by the assistant under the user\'s explicit stationary-test delegation. Only identity queries were sent; no upload, calibration/motor command or deliberate reset. Original serial hashes, source identity at known local Git checkpoint, profile, offsets, selected samples, conversions and timing were verified.','',
           'The board runs V2. Therefore roll/pitch below are calculated **on the PC from actual calibrated MCU acceleration**; they do not prove that the prepared V3 MCU angle code ran. Motionlessness is an assumption from the requested fixed-support test; no independent level measurement or camera observation was available.','',
           '## Measured results','',
           '| Quantity | Mean | Sample standard deviation |','|---|---:|---:|']
    for i,label in enumerate(['Roll','Pitch']):
        lines.append(f'| {label}, degrees | {result["mean_angle_deg"][i]:+.6f} | {result["sample_angle_std_deg"][i]:.6f} |')
    for i,label in enumerate(['Gyro sensor X','Gyro sensor Y','Gyro sensor Z']):
        lines.append(f'| {label}, degrees/s | {result["gyro_mean_sensor_deg_s"][i]:+.6f} | {result["gyro_sample_std_deg_s"][i]:.6f} |')
    lines+=['',f'Chip temperature: {temp.min():.3f}-{temp.max():.3f} C. Mean acceleration vector (m/s²): '+', '.join(f'{v:+.6f}' for v in mean)+'.',
            '', 'A nonzero angle is the observed gravity-vector direction. Without an independently level reference it cannot be attributed to calibration error, and it is not subtracted away. The reported standard deviations describe fluctuations of individual samples, not absolute accuracy or uncertainty of the mean.','',
            '![Original sensor data and computed angle results](stationary-results.png)','',
            '## 1. Recover roll and pitch from the measured vector','',
            'For stationary gravity observations in the project\'s calibrated Z-up drone frame, define roll phi and pitch theta with R = Rz(yaw) Ry(theta) Rx(phi). The gravity-opposing acceleration vector is','',
            '$$\\mathbf a=g[-\\sin\\theta,\\;\\sin\\phi\\cos\\theta,\\;\\cos\\phi\\cos\\theta]^T.$$','',
            'For |theta| < 90 degrees,','',
            '$$h=\\sqrt{a_y^2+a_z^2}=g\\cos\\theta,\\qquad\\phi=\\operatorname{atan2}(a_y,a_z),\\quad\\theta=\\operatorname{atan2}(-a_x,h).$$','',
            'The exact trigonometric formulas are used for every measured sample. No startup zeroing or angle fitting is performed. Yaw does not appear in the gravity direction and cannot be recovered by this observation. Translation adds non-gravity acceleration, so these formulas are not a complete dynamic flight estimator.','',
            '## 2. Measure the full covariance','',
            '$$\\bar{\\mathbf a}=\\frac1N\\sum_{k=1}^{N}\\mathbf a_k,\\qquad S_a=\\frac1{N-1}\\sum_{k=1}^{N}(\\mathbf a_k-\\bar{\\mathbf a})(\\mathbf a_k-\\bar{\\mathbf a})^T.$$','',
            'Each diagonal contains a component variance; each off-diagonal contains measured co-fluctuation of two components. All terms are retained. The same calculation is applied directly to the exact per-sample angle vectors to obtain empirical angle covariance. It does not require a jointly Gaussian model.','',
            '## 3. Explain how acceleration noise becomes angle noise','',
            'For u and v, differential angle obeys','',
            '$$d\\operatorname{atan2}(u,v)=\\frac{v\\,du-u\\,dv}{u^2+v^2}.$$','',
            'With u = ay and v = az,','',
            '$$d\\phi=\\frac{a_z\\,da_y-a_y\\,da_z}{h^2}.$$','',
            'For pitch, u = -ax and v = h. First differentiate h, then substitute:','',
            '$$dh=\\frac{a_y\\,da_y+a_z\\,da_z}{h},$$','',
            '$$d\\theta=\\frac{-h\\,da_x+a_x\\,dh}{a_x^2+h^2}.$$','',
            'Let d = ax²+h². Collect coefficients into the two-by-three Jacobian:','',
            '$$J=\\begin{bmatrix}0&a_z/h^2&-a_y/h^2\\\\-h/d&a_xa_y/(hd)&a_xa_z/(hd)\\end{bmatrix}.$$','',
            'The small-noise approximation is delta_angle approximately J delta_acceleration, evaluated at the measured mean vector. Thus','',
            '$$S_{\\rm angle}\\approx J S_a J^T.$$','',
            'This is a local uncertainty-propagation approximation, not a replacement for the exact angle formula. A central finite-difference check independently verifies the analytical derivatives.','',
            '| Angle | Empirical sample std (degrees) | Jacobian prediction (degrees) |','|---|---:|---:|']
    for i,label in enumerate(['Roll','Pitch']):
        lines.append(f'| {label} | {np.degrees(std[i]):.6f} | {np.degrees(predicted[i]):.6f} |')
    lines+=['','## 4. Why averaging does not automatically prove high precision','',
            '$$\\operatorname{Cov}(\\bar{\\mathbf a})=\\frac1{N^2}\\sum_{i=1}^{N}\\sum_{j=1}^{N}\\operatorname{Cov}(\\mathbf a_i,\\mathbf a_j).$$','',
            'Only if the cross-time covariance terms vanish does this reduce to Sa/N (as an estimated covariance). Filtered sensor outputs, temperature and slow motion can create time dependence. Therefore no standard error or confidence interval is obtained merely by dividing the sample standard deviation by sqrt(N). One-second block means are shown and their spread saved as diagnostics; their independence is not assumed either.','',
            '## What these results establish','',
            'A real stationary-intended data stream was acquired and its calibration arithmetic verified. Noise, gyro residuals, temperature and gravity-vector direction are quantified. No bias was refitted. These results do not independently prove the pose was level/still, validate V3 angle execution, establish all-temperature accuracy, or supply final 1000 Hz Kalman noise. Full numerical covariance/Jacobian values and original evidence hashes are in [analysis.json](analysis.json). Original raw logs remain private and unchanged.']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
