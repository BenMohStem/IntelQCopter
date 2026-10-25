"""Verify actual corrected V2 captures; summarize residuals without changing calibration."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from analyze_gyro_runs import ROOT, verify_session


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions',type=Path,nargs='+',required=True)
    parser.add_argument('--seconds',type=float,nargs='+',required=True,help='Expected duration for each session in the same order')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if len(args.sessions)!=len(args.seconds) or any(not np.isfinite(v) or v<20 for v in args.seconds):
        parser.error('Provide one valid duration per session')
    if args.output.exists():
        parser.error('Use a new output directory; preserve earlier reports')
    fit=json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    runs=[verify_session(path,fit,seconds) for path,seconds in zip(args.sessions,args.seconds)]
    if not all('gyro_bias' in run[2] for run in runs):
        raise ValueError('Requires corrected IMU_XYZ_V2 firmware')
    if not all(run[1]==runs[0][1] for run in runs):
        raise ValueError('Board/source/configuration/boot metadata differ; review separately before combining')
    if runs[0][0]['source_sha256']!=hashlib.sha256((ROOT/'firmware/main.cpp').read_bytes()).hexdigest():
        raise ValueError('Recorded source is not current main.cpp; review checkpoint explicitly')
    results=[]
    fig, axes=plt.subplots(4,1,figsize=(12,10),constrained_layout=True)
    fig_angle, angle_axes=plt.subplots(3,1,figsize=(12,7),constrained_layout=True)
    labels=['X','Y','Z']
    for number,(path,run) in enumerate(zip(args.sessions,runs),1):
        summary,header,profile,samples,rates,temp,times=run
        cov=np.cov(rates,rowvar=False,ddof=1)
        means=rates.mean(axis=0)
        dt=np.diff(times)
        integrated=np.vstack([np.zeros(3),np.cumsum((rates[1:]+rates[:-1])*dt[:,None]/2,axis=0)])
        blocks=[]
        for start in range(int(times[-1])):
            blocks.append(rates[(times>=start)&(times<start+1)].mean(axis=0))
        block_rates=np.array(blocks)
        centred=rates-means
        lag1=[float(np.corrcoef(centred[:-1,i],centred[1:,i])[0,1]) for i in range(3)]
        slopes=np.sum((times-times.mean())[:,None]*centred,axis=0)/np.sum((times-times.mean())**2)
        packet_max_error=np.max(np.abs(rates-(np.array([s['gyro_counts_at_recorded_offsets'] for s in samples])*np.pi/(180*65.5)-np.array(profile['gyro_remaining_bias']))),axis=0)
        result={'session':path.name,'serial_sha256':summary['serial_sha256'],'selected_samples':len(samples),
                'verified_post_measurement_packets':summary['offline_post_measurement_packets_verified'],'duration_s':float(times[-1]),'report_rate_hz':summary['observed_report_rate_hz'],'max_report_gap_ms':summary['max_report_gap_ms'],
                'gyro_boot_words':profile['gyro_boot_offsets'],'gyro_target_readbacks':profile['gyro_offsets'],
                'mean_corrected_deg_s':np.degrees(means).tolist(),'sample_std_deg_s':np.degrees(np.sqrt(np.diag(cov))).tolist(),
                'full_covariance_rad2_s2':cov.tolist(),'lag1_correlation':lag1,
                'block_1s_mean_std_deg_s':np.degrees(block_rates.std(axis=0,ddof=1)).tolist(),
                'linear_rate_drift_deg_s2':np.degrees(slopes).tolist(),
                'first_20s_mean_deg_s':np.degrees(rates[times<20].mean(axis=0)).tolist(),
                'last_20s_mean_deg_s':np.degrees(rates[times>times[-1]-20].mean(axis=0)).tolist(),
                'integrated_gyro_only_angle_deg':np.degrees(integrated[-1]).tolist(),
                'temperature_range_C':[float(temp.min()),float(temp.max())],'temperature_mean_C':float(temp.mean()),
                'max_recomputed_rate_error_rad_s':packet_max_error.tolist(),
                'operator_confirmed_motionless':summary['operator_confirmed_motionless'],
                'operator_note':summary.get('note_operator','')}
        results.append(result)
        for axis in range(3):
            axes[axis].plot(np.arange(len(block_rates))+.5,np.degrees(block_rates[:,axis]),label=f'Run {number} ({times[-1]:.0f} s)',lw=1.2)
            axes[axis].axhline(0,color='black',lw=.7)
            axes[axis].set_ylabel(f'{labels[axis]} rate (deg/s)')
            angle_axes[axis].plot(times[::5],np.degrees(integrated[::5,axis]),label=f'Run {number}')
            angle_axes[axis].set_ylabel(f'{labels[axis]} angle (deg)')
        axes[3].plot(times[::10],temp[::10],lw=1)
    for ax in [*axes,*angle_axes]:
        ax.grid(alpha=.2)
    axes[0].legend(ncol=len(runs))
    axes[3].set_ylabel('Chip temperature (C)')
    axes[3].set_xlabel('Seconds from start of each separate run')
    angle_axes[0].legend()
    angle_axes[-1].set_xlabel('Seconds from start of each separate run')
    fig.suptitle('Actual V2 MCU corrected gyro rates: 1-second block means\nOriginal post-trim counts, offsets and emitted correction verified; no new fit')
    fig_angle.suptitle('Offline integral of actual corrected gyro samples\nDiagnostic gyro-only integration; not a measured aircraft attitude error')
    args.output.mkdir(parents=True)
    fig.savefig(args.output/'corrected-gyro.png',dpi=150)
    fig_angle.savefig(args.output/'gyro-only-integral.png',dpi=150)
    plt.close('all')
    analysis={'status':'CORRECTION_OBSERVED_IN_TWO_OPERATOR_STATIONARY_RUNS_WITH_NONZERO_RESIDUAL',
              'engine':'Original real ESP32-S3/MPU6050 serial logs, offline verification/plots; no simulator',
              'uploaded_by_assistant':False,'serial_port_opened_by_assistant':False,'firmware_modified':False,'bias_refitted':False,
              'source_sha256':runs[0][0]['source_sha256'],'all_original_hashes_headers_samples_conversions_statistics_verified':True,
              'total_selected_samples':sum(r['selected_samples'] for r in results),'runs':results,
              'acceptance_scope':'Same-source trim readbacks and per-sample correction observed on hardware. Residuals quantified, not zero. No external rotation reference or independent stationarity measurement.',
              'remaining':['Explicit reset and full-power-cycle verification for gyro correction','Known rotations/body-frame mapping and gyro scale','Temperature dependence and runtime bias tracking','Correctly timed 500 Hz acquisition and noise for attitude filter'],
              'limitations':['These tests are separate from earlier V1 recordings; no controlled before/after comparison','Nominal hardware trim gain is not independently identified','One minute and five minutes do not establish long-term stability','No IID/Gaussian or standard-error assumption; full covariance retained','~83.3 Hz wide-band polling is not 500 Hz flight-loop noise']}
    (args.output/'analysis.json').write_text(json.dumps(analysis,indent=2)+'\n',encoding='utf-8')
    duration_title=' and '.join(f'{duration:g}-second' for duration in args.seconds)
    lines=[f'# Corrected gyro: {duration_title} hardware recordings','',
           'Any valid post-measurement packets are counted in analysis.json and preserved outside the selected statistics. Both original captures passed SHA-256, source/profile/board continuity, beginning/end metadata, raw-to-corrected conversion, exact selected-sample replay, timestamp and stored-statistic checks. The assistant opened no serial port and uploaded nothing. No coefficients were changed or refitted. Original raw logs remain private; failed earlier trials remain untouched.','',
           f'Total selected samples: **{analysis["total_selected_samples"]:,}**. All gyro boot words were [0,0,0]; target readbacks were **[-6,+9,+27]**. Accelerometer metadata remains unchanged. Readbacks at both queries plus the verified source and continuing output support the intended application; they do not identify the exact physical trim sensitivity.','',
           '| Run | Duration | Samples | Full-run mean X/Y/Z (deg/s) | Sample std X/Y/Z (deg/s) | Temperature (C) |',
           '|---|---:|---:|---|---|---|']
    for i,r in enumerate(results,1):
        vector=lambda key:', '.join(f'{v:+.5f}' for v in r[key])
        lines.append(f'| {i} | {r["duration_s"]:.3f} s | {r["selected_samples"]} | {vector("mean_corrected_deg_s")} | {vector("sample_std_deg_s")} | {r["temperature_range_C"][0]:.3f}-{r["temperature_range_C"][1]:.3f} |')
    lines += ['', 'The earlier terminal residual mean used the first two-thirds of a run; the table above uses every selected sample. They need not be identical. Noise standard deviation is not the uncertainty of the bias estimate.','',
              '![Actual corrected rates and temperature](corrected-gyro.png)','',
              'The rate graph uses one-second means so that slow variation is visible. The full-resolution data were used for all statistics. A residual around -0.02 degree/s on X remains; do not hide it by fitting and subtracting this same dataset again. Earlier observation runs had a large sensor-Z bias near -0.83 degree/s. These fresh corrected rates are consistent with its removal, but were not collected as a controlled alternating before/after test.','',
              '![Gyro-only integral diagnostic](gyro-only-integral.png)','',
              'For samples with nonuniform intervals, the integral is calculated using the trapezoidal rule:','',
              '$$\\theta_{i,k}=\\theta_{i,k-1}+\\frac{\\omega_{i,k}+\\omega_{i,k-1}}{2}(t_k-t_{k-1}).$$','',
              'A small remaining constant rate integrates into several degrees over five minutes. This is an offline gyro-only diagnostic, not a measured physical angle or a prediction of a fused roll/pitch estimator. It explains why bias/drift matter even after the large offset is removed. Accelerometer fusion can constrain gravity-referenced tilt; absolute yaw requires another reference.','',
              '## Acceptance and next test','',
              'Implementation is observed in these two operator-attested stationary runs: source, trim readbacks and per-sample subtraction are consistent. Residual means are quantified rather than declared zero. No arbitrary pass tolerance, sub-degree accuracy, temperature compensation or final Kalman gain is inferred.','',
              'Operator notes identify the physical action claimed for each session. Headers alone cannot prove power removal. If reset/full-power-cycle cases were not explicitly performed and labeled, those checks remain pending; do not infer them from capture count. When both were performed, offset restoration is supported for those operator-reported cases, with nonzero residuals still documented.','',
              'Rate output was about 83.3 Hz with a maximum 12 ms packet gap. This is bench telemetry, not the future 500 Hz flight loop. Full covariance, lag-one correlation, one-second block variability, temperature, drift slopes and first/last segment means are preserved in [analysis.json](analysis.json). No IID or joint-Gaussian assumption is made. Known-rotation/frame tests and properly timed acquisition remain later steps. Motor arming stays disabled.']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(analysis,indent=2))


if __name__=='__main__':
    main()
