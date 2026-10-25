"""Verify repeated real stationary captures and distinguish sensor rate from transport rate."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from analyze_gyro_runs import ROOT, verify_session
from analyze_stationary_imu import angles


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions',nargs=3,type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output directory to preserve evidence')
    fit=json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    runs=[verify_session(path,fit,60) for path in args.sessions]
    if not all(run[1]==runs[0][1] for run in runs):
        raise ValueError('Source/board/configuration/offset state differs across sessions')
    results=[]
    fig,axes=plt.subplots(4,1,figsize=(11,9),constrained_layout=True)
    labels=['X','Y','Z']
    for number,(path,run) in enumerate(zip(args.sessions,runs),1):
        summary,header,profile,samples,rates,temp,times=run
        acceleration=np.array([sample['drone_m_s2'] for sample in samples])
        tilt=angles(acceleration)
        lines=[line for line in (path/'serial.log').read_bytes().splitlines(keepends=True) if line.startswith(b'IMU,')]
        mean_line_bytes=float(np.mean([len(line) for line in lines]))
        blocks=np.array([rates[(times>=i)&(times<i+1)].mean(axis=0) for i in range(int(times[-1]))])
        entry={'session':path.name,'original_serial_sha256':summary['serial_sha256'],
               'samples':len(samples),'duration_s':float(times[-1]),'report_rate_hz':summary['observed_report_rate_hz'],
               'max_report_gap_ms':summary['max_report_gap_ms'],
               'mean_gyro_sensor_deg_s':np.degrees(rates.mean(axis=0)).tolist(),
               'sample_gyro_std_deg_s':np.degrees(rates.std(axis=0,ddof=1)).tolist(),
               'gyro_covariance_rad2_s2':np.cov(rates,rowvar=False,ddof=1).tolist(),
               'one_second_block_gyro_mean_std_deg_s':np.degrees(blocks.std(axis=0,ddof=1)).tolist(),
               'mean_PC_gravity_tilt_deg':np.degrees(tilt.mean(axis=0)).tolist(),
               'sample_PC_tilt_std_deg':np.degrees(tilt.std(axis=0,ddof=1)).tolist(),
               'temperature_mean_C':float(temp.mean()),'temperature_range_C':[float(temp.min()),float(temp.max())],
               'mean_serial_packet_bytes':mean_line_bytes,
               'estimated_uart_bits_s_for_same_text_at_1000Hz_8N1':mean_line_bytes*10*1000,
               'operator_motionless_confirmed':summary['operator_confirmed_motionless'],
               'stationary_assumed_from_delegation':summary.get('stationary_assumed_from_delegation',False),
               'first_uptime_ms':samples[0]['ms'],'last_uptime_ms':samples[-1]['ms']}
        results.append(entry)
        for i,label in enumerate(labels):
            axes[i].plot(np.arange(len(blocks))+.5,np.degrees(blocks[:,i]),label=f'Run {number}',lw=1.2)
            axes[i].set_ylabel(f'Sensor {label} (deg/s)')
            axes[i].axhline(0,color='black',lw=.6)
        axes[3].plot(times[::10],temp[::10],lw=.8)
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].legend(ncol=3)
    axes[3].set_ylabel('Chip temperature (C)')
    axes[3].set_xlabel('Seconds within each separate run')
    fig.suptitle('Three assistant-run stationary-intended captures: actual V2 data\nOne-second gyro block means; no new calibration or high-rate firmware')
    means=np.array([r['mean_gyro_sensor_deg_s'] for r in results])
    tilt_means=np.array([r['mean_PC_gravity_tilt_deg'] for r in results])
    gaps=[(runs[i][3][0]['ms']-runs[i-1][3][-1]['ms'])&0xFFFFFFFF for i in range(1,3)]
    if any(gap>60000 for gap in gaps):
        raise ValueError('Uptime gap/reset ambiguity; do not summarize as consecutive runs')
    analysis={'engine':'Three actual assistant-run COM3 captures verified offline; no simulator',
              'uploaded':False,'firmware_changed':False,'calibration_changed':False,'source_sha256':runs[0][0]['source_sha256'],
              'firmware_commit':runs[0][0].get('firmware_commit'),'total_samples':sum(r['samples'] for r in results),'runs':results,
              'gyro_mean_span_deg_s':np.ptp(means,axis=0).tolist(),
              'sample_std_of_three_run_gyro_means_deg_s':means.std(axis=0,ddof=1).tolist(),
              'PC_tilt_mean_span_deg':np.ptp(tilt_means,axis=0).tolist(),
              'uptime_gaps_between_selected_segments_ms':gaps,
              'hardware_limits':{'accelerometer_new_outputs_hz':1000,'gyro_max_outputs_hz':8000,'configured_divider':7,
                                 'configured_output_schedule_hz':1000,'i2c_hz':400000,'requested_10000_new_samples_hz_possible':False},
              'source':'https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf',
              'limitations':['Stationary support assumed from user delegation, not independently observed','Consecutive runs are not independent long-term trials','No fresh gyro scale/temperature law/bias fit','PC tilt is not actual V3 angle execution','Wide-band ~83 Hz data is not 500 Hz Kalman noise','No successful 1 kHz/8 kHz/10 kHz capture claimed']}
    args.output.mkdir(parents=True)
    fig.savefig(args.output/'repeatability.png',dpi=150)
    plt.close(fig)
    (args.output/'analysis.json').write_text(json.dumps(analysis,indent=2)+'\n',encoding='utf-8')
    report=['# Sampling limits and three real repeatability tests','',
            'The assistant ran three consecutive 60-second COM3 captures on the verified V2 firmware. Five settling seconds preceded each selected interval. No flash, calibration update, deliberate reset or motor command was sent. Stationarity was assumed from the user\'s delegated fixed-support test, not independently observed.','',
            f'All original SHA-256 hashes, source/profile/board continuity, offset readbacks, exact selected samples, per-sample conversions and timestamps were checked. Total: **{analysis["total_samples"]:,} selected samples**. Original logs remain private and unchanged.','',
            '## 1. Why 10,000 Hz cannot be obtained from this sensor','',
            'The [manufacturer register map, section 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) specifies gyro output at 8 kHz for DLPF_CFG = 0, or 1 kHz for the enabled low-pass modes. The accelerometer produces new outputs at 1 kHz. Higher scheduled rates can repeat acceleration samples.','',
            '$$f_s=\\frac{f_{\\rm gyro}}{1+\\mathrm{SMPLRT\\_DIV}}.$$','',
            'Current DLPF mode 0 and divider 7 therefore schedule','',
            '$$f_s=\\frac{8000}{1+7}=1000\\;\\mathrm{Hz}.$$','',
            'This is the sensor update schedule, not the rate recorded by the PC. The active loop polls, checks offsets and prints text with a delay. Its measured report rate is about 83.3 Hz. Intermediate sensor outputs are not all captured. A 10 kHz loop would reread old data; it would not create 10,000 fresh observations or 10,000 independent samples. Identical digital values alone also do not prove duplicate timing, because quantization can produce equal consecutive fresh values.','',
            '| Quantity | Rate or limit |','|---|---:|','| Requested fresh measurements | 10,000 Hz: unsupported |','| Maximum gyro output schedule | 8,000 Hz under specified configuration |','| Maximum new accelerometer outputs | 1,000 Hz |','| Current configured schedule | 1,000 Hz |','| Actual PC capture rate | About 83.3 Hz, measured |','',
            '## 2. Sensor rate, bus throughput and serial throughput are different','',
            'I²C requires eight data bits plus an ACK/NACK clock per transferred byte. Even ignoring addresses and register overhead, three gyro words require six bytes per sample. At 8 kHz this payload alone costs','',
            '$$8000\\cdot6\\cdot9=432000\\;\\mathrm{bits/s}>400000\\;\\mathrm{bits/s}.$$','',
            'Thus the present 400 kHz I²C link cannot continuously drain that complete 8 kHz gyro stream, even if a FIFO batches transfers. Full accel+temperature+gyro samples require 14 bytes. At 1 kHz the payload is','',
            '$$1000\\cdot14\\cdot9=126000\\;\\mathrm{bits/s}.$$','',
            'This fits the bus payload budget, leaving room for bounded protocol overhead. A single 14-byte register burst also needs three overhead bytes (write address, register index, read address), giving an idealized bound','',
            '$$f_{\\rm read}<\\frac{400000}{9(14+3)}\\approx2614\\;\\mathrm{reads/s}.$$','',
            'Real overhead lowers that bound further. Repeated six-axis offset checks in the current loop consume additional traffic. Therefore lossless 1 kHz acquisition needs a different FIFO/batched sampling path, with overflow detection and bounded logging. This is a feasible next acquisition design, not a test that has already run.','',
            'For UART 8N1, each transmitted byte consumes ten line bits. With C text bytes per sample, the required line rate is','',
            '$$B=10 C f_s.$$','',
            f'The measured average packet size in run 1 was {results[0]["mean_serial_packet_bytes"]:.2f} bytes. Reusing that format at 1 kHz would require approximately {results[0]["estimated_uart_bits_s_for_same_text_at_1000Hz_8N1"]:,.0f} bit/s, compared with the nominal configured 921,600 baud. A lean binary 14-byte payload at 1 kHz requires only 140,000 UART bit/s before framing. Native USB transports can have different limits; the UART calculation is explicitly an 8N1 line budget.','',
            '## 3. Actual repeatability results','',
            '| Run | Samples | Gyro mean X/Y/Z (degrees/s) | Gyro sample std X/Y/Z (degrees/s) | Temperature (C) |','|---|---:|---|---|---|']
    for i,r in enumerate(results,1):
        vector=lambda key:', '.join(f'{v:+.6f}' for v in r[key])
        report.append(f'| {i} | {r["samples"]} | {vector("mean_gyro_sensor_deg_s")} | {vector("sample_gyro_std_deg_s")} | {r["temperature_range_C"][0]:.3f}-{r["temperature_range_C"][1]:.3f} |')
    report+=['','![Actual gyro block means and temperature](repeatability.png)','',
             'The figure shows one-second block means, not instantaneous noise or confidence intervals. Sensor axes remain distinct from the calibrated drone axes used for PC gravity angles.','',
             '## 4. Repeatability is not accuracy','',
             'For each run j, calculate its gyro mean and sample covariance without refitting the firmware bias:','',
             '$$\\bar{\\boldsymbol\\omega}_j=\\frac1{N_j}\\sum_k\\boldsymbol\\omega_{j,k},\\qquad S_j=\\frac1{N_j-1}\\sum_k(\\boldsymbol\\omega_{j,k}-\\bar{\\boldsymbol\\omega}_j)(\\boldsymbol\\omega_{j,k}-\\bar{\\boldsymbol\\omega}_j)^T.$$','',
             'The standard deviation across the three run means describes this short-session repeatability:','',
             '$$s_{\\rm runs,i}=\\sqrt{\\frac1{3-1}\\sum_{j=1}^{3}(\\bar\\omega_{j,i}-\\bar{\\bar\\omega}_i)^2}.$$','',
             'Measured X/Y/Z values (degrees/s): '+', '.join(f'{v:.6f}' for v in analysis['sample_std_of_three_run_gyro_means_deg_s'])+'. Mean spans, max minus min: '+', '.join(f'{v:.6f}' for v in analysis['gyro_mean_span_deg_s'])+'.','',
             'These consecutive runs do not establish independent long-term trials, temperature compensation or zero bias. A stable nonzero mean can be repeatable and still biased. Full covariance and PC tilt means/noise are retained in [analysis.json](analysis.json). No current angle is forced to zero because no independent level reference was supplied.','',
             '## 5. Why these samples are not final Kalman-noise measurements','',
             '$$f_{\\rm Nyquist}=\\frac{f_{\\rm recorded}}2\\approx41.7\\;\\mathrm{Hz}.$$','',
             'The existing sensor bandwidth is much wider than this recorded Nyquist frequency. Faster variations can alias into the sampled signal. Noise covariance depends on bandwidth and sampling, so the approximately 83 Hz statistics cannot simply be copied into a 500 Hz filter and called validated. Full-rate FIFO acquisition plus suitable anti-alias filtering/timing is needed for that later measurement. Neither Gaussianity nor independence is inferred, and sample standard deviations are not divided by sqrt(N) to promise bias accuracy.','',
             '**Result:** 10 kHz fresh MPU6050 data is unavailable; these three actual lower-rate runs were completed and preserved. No higher-rate capture, firmware upload, bias refit or motor activation is claimed.']
    (args.output/'report.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
    print(json.dumps(analysis,indent=2))


if __name__=='__main__':
    main()
