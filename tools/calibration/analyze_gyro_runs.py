"""Verify original stationary gyro captures, freeze a bias and test later runs offline."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import record_gyro as recording

ROOT = Path(__file__).resolve().parents[2]


def verify_selection(original, selected, verified_packets):
    if not selected:
        raise ValueError('No selected samples')
    starts = [i for i, sample in enumerate(original) if sample['ms'] == selected[0]['ms']]
    if len(starts) != 1:
        raise ValueError('Missing/ambiguous selected start')
    begin = starts[0]
    end = begin + len(selected)
    if original[begin:end] != selected or end != verified_packets or len(original) < verified_packets:
        raise ValueError('Selected samples do not match the exact original measurement segment')
    # The final query can preserve a packet produced after the timed capture ended.
    # Verify its timestamp too; keep it outside the statistics without modifying originals.
    for previous, current in zip(original, original[1:]):
        if not 0 < ((current['ms']-previous['ms']) & 0xFFFFFFFF) <= 100:
            raise ValueError('Invalid timestamp/gap in original capture including boundary packets')
    return len(original)-end


def verify_session(path, fit, requested_seconds=60):
    summary = json.loads((path/'summary.json').read_text(encoding='utf-8'))
    if summary['status'] != 'CAPTURE_COMPLETE_REVIEW_REQUIRED':
        raise ValueError('Incomplete recording: '+path.name)
    raw = (path/'serial.log').read_bytes()
    if hashlib.sha256(raw).hexdigest() != summary['serial_sha256']:
        raise ValueError('Original serial hash changed')
    class SavedDevice:
        def __init__(self, data):
            self.stream = io.BytesIO(data)
        def write(self, data):
            if data != b'?':
                raise ValueError('Unexpected offline query')
        def readline(self):
            return self.stream.readline()
    # Replay the same strict header verifier from preserved bytes, not a serial port.
    device = SavedDevice(raw)
    header, profile = recording.read_header(device, io.BytesIO(), summary['source_sha256'], fit)
    if header != summary['header'] or profile != summary['profile']:
        raise ValueError('Stored header/profile differs from original')
    original = []
    while True:
        line = device.readline()
        if not line:
            raise ValueError('Missing final query/header')
        if line.strip() == b'QUERY_REPLY':
            final_data = line + device.stream.read()
            break
        original.append(recording.parse_sample(line.decode('ascii').strip(), profile))
    final_header, final_profile = recording.read_header(SavedDevice(final_data), io.BytesIO(), summary['source_sha256'], fit)
    if final_header != header or final_profile != profile:
        raise ValueError('State changed during recording')
    selected = json.loads((path/'samples.json').read_text(encoding='utf-8'))['selected_after_settling']
    if len(selected) != summary['samples']:
        raise ValueError('Sample count mismatch')
    summary['offline_post_measurement_packets_verified'] = verify_selection(
        original, selected, summary['all_packets_verified_including_settling'])
    timing = recording.validate_timing(selected, requested_seconds)
    stats = recording.summarize(selected)
    if "gyro_bias" in profile:
        stats["residual_bias_sensor_rad_s"] = stats.pop("candidate_bias_sensor_rad_s")
        stats["residual_bias_sensor_deg_s"] = stats.pop("candidate_bias_sensor_deg_s")
        stats.pop("offline_predicted_held_out_mean_after_bias_rad_s")
        stats["held_out_mean_corrected_rad_s"] = stats.pop("held_out_mean_raw_rad_s")
        stats["note"] = "Rates already corrected by uploaded firmware. Statistics are residuals, not a new bias to add automatically. No correction was written by this recorder."
        if not summary.get("firmware_gyro_correction_active") or summary.get("gyro_offsets_written_by_recorder") is not False:
            raise ValueError("Unexpected correction/recorder write state")
    for key, value in {**timing, **stats}.items():
        if key == 'status':
            continue
        if value != summary[key]:
            raise ValueError('Recomputed statistic differs: '+key)
    rates = np.array([sample['gyro_sensor_rad_s'] for sample in selected])
    temperature = np.array([sample['temperature_C'] for sample in selected])
    times = np.array([((sample['ms']-selected[0]['ms']) & 0xFFFFFFFF)/1000 for sample in selected])
    # The same chip and offset state are compared internally; never export the board ID.
    return summary, header, profile, selected, rates, temperature, times


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions', nargs=3, type=Path, required=True,
                        help='Chronological runs; first two estimation segments fit, third entirely held out')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output directory to preserve earlier evidence')
    fit = json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    runs = [verify_session(path, fit) for path in args.sessions]
    if not all(run[1] == runs[0][1] for run in runs):
        raise ValueError('Source/board/profile/offsets differ across runs')
    if any("gyro_bias" in run[2] for run in runs):
        raise ValueError("Bias fitting requires V1 observation data; use analyze-corrected-gyro for V2 residuals")
    training = np.concatenate([run[4][:len(run[4])*2//3] for run in runs[:2]])
    bias = training.mean(axis=0)
    covariance = np.cov(training, rowvar=False, ddof=1)
    # No data from run 3, nor the temporal holdouts of runs 1/2, enters this fit.
    results = []
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), constrained_layout=True)
    axes_names = ['X', 'Y', 'Z']
    colors = ['#2563eb', '#ea580c', '#059669']
    for index, (path, run) in enumerate(zip(args.sessions, runs)):
        summary, _, profile, selected, rates, temperature, times = run
        corrected = rates - bias
        n = len(rates)
        centred = rates-rates.mean(axis=0)
        lag1 = [float(np.corrcoef(centred[:-1,axis],centred[1:,axis])[0,1]) for axis in range(3)]
        blocks = [corrected[(times >= second) & (times < second+1)].mean(axis=0)
                  for second in range(int(times[-1]))]
        block_values = np.array(blocks)
        integrated = np.sum((corrected[1:]+corrected[:-1])*np.diff(times)[:,None]/2,axis=0)
        holdout = corrected[n*2//3:]
        entry = {'session':path.name,'samples':n,'duration_s':float(times[-1]),
                 'serial_sha256':summary['serial_sha256'],
                 'mean_raw_deg_s':np.degrees(rates.mean(axis=0)).tolist(),
                 'sample_std_deg_s':np.degrees(rates.std(axis=0,ddof=1)).tolist(),
                 'covariance_rad2_s2':np.cov(rates,rowvar=False,ddof=1).tolist(),
                 'lag1_correlation_per_axis':lag1,
                 'mean_after_frozen_bias_deg_s':np.degrees(corrected.mean(axis=0)).tolist(),
                 'last_third_mean_after_frozen_bias_deg_s':np.degrees(holdout.mean(axis=0)).tolist(),
                 'one_second_block_mean_std_deg_s':np.degrees(block_values.std(axis=0,ddof=1)).tolist(),
                 'offline_integrated_residual_angle_deg':np.degrees(integrated).tolist(),
                 'temperature_range_C':[float(temperature.min()),float(temperature.max())],
                 'temperature_mean_C':float(temperature.mean()),
                 'operator_confirmed_motionless':summary['operator_confirmed_motionless'],
                 'gyro_offset_registers':profile['gyro_offsets']}
        results.append(entry)
        for axis in range(3):
            axes[axis].plot(np.arange(len(block_values))+.5,np.degrees(block_values[:,axis]),color=colors[index],label=f'Run {index+1}',lw=1.3)
            axes[axis].axhline(0,color='black',lw=.7)
            axes[axis].set_ylabel(f'{axes_names[axis]} residual (deg/s)')
        axes[3].plot(times[::10],temperature[::10],color=colors[index],label=f'Run {index+1}')
    axes[0].legend(ncol=3)
    for ax in axes:
        ax.grid(alpha=.2)
    axes[3].set_ylabel('Chip temperature (C)')
    axes[3].set_xlabel('Seconds within each run (separate sessions overlaid)')
    fig.suptitle('Actual MPU6050 data: 1-second mean residual after OFFLINE frozen bias\nFirst two estimation segments fit; whole third run held out. No corrected firmware test yet.')
    args.output.mkdir(parents=True)
    fig.savefig(args.output/'gyro-runs.png',dpi=150)
    plt.close(fig)
    report = {'engine':'Offline verification/replay of real ESP32-S3/MPU6050 recordings; no simulation',
              'physical_test_by_assistant':False,'uploaded':False,'gyro_calibration_applied_to_firmware':False,
              'all_original_hashes_headers_conversions_samples_and_statistics_verified':True,
              'source_sha256':runs[0][0]['source_sha256'],
              'fit_policy':'First two thirds of runs 1 and 2; last thirds and entire run 3 reserved',
              'candidate_bias_sensor_rad_s':bias.tolist(),'candidate_bias_sensor_deg_s':np.degrees(bias).tolist(),
              'training_covariance_rad2_s2':covariance.tolist(),
              'training_std_rad_s':np.sqrt(np.diag(covariance)).tolist(),
              'runs':results,
              'acceptance':'Candidate supports a provisional bias-application bench lesson; corrected hardware validation remains required.',
              'limitations':['Operator-attested stationarity, not independently observed','Chip temperature changes; no fitted temperature compensation','Telemetry ~90.9 Hz with wide sensor bandwidth, not final 500 Hz noise','No joint Gaussian/IID assumption or standard error from sigma/sqrt(N)','Known angular rate/mounting/temperature and restart validation remain pending','Constant bias subtraction cannot remove random noise or all slow drift']}
    (args.output/'analysis.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    lines=['# Three actual gyro recordings: review','',
           'Original hashes, both headers, board/source continuity, conversions, every selected original sample and stored statistics were independently replayed and verified. No serial port was opened; no upload or firmware bias change was made. Failed and interrupted captures remain untouched.','',
           'The data were recorded by the user on a support, with motionlessness confirmed at the recorder prompt. This is a real-sensor analysis, not a simulation.','',
           '## Frozen candidate bias','',
           'Fit only the first two-thirds of runs 1 and 2. Reserve both final thirds and the entire third run. Rates are in sensor axes.','',
           '$$\\widehat{\\mathbf b}_g=['+', '.join(f'{v:.9f}' for v in bias)+']^T\\ \\mathrm{rad/s}.$$','',
           'Equivalent degrees/s: '+', '.join(f'{v:+.6f}' for v in np.degrees(bias))+'.','',
           '$$\\boldsymbol\\omega_{\\mathrm{corrected}}=\\boldsymbol\\omega_{\\mathrm{reported}}-\\widehat{\\mathbf b}_g.$$','',
           'Here that subtraction is **offline only**. All three actual MPU gyro offset words remained zero. The accelerometer calibration was unchanged.','',
           '| Run | Samples | Raw mean X/Y/Z (deg/s) | Frozen-bias residual X/Y/Z (deg/s) | Temperature (C) |',
           '|---|---:|---|---|---|']
    for i,r in enumerate(results):
        vector=lambda key:', '.join(f'{v:+.5f}' for v in r[key])
        lines.append(f'| {i+1} | {r["samples"]} | {vector("mean_raw_deg_s")} | {vector("mean_after_frozen_bias_deg_s")} | {r["temperature_range_C"][0]:.3f}-{r["temperature_range_C"][1]:.3f} |')
    lines += ['', '![Measured residual block means and temperature](gyro-runs.png)','',
              'The graph shows one-second block means to expose slow changes; it does not show the full instantaneous noise amplitude. Each session has its own time zero. Run 3 never alters the fitted bias.','',
              '## Interpretation and next small step','',
              'A fixed bias removes the large repeatable mean, especially sensor Z. Run-to-run differences and temperature changes remain; a constant correction does not promise zero drift. The JSON retains full covariance, individual sample deviations, lag-one correlation, one-second block variability and the offline integrated residual angle. Integrated angles are diagnostic replay, not measured orientation error.','',
              'Sample deviations describe this subsampled wide-band bench stream. They are not final Kalman noise at 500 Hz. Do not divide by the square root of sample count to claim a precision without accounting for temporal correlation. Neither independence nor Gaussianity is established. A single support test cannot validate scale/mounting errors; chip temperature is not an external thermometer.','',
              '**Next:** apply this frozen candidate in a small gyro-bias firmware lesson, with recorded baseline and rounding accounted for once, then make a new stationary recording and repeat reset/power-cycle checks. Keep attitude estimation behind that gate. No motor output is enabled.','',
              'Original raw console logs remain local under artifacts/gyro-recordings. Only reviewed derived values and graphs are published. Exact session IDs and SHA-256 hashes are in [analysis.json](analysis.json).']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'candidate_deg_s':report['candidate_bias_sensor_deg_s'],'runs':results},indent=2))


if __name__ == '__main__':
    main()
