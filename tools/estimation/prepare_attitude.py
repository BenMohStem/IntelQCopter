"""Offline angle-filter preparation. Never opens serial, flashes, or commands motors."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/calibration'))
from analyze_fifo_runs import verify

DT = 0.001
TRAINING = ['20261026T082139649708Z', '20261026T082236554647Z', '20261026T082256835439Z']
HOLDOUT = '20261026T083102215018Z'


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def kalman_angle(observed, rate, dt, q, r):
    """One angle state, rate input; radians throughout. Near-level p/q approximation."""
    observed = np.asarray(observed, dtype=float)
    rate = np.asarray(rate, dtype=float)
    if observed.ndim != 1 or observed.shape != rate.shape or len(observed) < 2:
        raise ValueError('Need matching one-dimensional angle and rate records')
    if not np.all(np.isfinite(observed)) or not np.all(np.isfinite(rate)):
        raise ValueError('Nonfinite sensor values')
    if not np.all(np.isfinite([dt, q, r])) or dt <= 0 or q < 0 or r <= 0:
        raise ValueError('Require finite dt > 0, Q >= 0 and R > 0')
    angle = wrap(observed[0])
    variance = r
    estimates = np.empty(len(observed))
    covariances = np.empty(len(observed))
    gains = np.zeros(len(observed))
    estimates[0] = angle
    covariances[0] = variance
    for i in range(1, len(observed)):
        predicted = wrap(angle + dt * rate[i])
        predicted_variance = variance + q
        gain = predicted_variance / (predicted_variance + r)
        innovation = wrap(observed[i] - predicted)
        angle = wrap(predicted + gain * innovation)
        # Joseph form: preserves a nonnegative variance under finite precision.
        variance = (1 - gain) ** 2 * predicted_variance + gain ** 2 * r
        estimates[i] = angle
        covariances[i] = variance
        gains[i] = gain
    return estimates, covariances, gains


def pooled_covariance(records):
    """Within-record covariance: do not mistake differences of pose for noise."""
    degrees = sum(len(values) - 1 for values in records)
    if degrees <= 0:
        raise ValueError('Insufficient samples')
    return sum((values - values.mean(axis=0)).T @ (values - values.mean(axis=0))
               for values in records) / degrees


def lag_correlation(values, maximum_lag=30):
    residual = values - values.mean(axis=0)
    denominator = np.sum(residual ** 2, axis=0)
    if np.any(denominator <= 0):
        raise ValueError('Zero-variance record has undefined correlation')
    return np.array([np.sum(residual[lag:] * residual[:len(values) - lag], axis=0)
                     / denominator for lag in range(maximum_lag + 1)])


def load_capture(folder, rotation):
    evidence = verify(folder)
    if evidence['status'] != 'ORIGINAL_COMPLETE_VERIFIED':
        raise ValueError('Only complete original captures may enter this study')
    summary = json.loads((folder / 'summary.json').read_text(encoding='utf-8'))
    condition = summary.get('operator_condition', '')
    legacy_condition = summary.get('stationary_condition', '').lower()
    if condition != 'stationary_assumed' and not (not condition and 'assumed' in legacy_condition):
        raise ValueError('Stationary-intended data only; do not fit moving/unknown records')
    profile = summary['profile']
    rows = np.array(json.loads((folder / 'samples.json').read_text(encoding='utf-8'))['raw_post_trim_words'], dtype=float)
    accel = (rows[:, :3] * 9.80665 / 4096 - profile['remaining_bias']) @ np.array(profile['matrix']).T
    gyro = (rows[:, 4:] * np.pi / (180 * 65.5) - profile['gyro_remaining_bias']) @ rotation.T
    angles = np.column_stack((np.arctan2(accel[:, 1], accel[:, 2]),
                             np.arctan2(-accel[:, 0], np.hypot(accel[:, 1], accel[:, 2]))))
    if np.max(np.abs(angles)) > np.radians(20):
        raise ValueError('This candidate study is limited to stationary near-level poses')
    values = np.column_stack((angles, gyro))
    block_means = values.reshape(-1, 1000, 5).mean(axis=1)
    temperature = rows[:, 3] / 340 + 36.53
    metadata = {
        'session': folder.name,
        'firmware_sha256': summary['source_sha256'],
        'original_serial_sha256': summary['serial_sha256'],
        'samples': len(rows),
        'observed_rate_hz': evidence['observed_capture_rate_hz'],
        'condition': 'Stationary assumed from delegation; no measured level reference',
        'means_angle_deg_then_body_rate_deg_s': np.degrees(values.mean(axis=0)).tolist(),
        'standard_deviations_angle_deg_then_body_rate_deg_s': np.degrees(values.std(axis=0, ddof=1)).tolist(),
        'within_run_covariance_angle_rad_then_body_rate_rad_s': np.cov(values, rowvar=False).tolist(),
        'lag1_correlations': lag_correlation(values)[1].tolist(),
        'one_second_block_mean_span_angle_deg_then_body_rate_deg_s': np.degrees(np.ptp(block_means, axis=0)).tolist(),
        'temperature_mean_C_nominal': float(temperature.mean()),
    }
    return angles, gyro, metadata


def factorial_cases():
    """Two factors at two levels: all four combinations, no fractional aliasing."""
    return list(itertools.product([0.0, 0.1], [0.0, 0.1]))


def model_experiment(q, r, gyro_variance):
    t = np.arange(20000) * DT
    # Known pure-roll trajectory. No physics engine or real movement is implied.
    truth = np.radians(10) * np.sin(2 * np.pi * 0.5 * t)
    interval_rate = np.r_[0.0, np.diff(truth) / DT]
    rng = np.random.default_rng(2601026)
    angle_noise = rng.normal(0, np.sqrt(r), len(t))
    rate_noise = rng.normal(0, np.sqrt(gyro_variance), len(t))
    results = []
    traces = []
    for bias_deg_s, acceleration_g in factorial_cases():
        disturbance = acceleration_g * ((t >= 8) & (t < 12))
        observed = np.arctan2(np.sin(truth) + disturbance, np.cos(truth)) + angle_noise
        rate = interval_rate + np.radians(bias_deg_s) + rate_noise
        estimated, _, _ = kalman_angle(observed, rate, DT, q, r)
        error_deg = np.degrees(wrap(estimated - truth))
        evaluated = error_deg[t >= 2]
        results.append({
            'gyro_bias_deg_s': bias_deg_s,
            'body_y_acceleration_g_8_to_12_s': acceleration_g,
            'rmse_deg_after_2_s': float(np.sqrt(np.mean(evaluated ** 2))),
            'max_absolute_error_deg_after_2_s': float(np.max(np.abs(evaluated))),
        })
        traces.append(error_deg)
    responses = np.array([item['rmse_deg_after_2_s'] for item in results]).reshape(2, 2)
    effects = {
        'gyro_bias_high_minus_low_average_rmse_deg': float(responses[1].mean() - responses[0].mean()),
        'acceleration_high_minus_low_average_rmse_deg': float(responses[:, 1].mean() - responses[:, 0].mean()),
        'interaction_difference_of_differences_rmse_deg': float(responses[1, 1] - responses[1, 0] - responses[0, 1] + responses[0, 0]),
        'scope': 'Descriptive fixed-seed model contrast; no p-values or physical effect estimate',
    }
    return t, results, traces, effects


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a fresh output folder; evidence is never overwritten')
    rotation = np.array(json.loads((ROOT / 'config/gyro-frame.json').read_text(encoding='utf-8'))['rotation_sensor_to_drone'])
    base = ROOT / 'artifacts/fifo1000-recordings'
    training = [load_capture(base / name, rotation) for name in TRAINING]
    # Freeze parameters here, BEFORE opening the reserved recording.
    angle_covariance = pooled_covariance([item[0] for item in training])
    gyro_covariance = pooled_covariance([item[1] for item in training])
    r = np.diag(angle_covariance).copy()
    q = DT ** 2 * np.diag(gyro_covariance)[:2]
    holdout = load_capture(base / HOLDOUT, rotation)
    observed, rates, _ = holdout
    traces = [kalman_angle(observed[:, axis], rates[:, axis], DT, q[axis], r[axis])
              for axis in range(2)]
    p_steady = (2 * q * r) / (np.sqrt(q * q + 4 * q * r) + q)
    k_steady = (p_steady + q) / (p_steady + q + r)
    holdout_metrics = []
    for axis, (estimated, variance, gain) in enumerate(traces):
        holdout_metrics.append({
            'axis': ['roll', 'pitch'][axis],
            'accel_angle_std_deg_after_2_s': float(np.degrees(observed[2000:, axis].std(ddof=1))),
            'estimate_std_deg_after_2_s': float(np.degrees(estimated[2000:].std(ddof=1))),
            'estimate_minus_accel_mean_deg_after_2_s': float(np.degrees(wrap(estimated[2000:] - observed[2000:, axis])).mean()),
            'final_formal_sigma_deg': float(np.degrees(np.sqrt(variance[-1]))),
            'final_gain': float(gain[-1]),
            'scope': 'Dispersion and mean difference only, not accuracy; true attitude unmeasured',
        })
    t, cases, model_errors, effects = model_experiment(q[0], r[0], gyro_covariance[0, 0])
    report = {
        'status': 'OFFLINE_CANDIDATE_NOT_ACCEPTED_FOR_FLIGHT_OR_MCU',
        'engine': 'Python float64 scalar estimator; real original-log replay plus explicit synthetic pure-roll model; no Gazebo/ESP emulation',
        'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'unchanged_firmware_sha256': hashlib.sha256((ROOT / 'firmware/main.cpp').read_bytes()).hexdigest(),
        'rotation_config_sha256': hashlib.sha256((ROOT / 'config/gyro-frame.json').read_bytes()).hexdigest(),
        'new_hardware_recordings': 0,
        'uploaded': False,
        'calibration_changed': False,
        'motor_outputs_enabled': False,
        'dt_s_nominal': DT,
        'training': [item[2] for item in training],
        'reserved_recording': holdout[2],
        'within_record_angle_covariance_R_rad2': angle_covariance.tolist(),
        'within_record_body_gyro_covariance_rad2_s2': gyro_covariance.tolist(),
        'white_rate_only_Q_rad2_per_step': q.tolist(),
        'scalar_R_rad2': r.tolist(),
        'conditional_steady_state_gain': k_steady.tolist(),
        'conditional_correction_time_constant_s': (-DT / np.log1p(-k_steady)).tolist(),
        'reserved_recording_results': holdout_metrics,
        'model': {
            'truth': 'Pure roll, 10 degree amplitude, 0.5 Hz, 20 seconds; rate is exact interval-average difference',
            'noise': 'Independent Gaussian white angle/rate draws using training variances, fixed seed 2601026; an illustrative model, not a claim about real noise',
            'design': '2 x 2 full factorial; same noise draws/trajectory in every combination',
            'cases': cases,
            'effects': effects,
        },
        'limits': [
            'Physical gyro axis direction/scale and mounting rotation remain pending operator tests',
            'Stationarity assumed; zero angle/true attitude and absolute sensor accuracy not measured',
            'Two scalar filters use p as roll rate and q as pitch rate only near level; general body-rate/Euler coupling omitted',
            'Q=dt^2*variance(rate) assumes independent rate errors across time, state and measurement; bias, colored noise and acceleration violate this model',
            'R describes stationary repeatability; full angle and gyro covariance retained, but this scalar candidate ignores cross-axis correlation',
            'Formal P is conditional model covariance, not demonstrated real error uncertainty; no Gaussian/IID assumption proven by these recordings',
            'No parameters selected by minimizing held-out dispersion and no calibration residuals refitted',
            'No physical timing, yaw heading, altitude, PID or actuator validation in this stage',
        ],
    }
    args.output.mkdir(parents=True)
    (args.output / 'analysis.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), constrained_layout=True)
    for axis in range(2):
        # Display every tenth sample; estimates and statistics use every sample.
        reference = np.degrees(observed[:, axis].mean())
        axes[axis].plot(t[::10], np.degrees(observed[::10, axis]) - reference, alpha=.45, lw=.5, label='Accel angle')
        axes[axis].plot(t[::10], np.degrees(traces[axis][0][::10]) - reference, lw=1.2, label='Offline Kalman candidate')
        axes[axis].set_ylabel(['Roll', 'Pitch'][axis] + ' relative to accel mean (deg)')
        axes[axis].legend(loc='upper right')
        axes[axis].grid(alpha=.2)
    correlation = lag_correlation(np.column_stack((observed, rates)))
    for axis, name in enumerate(['roll angle', 'pitch angle', 'gyro X', 'gyro Y', 'gyro Z']):
        axes[2].plot(np.arange(31), correlation[:, axis], label=name)
    axes[2].set(xlabel='Lag (nominal ms)', ylabel='Sample correlation')
    axes[2].legend(ncol=3)
    axes[2].grid(alpha=.2)
    fig.suptitle('Reserved real 1 kHz recording: offline filtering, not an accuracy test\nPhysical gyro frame remains provisional; no firmware change')
    fig.savefig(args.output / 'reserved-replay.png', dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    for case, error in zip(cases, model_errors):
        ax.plot(t[::10], error[::10], label=f"bias {case['gyro_bias_deg_s']} deg/s, acceleration {case['body_y_acceleration_g_8_to_12_s']} g")
    ax.axvspan(8, 12, color='grey', alpha=.12)
    ax.set(xlabel='Model time (s)', ylabel='Estimated minus known roll (deg)', title='Four-case full factorial MODEL: same motion and noise draws\nGrey interval: optional translational acceleration; no physical flight')
    ax.legend()
    ax.grid(alpha=.2)
    fig.savefig(args.output / 'factorial-model.png', dpi=150)
    plt.close(fig)
    print(json.dumps({'verified_real_samples_reused': sum(item[2]['samples'] for item in training) + holdout[2]['samples'],
                      'new_physical_tests': 0, 'modeled_combinations': len(cases), 'candidate_K': k_steady.tolist(),
                      'reserved_metrics': holdout_metrics, 'model_results': cases}, indent=2))


if __name__ == '__main__':
    main()
