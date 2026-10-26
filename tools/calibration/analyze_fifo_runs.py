"""Replay original FIFO captures, including failed trials, and report measured timing only."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import record_fifo1000 as fifo
import record_gyro as gyro


class SavedDevice:
    def __init__(self,data):
        self.data=io.BytesIO(data)
    def write(self,value):
        if value!=b'?':
            raise ValueError('Offline replay must not issue capture/motor commands')
    def readline(self):
        return self.data.readline()


def verify(path):
    summary=json.loads((path/'summary.json').read_text(encoding='utf-8'))
    raw=(path/'serial.log').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=summary['serial_sha256']:
        raise ValueError('Original raw log hash mismatch')
    fit=json.loads((gyro.ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    device=SavedDevice(raw)
    header,profile=gyro.read_header(device,io.BytesIO(),summary['source_sha256'],fit)
    if header!=summary['header'] or profile!=summary['profile']:
        raise ValueError('Original header/profile differs from saved metadata')
    stream=fifo.Samples(profile)
    started=False
    observed_failure=None
    final=None
    profiling=None
    while True:
        raw_line=device.readline()
        if not raw_line:
            break
        line=raw_line.decode('ascii').strip()
        if not started:
            if line=='BEGIN,20000':
                started=True
            elif not line.startswith('D,'):
                raise ValueError('Unexpected original pre-capture packet')
            continue
        try:
            if line.startswith('S,'):
                stream.add(line)
            elif line.startswith('D,'):
                stream.diagnostic(line)
            elif line.startswith('PROFILE,'):
                values=[int(v) for v in line.split(',')[1:]]
                profiling=dict(zip(['maximum_i2c_us','maximum_logging_us','maximum_offset_us','handler_overruns','maximum_queue_depth'],values))
            elif line.startswith('END,'):
                final=stream.finish(line)
                break
            else:
                raise ValueError('Unexpected original packet: '+repr(line[:80]))
        except ValueError as error:
            observed_failure=str(error)
            break
    stored=json.loads((path/'samples.json').read_text(encoding='utf-8'))['raw_post_trim_words']
    if stored!=stream.rows:
        raise ValueError('Stored raw samples differ from original serial segment')
    if summary['status']=='FAILED':
        if final is not None or observed_failure is None:
            raise ValueError('Failed-trial replay did not reproduce a capture failure')
        return {'session':path.name,'status':'FAILED_ORIGINAL_REPRODUCED','source_sha256':summary['source_sha256'],
                'serial_sha256':summary['serial_sha256'],'verified_partial_samples':len(stream.rows),'failure':observed_failure}
    if final is None or observed_failure:
        raise ValueError('Completed capture does not replay completely')
    for key,value in final.items():
        if summary[key]!=value:
            raise ValueError('Stored final statistic differs: '+key)
    if profiling!=summary.get('profile_timings'):
        raise ValueError('Profiling metadata differs')
    final_header,_=gyro.read_header(device,io.BytesIO(),summary['source_sha256'],fit)
    if final_header!=header:
        raise ValueError('Final original header differs')
    rows=np.array(stream.rows,dtype=float)
    accel=(rows[:,:3]*9.80665/4096-np.array(profile['remaining_bias']))@np.array(profile['matrix']).T
    rates=rows[:,4:]*np.pi/(180*65.5)-np.array(profile['gyro_remaining_bias'])
    result={'session':path.name,'status':'ORIGINAL_COMPLETE_VERIFIED','source_sha256':summary['source_sha256'],
            'serial_sha256':summary['serial_sha256'],**final,'profile_timings':profiling,
            'sensor_gyro_mean_deg_s':np.degrees(rates.mean(axis=0)).tolist(),
            'sensor_gyro_std_deg_s':np.degrees(rates.std(axis=0,ddof=1)).tolist(),
            'sensor_gyro_covariance_rad2_s2':np.cov(rates,rowvar=False,ddof=1).tolist(),
            'acceleration_mean_body_m_s2':accel.mean(axis=0).tolist(),
            'rotation_status':'not present'}
    result['status']='ORIGINAL_COMPLETE_VERIFIED'
    if 'gyro_to_drone' in profile:
        body=rates@np.array(profile['gyro_to_drone']).T
        result.update({'rotation_status':'Provisional mounting candidate; actual MCU output arithmetic checked, physical known-rotation validation pending',
                       'body_gyro_mean_deg_s':np.degrees(body.mean(axis=0)).tolist(),
                       'body_gyro_covariance_rad2_s2':np.cov(body,rowvar=False,ddof=1).tolist()})
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions',nargs='+',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh directory to preserve original reports')
    trials=[verify(path) for path in args.sessions]
    args.output.mkdir(parents=True)
    analysis={'engine':'Offline exact replay of real ESP32-S3/MPU6050 COM3 serial logs; no simulator',
              'trials':trials,'failed_trials_preserved':True,'motors_enabled':False,
              'note':'Original integrity, rate and observed handler budget are separate from strict periodic FC timing and flight/dynamic validation.'}
    (args.output/'analysis.json').write_text(json.dumps(analysis,indent=2)+'\n',encoding='utf-8')
    complete=[r for r in trials if r['status']=='ORIGINAL_COMPLETE_VERIFIED']
    fig,axes=plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
    labels=[f'Trial {i+1}' for i,r in enumerate(trials) if r['status']=='ORIGINAL_COMPLETE_VERIFIED']
    values=[r['maximum_sample_handler_us'] for r in complete]
    axes[0].bar(labels,values,color=['#059669' if v<1000 else '#dc2626' for v in values])
    axes[0].axhline(1000,color='black',linestyle='--',label='1000 us budget')
    axes[0].set_ylabel('Max sample handler (us)')
    axes[0].legend()
    for i,value in enumerate(values):
        axes[0].text(i,value+10,str(value),ha='center')
    axes[1].bar(labels,[r['maximum_sample_service_gap_us'] for r in complete],color='#2563eb')
    axes[1].axhline(1000,color='black',linestyle='--')
    axes[1].set_ylabel('Max inter-service gap (us)')
    for ax in axes:
        ax.grid(axis='y',alpha=.2)
    fig.suptitle('Actual 20,000-sample hardware captures: failed budgets remain visible\nHandler execution and inter-service jitter are different measurements')
    fig.savefig(args.output/'actual-timing.png',dpi=150)
    plt.close(fig)
    lines=['# Real 1 kHz tests and next gyro-frame step','',
           'All completed captures below were replayed from original raw bytes, not trusted only from summaries. Source/configuration/calibration headers, sample order, FNV payload checksum, all 200 MCU diagnostics, final metadata and saved raw words agree. The malformed-serial trial reproduces its original failure and remains preserved. No motor outputs were enabled.','',
           '| Trial | Samples verified | Rate (Hz) | Maximum handler (us) | Overruns | Result |','|---|---:|---:|---:|---:|---|']
    for i,r in enumerate(trials,1):
        if r['status']=='FAILED_ORIGINAL_REPRODUCED':
            lines.append(f'| {i} | {r["verified_partial_samples"]} partial | unavailable | unavailable | unavailable | Rejected original malformed record |')
        else:
            overruns=r['profile_timings'].get('handler_overruns','not instrumented') if r['profile_timings'] else 'not instrumented'
            decision='Observed handler budget passed' if r['handler_within_1000us_budget'] else 'Handler budget failed'
            lines.append(f'| {i} | {r["samples"]} | {r["observed_capture_rate_hz"]:.2f} | {r["maximum_sample_handler_us"]} | {overruns} | {decision} |')
    lines+=['','![Actual timing before and after repair](actual-timing.png)','',
            'Synchronous logging caused real overruns despite average throughput near 1 kHz. The repaired path uses a bounded FreeRTOS packet queue: core 1 acquires/calculates; core 0 formats/transmits. Queue-full/drain failure rejects capture. Buffered Windows serial reads retain strict validation; the earlier malformed-record cause is not conclusively established. No record was ignored to manufacture a pass.','',
            'The successful handler budget does not establish a perfectly periodic hard deadline. Inter-service gaps above 1000 us remain, FIFO timestamps are nominal sample-clock positions, and clocks have not been calibrated against an external reference. No attitude filter/PID/actuator performance is inferred.','',
            '## Next implemented step: body-frame gyro rates','',
            'The latest source applies a **provisional rotation-only mounting transform** after sensor-frame gyro bias correction. It does not reuse the full accelerometer scale matrix. Actual MCU diagnostics include both sensor and body rates; their arithmetic is checked against original FIFO words. A stationary run validates computation/timing, not physical direction, gyro scale or dynamic attitude accuracy.','',
            'Known physical rotations are the next reference check. The proper-rotation candidate and its assumptions are recorded in config/gyro-frame.json and explained in [the frame guide](../../../docs/gyro-frame-stage.md). Gravity-angle conventions are unchanged; calibration biases and register targets are unchanged.','',
            'Original successful/failed logs and source fingerprints remain local; exact hashes and measured statistics are in [analysis.json](analysis.json).']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'verified_complete_trials':len(complete),'verified_complete_samples':sum(r['samples'] for r in complete),
                      'reproduced_failures':sum(r['status']=='FAILED_ORIGINAL_REPRODUCED' for r in trials)},indent=2))


if __name__=='__main__':
    main()
