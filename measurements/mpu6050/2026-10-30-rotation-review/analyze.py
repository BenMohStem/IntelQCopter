from pathlib import Path
import json
import hashlib
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(r/'tools/validation'))
from record_xbox_menu import summarize_rotation
out=r/'measurements/mpu6050/2026-10-30-rotation-review'
out.mkdir(exist_ok=True,parents=True)
rotation=np.asarray(json.loads((r/'config/gyro-frame.json').read_text())['rotation_sensor_to_drone'])
summaries=[]
for stamp in ['20261030T201749Z','20261030T202029Z']:
    folder=r/'artifacts/acquisition-comparison/xbox-menu'/stamp
    info=json.loads((folder/'results.json').read_text())
    original=(folder/'session-original.log').read_bytes()
    assert hashlib.sha256(original).hexdigest()==info['session_log_sha256']
    rows=[[int(v) for v in line.decode().split(',')[1:]] for line in original.splitlines() if line.startswith(b'G,')]
    saved=np.loadtxt(folder/'run-01-r.csv',delimiter=',',skiprows=1,dtype=np.int64)
    assert np.array_equal(saved,np.asarray(rows)), 'Original serial differs from CSV'
    verified=summarize_rotation(rows,info['runs'][0])
    data=np.asarray(rows)
    # FIFO nominal sample interval, independent of variable retrieval gaps.
    t=np.arange(len(data))*.001
    sensor=data[:,2:]/65.5
    frozen_bias=np.rad2deg(np.array([.003098404,-.004985037,-.014435425]))
    body=(sensor-frozen_bias)@rotation.T
    integral=np.cumsum(body,axis=0)*.001
    # This is a diagnostic baseline check, not a refitted calibration.
    before=body[t<2]
    after=body[t>18]
    stats={'recording':stamp,'operator_role':'preferred run; physical direction/reference pending' if stamp.endswith('202029Z') else 'exploratory; operator says motion unsuitable',
           'source_sha256':info['source_sha256'],'original_log_sha256':info['session_log_sha256'],
           'verified_samples':len(data),'duration_us':verified['elapsed_us'],'read_rate_hz':verified['read_rate_hz'],
           'max_retrieval_gap_us':verified['max_read_gap_us'],'raw_clipped_samples':int(np.sum(np.any(np.abs(data[:,2:])>=32760,axis=1))),
           'candidate_body_peak_abs_deg_s':np.abs(body).max(axis=0).tolist(),
           'candidate_body_rate_integrals_deg':integral[-1].tolist(),
           'candidate_absolute_axis_integrals_deg':(np.abs(body).sum(axis=0)*.001).tolist(),
           'first2s_candidate_mean_deg_s':before.mean(axis=0).tolist(),'first2s_candidate_std_deg_s':before.std(axis=0,ddof=1).tolist(),
           'last2s_candidate_mean_deg_s':after.mean(axis=0).tolist(),
           'physical_axis_sign_validated':False,'gyro_scale_validated':False,
           'assumptions':'65.5counts/(deg/s), nominal1ms, historical frozen bias and config/gyro-frame.json candidate rotation; no calibration written; integrals are NOT general Euler angles'}
    summaries.append(stats)
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    colors=['#d34c4c','#269b70','#3675ce']
    stride=10
    for i,label in enumerate(['X','Y','Z']):
        axes[0].plot(t[::stride],body[::stride,i],label=label,color=colors[i],lw=1)
        axes[1].plot(t[::stride],integral[::stride,i],label=label,color=colors[i],lw=1.4)
    axes[0].set_ylabel('Candidate body rate (deg/s)')
    axes[1].set_ylabel('Rate integral (deg)\nnot general Euler angles')
    axes[2].plot(t,data[:,1]/1000,color='#677587',lw=.7)
    axes[2].set_ylabel('Read gap (ms)')
    axes[2].set_xlabel('Nominal FIFO sample time (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.axhline(0,color='black',lw=.5)
    axes[0].legend(ncol=3)
    fig.suptitle(stamp+' | Actual ESP32-S3 recording, offline candidate mapping\nPhysical motion reference unconfirmed; no fit or flight acceptance')
    fig.savefig(out/(stamp+'.png'),dpi=160)
    plt.close(fig)
(out/'analysis.json').write_text(json.dumps({'engine':'Offline NumPy analysis of actual ESP32-S3 FIFO gyro recordings; not simulation','runs':summaries,'mapping_source':'config/gyro-frame.json','mapping_sha256':hashlib.sha256((r/'config/gyro-frame.json').read_bytes()).hexdigest()},indent=2)+'\n')
preferred=summaries[1]
text='''# Two 20-second gyro recordings — 30 October 2026

Engine: offline NumPy processing of actual ESP32-S3 FIFO recordings, not a simulator. Original serial logs and CSVs remain privately preserved. Both original log hashes, every gyro CSV row against serial, sequence/count/payload checksum, duration and retrieval gap checks passed independently. No calibration or firmware changed.

The operator says the first motion was unsuitable and the second is better. Keep the first as exploratory, never delete it or treat it as a failed sensor calibration. The physical direction and reference angle of the second are still unspecified.

## Preferred run

![Actual preferred recording; provisional axis mapping](20261030T202029Z.png)

NUMBERS

Top: candidate drone-axis angular rates. Middle: sums of rate × nominal 0.001s. Bottom: intervals between FIFO retrieval completions, not sensor measurement times. Downsampling is for drawing only; calculations use all samples. Integrating body angular-rate components is not general 3D Euler-angle reconstruction; a near-pure axis motion is needed for simple angle interpretation.

Conversion: sensor deg/s = raw / 65.5. Candidate body rate = R × (sensor rate − historical frozen sensor bias). R is the proper-rotation candidate from config/gyro-frame.json, not the accelerometer scaling matrix. That file retains the reviewed historical value; the separate unreviewed production source coefficient edit is untouched. The harness did not write gyro calibration offsets; applying the frozen bias in this offline diagnostic is an assumption, not fresh bias acceptance. First/last2second means and standard deviations are diagnostics, not refitted offsets or proof the assembly was stationary.

No full-scale clip observed if the clipped count is zero. A large integral alone does not prove a correct +90 degree rotation: returning motion can cancel net angle, cross-axis motion can contaminate it, and the physical sign/reference is still required. No sign/scale pass is inferred from the same sensor data being tested. Full accepted_record in transport-results refers to recording integrity only.

## Exploratory first run

![Preserved exploratory recording](20261030T201749Z.png)

Next: identify which side physically rose and whether the second motion was approximately90degrees once or included a return. If not remembered, use a labelled fixture or short phone video on the next run; do not retroactively guess its sign. No PID or attitude flight acceptance yet.
'''
numbers=f"Samples: **{preferred['verified_samples']}**, observed read throughput **{preferred['read_rate_hz']:.2f} Hz**; maximum retrieval gap **{preferred['max_retrieval_gap_us']/1000:.3f} ms**. Near-full-scale samples: **{preferred['raw_clipped_samples']}**. Candidate net axis integrals X/Y/Z: **{', '.join(f'{v:.2f}' for v in preferred['candidate_body_rate_integrals_deg'])} degrees**. These are provisional rate integrals, not validated physical angles."
(out/'README.md').write_text(text.replace('NUMBERS',numbers),encoding='utf-8')
