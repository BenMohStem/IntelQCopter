from pathlib import Path
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/imu-acquisition-benchmark/test_analysis.py'
p.write_text(r'''"""Synthetic negative controls for the PC checker; no device access."""
import copy
import importlib.util
from pathlib import Path
import struct
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('comparison',ROOT/'tools/validation/compare_imu_acquisition.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class AnalysisChecks(unittest.TestCase):
    def setUp(self):
        self.rows=[[i,i*1000,i*1000+100,i*1000+200,i*1000+651,0,1,2,4096,0,4,5,6] for i in range(100)]
        checksum=2166136261
        for row in self.rows:
            for value in struct.pack('>7h',*row[6:]):checksum=((checksum^value)*16777619)&0xffffffff
        self.meta={'mode':'d','samples':100,'elapsed_us':4000000,'injected_pauses':0,'tail_bytes':0,'checksum':checksum}

    def test_known_durations_and_units(self):
        result=module.summarize(self.rows,self.meta)
        self.assertEqual(result['data_read_us']['median'],451)
        self.assertEqual(result['successful_service_us']['median'],651)
        self.assertEqual(result['gyro_mean_raw'],[4,5,6])

    def test_corrupt_payload(self):
        self.rows[10][6]+=1
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_missing_sequence(self):
        del self.rows[10]
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_timestamp_order(self):
        self.rows[10][4]=self.rows[10][3]-1
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_wrong_window(self):
        self.meta['elapsed_us']=3999999
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_fake_fifo_in_direct(self):
        self.rows[10][5]=14
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_wrong_pause_condition(self):
        self.meta['mode']='D'
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)

    def test_valid_fifo_and_impossible_fifo(self):
        self.meta['mode']='f'
        for row in self.rows:row[5]=42
        result=module.summarize(self.rows,self.meta)
        self.assertEqual(result['fifo_oldest_extra_queue_age_proxy_us'],2000)
        self.rows[10][5]=1024
        with self.assertRaises(ValueError):module.summarize(self.rows,self.meta)


if __name__=='__main__':unittest.main(verbosity=2)
''',encoding='utf-8',newline='\n')
p=r/'lessons/imu-acquisition-benchmark/analyze.py'
p.write_text(r'''"""Recheck original real trials, then generate figures and a teaching report."""
from pathlib import Path
import base64,csv,hashlib,importlib.util,json,html,statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
RAW=ROOT/'artifacts/acquisition-comparison/2026-10-30/experiment-01'
OUT=ROOT/'measurements/mpu6050/2026-10-30-acquisition-comparison'
spec=importlib.util.spec_from_file_location('comparison',ROOT/'tools/validation/compare_imu_acquisition.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
previous=json.loads((RAW/'results.json').read_text())
data={};runs=[];manifest=[]
for i,old in enumerate(previous['runs'],1):
    stem=f'run-{i:02d}-{old["mode"]}'
    raw=RAW/(stem+'.log');lines=raw.read_text(encoding='ascii').splitlines()
    rows=[[int(v) for v in line.split(',')[1:]] for line in lines if line.startswith('S,')]
    saved=np.loadtxt(RAW/(stem+'.csv'),delimiter=',',skiprows=1,dtype=np.int64)
    assert np.array_equal(rows,saved)
    header=next(line.split(',') for line in lines if line.startswith('DATA,'))
    metadata={'mode':header[1],'block':old['block'],'samples':int(header[2]),'elapsed_us':int(header[3]),'injected_pauses':int(header[4]),'tail_bytes':int(header[5]),'checksum':int(header[6],16)}
    assert lines[-1]==f'DONE,{old["mode"]},OK'
    result=module.summarize(rows,metadata)
    result['run']=i;result['raw_log_sha256']=hashlib.sha256(raw.read_bytes()).hexdigest()
    runs.append(result);data[i]=np.asarray(rows)
    manifest.append({'path':raw.relative_to(ROOT).as_posix(),'sha256':result['raw_log_sha256']})
groups={m:[x for x in runs if x['mode']==m] for m in 'dfDF'}
labels={'d':'Direct / normal','f':'FIFO / normal','D':'Direct / pauses','F':'FIFO / pauses'}
colors={'d':'#2877ab','f':'#198548','D':'#2877ab','F':'#198548'}
def median(m,key,stat=None):
    return statistics.median(x[key][stat] if stat else x[key] for x in groups[m])

plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
ax=axes[0,0]
for j,m in enumerate('dfDF'):
    ax.scatter([j]*3,[x['read_rate_hz'] for x in groups[m]],c=colors[m],s=42,zorder=3)
    ax.plot([j-.22,j+.22],[median(m,'read_rate_hz')]*2,c=colors[m],lw=2)
ax.axhline(1000,c='#777777',ls=':',lw=1)
ax.set(xticks=range(4),xticklabels=['Direct','FIFO','Direct + pauses','FIFO + pauses'],ylim=(975,1005),ylabel='Captured records per second (zoomed axis)',title='A. Both keep up normally; direct loses throughput under pauses')
ax.tick_params(axis='x',labelsize=8);ax.grid(axis='y',alpha=.25)
ax=axes[0,1];x=np.arange(2)
ax.bar(x-.18,[median(m,'data_read_us','median') for m in 'df'],.36,color='#9dc7ce',label='14-byte data read')
ax.bar(x+.18,[median(m,'successful_service_us','median') for m in 'df'],.36,color=['#2877ab','#198548'],label='Successful service incl. status/count')
ax.set(xticks=x,xticklabels=['Direct','FIFO'],ylabel='Median duration (microseconds)',title='B. Same data transfer; FIFO adds a count-register read')
ax.legend(fontsize=8);ax.grid(axis='y',alpha=.25)
ax=axes[1,0]
for m in 'DF':
    row=next(v for v in runs if v['mode']==m);a=data[row['run']]
    t=a[1:,4]/1000;gap=np.diff(a[:,4])/1000;mask=(t>=80)&(t<=320)
    ax.plot(t[mask],gap[mask],c=colors[m],label=labels[m],lw=1.2)
ax.axhline(1,c='#777777',ls=':',lw=1)
ax.set(xlabel='MCU elapsed time (ms)',ylabel='Gap between read completions (ms)',title='C. A CPU pause delays BOTH readers; FIFO cannot prevent it')
ax.legend(fontsize=8);ax.grid(alpha=.25)
ax=axes[1,1];row=next(v for v in runs if v['mode']=='F');a=data[row['run']];mask=(a[:,4]>=80000)&(a[:,4]<=320000)
ax.step(a[mask,4]/1000,a[mask,5]//14,where='post',c='#198548')
ax.set(xlabel='MCU elapsed time (ms)',ylabel='Complete FIFO packets before read',yticks=[1,2,3],title='D. FIFO queues up to 3 packets, then the reader catches up')
ax.grid(alpha=.25)
fig.suptitle('REAL ESP32-S3 + MPU6050: direct vs FIFO / 12 x 4-second captures\n1 kHz nominal, I2C 400 kHz, same settings; no BLE/PID/motor output',fontsize=13)
fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)

fields=['run','mode','block','samples','elapsed_us','read_rate_hz','injected_pauses','read_median_us','read_p99_us','read_max_us','service_median_us','service_p99_us','service_max_us','gap_p99_us','gap_max_us','gaps_over_1500us','max_fifo_packets']
with (OUT/'runs-summary.csv').open('w',newline='',encoding='utf-8') as f:
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
    for v in runs:
        w.writerow({**{k:v[k] for k in fields if k in v},'read_median_us':v['data_read_us']['median'],'read_p99_us':v['data_read_us']['p99'],'read_max_us':v['data_read_us']['max'],'service_median_us':v['successful_service_us']['median'],'service_p99_us':v['successful_service_us']['p99'],'service_max_us':v['successful_service_us']['max'],'gap_p99_us':v['completion_gap_us']['p99'],'gap_max_us':v['completion_gap_us']['max'],'max_fifo_packets':v['maximum_fifo_packets_before_read']})

table='| Mode | Records in 4 s (3 trials) | Median read | Median successful service | Largest completion gap | Max FIFO queue |\n|---|---|---|---|---|---|\n'
for m in 'dfDF':
    table+=f"| {labels[m]} | {', '.join(str(v['samples']) for v in groups[m])} | {median(m,'data_read_us','median'):.0f} us | {median(m,'successful_service_us','median'):.0f} us | {max(v['completion_gap_us']['max'] for v in groups[m]):.0f} us | {max(v['maximum_fifo_packets_before_read'] for v in groups[m]) if m.lower()=='f' else 'not applicable'} |\n"
example=data[runs[0]['run']][10];read=int(example[4]-example[3]);service=int(example[4]-example[1])
total=sum(v['samples'] for v in runs)
summary={'date':'2026-10-30','engine':previous['engine'],'source_sha256':previous['source_sha256'],'board':'Confirmed ESP32-S3 on operator-approved COM3; flash capacity16MB; private identifiers omitted','config':{'accel_g':8,'gyro_dps':500,'DLPF_CFG':0,'divider':7,'clock':'gyro X PLL','i2c_hz':400000,'packet_bytes':14},'trials':12,'samples_verified':total,'window_us':4000000,'pause_condition':'2500us busy delay every100ms;39observed injections per stressed trial','runs':runs,'checker':'Reparsed every original raw serial sample, matched saved CSV, independently recomputed payload checksum and timing statistics','claim':'In this implementation, direct service is about177us shorter; payload read duration is the same. FIFO preserves throughput across these artificial pauses but still has delayed reads and queued data.','decision':'No automatic switch of production acquisition. Compare synchronized direct reads and full-load deadlines later.','limitations':previous['limitations'],'original_flash_backup':'Saved privately; not published because it may contain credentials/NVS/bonds','left_installed':'Motor-disabled acquisition benchmark; not original motor-capable firmware'}
(OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8',newline='\n')
(OUT/'original-evidence-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8',newline='\n')
text=f"""# Real direct-read versus FIFO comparison

30 October 2026. **12 real ESP32-S3/MPU6050 trials; {total:,} samples independently verified.** No simulated timing is presented as hardware evidence. No motors, BLE, PIDs or calibration acceptance were involved.

![Actual MCU measurements and queue behaviour](comparison.png)

{table}
## What we changed - and kept the same

Both paths used GPIO9/8,400kHz I2C,8g,500deg/s,DLPF_CFG0,divider7,gyro-X PLL and14-byte raw packets. Both polled INT_STATUS and used the same Adafruit BusIO burst-read primitive. FIFO additionally read its byte-count register before each packet. Capture duration was4seconds; raw data/timestamps were buffered in ESP RAM, then printed afterward. Three randomized blocks included both readers with and without injected2500us CPU pauses every100ms. All twelve completed without reported I2C failure or FIFO overflow; sequence/checksum/timestamp and CSV-vs-original replay checks passed. Physical stationarity/level was not independently attested and was not used to claim angle accuracy.

The sample clock is the MPU and timestamps use ESP micros; no external clock calibration was available. About1000.5-1000.7records/s is observed in the unpaused trials, not an externally certified exact frequency. The independent baseline of each four-second window is not phase-synchronized, so one-sample count differences are expected.

## Interpretation

1. The14-byte payload read has the SAME451us median with either method. FIFO did not make that transaction slower.
2. Successful direct service is603us median; FIFO is about780us, including its count read: approximately177us additional overhead for THIS implementation. This is not an inherent minimum or total CPU utilization. Empty-status polling, scheduling, calibration/filter/PID work and motor output are outside that service-only comparison.
3. With pauses, direct delivered3925records per window versus roughly4003in its unpaused baseline: about78fewer (about1.95%). These are throughput differences, not an exact hardware count of which physical samples were skipped. There is no per-sample hardware sequence number or external DRDY trace here.
4. FIFO retained roughly4002-4003records per window and reached3queued packets. The oldest had approximately2ms MORE queue history than the newest at that count observation. This nominal packet-spacing proxy is not its absolute measurement age; read transfer, polling and sensor filtering add latency.
5. BOTH readers had about3.6ms maximum read-completion gaps under pauses. FIFO cannot compute a control output while the CPU is deliberately stalled. Retaining sample history and meeting a fresh1kHz actuation deadline are different requirements.

**Decision:** direct is a credible lower-overhead candidate if scheduling keeps it timely. FIFO is useful for retaining history when servicing is delayed. Do not switch the production path or declare flight readiness from these short standalone trials. A next comparison can use data-ready synchronization and representative BLE/estimation load; external DRDY timing is needed to measure sample age accurately. This study does not compare getEvent() overhead, FIFO batch optimizations or different filter settings.

## Learn to analyse these data with me

### Step1 - identify the columns and units

Each private raw CSV contains index,service_start_us,ready_observed_us,read_start_us,read_end_us,fifo_bytes,ax,ay,az,temp,gx,gy,gz. Times are relative MCU microseconds; raw sensor words are not calibrated physical values. ready_observed is when software finished the status check, NOT the sensor's physical sampling instant.

### Step2 - subtract the correct timestamps

For real row10 of the first recorded trial:

read duration = {int(example[4])} - {int(example[3])} = **{read}us**

successful service duration = {int(example[4])} - {int(example[1])} = **{service}us**

Neither equals complete physical sensor latency. For consecutive records, completion gap = current read_end - previous read_end. A1ms average can hide occasional3.6ms gaps.

### Step3 - inspect distribution, not only average

Sort durations. Median is the middle value; p99 has approximately99%of samples at or below it; maximum is the largest observed value, not a guaranteed worst case. In the stressed direct runs, p99 gaps are near1.075ms but maxima reach3.573ms: the39injected gaps occupy only about1%of intervals. Read the CSV's long-gap count too; a percentile alone can hide rare deadline failures.

### Step4 - compare repetitions before deciding

Open runs-summary.csv. Modes d/f are unpaused direct/FIFO; D/F include pauses. Compare like with like inside each block. Do not treat thousands of samples from one run as thousands of independent experiments: timing is periodic and correlated. Three short blocks show repeatability here, not statistical certification across hardware, temperatures or flight workloads.

### Step5 - verify before plotting

The tool checks record order, all raw-word checksum bytes, timestamp ordering, capture windows, FIFO limits, pause metadata and saved CSV equality with original serial data. It must reject corrupt data rather than silently cleaning away failed samples. Eight synthetic checker controls are provided separately; they are not additional hardware trials.

### Step6 - ask the useful engineering question

Does the full FC receive a sufficiently recent observation and finish the whole estimator/PID/mixer/output path within its deadline under realistic load? Our graphs answer only part of that question. Speed, data preservation and sample freshness must be reported separately.

## Reproduce the analysis without touching the drone

```powershell
$env:PYTHONPATH='C:\\Users\\Mohamed\\Desktop\\Work\\QCopter\\build\\route-pcb\\deps'
& C:\\Users\\Mohamed\\.platformio\\penv\\Scripts\\python.exe lessons/imu-acquisition-benchmark/analyze.py
```

This reuses the private experiment-01 originals. The public repo contains timing summaries, figures and hashes, not the full flash backup or private serial identifiers. Do not run the recorder as a replacement for this analysis command: it would start NEW hardware measurements. The uploaded source is lessons/imu-acquisition-benchmark/main.cpp. The original firmware/main.cpp and its local user edit remain untouched. The board is left on the motor-disabled benchmark; the old program's complete16MB flash backup is preserved privately and is not automatically restored.
"""
(OUT/'report.md').write_text(text,encoding='utf-8',newline='\n')
# Standalone, offline HTML: include the plotted image rather than network assets.
pic=base64.b64encode((OUT/'comparison.png').read_bytes()).decode()
parts=[]
for line in text.splitlines():
    if line.startswith('!['):parts.append(f'<img src="data:image/png;base64,{pic}" alt="Actual acquisition measurements">')
    elif line.startswith('# '):parts.append('<h1>'+html.escape(line[2:])+'</h1>')
    elif line.startswith('## '):parts.append('<h2>'+html.escape(line[3:])+'</h2>')
    elif line.startswith('### '):parts.append('<h3>'+html.escape(line[4:])+'</h3>')
    elif line.strip():parts.append('<p>'+html.escape(line)+'</p>')
(OUT/'report.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Direct vs FIFO - real measurements</title><style>body{max-width:1100px;margin:30px auto;padding:20px;background:#f3f7f1;color:#18382c;font:17px/1.6 Segoe UI,Arial}h1,h2,h3{color:#198548}img{width:100%;background:white}p{overflow-wrap:anywhere}h2{border-top:2px solid #198548;padding-top:20px}</style>'+''.join(parts)+'</html>',encoding='utf-8')
print('Independent replay passed:',total,'actual samples; wrote report, graph, analysis and timing CSV.')
''',encoding='utf-8',newline='\n')
print('Created analysis replay, figures and tutorial generator plus negative controls.')
