#!/usr/bin/env python3
"""Audit the *observed* IntelQCopter acquisition timing from CSV microsecond timestamps.

Distinguishes target frequency, host-observed packet rate and sensor conversion
rate. NOT proof of hardware DRDY synchronization or hard real-time deadlines.
"""
import argparse,csv,json,math
from pathlib import Path
import numpy as np

MOD32=2**32

def analyze(paths):
    results=[]
    for path in paths:
        t=[];seq=[];late=[];miss=[];mpu=[]
        with open(path,newline='',encoding='utf-8-sig') as fh:
            for r in csv.DictReader(fh):
                try:
                    ti=int(r['t_us']);s=int(r['seq']);lag=int(r.get('late_us',0));
                    missed=int(r.get('missed_ticks',0));valid=r.get('mpu_ok')=='1'
                except (ValueError,KeyError,TypeError):continue
                t.append(ti);seq.append(s);late.append(lag);miss.append(missed);mpu.append(valid)
        if len(t)<2:
            results.append({'file':str(path),'error':'<2 timestamped samples'});continue
        t=np.asarray(t,dtype=np.int64);seq=np.asarray(seq,dtype=np.int64)
        late=np.asarray(late,dtype=np.int64);miss=np.asarray(miss,dtype=np.int64)
        dt=np.mod(np.diff(t),MOD32)
        seq_delta=np.mod(np.diff(seq),MOD32)
        mask=(dt>0)&(dt<2_000_000)&(seq_delta>0)&(seq_delta<100000)
        good_dt=dt[mask]
        gaps=seq_delta[mask]-1
        consecutive=mask & (seq_delta==1)
        consecutive_dt=dt[consecutive]
        if not len(good_dt):results.append({'file':str(path),'error':'no valid timestamp differences'});continue
        duration_s=float(np.sum(good_dt)/1e6)
        avg_hz=float(len(good_dt)/duration_s) if duration_s>0 else None
        stats={'file':str(path),'rows':len(t), 'good_intervals':len(good_dt),
          'target_period_us':2000,'target_frequency_hz':500,
          'timestamped_row_rate_hz':avg_hz,
          'dt_us_mean':float(good_dt.mean()),
          'dt_us_median':float(np.median(good_dt)),
          'dt_us_p95':float(np.percentile(good_dt,95)),
          'dt_us_p99':float(np.percentile(good_dt,99)),
          'dt_us_max':int(good_dt.max()),
          'intervals_over_2500us':int(np.count_nonzero(good_dt>2500)),
          'consecutive_packet_intervals':int(np.count_nonzero(consecutive)),
          'consecutive_dt_us_p95':float(np.percentile(consecutive_dt,95)) if len(consecutive_dt) else None,
          'consecutive_dt_us_p99':float(np.percentile(consecutive_dt,99)) if len(consecutive_dt) else None,
          'intervals_over_5000us':int(np.count_nonzero(consecutive_dt>5000)),
          'intervals_over_10000us':int(np.count_nonzero(consecutive_dt>10000)),
          'interrupt_like_gaps_over_10ms_per_min':float(np.count_nonzero(consecutive_dt>10000)*60/duration_s) if duration_s>0 else None,
          'observed_wallclock_fraction_in_over_10ms_gaps':float(np.sum(consecutive_dt[consecutive_dt>10000])/np.sum(good_dt)) if len(good_dt) else None,
          'comment':'A small time-occupancy fraction does NOT certify attitude/control stability; assess actual deadlines and worst-case response.',
          'fraction_over_2500us':float(np.mean(good_dt>2500)),
          'phase_lateness_us_p95':float(np.percentile(late,95)),
          'phase_lateness_us_p99':float(np.percentile(late,99)),
          'phase_lateness_us_max':int(np.max(late)),
          'mpu_valid_fraction':float(np.mean(mpu)),
          'serial_sequence_gaps':int(np.sum(gaps)),
          'device_reported_deadline_miss_counter_last':int(miss[-1]),
          'measurement_limits':['timestamps are acquisition-call start times, not MPU hardware DRDY times',
           'USB/serial row rates may be reduced by buffer drops',
           'consecutive_dt statistics exclude known serial packet sequence gaps',
           'slow shared-bus sensor reads may introduce missed deadlines']}
        results.append(stats)
    return results

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv',type=Path,nargs='+')
    ap.add_argument('--output',type=Path)
    ns=ap.parse_args()
    report=analyze(ns.csv)
    for x in report:
        print(x['file'])
        if 'error' in x:print(' ERROR',x['error']);continue
        print(' Observed row rate:',f"{x['timestamped_row_rate_hz']:.2f} Hz",'target 500 Hz')
        print(' Period p50/p95/p99/max:',x['dt_us_median'],x['dt_us_p95'],x['dt_us_p99'],x['dt_us_max'],'us')
        print(' Intervals >2.5 ms:',x['intervals_over_2500us'],'/',x['good_intervals'])
        print(' Genuine consecutive decoded-frame >10ms gaps/min:',x['interrupt_like_gaps_over_10ms_per_min'])
        print(' Serial sequence gaps:',x['serial_sequence_gaps'], 'device miss counter:',x['device_reported_deadline_miss_counter_last'])
    if ns.output:
        ns.output.parent.mkdir(parents=True,exist_ok=True)
        ns.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print('Saved',ns.output)
if __name__=='__main__':main()
