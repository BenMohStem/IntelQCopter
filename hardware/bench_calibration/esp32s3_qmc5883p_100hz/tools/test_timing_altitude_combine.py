#!/usr/bin/env python3
"""Synthetic physics & merge-tests; no hardware validation inferred."""
from __future__ import annotations
import sys, math, json, tempfile, csv
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from altitude_fusion import VerticalKF, lidar_vertical_height
from combine_calibrations import aggregate_json_only,combine
from live_calibration_gui import BinaryDecoder,make_test_packet, Recorder, CSV_COLUMNS,Stage
import queue,io,time


def run():
    f=VerticalKF(sigma_acc=0.2,baro_sigma_m=.5,lidar_sigma_m=.05)
    # Exact 10 ms gap must advance position by v*0.010, not v*0.002.
    f.x[1]=2.0;f.predict(0.010,0.0)
    assert abs(f.x[0]-.020)<1e-9, f.x
    f.predict(.002,0.0)
    assert abs(f.x[0]-.024)<1e-9
    for i in range(1000):
        t=i*.002
        f.predict(.002,0.)
        if i%10==0:f.update('lidar', 0.024+2.0*t, sigma=.05)
        if i%50==0:f.update('baro',.024+2.0*t+0.5, sigma=.3)
    assert np.linalg.eigvalsh(f.P).min()>-1e-9
    assert math.isfinite(float(f.x[0])) and f.stats['lidar_accepted']>20
    assert abs(lidar_vertical_height(2.,math.radians(30),0.)-math.sqrt(3))<1e-8
    try:lidar_vertical_height(2.,math.radians(70),0.)
    except ValueError:pass
    else:raise AssertionError('Tilt gating failed')
    # Previous calibration summaries must NOT be treated as 500Hz IID samples.
    r=lambda x:{'input_file':f'run{x}.csv','gyro':{'bias_dps':[x,x*2,0], 'mpu_temperature_c_at_calibration':22+x},
       'accelerometer':{'bias_mps2':[x/100,0,0],'scale_matrix_3x3':np.eye(3).tolist()}}
    res=aggregate_json_only({'one':r(1),'two':r(3)})
    assert abs(res['gyro']['bias_dps'][0]-2.0)<1e-9
    assert res['gyro']['independent_sessions']==2
    # 32-bit MCU time rollover, useful diagnostic of full elapsed time in host CSV.
    d=BinaryDecoder();p0=d.feed(make_test_packet(1))[0]
    rr=Recorder('none','/tmp/notreal.csv',queue.Queue(),[[1,0,0],[0,1,0],[0,0,1]],settle_seconds=0)
    rr.run_started=time.monotonic();rr.start_stage(Stage('REST','STILL',2,'mpu','test'))
    rr.phase_start=0
    output=io.StringIO();writer=csv.writer(output);writer.writerow(CSV_COLUMNS)
    rr._handle_frame(p0,writer)
    p2=d.feed(make_test_packet(2))[0];p2.t_us=p0.t_us+10000
    rr._handle_frame(p2,writer)
    csvrows=list(csv.DictReader(io.StringIO(output.getvalue())))
    assert csvrows[-1]['observed_packet_dt_us']=='10000'
    assert csvrows[-1]['gap_gt_5ms']=='1'
    print('PASS: actual 10ms propagation, SPD covariance, delayed/tilt gates, session-level averaging, GUI timing columns')

if __name__=='__main__':run()
