#!/usr/bin/env python3
"""Pure offline regression of user-requested v5 features; no hardware claims."""
from __future__ import annotations
import sys,tempfile,csv,json,math,time,io,queue
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from thermal_error_fusion import correct_one,process_file,normalised_innovation_squared
from live_calibration_gui import Recorder,Stage,BinaryDecoder,make_test_packet,CSV_COLUMNS
from attempt_filter import only_latest
from lidar_calibration import NLinkDecoder,make_test_frame


def run():
    m={'status':'EXPLORATORY_FIT','temperature_min_c':15.,'temperature_max_c':35.,
       'temperature_ref_c':25.,'intercept':[.1,-.2,.3],
       'slope_per_c':[.02,0.,-.01]}
    nv={'status':'EXPLORATORY_FIT','temperature_min_c':15.,'temperature_max_c':35.,
        'temperature_ref_c':25.,'intercept':list(np.log([.04,.09,.16])),
        'slope_per_c':[0.,0.,0.]}
    b={'status':'REFERENCE_BASED_PROVISIONAL','temperature_min_c':15.,'temperature_max_c':35.,
       'temperature_ref_c':25.,'intercept':2.,'slope_per_c':.5}
    thermal={'gyro_bias_model':m,'gyro_log_variance_model':nv,'bmp_pressure_residual_model':b}
    r={'mpu_ok':'1','mpu_temp_c':30,'gx_dps':'.5','gy_dps':'1','gz_dps':'2',
       'bmp_ok':'1','bmp_temp_c':30,'pressure_pa':'101325'}
    untrusted=correct_one(r,thermal,None,False)
    assert untrusted['gyro_thermal_status']=='CONSTANT_BIAS_ONLY'
    assert untrusted['bmp_thermal_status']=='FACTORY_COMPENSATED_ONLY'
    trusted=correct_one(r,thermal,None,True)
    assert abs(trusted['gx_temp_corrected_dps']-.3)<1e-9
    assert abs(trusted['pressure_temp_reference_corrected_pa']-(101325-4.5))<1e-9
    assert abs(trusted['gz_meas_variance_dps2']-.16)<1e-9
    assert normalised_innovation_squared(2,0,1,1)==2
    assert math.isnan(correct_one({'mpu_ok':'0'},thermal)['gx_temp_corrected_dps'])
    outside=correct_one({**r,'mpu_temp_c':50},thermal,None,True)
    assert outside['gyro_thermal_status']=='MODEL_OUT_OF_RANGE_OR_MALFORMED'
    # Preserve provenance and raw CSV, refusing in-place output.
    with tempfile.TemporaryDirectory() as d:
        d=Path(d); raw=d/'raw.csv';rep=d/'thermal.json';out=d/'corrected.csv'
        with raw.open('w',newline='') as fd:
            w=csv.DictWriter(fd,fieldnames=list(r));w.writeheader();w.writerow(r)
        rep.write_text(json.dumps(thermal));result=process_file(raw,rep,out,apply_provisional=True)
        assert result['rows']==1 and out.exists()
        assert next(csv.DictReader(out.open()))['gx_dps']==r['gx_dps']
        try:process_file(raw,rep,raw)
        except ValueError:pass
        else:raise AssertionError('Raw input overwrite should have been refused')
    # Preparation countdown and redo: a new attempt retains previous raw data
    # but only_latest() must return latest labelled attempt.
    t=Recorder('none','/tmp/v5-not-opened.csv',queue.Queue(),[[1,0,0],[0,1,0],[0,0,1]],settle_seconds=2.5)
    t.run_started=time.monotonic(); stage=Stage('+X','STILL',1,'mpu','test')
    packet=BinaryDecoder().feed(make_test_packet(1,version=3,flags=0x1F))[0]
    sink=io.StringIO();wr=csv.writer(sink);wr.writerow(CSV_COLUMNS)
    assert t.start_stage(stage)
    assert 2.3<t.phase_start-time.monotonic()<2.5
    t.phase_start=0;t._handle_frame(packet,wr);assert t.active is None
    assert t.start_stage(stage);t.phase_start=0
    t._handle_frame(packet,wr)
    rows=list(csv.DictReader(io.StringIO(sink.getvalue())))
    assert len(rows)==2
    assert [int(q['phase_attempt']) for q in rows]==[1,2]
    assert len(only_latest([dict(q,_source_file='/tmp/iq_redo.csv') for q in rows]))==1
    # Lidar optional module remains protocol-compatible and independent of the FC.
    ld=NLinkDecoder()
    frame=ld.feed(make_test_frame(distance_mm=1000))
    assert len(frame)==1 and frame[0].valid and abs(frame[0].distance_m-1)<1e-9
    print('PASS: temperature biases/uncertainties/reference gates, stale-temperature handling,')
    print('      lossless raw CSV, in-place safety, redo attempt provenance, optional LiDAR parser.')

if __name__=='__main__':run()
