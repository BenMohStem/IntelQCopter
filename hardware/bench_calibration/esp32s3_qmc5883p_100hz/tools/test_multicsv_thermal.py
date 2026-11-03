#!/usr/bin/env python3
"""Synthetic-only multi-CSV thermal regression and checkpoint tests.
Not a hardware sensor or flight validation.
"""
import csv
import io
import json
import math
import queue
import subprocess
import sys
import tempfile
import threading
import time
import types
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent
sys.path.insert(0,str(P))
from live_calibration_gui import CSV_COLUMNS, Recorder, G, make_test_packet

def generate(path,T,rng):
    seq=0
    gyro_ref=np.array([.2,-.3,.6])
    gyro_slope=np.array([.02,-.012,.015])
    acc_ref=np.array([.1,-.2,.04])
    acc_slope=np.array([.005,-.002,.001])
    def emit(writer,pose,acc,gyro,mag=(0,0,0),mag_new=False,bmp_new=False,stage='STILL'):
        nonlocal seq
        r=dict.fromkeys(CSV_COLUMNS,'0')
        r.update(kind='D',seq=str(seq),t_us=str(seq*2000),pose=pose,mpu_ok='1',bmp_ok='1',mag_ok='1',mag_new=str(int(mag_new)),bmp_new=str(int(bmp_new)),mag_stage=stage,phase_accepted='1',mag_sensor_type='QMC5883P')
        r.update({key:str(val) for key,val in zip(('ax_mps2','ay_mps2','az_mps2'),acc)})
        r.update({key:str(val) for key,val in zip(('gx_dps','gy_dps','gz_dps'),gyro)})
        r.update({key:str(val) for key,val in zip(('mx_uT','my_uT','mz_uT'),mag)})
        r['mpu_temp_c']=str(T)
        r['bmp_temp_c']=str(T-5)
        r['pressure_pa']=str(101300+T*.05+rng.normal(0,1))
        writer.writerow(r);seq+=1
    bias_g=gyro_ref+gyro_slope*(T-27)
    bias_a=acc_ref+acc_slope*(T-27)
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=CSV_COLUMNS);w.writeheader()
        for i in range(1300):
            emit(w,'REST',np.array([0,0,G])+bias_a+rng.normal(0,.012,3),bias_g+rng.normal(0,.01,3),bmp_new=i%50==0)
        for pose,ideal in [('+X',(G,0,0)),('-X',(-G,0,0)),('+Y',(0,G,0)),('-Y',(0,-G,0)),('+Z',(0,0,G)),('-Z',(0,0,-G))]:
            for i in range(650):
                emit(w,pose,np.array(ideal)+bias_a+rng.normal(0,.02,3),bias_g+rng.normal(0,.01,3))
        for st in ('YAW','PITCH','ROLL','TUMBLE'):
            for i in range(350):
                unit=rng.normal(size=3);unit/=np.linalg.norm(unit)
                emit(w,'MAG',[0,0,G],bias_g,unit*47+np.array([4,-8,12])+rng.normal(0,.15,3),mag_new=True,stage=st)
        for i in range(100):
            emit(w,'BMP',[0,0,G],bias_g,bmp_new=True)

def test_multi_csv():
    rng=np.random.default_rng(8)
    with tempfile.TemporaryDirectory() as td:
        root=Path(td)
        paths=[root/f'capture_{t}.csv' for t in (23,27,31)]
        for p,t in zip(paths,(23,27,31)):generate(p,t,rng)
        out=root/'combined'
        run=subprocess.run([sys.executable,str(P/'calibrate.py'),*[str(p) for p in paths],'-o',str(out)],capture_output=True,text=True,timeout=180)
        assert run.returncode==0,(run.stdout,run.stderr)
        report=json.loads((root/'combined_calibration.json').read_text())
        assert len(report['input_files'])==3
        assert abs(report['logging_quality']['phase_metrics']['REST/STILL']['mpu_die_temp_c']['median']-27)<.5
        assert 'temperature_uncertainty_analysis' in report
        assert 'temperature_dependent_measurement_equations' in report
        assert report['temperature_analysis']['gyro_bias_versus_mpu_temperature']['status']=='ESTIMATED_NOT_VALIDATED'
        slope=report['temperature_analysis']['gyro_bias_versus_mpu_temperature']['slope_per_c']
        assert np.max(abs(np.array(slope)-[.02,-.012,.015]))<.003,slope
        assert report['temperature_analysis']['accelerometer_bias_versus_mpu_temperature']['status']=='ESTIMATED_NOT_VALIDATED'
        assert report['temperature_analysis']['bmp280_pressure_versus_temperature_diagnostic']['status']=='ESTIMATED_NOT_VALIDATED'
        assert 'GYRO_BIAS_TEMP_SLOPE_DPS_PER_C' in (root/'combined_calibration.h').read_text()
        assert report['magnetometer']['sensor'].startswith('QMC5883P')
        print('PASS: 3 CSV files; 3 fitted thermal models; gyro slope recovered; BMP diagnostic-only')

def test_checkpoint():
    fake=types.ModuleType('serial')
    class FakePort:
        def __init__(self,*a,**k):self.i=0
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def read(self,n):
            time.sleep(.004)
            self.i+=1
            return make_test_packet(self.i)
    fake.Serial=FakePort
    original=sys.modules.get('serial')
    sys.modules['serial']=fake
    try:
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'sample.csv'
            r=Recorder('FAKE',p,queue.Queue(),[[1,0,0],[0,1,0],[0,0,1]],settle_seconds=0)
            r.start();assert r.started.wait(3)
            assert r.start_stage(__import__('live_calibration_gui').Stage('REST','STILL',50,'mpu','test'))
            time.sleep(.06)
            r.save_checkpoint();time.sleep(.1)
            assert r.completed_files==[p]
            r.stop_event.set();r.join(timeout=3)
            assert not r.is_alive() and r.error is None,r.error
            assert p.exists() and r.current_file.exists()
            assert r.current_file.name.endswith('_part02.csv')
            for f in [p,r.current_file]:
                with f.open(newline='') as stream:
                    rows=list(csv.DictReader(stream))
                assert rows and all(x['kind']=='D' for x in rows)
            print('PASS: close/save checkpoint without stopping active 500Hz-style capture')
    finally:
        if original is not None:sys.modules['serial']=original
        else:sys.modules.pop('serial',None)

if __name__=='__main__':
    test_multi_csv()
    test_checkpoint()
