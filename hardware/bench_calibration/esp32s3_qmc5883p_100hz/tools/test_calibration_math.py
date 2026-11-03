#!/usr/bin/env python3
"""Synthetic test for offline calibration estimator; no real sensor validation."""
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

P=Path(__file__).resolve().parent
sys.path.insert(0,str(P))
from live_calibration_gui import CSV_COLUMNS,G


def run():
    rng=np.random.default_rng(1234)
    b_acc=np.array([.18,-.11,.07]);S_acc=np.diag([1.02,.99,1.04])
    b_gyro=np.array([.45,-.28,.12]);b_mag=np.array([13.0,-7.0,4.0]);
    A_mag=np.array([[1.16,.05,.02],[.06,.87,-.07],[.03,.02,1.06]])
    s=0
    with tempfile.TemporaryDirectory() as tmp:
        f=Path(tmp)/'synthetic.csv'
        with f.open('w',newline='',encoding='utf-8') as stream:
            w=csv.DictWriter(stream,fieldnames=CSV_COLUMNS)
            w.writeheader()
            def emit(pose,acc,gyro,mag,stage='STILL',new_mag=False,new_bmp=False):
                nonlocal s
                row=dict.fromkeys(CSV_COLUMNS,'0')
                row.update(kind='D',seq=str(s),t_us=str((s*2000)&0xFFFFFFFF),pose=pose,mpu_ok='1',
                           mag_ok='1',bmp_ok='1',mag_new=str(int(new_mag)),bmp_new=str(int(new_bmp)),
                           mag_stage=stage,late_us='0',missed_ticks='0',phase_accepted='1')
                row.update(dict(zip(('ax_mps2','ay_mps2','az_mps2'),map(str,acc))))
                row.update(dict(zip(('gx_dps','gy_dps','gz_dps'),map(str,gyro))))
                row.update(dict(zip(('mx_uT','my_uT','mz_uT'),map(str,mag))))
                row['pressure_pa']=str(101325+rng.normal(0,1.2))
                row['bmp_temp_c']='24.0'
                w.writerow(row)
                s+=1
            for i in range(2500):
                emit('REST',rng.normal(0,.03,3)+np.array([0,0,G]),b_gyro+rng.normal(0,.02,3),[0,0,0],new_bmp=i%50==0)
            for name,v in [('+X',[G,0,0]),('-X',[-G,0,0]),('+Y',[0,G,0]),('-Y',[0,-G,0]),('+Z',[0,0,G]),('-Z',[0,0,-G])]:
                for _ in range(1050):
                    acc=np.linalg.solve(S_acc,np.array(v))+b_acc+rng.normal(0,.03,3)
                    emit(name,acc,b_gyro+rng.normal(0,.02,3),[0,0,0])
            for stage,count in [('YAW',300),('PITCH',300),('ROLL',300),('TUMBLE',1000)]:
                for _ in range(count):
                    v=rng.normal(size=3);v/=np.linalg.norm(v)
                    z=A_mag@(v*47)+b_mag+rng.normal(0,.15,3)
                    emit('MAG',[0,0,G],b_gyro,z,stage=stage,new_mag=True)
            for i in range(400):
                emit('BMP',[0,0,G],b_gyro,[0,0,0],new_bmp=True)
        result=subprocess.run([sys.executable,str(P/'calibrate.py'),str(f)],capture_output=True,text=True)
        assert result.returncode==0,(result.stdout,result.stderr)
        report=json.loads(f.with_name('synthetic_calibration.json').read_text())
        print(result.stdout.strip())
        assert np.max(abs(np.array(report['gyro']['bias_dps'])-b_gyro))<.02
        assert np.max(abs(np.array(report['accelerometer']['bias_mps2'])-b_acc))<.02
        assert np.max(abs(np.array(report['magnetometer']['hard_iron_offset_uT'])-b_mag))<1.5
        assert report['magnetometer']['post_fit_radius_cv']<.025
        print('PASS: gyro, six-face accel, 3-D ellipsoid, BMP and output JSON + C++ header on synthetic data')

if __name__=='__main__':run()
