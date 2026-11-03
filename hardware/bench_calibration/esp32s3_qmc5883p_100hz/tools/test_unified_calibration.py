#!/usr/bin/env python3
"""Offline unified flow tests. No physical sensor, UART or radio is assumed."""
from pathlib import Path
import csv, io, json, queue, subprocess, sys, tempfile, time, math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from live_calibration_gui import Recorder, Stage, stages, CSV_COLUMNS, OptionalLidarSource, make_test_packet, BinaryDecoder
from lidar_calibration import NLinkDecoder, make_test_frame
from attempt_filter import only_latest
from calibrate import fit_lidar
from argparse import Namespace

def run():
    a=Namespace(gyro_samples=10000,accel_samples=5000,mag_turn_samples=2000,
       mag_tumble_samples=5000,bmp_samples=300,skip_bmp=False,skip_lidar=False,
       lidar_samples=40,mag_only=False)
    q=stages(a)
    assert len(q)==16 and q[-1].sensor=='lidar' and sum(s.sensor=='lidar' for s in q)==4
    a.skip_lidar=True
    assert len(stages(a))==12
    a.mag_only=True
    assert len(stages(a))==4
    recorder=Recorder('COM3','/tmp/sensor.csv',queue.Queue(),[[1,0,0],[0,1,0],[0,0,1]],lidar_port='COM4',settle_seconds=0)
    recorder.run_started=time.monotonic()
    frame=BinaryDecoder().feed(make_test_packet(3,flags=0x5F,version=3))[0]
    recorder.last_frame=frame
    writer_stream=io.StringIO(); writer=csv.writer(writer_stream)
    writer.writerow(CSV_COLUMNS)
    st=Stage('LIDAR','R0250',2,'lidar','test')
    assert recorder.start_stage(st)
    recorder.phase_start=0
    decoded=NLinkDecoder().feed(make_test_frame(251))[0]
    recorder._handle_lidar(decoded,time.monotonic(),writer)
    assert recorder.sample_count==1
    recorder._handle_lidar(decoded,time.monotonic(),writer)
    assert recorder.sample_count==2 and recorder.active is None
    rows=list(csv.DictReader(io.StringIO(writer_stream.getvalue())))
    assert len(rows)==2 and all(r['kind']=='L' and r['pose']=='LIDAR' for r in rows)
    assert abs(float(rows[0]['lidar_reference_m'])-.25)<1e-8
    assert rows[0]['mpu_temp_c'] and rows[0]['bmp_temp_c']
    assert len(only_latest([dict(r,_source_file='/tmp/run.csv') for r in rows]))==2
    # Known linear measurement: z = 1.02 * d + 0.006; fit from four distances.
    rng=np.random.default_rng(6)
    samples=[]
    for d in (.25,.50,1.0,2.0):
        for z in 1.02*d+0.006+rng.normal(0,.001,100):
            samples.append(dict(kind='L',pose='LIDAR',mag_stage=f'R{round(d*1000):04d}',
                lidar_reference_m=str(d),lidar_distance_m=str(z),phase_accepted='1'))
    fit=fit_lidar(samples)
    assert abs(fit['measured_equals_a_times_reference_plus_b']['a']-1.02)<.005
    assert fit['leave_one_distance_out_rms_m']<.005
    with tempfile.TemporaryDirectory() as td:
        path=Path(td)/'lidar_only.csv'
        with path.open('w',newline='') as fd:
            w=csv.DictWriter(fd,fieldnames=CSV_COLUMNS);w.writeheader()
            for r in samples:w.writerow(r)
        cmd=[sys.executable,str(Path(__file__).resolve().parent/'calibrate.py'),str(path),'-o',str(Path(td)/'result')]
        proc=subprocess.run(cmd,capture_output=True,text=True,timeout=40)
        assert proc.returncode==0,(proc.stdout,proc.stderr)
        report=json.loads((Path(td)/'result_calibration.json').read_text())
        assert 'lidar' in report and 'temperature_analysis' in report
    assert not (Path(__file__).resolve().parent/'start_gui.ps1').exists()
    print('PASS: guided optional LiDAR stages, nonblocking UART decoder, CSV/attempts, auto calibration and single launcher')

if __name__=='__main__':run()
