#!/usr/bin/env python3
"""Offline decoder, phase scheduling, pose gating, CSV compatibility tests."""
import csv
import io
import math
import queue
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from live_calibration_gui import (BinaryDecoder, make_test_packet, Frame, Recorder,
    Stage, CSV_COLUMNS, load_axis_map, map_mag, determinant, G, stages, selftest)


def run():
    selftest()
    import argparse
    a=argparse.Namespace(gyro_samples=10000, accel_samples=5000,
        mag_turn_samples=300, mag_tumble_samples=900,bmp_samples=300,skip_bmp=False,skip_lidar=True)
    seq=stages(a)
    assert [x.pose for x in seq]==['REST','+X','-X','+Y','-Y','+Z','-Z','MAG','MAG','MAG','MAG','BMP']
    assert [x.mag_stage for x in seq if x.pose=='MAG']==['YAW','PITCH','ROLL','TUMBLE']
    name,m=load_axis_map(Path(__file__).resolve().parent.parent/'sensor_config.json')
    assert map_mag(m,(1,2,3))==(2,1,3)
    assert determinant(m)==-1
    d=BinaryDecoder()
    f=d.feed(make_test_packet(1, ax=16384,ay=0,az=0))[0]
    wrong=d.feed(make_test_packet(2, ax=0,ay=0,az=16384))[0]
    assert Recorder._accept_phase_frame(f, seq[1])
    # Sensor data arriving in the wrong pose must NOT block the live counter.
    assert Recorder._accept_phase_frame(wrong,seq[1])
    assert Recorder._accept_phase_frame(minus:=d.feed(make_test_packet(3, ax=-14000,ay=0,az=1200))[0],seq[1])
    qmc=d.feed(make_test_packet(4, flags=0x9F, mag=(3000,6000,-1500)))[0]
    assert qmc.mag_sensor_type == 'QMC5883L'
    assert abs(qmc.mag()[0]-100)<1e-6 and abs(qmc.mag()[1]-200)<1e-6
    qmc_p=d.feed(make_test_packet(8, flags=0x5F, mag=(3750,7500,-1500), version=3))[0]
    assert qmc_p.mag_sensor_type == 'QMC5883P'
    assert abs(qmc_p.mag()[0]-100)<1e-6 and abs(qmc_p.mag()[1]-200)<1e-6
    assert qmc_p.mag_new and qmc_p.mag_ok
    legacy_fifo=d.feed(make_test_packet(9,flags=0x5F,version=2))[0]
    assert legacy_fifo.mag_sensor_type == 'HMC5883L' # bit6 in v2 was FIFO-reset, not QMC-P
    # Count only real sensors (invalid MPU and stale MAG/BMP don't count).
    assert not Recorder._accept_phase_frame(Frame(0,5,10000,0,*([0]*15)),seq[1])
    assert not Recorder._accept_phase_frame(Frame(1,5,10000,0,*([0]*15)),seq[-1])

    recorder=Recorder('none','/tmp/not-opened.csv',queue.Queue(),m, settle_seconds=3.0)
    recorder.run_started=time.monotonic()
    dest=io.StringIO(); w=csv.writer(dest); w.writerow(CSV_COLUMNS)
    assert recorder.start_stage(Stage('+X','STILL',2,'mpu','test'))
    assert 2.8 < recorder.phase_start - time.monotonic() <=3.0
    recorder._handle_frame(f,w)
    assert recorder.sample_count == 0 and recorder.active is not None
    assert len(list(csv.DictReader(io.StringIO(dest.getvalue())))) == 0
    # After preparation delay, wrong orientation is STILL counted.
    recorder.phase_start=0
    recorder._handle_frame(wrong,w)
    assert recorder.sample_count==1
    recorder._handle_frame(minus,w)
    assert recorder.sample_count==2 and recorder.active is None
    rows=list(csv.DictReader(io.StringIO(dest.getvalue())))
    assert len(rows)==2 and all(r['phase_accepted']=='1' for r in rows)
    assert all(r['mag_sensor_type']=='HMC5883L' for r in rows)
    assert any(event[0]=='stage_done' for event in list(recorder.events.queue))
    # Pausing stops counting and resuming starts a new 3-second countdown.
    r2=Recorder('none','/tmp/not-opened-2.csv',queue.Queue(),m,settle_seconds=3.0)
    r2.run_started=time.monotonic()
    assert r2.start_stage(Stage('REST','STILL',5,'mpu','test'))
    assert r2.pause_stage()
    dummy=io.StringIO(); wr=csv.writer(dummy)
    r2._handle_frame(f,wr)
    assert not dummy.getvalue() and r2.sample_count==0
    assert r2.resume_stage() and r2.phase_start-time.monotonic() > 2.8
    assert not r2.resume_stage()
    # MAG cannot be faked: missing readings are not counted.
    r3=Recorder('none','/tmp/not-opened-3.csv',queue.Queue(),m,settle_seconds=0)
    r3.run_started=time.monotonic()
    assert r3.start_stage(Stage('MAG','YAW',1,'mag','test'))
    imu_only=d.feed(make_test_packet(6, flags=0x01))[0]
    r3._handle_frame(imu_only, csv.writer(io.StringIO()))
    assert r3.sample_count == 0
    r3._handle_frame(qmc, csv.writer(io.StringIO()))
    assert r3.sample_count == 1
    print('PASS: enter→3s countdown, no pose/stillness gates, real-only MAG/BMP, CSV, stage completion')

if __name__=='__main__':run()
