#!/usr/bin/env python3
"""Synthetic physics & merge-tests; no hardware validation inferred."""
from __future__ import annotations
import sys, math, json, tempfile, csv
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from altitude_fusion import VerticalKF, lidar_com_height_above_hit, lidar_vertical_height, replay
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
    # LiDAR observes local_z - terrain_z; it does not silently label a new
    # terrain-relative clearance as an absolute navigation-height measurement.
    terrain_filter=VerticalKF(lidar_sigma_m=.01)
    assert terrain_filter.update('lidar',1.0)
    assert abs((terrain_filter.x[0]-terrain_filter.x[4])-1.0)<1e-3
    try:terrain_filter.initialize_confirmed_ground_reference(stationary_confirmed=False,ground_confirmed=True)
    except ValueError:pass
    else:raise AssertionError('Ground initialization requires stationary confirmation')
    terrain_filter.initialize_confirmed_ground_reference(stationary_confirmed=True,ground_confirmed=True)
    try:terrain_filter.initialize_confirmed_ground_reference(stationary_confirmed=True,ground_confirmed=True)
    except RuntimeError:pass
    else:raise AssertionError('Ground reference must be one-shot')
    outage_filter=VerticalKF()
    outage_filter.predict(.6,0.)
    assert np.all(np.isfinite(outage_filter.P))
    assert np.linalg.eigvalsh(outage_filter.P).min()>-1e-9
    assert abs(lidar_vertical_height(2.,math.radians(30),0.)-math.sqrt(3))<1e-8
    # User-reported preliminary centered mount: optical reference is 26.658 mm
    # below COM. At level, this adds that vertical lever arm to slant range.
    offset=(0.,0.,-0.026658)
    assert abs(lidar_com_height_above_hit(2.,0.,0.,sensor_offset_body_m=offset)-2.026658)<1e-9
    # A centered vertical lever arm follows the same roll/pitch projection.
    tilted=(2.+0.026658)*math.cos(math.radians(30))
    assert abs(lidar_com_height_above_hit(2.,math.radians(30),0.,sensor_offset_body_m=offset)-tilted)<1e-9
    assert abs(lidar_com_height_above_hit(2.,0.,0.,math.pi/2,offset)-2.026658)<1e-9
    # A forward lever arm contributes at pitch; the vertical projection is
    # independent of ZYX yaw, as expected for world-up height.
    offset_xy=(0.1,0.,-0.026658)
    pitched_expected=0.1*math.sin(math.radians(30))+(2.+0.026658)*math.cos(math.radians(30))
    assert abs(lidar_com_height_above_hit(2.,0.,math.radians(30),0.,offset_xy)-pitched_expected)<1e-9
    assert abs(lidar_com_height_above_hit(2.,0.,math.radians(30),math.pi/2,offset_xy)-pitched_expected)<1e-9
    try:lidar_vertical_height(2.,math.radians(70),0.)
    except ValueError:pass
    else:raise AssertionError('Tilt gating failed')
    try:lidar_com_height_above_hit(0.01,0.,0.)
    except ValueError:pass
    else:raise AssertionError('LiDAR minimum-range gate failed')
    # Replay must require an explicit pressure baseline or a tagged, stationary
    # ground-startup window. Missing LiDAR age is rejected; a fresh row is used.
    with tempfile.TemporaryDirectory() as td:
        source=Path(td)/'vertical.csv'
        fields=['kind','t_us','pressure_pa','bmp_temp_c','bmp_ok','bmp_new',
                'ground_startup_confirmed','stationary','lidar_new','lidar_ok',
                'lidar_range_m','roll_rad','pitch_rad','lidar_age_s']
        with source.open('w',newline='',encoding='utf-8') as fp:
            writer=csv.DictWriter(fp,fieldnames=fields);writer.writeheader()
            for i in range(35):
                row={'kind':'D','t_us':i*10000,'pressure_pa':101325.,'bmp_temp_c':22.,
                     'bmp_ok':1,'bmp_new':1,'ground_startup_confirmed':int(i<30),
                     'stationary':int(i<30),'lidar_new':0,'lidar_ok':0}
                if i==30:
                    row.update(lidar_new=1,lidar_ok=1,lidar_range_m=2.,roll_rad=0.,pitch_rad=0.,lidar_age_s='')
                if i==31:
                    row.update(pressure_pa='',lidar_new=1,lidar_ok=1,lidar_range_m=.05,
                               roll_rad=0.,pitch_rad=0.,lidar_age_s=.01)
                writer.writerow(row)
        altitude_path,report_path=replay(source,Path(td)/'result')
        report=json.loads(report_path.read_text(encoding='utf-8'))
        assert report['vertical_reference']['initialized_from_confirmed_ground']
        assert report['filter_statistics']['lidar_rejected']==1,report['filter_statistics']
        assert report['filter_statistics']['lidar_accepted']==1,report['filter_statistics']
        with altitude_path.open(newline='',encoding='utf-8') as fp:
            columns=next(csv.reader(fp))
        for name in ('baro_relative_height_m','lidar_clearance_measurement_m',
                     'local_z_m','terrain_z_m','lidar_clearance_est_m'):
            assert name in columns,name
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
    print('PASS: timing/covariance, terrain-separated LiDAR geometry, startup reference, range/age/tilt gates, session averaging, GUI timing')

if __name__=='__main__':run()
