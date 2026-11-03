#!/usr/bin/env python3
"""Offline host-side tests; no physical Wi-Fi or sensor test implied."""
from pathlib import Path
import tempfile, socket, csv, json, math
import numpy as np
from live_calibration_gui import WifiPacketStream, BinaryDecoder, make_test_packet
from thermal_characterization import analyze


def test_transport():
    ctrl=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    ctrl.bind(('127.0.0.1',0));ctrl.settimeout(1)
    recv=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    recv.bind(('127.0.0.1',0))
    port=recv.getsockname()[1]; recv.close()
    with WifiPacketStream(host='127.0.0.1',local_port=port,
                          control_port=ctrl.getsockname()[1]) as client:
        assert client.read()==b''
        data,addr=ctrl.recvfrom(50)
        assert data==b'IQ_HELLO_V1' and addr[1]==port
        two=make_test_packet(10,version=3)+make_test_packet(11,version=3)
        ctrl.sendto(two,('127.0.0.1',port))
        got=client.read();frames=BinaryDecoder().feed(got)
        assert [f.seq for f in frames]==[10,11]
    ctrl.close()
    print('PASS UDP discovery and binary-batch receiver mock with CRC decoding')


def test_thermal():
    with tempfile.TemporaryDirectory() as d:
        root=Path(d);paths=[];refs=[]
        for i,t in enumerate([18.,23.,29.,34.]):
            p=root/f'session{i}.csv';paths.append(p)
            ref=101325.+i*15
            refs.append({'csv':str(p),'reference_pressure_pa':ref})
            cols=['kind','pose','phase_accepted','mpu_ok','bmp_ok','bmp_new',
              'gx_dps','gy_dps','gz_dps','ax_mps2','ay_mps2','az_mps2',
              'mpu_temp_c','bmp_temp_c','pressure_pa']
            rng=np.random.default_rng(100+i)
            with p.open('w',newline='') as fh:
                w=csv.DictWriter(fh,fieldnames=cols);w.writeheader()
                for j in range(300):
                    x=.1+.025*(t-20)
                    r={'kind':'D','pose':'REST','phase_accepted':1,'mpu_ok':1,
                        'bmp_ok':1,'bmp_new':int(j%5==0),
                        'gx_dps':x+rng.normal(0,.02),'gy_dps':-.2+rng.normal(0,.02),
                        'gz_dps':.3+rng.normal(0,.02),
                        'ax_mps2':rng.normal(0,.05),'ay_mps2':rng.normal(0,.05),
                        'az_mps2':9.8+rng.normal(0,.05),
                        'mpu_temp_c':t+rng.normal(0,.03),'bmp_temp_c':t,
                        'pressure_pa':ref+2.0+.7*(t-20)+rng.normal(0,.1)}
                    w.writerow(r)
        ref_file=root/'refs.json';ref_file.write_text(json.dumps({'sessions':refs}))
        data=analyze(paths,ref_file)
        assert data['gyro_bias_model']['status']=='EXPLORATORY_FIT'
        assert data['bmp_pressure_residual_model']['status']=='REFERENCE_BASED_PROVISIONAL'
        assert abs(data['gyro_bias_model']['slope_per_c'][0]-.025)<.003
        assert abs(data['bmp_pressure_residual_model']['slope_per_c']-.7)<.02
        no_ref=analyze(paths)
        assert no_ref['bmp_pressure_residual_model']['status']!='REFERENCE_BASED_PROVISIONAL'
        print('PASS thermal: 4 independent sessions, known coefficients, reference-gating and regression')


if __name__=='__main__':
    test_transport();test_thermal()
