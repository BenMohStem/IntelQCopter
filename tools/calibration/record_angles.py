"""Live MCU roll/pitch viewer with automatic original-log saving; no calibration, upload or motor commands."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import serial
import record_gyro as gyro
import record_poses as transport


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--seconds',type=float,default=90)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 10 <= args.seconds <= 600:
        parser.error('Choose 10-600 seconds')
    root=gyro.ROOT
    source=hashlib.sha256((root/'firmware/main.cpp').read_bytes()).hexdigest()
    fit=json.loads((root/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    folder=root/'artifacts/angle-recordings'/transport.stamp()
    folder.mkdir(parents=True)
    result={'status':'FAILED','source_sha256':source,'engine':'Real MCU gravity-only angle observation; no simulator',
            'uploaded':False,'calibration_changed':False,'operator_condition':'Hand-positioned poses and transitions; not stationary gyro-bias data','commands_sent':[]}
    samples=[]
    device=serial.Serial(port=None,baudrate=921600,timeout=.5,write_timeout=2)
    device.port=args.port
    device.dtr=False
    device.rts=False
    try:
        print('Close Serial Monitor. Show level, +Y side up, -Y side up, -X side up, +X side up; pause at each pose.')
        print('Move gently; readings during movement are not a reliable gravity reference. About 20-30 degrees per tilt is enough.')
        input('Press Enter when ready; Ctrl+C saves a partial session: ')
        device.open()
        with (folder/'serial.log').open('xb') as log:
            result['commands_sent'].append('?')
            header,profile=gyro.read_header(device,log,source,fit)
            if not profile.get('tilt_observation'):
                raise ValueError('Upload current V3 bench firmware first')
            result.update({'header':header,'profile':profile})
            started=time.monotonic()
            progress=0
            last_ms=None
            while time.monotonic()-started < args.seconds:
                line=transport.receive(device,log)
                if not line:
                    continue
                sample=gyro.parse_sample(line,profile)
                if last_ms is not None and not 0 < ((sample['ms']-last_ms)&0xFFFFFFFF) <= 100:
                    raise ValueError('Reset/duplicate timestamp or report gap')
                last_ms=sample['ms']
                samples.append(sample)
                elapsed=time.monotonic()-started
                if elapsed >= progress:
                    progress=elapsed+.2
                    norm=sample['norm_m_s2']
                    tilt=sample['tilt_deg']
                    if tilt is None:
                        print(f'{elapsed:5.1f}s  Tilt unavailable near singular orientation')
                    else:
                        warning='  acceleration magnitude differs from g' if abs(norm-9.80665)>1.5 else ''
                        print(f'{elapsed:5.1f}s  ROLL={tilt[0]:+7.2f} deg  PITCH={tilt[1]:+7.2f} deg  |a|={norm:.3f} m/s2{warning}')
                    log.flush()
            result['commands_sent'].append('?')
            final_header,_=gyro.read_header(device,log,source,fit)
            if final_header!=header:
                raise ValueError('Source/calibration state changed')
            result['status']='CAPTURE_COMPLETE_REVIEW_REQUIRED'
    except KeyboardInterrupt:
        result['status']='PARTIAL_OPERATOR_STOPPED'
    except Exception as error:
        result['error']=str(error)
        raise
    finally:
        device.close()
        if samples:
            transport.save_json(folder/'samples.json',{'samples':samples})
        result['selected_samples']=len(samples)
        if (folder/'serial.log').exists():
            result['serial_sha256']=hashlib.sha256((folder/'serial.log').read_bytes()).hexdigest()
        transport.save_json(folder/'summary.json',result)
        print(f'Saved original evidence: {folder}')


if __name__=='__main__':
    main()
