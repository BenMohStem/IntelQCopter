"""Record and verify 20,000 actual calibrated-FIFO samples; no flash or motor commands."""
import argparse
import hashlib
import json
import math
import struct
import time

import serial
import record_gyro as gyro
import record_poses as transport

ROOT=gyro.ROOT
COUNT=20000


class Samples:
    def __init__(self,profile):
        self.profile=profile
        self.rows=[]
        self.checksum=2166136261
        self.diagnostics=0
        self.last_diagnostic=-1

    def add(self,line):
        fields=line.split(',')
        if len(fields)!=9 or fields[0]!='S' or int(fields[1])!=len(self.rows) or len(self.rows)>=COUNT:
            raise ValueError('Missing/duplicate/reordered sample or wrong packet size')
        counts=[int(v) for v in fields[2:]]
        if any(not -32768 < value < 32767 for value in counts):
            raise ValueError('Invalid/clipped raw sensor sample')
        for value in struct.pack('>7h',*counts):
            self.checksum=((self.checksum^value)*16777619)&0xFFFFFFFF
        self.rows.append(counts)

    def diagnostic(self,line):
        fields=line.split(',')
        if len(fields)!=10 or fields[0]!='D':
            raise ValueError('Malformed MCU diagnostic')
        sequence=int(fields[1])
        if sequence!=len(self.rows)-1 or sequence%100 or sequence<=self.last_diagnostic:
            raise ValueError('Diagnostic detached from its actual raw sample')
        c=self.rows[sequence]
        centred=[c[i]*9.80665/4096-self.profile['remaining_bias'][i] for i in range(3)]
        body=[sum(row[i]*centred[i] for i in range(3)) for row in self.profile['matrix']]
        rates=[c[i+4]*math.pi/(180*65.5)-self.profile['gyro_remaining_bias'][i] for i in range(3)]
        yz=math.hypot(body[1],body[2])
        expected=body+rates+[math.degrees(math.atan2(body[1],body[2])),math.degrees(math.atan2(-body[0],yz))]
        actual=[float(v) for v in fields[2:]]
        for i,(value,reference) in enumerate(zip(actual,expected)):
            if i>=6 and yz<=.001002:
                if not math.isnan(value):
                    raise ValueError('Expected unavailable angle near singularity')
            elif not math.isfinite(value) or abs((value-reference+180)%360-180 if i>=6 else value-reference)>(.001+math.degrees(.000002/max(yz,.000001)) if i>=6 else .00005):
                raise ValueError('MCU calibrated calculation differs from raw packet')
        self.last_diagnostic=sequence
        self.diagnostics+=1

    def finish(self,line):
        fields=line.split(',')
        if len(fields)!=8 or fields[0]!='END':
            raise ValueError('Malformed timing/checksum trailer')
        count,elapsed,fifo,gap,processing,handler=map(int,fields[1:7])
        if count!=COUNT or len(self.rows)!=COUNT or self.diagnostics!=COUNT//100:
            raise ValueError('Incomplete samples or MCU diagnostics')
        if int(fields[7],16)!=self.checksum:
            raise ValueError('Raw payload checksum mismatch')
        if not 0<elapsed<=25000000 or not 14<=fifo<980 or not 0<processing<=handler or not 0<gap<=25000000:
            raise ValueError('Invalid timing or FIFO evidence')
        rate=COUNT*1e6/elapsed
        return {'samples':COUNT,'elapsed_us':elapsed,'observed_capture_rate_hz':rate,'maximum_fifo_bytes':fifo,
                'maximum_sample_service_gap_us':gap,'maximum_calculation_us':processing,'maximum_sample_handler_us':handler,
                'verified_mcu_diagnostics':self.diagnostics,'fnv1a_raw_payload':f'{self.checksum:08X}',
                'average_rate_within_2_percent':980<=rate<=1020,'handler_within_1000us_budget':handler<1000,
                'status':'CAPTURE_COMPLETE_REVIEW_REQUIRED',
                'scope':'Average rate, backlog and service gaps are separate. No PID, motor or strict control-deadline pass is implied.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    args=parser.parse_args()
    source=hashlib.sha256((ROOT/'firmware/main.cpp').read_bytes()).hexdigest()
    fit=json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    folder=ROOT/'artifacts/fifo1000-recordings'/transport.stamp()
    folder.mkdir(parents=True)
    report={'status':'FAILED','engine':'Real MPU6050 FIFO via ESP32-S3; no simulator','source_sha256':source,
            'uploaded':False,'calibration_changed':False,'stationary_condition':'Assumed from delegated bench testing','commands_sent':[]}
    device=serial.Serial(port=None,baudrate=921600,timeout=.5,write_timeout=2)
    device.port=args.port
    device.dtr=False
    device.rts=False
    stream=None
    try:
        device.open()
        with (folder/'serial.log').open('xb') as log:
            report['commands_sent'].append('?')
            header,profile=gyro.read_header(device,log,source,fit)
            if not profile.get('fifo_acquisition'):
                raise ValueError('Upload current FIFO1000 bench firmware first; no firmware was uploaded by this recorder')
            report.update({'header':header,'profile':profile})
            stream=Samples(profile)
            report['commands_sent'].append('s')
            device.write(b's')
            started=False
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                line=transport.receive(device,log)
                if not line:
                    continue
                if line.startswith('ERROR,'):
                    raise ValueError(line)
                if not started:
                    if line==f'BEGIN,{COUNT}':
                        started=True
                    elif not line.startswith('D,'):
                        raise ValueError('Unexpected pre-capture data')
                    continue
                if line.startswith('S,'):
                    stream.add(line)
                    if len(stream.rows)%1000==0:
                        print(f'{len(stream.rows)}/{COUNT} samples verified',flush=True)
                        log.flush()
                elif line.startswith('D,'):
                    stream.diagnostic(line)
                elif line.startswith('END,'):
                    report.update(stream.finish(line))
                    break
                else:
                    raise ValueError('Unexpected capture output')
            else:
                raise ValueError('Capture timeout; original partial data preserved')
            report['commands_sent'].append('?')
            final,_=gyro.read_header(device,log,source,fit)
            if final!=header:
                report['status']='FAILED'
                raise ValueError('Calibration/source/configuration metadata changed')
            report['final_header_unchanged']=True
            print(f"Rate {report['observed_capture_rate_hz']:.2f} Hz; max service gap {report['maximum_sample_service_gap_us']} us; max handler {report['maximum_sample_handler_us']} us",flush=True)
    except (Exception,KeyboardInterrupt) as error:
        report['status']='FAILED'
        report['error']=str(error) or 'User interrupted'
        if device.is_open:
            try:
                report['commands_sent'].append('x')
                device.write(b'x')
            except serial.SerialException:
                pass
        raise
    finally:
        device.close()
        if stream is not None:
            transport.save_json(folder/'samples.json',{'raw_post_trim_words':stream.rows})
        if (folder/'serial.log').exists():
            report['serial_sha256']=hashlib.sha256((folder/'serial.log').read_bytes()).hexdigest()
        transport.save_json(folder/'summary.json',report)
        print(f'Original evidence: {folder}',flush=True)


if __name__=='__main__':
    main()
