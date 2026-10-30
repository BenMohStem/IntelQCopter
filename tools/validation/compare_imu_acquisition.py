"""Record the real motor-disabled direct/FIFO harness; never flashes firmware."""
import argparse
import hashlib
import json
from pathlib import Path
import random
import struct
import time
import serial
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def receive(device, log, deadline):
    while time.monotonic() < deadline:
        raw = device.readline()
        if not raw:
            continue
        log.write(raw)
        if not raw.endswith(b'\n'):
            raise ValueError('Truncated serial line')
        return raw.decode('ascii').strip()
    raise TimeoutError('No complete line before deadline')


def summarize(rows, metadata):
    a = np.asarray(rows, dtype=np.int64)
    if len(a) < 100:
        raise ValueError('Too few samples')
    if not np.array_equal(a[:, 0], np.arange(len(a))):
        raise ValueError('Missing, duplicated or reordered host records')
    if not np.all((a[:, 1] <= a[:, 2]) & (a[:, 2] <= a[:, 3]) & (a[:, 3] < a[:, 4])):
        raise ValueError('Non-monotonic within-record timestamps')
    if not np.all(np.diff(a[:, 4]) > 0):
        raise ValueError('Non-monotonic read completion timestamps')
    if a[-1, 4] > metadata['elapsed_us']:
        raise ValueError('Elapsed time does not contain samples')
    if metadata['mode'] not in 'dfDF' or not 4000000 <= metadata['elapsed_us'] <= 4100000:
        raise ValueError('Invalid acquisition mode/window')
    if not 0 <= metadata['tail_bytes'] < 1024:
        raise ValueError('Invalid FIFO tail')
    fifo = metadata['mode'].lower() == 'f'
    if fifo and not np.all((a[:,5] >= 14) & (a[:,5] < 1024)):
        raise ValueError('Invalid FIFO backlog')
    if not fifo and (np.any(a[:,5]) or metadata['tail_bytes']):
        raise ValueError('Direct stream unexpectedly reports FIFO packets')
    if metadata['mode'].isupper() != (metadata['injected_pauses'] > 0):
        raise ValueError('Wrong injected-pause condition')
    checksum = 2166136261
    for row in rows:
        for value in struct.pack('>7h', *row[6:]):
            checksum = ((checksum ^ value) * 16777619) & 0xFFFFFFFF
    if checksum != metadata['checksum'] or len(a) != metadata['samples']:
        raise ValueError('Raw payload checksum/count mismatch')
    def stats(v):
        return {'min':float(np.min(v)), 'median':float(np.median(v)),
                'p95':float(np.percentile(v,95)), 'p99':float(np.percentile(v,99)),
                'max':float(np.max(v)), 'mean':float(np.mean(v))}
    return {**metadata, 'data_read_us':stats(a[:,4]-a[:,3]),
            'successful_service_us':stats(a[:,4]-a[:,1]),
            'observed_ready_to_completion_us':stats(a[:,4]-a[:,2]),
            'completion_gap_us':stats(np.diff(a[:,4])),
            'gaps_over_1500us':int(np.sum(np.diff(a[:,4])>1500)),
            'maximum_fifo_packets_before_read':int(np.max(a[:,5])//14),
            'fifo_oldest_extra_queue_age_proxy_us':int(max(0,np.max(a[:,5])//14-1)*1000),
            'read_rate_hz':len(a)*1e6/metadata['elapsed_us'],
            'tail_packets':metadata['tail_bytes']//14,
            'gyro_mean_raw':a[:,10:13].mean(axis=0).tolist(),
            'temperature_mean_C':float((a[:,9]/340+36.53).mean()),
            'serial_integrity':'sequence, raw checksum and timestamp ordering verified'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    expected = hashlib.sha256((ROOT/'lessons/imu-acquisition-benchmark/main.cpp').read_bytes()).hexdigest()
    results = []
    order = []
    rng = random.Random(30102026)
    for block in range(args.repetitions):
        modes = list('dfDF')
        rng.shuffle(modes)
        order.extend((block,mode) for mode in modes)
    device = serial.Serial()
    device.port = args.port
    device.baudrate = 921600
    device.timeout = 1
    device.dtr = False
    device.rts = False
    try:
        device.open()
        if hasattr(device,'set_buffer_size'):
            device.set_buffer_size(rx_size=1048576,tx_size=4096)
        device.reset_input_buffer()
        with (args.output/'identity.log').open('xb') as log:
            device.write(b'?')
            deadline = time.monotonic()+5
            while True:
                line = receive(device,log,deadline)
                if line.startswith('ACQ_BENCH_V1,'):
                    if expected not in line or 'READY=1' not in line or 'WHO=104' not in line:
                        raise ValueError('Wrong source/sensor or initialization failure: '+line)
                    break
        for number,(block,mode) in enumerate(order):
            print(f'Run {number+1}/{len(order)}: block={block+1}, mode={mode}',flush=True)
            rows=[];metadata=None;failure=None
            with (args.output/f'run-{number+1:02d}-{mode}.log').open('xb') as log:
                device.write(mode.encode('ascii'))
                deadline=time.monotonic()+18
                while True:
                    line=receive(device,log,deadline)
                    fields=line.split(',')
                    if line.startswith('ERROR,'):
                        raise ValueError(line)
                    if fields[0]=='DATA':
                        if len(fields)!=7 or metadata is not None or fields[1]!=mode:
                            raise ValueError('Malformed DATA header')
                        metadata={'mode':mode,'block':block+1,'samples':int(fields[2]),
                                  'elapsed_us':int(fields[3]),'injected_pauses':int(fields[4]),
                                  'tail_bytes':int(fields[5]),'checksum':int(fields[6],16)}
                    elif fields[0]=='S':
                        if metadata is None or len(fields)!=14:
                            raise ValueError('Malformed/detached sample')
                        rows.append([int(value) for value in fields[1:]])
                    elif fields[0]=='DONE':
                        if len(fields)!=3 or fields[1]!=mode or fields[2]!='OK':
                            raise ValueError('Acquisition rejected: '+line)
                        break
                    elif fields[0] not in ('START','SETTINGS'):
                        raise ValueError('Unexpected protocol line: '+line)
            if metadata is None:
                raise ValueError('Missing DATA header')
            result=summarize(rows,metadata)
            np.savetxt(args.output/f'run-{number+1:02d}-{mode}.csv',rows,delimiter=',',fmt='%d',
                       header='index,service_start_us,ready_observed_us,read_start_us,read_end_us,fifo_bytes,ax,ay,az,temp,gx,gy,gz',comments='')
            result['raw_log_sha256']=hashlib.sha256((args.output/f'run-{number+1:02d}-{mode}.log').read_bytes()).hexdigest()
            results.append(result)
            (args.output/'partial-results.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
            print(f"  {result['samples']} samples, {result['read_rate_hz']:.2f} Hz; read median {result['data_read_us']['median']:.1f} us",flush=True)
            time.sleep(.2)
    except Exception as error:
        (args.output/'failure.json').write_text(json.dumps({'error':str(error),'completed_runs':len(results)},indent=2)+'\n',encoding='utf-8')
        raise
    finally:
        device.close()
    output={'engine':'Real ESP32-S3/MPU6050 measurements; MCU micros timestamps; Python integrity/statistics',
            'source_sha256':expected,'order':order,'runs':results,'uploaded_by_recorder':False,
            'limitations':['No external data-ready timestamp or motion reference',
                'Direct physical sample losses not counted exactly; sample throughput/gaps are observations',
                'FIFO age is a nominal1ms-per-packet proxy, not absolute sensor latency',
                'No BLE, PID, motor output, calibration or flight acceptance',
                'Raw sensor words without project calibration; support/motion not independently attested',
                'Serial printing occurs after each4s capture, not inside timed loop']}
    (args.output/'results.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print('Saved verified comparison:',args.output,flush=True)


if __name__=='__main__':
    main()
