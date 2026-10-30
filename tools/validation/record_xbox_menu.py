"""Save Xbox-menu-selected tests automatically; --run-all uses serial test requests."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import time
import struct
import serial
import numpy as np
from compare_imu_acquisition import summarize

ROOT=Path(__file__).resolve().parents[2]


def summarize_rotation(rows, metadata):
    a=np.asarray(rows,dtype=np.int64)
    if a.ndim!=2 or a.shape[1]!=5 or len(a)<100:raise ValueError('Invalid rotation rows')
    if not np.array_equal(a[:,0],np.arange(len(a))):raise ValueError('Rotation sequence mismatch')
    if np.any(a[:,1]<=0) or np.any(a[:,1]>65535):raise ValueError('Invalid read gaps')
    if not 20000000<=metadata['elapsed_us']<=20100000:raise ValueError('Wrong rotation duration')
    if a[:,1].sum()>metadata['elapsed_us']:raise ValueError('Read times exceed duration')
    checksum=2166136261
    for row in rows:
        for byte in struct.pack('>3h',*row[2:]):checksum=((checksum^byte)*16777619)&0xffffffff
    if len(a)!=metadata['samples'] or checksum!=metadata['checksum']:raise ValueError('Gyro payload integrity mismatch')
    if not 0<=metadata['tail_bytes']<1024:raise ValueError('Invalid FIFO tail')
    return {**metadata,'read_rate_hz':len(a)*1e6/metadata['elapsed_us'],
            'max_read_gap_us':int(a[:,1].max()),'serial_integrity':'gyro count, sequence, checksum and read-gap sum verified',
            'measurement_interval':'nominal 1 ms FIFO; read gaps are not sensor sample times',
            'retained_channels':'raw sensor gyro only; no accelerometer or temperature in rotation mode'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--run-all',action='store_true')
    parser.add_argument('--inputs-only',action='store_true',help='Show raw controller inputs; disable test starts')
    parser.add_argument('--single',choices=list('dfDFr'),help='Request one acquisition test, save and exit')
    parser.add_argument('--note',default='',help='Physical condition/reference; retained with evidence')
    args=parser.parse_args()
    if args.single and (args.run_all or args.inputs_only):parser.error('Choose single, inputs-only or run-all')
    if args.inputs_only and args.run_all:parser.error('Choose inputs-only or run-all')
    output=args.output or ROOT/'artifacts/acquisition-comparison/xbox-menu'/dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output.mkdir(parents=True,exist_ok=False)
    expected=hashlib.sha256((ROOT/'lessons/xbox-test-menu/main.cpp').read_bytes()).hexdigest()
    device=serial.Serial();device.port=args.port;device.baudrate=921600;device.timeout=1;device.dtr=False;device.rts=False
    runs=[];rows=[];metadata=None;mode=None;core=None;identity=None
    xbox_status=[];menu_lines=[];pending=list('dfDF') if args.run_all else ([args.single] if args.single else [])
    log=(output/'session-original.log').open('xb')
    def line():
        raw=device.readline()
        if not raw:return None
        log.write(raw);log.flush()
        if not raw.endswith(b'\n'):raise ValueError('Truncated serial record')
        return raw.decode('ascii').strip()
    try:
        device.open()
        device.reset_input_buffer();device.write(b'?')
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            value=line()
            if not value:continue
            if value.startswith('ACQ_MENU_V1,'):
                if value.split(',')[1]!=expected or 'READY=1' not in value or 'WHO=104' not in value:
                    raise ValueError('Wrong menu source or sensor initialization')
                identity=value
            if identity and value.startswith('CORE,'):
                if 'acquisition=1,slow=0,' not in value:raise ValueError('Unexpected core assignment')
                core=value;break
        if not core:raise TimeoutError('Menu identity/core verification timed out')
        print(identity,flush=True);print(core,flush=True)
        print('Automatic recording folder:',output,flush=True)
        print('D-pad LEFT/RIGHT selects; A starts; B cancels; Y shows selection.',flush=True)
        print('1 DIRECT_NORMAL | 2 FIFO_NORMAL | 3 DIRECT_PAUSES | 4 FIFO_PAUSES',flush=True)
        print('Initial selection: DIRECT_NORMAL. Timing tests: 4 seconds. Rotation r: 20 seconds, then automatic saving.',flush=True)
        print('RECENT_INPUT=0 means no report in 300 ms; it does not mean disconnected.',flush=True)
        if args.single:
            duration=20 if args.single=='r' else 4
            print(f'Capture {duration} seconds: stay still first, make the labelled motion gently, then hold.',flush=True)
            input('Press Enter when positioned and ready: ')
        if args.inputs_only:
            device.write(b'i')
            print('INPUT MONITOR ONLY: tests disabled. Press each button separately, then move sticks/triggers.',flush=True)
        if pending:device.write(pending.pop(0).encode())
        while True:
            value=line()
            if not value:continue
            fields=value.split(',')
            if fields[0]=='START':
                if mode is not None:raise ValueError('Overlapping tests')
                mode=fields[1];rows=[];metadata=None
                print('Recording test:',mode,flush=True)
            elif fields[0] in ('DATA','RDATA'):
                if len(fields)!=7 or fields[1]!=mode or metadata is not None:raise ValueError('Malformed DATA')
                metadata={'mode':mode,'samples':int(fields[2]),'elapsed_us':int(fields[3]),'injected_pauses':int(fields[4]),'tail_bytes':int(fields[5]),'checksum':int(fields[6],16)}
            elif fields[0]=='G':
                if mode!='r' or metadata is None or len(fields)!=6:raise ValueError('Malformed gyro row')
                rows.append([int(x) for x in fields[1:]])
                if len(rows)%1000==0:print(f'  Saved {len(rows)} gyro samples',flush=True)
            elif fields[0]=='S':
                if mode is None or metadata is None or len(fields)!=14:raise ValueError('Detached/malformed sample')
                rows.append([int(x) for x in fields[1:]])
                if len(rows)%1000==0:print(f'  Saved {len(rows)} samples',flush=True)
            elif fields[0]=='DONE':
                if len(fields)!=3 or fields[1]!=mode or metadata is None:raise ValueError('Malformed completion')
                np.savetxt(output/f'run-{len(runs)+1:02d}-{mode}.csv',rows,delimiter=',',fmt='%d',header=('index,read_gap_us,gx,gy,gz' if mode=='r' else 'index,service_start_us,ready_observed_us,read_start_us,read_end_us,fifo_bytes,ax,ay,az,temp,gx,gy,gz'),comments='')
                if fields[2]=='OK':
                    result=summarize_rotation(rows,metadata) if mode=='r' else summarize(rows,metadata)
                    result['accepted_record']=True
                else:
                    result={**metadata,'accepted_record':False,'reason':fields[2]}
                runs.append(result);print('Saved test:',mode,fields[2],flush=True);mode=None
                if args.run_all or args.single:
                    if fields[2]!='OK':raise ValueError('Automatic test rejected: '+fields[2])
                    if not pending:break
                    time.sleep(.2);device.write(pending.pop(0).encode())
            elif fields[0]=='XBOX':
                xbox_status.append(value)
                print(value,flush=True)
            elif fields[0]=='MENU':
                menu_lines.append(value);print(value,flush=True)
            elif fields[0] in ('INPUT','INPUT_MODE'):
                print(value,flush=True)
            elif fields[0]=='ERROR':raise ValueError(value)
    except KeyboardInterrupt:
        device.write(b'x')
        print('Recorder stopped; cancel requested if a capture was active. Originals retained.',flush=True)
    except Exception as error:
        (output/'failure.json').write_text(json.dumps({'error':str(error),'mode':mode,'partial_rows':len(rows)},indent=2)+'\n',encoding='utf-8')
        raise
    finally:
        if device.is_open and args.inputs_only:device.write(b't')
        device.close();log.close()
        result={'engine':'Real ESP32-S3 core assignment and capture timings; Python serial recorder/integrity checks',
                'source_sha256':expected,'identity':identity,'core_report':core,'runs':runs,'xbox_status':xbox_status,'menu_events':menu_lines,
                'request_source':'PC serial through slow-core request queue' if (args.run_all or args.single) else 'Xbox menu (or manual serial)',
                'physical_condition_note':args.note,
                'physical_button_validation':'pending unless operator separately attests button actions',
                'session_log_sha256':hashlib.sha256((output/'session-original.log').read_bytes()).hexdigest(),
                'motor_outputs_enabled':False,'PID_implemented':False}
        (output/'results.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print('Saved session:',output,flush=True)


if __name__=='__main__':main()
