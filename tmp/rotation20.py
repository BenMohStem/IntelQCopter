from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/xbox-test-menu/main.cpp'
s=p.read_text().replace('static Sample samples[capacity];','''struct RotationSample
{
    uint16_t read_gap_us;
    int16_t gyro[3];
};
static const uint16_t rotation_capacity = 20200;
static const uint32_t rotation_window_us = 20000000;
static union
{
    Sample samples[capacity];
    RotationSample rotations[rotation_capacity];
} records;
static Sample (&samples)[capacity] = records.samples;''')
marker='static void capture(char mode)'
func='''// Rotation-only FIFO capture: retain gyro, not the full timing benchmark rows.
static void capture_rotation(void)
{
    Trial result = {};
    result.mode = 'r';
    result.checksum = 2166136261u;
    bool ok = ready && check_config();
    ok = ok && write_register(0x23, 0) && write_register(0x6A, 0x04);
    delay(2);
    ok = ok && write_register(0x6A, 0x40) && write_register(0x23, 0xF8);
    uint32_t started = micros();
    uint32_t previous = 0;
    while (ok && (uint32_t)(micros() - started) < rotation_window_us)
    {
        if (xEventGroupGetBits(events) & abort_bit)
        {
            result.failure = "USER_ABORT";
            break;
        }
        uint8_t status;
        uint8_t size[2];
        ok = read_registers(0x3A, &status, 1) && read_registers(0x72, size, 2);
        if (!ok)
        {
            break;
        }
        uint16_t queued = ((uint16_t)size[0] << 8) | size[1];
        if ((status & 0x10) || queued >= 1024)
        {
            result.failure = "FIFO_OVERFLOW";
            break;
        }
        if (queued < 14)
        {
            continue;
        }
        if (result.count >= rotation_capacity)
        {
            result.failure = "RECORD_BUFFER_FULL";
            break;
        }
        uint8_t bytes[14];
        ok = read_registers(0x74, bytes, 14);
        if (!ok)
        {
            break;
        }
        uint32_t now = micros() - started;
        if (now - previous > 65535)
        {
            result.failure = "READ_GAP_TOO_LONG";
            break;
        }
        RotationSample &row = records.rotations[result.count++];
        row.read_gap_us = now - previous;
        previous = now;
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            row.gyro[axis] = (int16_t)(((uint16_t)bytes[8 + 2 * axis] << 8) | bytes[9 + 2 * axis]);
        }
        for (uint8_t j = 8; j < 14; j++)
        {
            result.checksum = (result.checksum ^ bytes[j]) * 16777619u;
        }
    }
    result.elapsed = micros() - started;
    uint8_t tail[2] = {};
    ok = read_registers(0x72, tail, 2) && ok;
    result.tail = ((uint16_t)tail[0] << 8) | tail[1];
    ok = write_register(0x23, 0) && write_register(0x6A, 0) && ok;
    if (!ok && !result.failure)
    {
        result.failure = "I2C_OR_CONFIG_FAILURE";
    }
    xQueueSend(completed, &result, 0);
}

'''
s=s.replace(marker,func+marker)
s=s.replace("    bool fifo = mode == 'f' || mode == 'F';", "    if (mode == 'r')\n    {\n        capture_rotation();\n        return;\n    }\n    bool fifo = mode == 'f' || mode == 'F';")
s=s.replace("command == 'F'))", "command == 'F' || command == 'r'))")
s=s.replace('Serial.printf("DATA,%c,%u,%lu,%lu,%u,%08lX\\n", result.mode, result.count,','Serial.printf("%s,%c,%u,%lu,%lu,%u,%08lX\\n", result.mode == \'r\' ? "RDATA" : "DATA", result.mode, result.count,')
needle='''                const Sample &row = samples[dump_index];'''
s=s.replace(needle,'''                if (result.mode == 'r')
                {
                    const RotationSample &row = records.rotations[dump_index];
                    Serial.printf("G,%u,%u,%d,%d,%d\\n", dump_index, row.read_gap_us,
                        row.gyro[0], row.gyro[1], row.gyro[2]);
                    dump_index++;
                    continue;
                }
'''+needle)
p.write_text(s,encoding='utf-8',newline='\n')
p=r/'tools/validation/record_xbox_menu.py'
s=p.read_text().replace('import time\n','import time\nimport struct\n')
s=s.replace("choices=list('dfDF')", "choices=list('dfDFr')")
s=s.replace("Each capture is 4 seconds, then automatic saving.","Timing tests: 4 seconds. Rotation r: 20 seconds, then automatic saving.")
s=s.replace("            print('Keep still. After START, make the labelled motion gently within 4 seconds, then hold.',flush=True)","            duration=20 if args.single=='r' else 4\n            print(f'Capture {duration} seconds: stay still first, make the labelled motion gently, then hold.',flush=True)")
s=s.replace("elif fields[0]=='DATA':", "elif fields[0] in ('DATA','RDATA'):")
s=s.replace("            elif fields[0]=='S':", "            elif fields[0]=='G':\n                if mode!='r' or metadata is None or len(fields)!=6:raise ValueError('Malformed gyro row')\n                rows.append([int(x) for x in fields[1:]])\n                if len(rows)%1000==0:print(f'  Saved {len(rows)} gyro samples',flush=True)\n            elif fields[0]=='S':")
s=s.replace("header='index,service_start_us,ready_observed_us,read_start_us,read_end_us,fifo_bytes,ax,ay,az,temp,gx,gy,gz'", "header=('index,read_gap_us,gx,gy,gz' if mode=='r' else 'index,service_start_us,ready_observed_us,read_start_us,read_end_us,fifo_bytes,ax,ay,az,temp,gx,gy,gz')")
s=s.replace("                    result=summarize(rows,metadata);result['accepted_record']=True", "                    result=summarize_rotation(rows,metadata) if mode=='r' else summarize(rows,metadata)\n                    result['accepted_record']=True")
marker='def main():'
helper='''def summarize_rotation(rows, metadata):
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


'''
s=s.replace(marker,helper+marker)
p.write_text(s,encoding='utf-8',newline='\n')
compile(s,str(p),'exec')
out=r/'measurements/software/2026-10-30-rotation20'
out.mkdir(exist_ok=True)
uncertain={'operator_statement':'Three runs; +X/-X positions may have been switched','runs':['20261030T200128Z','20261030T200145Z','20261030T200157Z'],'axis_sign_acceptance':False,'treatment':'Preserve originals; do not relabel from gyro output or use to validate sign'}
(out/'uncertain-runs.json').write_text(json.dumps(uncertain,indent=2)+'\n')
guide='''# Twenty-second gyro rotation check

New dedicated `r` mode retains all three RAW gyro axes at nominal 1 kHz for 20 seconds. It uses a compact 20200-row buffer shared with the old timing buffer, rather than allocating a too-large full timing buffer. No accelerometer or temperature is retained in this mode. No calibration, PID or motor changes. The old d/f/D/F timing tests stay four seconds.

Three earlier operator runs have uncertain +X/-X labels. Originals remain untouched; no physical sign validation is claimed. See uncertain-runs.json. Do not infer the intended physical label from the same gyro output being tested.

```powershell
.\\qcopter.ps1 record-xbox-menu --port COM3 --single r --note "rotation about marked drone X; physical direction described after run; approx 90 deg"
```

Mark the existing drone +X direction with tape. Right thumb along +X, curled fingers give positive rotation ABOUT X (not raising the X end). If unsure, use a literal description of which side rises and include a photo; leave sign unknown instead of guessing.

Press Enter ready. At `Recording test: r`: hold still for about 3 seconds; rotate once gently about X over 3–5 seconds; hold the new position until 20 seconds ends. Do not rotate back during capture. Wait for Saved test and automatic exit before returning the drone. No Xbox button needed. One run first; sign and opposite direction can be checked after reviewing the physical description. Approximate hand angle is not precision gyro scale evidence.

Data saves afterward; every 1000 saved samples is reported during transfer. Sensor measurement times are nominal 1 ms apart; saved read_gap_us is retrieval timing, not exact measurement timing. Overflow, excessive gap, I2C errors, full buffer and cancellation reject the run. Checksum covers raw gyro words; count, sequence, duration and timestamp accumulation are also checked. No model-derived physical pass.
'''
(r/'docs/gyro-single-check.md').write_text(guide,encoding='utf-8')
(out/'README.md').write_text(guide,encoding='utf-8')
event={'schema':2,'id':'2026-10-30-rotation20','date':'2026-10-30','domains':['acquisition-performance'],'summary':'User requests15–20second capture and reports uncertain signs in three earlier runs.','outcome':'Prepare compact20second gyro-only FIFO mode r; preserve ambiguous runs without sign acceptance; existing timing tests remain4seconds. Hardware result recorded separately.'}
(r/'knowledge/events/2026-10-30-rotation20.json').write_text(json.dumps(event,indent=2)+'\n')
state_path=r/'knowledge/state.json'
state=json.loads(state_path.read_text())
state['current_stage']='20-second gyro-only rotation capture r prepared. Three previous rotations have uncertain physical signs, preserved and not accepted. Hardware installation/test evidence in 2026-10-30-rotation20 report; no calibration or PID change.'
state_path.write_text(json.dumps(state,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:f.write('\n[Twenty-second gyro-only rotation recording](../docs/gyro-single-check.md): --single r; old timing tests stay four seconds.\n')
