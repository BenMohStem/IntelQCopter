from pathlib import Path
import json
import hashlib

r = Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
out = r/'measurements/software/2026-10-30-xbox-test-menu'
results = json.loads((r/'artifacts/acquisition-comparison/xbox-menu/20261030T125921Z/results.json').read_text())
(out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
rows = ['| Test | Samples | Read rate (Hz) | Median service (µs) | Maximum read gap (µs) |', '|---|---:|---:|---:|---:|']
for run in results['runs']:
    rows.append(f"| {run['mode']} | {run['samples']} | {run['read_rate_hz']:.2f} | {run['successful_service_us']['median']:.0f} | {run['completion_gap_us']['max']:.0f} |")
report = '''# Two-core Xbox test menu — 30 October 2026

Engine: actual ESP32-S3 and MPU6050, GPIO9 SDA / GPIO8 SCL, 400 kHz I²C. Separate motor-disabled measurement firmware; **no PID or actuator control**. Production `firmware/main.cpp` and calibration were not changed.

## Actual checks

PlatformIO build and upload succeeded. Source SHA256: `SOURCE`.

Device reported `CORE,acquisition=1,slow=0,heap=150276,min_heap=149600,PID=NOT_IMPLEMENTED`.

TABLE

All four captures passed record sequence, timestamp order, raw count and payload checksum checks. These are four-second observations, one trial per condition, not a hard real-time guarantee. `D`/`F` deliberately pause acquisition 2.5 ms every 100 ms; these measure backlog behavior, not flight behavior. No overflow occurred in these runs. Normal conditions must not be pooled with deliberately paused conditions.

**Xbox was disconnected throughout.** BLE scanning ran, but connected notification traffic, physical Menu/A/B/Y actions, reconnect behavior and cancellation still require operator testing. Serial requests used the same queue as Xbox selections; that does not prove the physical gamepad path.

Original serial log, per-run raw CSVs and installation logs remain privately preserved under `artifacts/acquisition-comparison/`. `results.json` contains original-log SHA256 and statistics. The old full-flash backup is private and must never be published. Upload log verified written flash hashes. The board now runs this menu harness, not the earlier benchmark or production FC.

## How to use

Close Serial Monitor and other COM3 users. From `QCopter-Stepwise`:

```powershell
.\\qcopter.ps1 record-xbox-menu --port COM3
```

Turn on the Xbox Series X/S controller and use pairing mode if needed. Observe `CONNECTED=1,FRESH=1` before testing buttons. Menu (≡) opens the selection; each further press advances. Release between presses. A runs the selected test; B requests cancellation and closes the menu; Y prints information. The recorder saves every completed run automatically and reports each 1000 received samples. Ctrl+C stops recording and requests cancellation. This is a bench menu, never a flight mode or motor control.

Four choices: direct normal (`d`), FIFO normal (`f`), direct with injected pauses (`D`), FIFO with injected pauses (`F`). PC-only Python/host tests are not executed on the ESP32.

```powershell
# Automatically request all four tests through the slow-core queue:
.\\qcopter.ps1 record-xbox-menu --port COM3 --run-all
# Build only; does not upload:
.\\qcopter.ps1 build-xbox-menu
```

## Core ownership

```text
core 0: Xbox/BLE → button edges → request queue → core 1: sensor acquisition
core 0: chunked serial dump ← completion queue ← core 1: capture buffer
                     buffer semaphore prevents overlapping captures
```

Only core 1 touches sensor I²C after initialization. Only core 0 handles Xbox and normal serial output. No serial dump occurs inside timed acquisition. A bounded queue and semaphore prevent overlapping tests; cancellation uses an event bit. Capture runs for four seconds and then dumps buffered samples. This is not a continuous flight-control scheduler. Splitting tasks cannot remove shared radio interrupts, memory contention or I²C time; a future 1 kHz PID still needs end-to-end deadline measurements under connected Xbox load.

## Sensor choices explained

FIFO overflow loses **oldest data** while new data continues entering. Reads return oldest retained bytes, not automatically the freshest sample. After overflow, reject the interval and reset/reframe FIFO before using packets. Manufacturer reference: [MPU6050 register map, FIFO_R_W](https://naylampmechatronics.com/img/cms/000033/MPU-6000-Register-Map1.pdf).

±4 g is usable for gentle flight, but gravity already contributes about 1 g at rest. Vibration, impacts and maneuvers can clip an axis. ±250°/s is a rotation-rate limit, not a tilt-angle limit: 90° in 0.25 s averages 360°/s and exceeds it. Smaller ranges improve quantization, not necessarily accuracy. We retain ±8 g and ±500°/s until representative motion/vibration data justify smaller ranges.

94/184 Hz describe filter bandwidth, not measurement range or CPU speed:

| Adafruit setting | Accel bandwidth / delay | Gyro bandwidth / delay |
|---|---|---|
| 184 Hz | 184 Hz / 2.0 ms | 188 Hz / 1.9 ms |
| 94 Hz | 94 Hz / 3.0 ms | 98 Hz / 2.8 ms |

Both can output at 1 kHz with divider 0. Filtering trades noise rejection against delay; the delay is not CPU execution time. This comparison retains 260 Hz mode and divider 7, matching the previous benchmark. Changing range/filter requires reviewing scaling and Kalman noise assumptions; no new flight setting is accepted here.

Clock source controls the MPU's internal sample timing. The power-up internal 8 MHz oscillator is one option; a phase-locked loop (PLL) can instead use X/Y/Z gyro references or supported external references. Current `MPU6050_PLL_GYROX` locks timing to the X-gyro reference; it does not use measured rotation as a clock and all axes still work. It is independent of the ESP32 CPU clock. Clock-stop is not a sampling mode. Sensor timing affects `angle += rate × dt`; PLL selection does not itself guarantee an exact measured 1 ms interval. See the manufacturer register map's `PWR_MGMT_1`, `SMPLRT_DIV` and `CONFIG` sections.
'''.replace('SOURCE', results['source_sha256']).replace('TABLE', '\n'.join(rows))
(out/'README.md').write_text(report, encoding='utf-8')
(r/'lessons/xbox-test-menu/README.md').write_text('# Separate motor-disabled Xbox test-menu lesson\n\n[Instructions, actual measurements and limitations](../../measurements/software/2026-10-30-xbox-test-menu/README.md). One runtime main.cpp; production FC source unchanged.\n')
p=r/'project.json'
catalog=json.loads(p.read_text())
catalog['commands']['record-xbox-menu']={'runtime':'python','path':'tools/validation/record_xbox_menu.py','args':[],'description':'Automatically save Xbox-selected bench captures; --port COM3, optional --run-all'}
catalog['commands']['build-xbox-menu']={'runtime':'pio','path':'lessons/xbox-test-menu/platformio.ini','args':['run','-d','lessons/xbox-test-menu'],'description':'Build separate two-core motor-disabled Xbox test-menu firmware'}
p.write_text(json.dumps(catalog,indent=2)+'\n')
p=r/'important_sources/README.md'
with p.open('a',encoding='utf-8') as f:
    f.write('\n## Current installed bench firmware — 30 October 2026\n\n[Two-core Xbox menu main.cpp](../lessons/xbox-test-menu/main.cpp), [recording instructions and actual results](../measurements/software/2026-10-30-xbox-test-menu/README.md). Supersedes the earlier installed-benchmark statement above. Four serial-requested captures passed; Xbox disconnected, physical buttons pending. No PID/motor output.\n')
p=r/'knowledge/state.json'
state=json.loads(p.read_text())
state['current_stage']='Motor-disabled two-core Xbox test-menu harness installed on COM3; four serial-requested captures passed integrity checks. Xbox disconnected; physical buttons and connected-load timing pending. Production source unchanged, existing user edit remains unreviewed.'
p.write_text(json.dumps(state,indent=2)+'\n')
event={'schema':2,'id':'2026-10-30-xbox-test-menu','date':'2026-10-30','domains':['acquisition-performance'],'summary':'User requested dual-core slow task isolation, Xbox-selected tests and MPU range/filter/clock explanations.','outcome':'Separate one-file menu firmware built/uploaded; core1 acquisition/core0 BLE and serial verified; four serial-requested tests passed integrity. Physical Xbox disconnected; no PID, motors or calibration changes. Production fingerprint intentionally unchanged.'}
(r/'knowledge/events/2026-10-30-xbox-test-menu.json').write_text(json.dumps(event,indent=2)+'\n')
p=r/'knowledge/domains/acquisition-performance.json'
domain=json.loads(p.read_text())
domain['entries'].append({'id':'acquisition.two-core-menu','status':'verified_artifact','summary':event['outcome'],'sources':[{'kind':'file','path':'measurements/software/2026-10-30-xbox-test-menu/README.md'}]})
p.write_text(json.dumps(domain,indent=2)+'\n')
print('\n'.join(rows))
