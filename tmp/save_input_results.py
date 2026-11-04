from pathlib import Path
from collections import Counter
import hashlib
import json
import sys
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
sys.path.insert(0,str(r/'tools/knowledge'))
import knowledge
c=knowledge.database()
token=knowledge.reserve(c,'input-results','input-results',['knowledge','measurements/software/2026-10-30-input-results','important_sources/README.md','git:integrator'],30)
print('CLAIM',token)
c.close()
p=r/'artifacts/acquisition-comparison/xbox-menu/20261030T193739Z/session-original.log'
rows=[dict(x.split('=',1) for x in line.split(',')[1:]) for line in p.read_text().splitlines() if line.startswith('INPUT,')]
analog=['LX','LY','RX','RY','LT','RT']
result={'engine':'Offline analysis of actual ESP32-S3 Xbox serial recording','recording':'20261030T193739Z','original_log_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'snapshots':len(rows),'snapshot_rate_hz':5,'buttons_nonzero_snapshots':{k:sum(int(row[k])>0 for row in rows) for k in rows[0] if k not in analog},'analog_observed_min_max':{k:[min(int(row[k]) for row in rows),max(int(row[k]) for row in rows)] for k in analog},'operator_statement':'Done testing buttons; small blue/activity LED intended instead of RGB','limits':['Snapshot counts are not press counts','Physical labels/direction/center drift/deadband not established solely by min/max','No PID or actuator test','No GPS or battery sensing available']}
out=r/'measurements/software/2026-10-30-input-results'
out.mkdir(exist_ok=True)
(out/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
(out/'README.md').write_text('''# Xbox input results and LED clarification — 30 October 2026

Actual ESP32-S3 recording, analyzed offline; no new firmware uploaded. Original recording preserved privately; SHA256 and statistics are in analysis.json.

466 snapshots at the configured 5 Hz diagnostic rate. All 11 logged button fields have nonzero samples. Each stick axis reached 0–65535; LT/RT reached 0–1023. This demonstrates observed decoding and endpoint coverage, not exact centers, deadband, physical label identity, BLE packet latency or an accepted flight input interface. Snapshot counts are not press counts. No calibration/PID/motors changed.

User confirms completion. Their previous physical Menu/Y mismatch is retained as operator evidence; application swaps these two roles for this controller. Full label validation still requires associating each press with its physical label, rather than inferring identity from arbitrary event order.

## LED decision

User intended the small blue flashing/activity LED or the adjacent small LED, not the large RGB. Exact board circuit remains unidentified (CH343 USB bridge previously observed). It may be USB activity or power, so no guessed GPIO is configured. RGB code remains as previously installed pending identification; this iteration does not claim the requested small LED was made programmable. A clear board photo showing both LEDs/nearby markings or the exact schematic is needed.

Official Espressif DevKitC-1 reference documents power and RGB LEDs; a clone's USB/activity LED cannot be inferred from that reference: https://documentation.espressif.com/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html

## Future battery behavior (not implemented)

Reserve the large RGB for battery indication, including red at estimated 20% remaining. User now proposes safe landing or GPS-assisted return/landing in the future. This supersedes the earlier requested 20% immediate-disarm behavior as a future direction; exact flight response remains to be designed and validated. No voltage/current sensors or GPS yet: no charge estimate, threshold action or landing is enabled. GPS alone is insufficient for a validated return/landing system. Physical motor outputs remain disabled in this bench stage.

## Next

Input endpoint observation is complete; do not repeat the four artificial-pause tests just for repetition. Next meaningful physical gate: corrected gyro signs and scale against known gentle rotations, followed by attitude estimation. This menu harness is not the calibrated attitude firmware; do not use its raw acquisition samples as calibrated drone rates. Review the unapproved production gyro coefficient edit before restoring that firmware. No PID flight acceptance follows from this input test.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-30-input-results-led-preference','date':'2026-10-30','domains':['acquisition-performance'],'summary':'Operator completed input test and clarified small activity LED intended; RGB reserved for future battery warning/landing.','outcome':'466 real snapshots reviewed; all logged button groups responded, stick full endpoints and trigger0..1023 observed. Small LED circuit unknown; no guessed GPIO or firmware change. Battery sensing/GPS/landing future only.'}
(r/'knowledge/events/2026-10-30-input-results-led-preference.json').write_text(json.dumps(event,indent=2)+'\n')
path=r/'knowledge/state.json'
state=json.loads(path.read_text())
state['current_stage']='Operator Xbox input session reviewed: 466 snapshots, all logged button fields active, sticks0..65535/triggers0..1023. Physical label/direction/deadband and gyro-motion gate remain. Small activity LED circuit unknown; large RGB reserved by user for future battery indication/landing, not implemented.'
path.write_text(json.dumps(state,indent=2)+'\n')
path=r/'knowledge/domains/acquisition-performance.json'
domain=json.loads(path.read_text())
domain['entries'].append({'id':'acquisition.xbox-input-endpoints','status':'verified_artifact','summary':event['outcome'],'sources':[{'kind':'file','path':'measurements/software/2026-10-30-input-results/README.md'}]})
path.write_text(json.dumps(domain,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:
    f.write('\n[Operator Xbox input results and future battery LED requirement](../measurements/software/2026-10-30-input-results/README.md). No firmware/actuator changes.\n')
(out/'claim-token.txt').write_text(token)
