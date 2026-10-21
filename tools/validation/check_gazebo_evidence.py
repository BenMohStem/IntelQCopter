"""Audit original selected run files; this does not run or repair simulations."""
import argparse,csv,hashlib,json,math,struct,zlib
from pathlib import Path
import cv2

ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--report',default='artifacts/reports/gazebo-verified-20261021/summary.json');args=p.parse_args()
report_path=(ROOT/args.report).resolve();assert report_path.is_relative_to(ROOT)
summary=json.loads(report_path.read_text(encoding='utf8'));runs=summary['runs']
assert len(runs)==15 and summary['manual_passed']==5 and summary['navigation_passed']==10
assert len({r['environment'] for r in runs if r['kind']=='navigation'})==10
assert len({json.dumps(r['engine']['environment']['panels']) for r in runs if r['kind']=='navigation'})==10
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
core_sha=sha(ROOT/'firmware/src/flight_controller.c');dll_sha=sha(ROOT/'build/sitl/esp32_flight_core.dll')
checks=[]
for r in runs:
    folder=ROOT/'artifacts/gazebo-suite'/r['name'];tr=r['transport'];engine=r['engine']
    assert r['passed'] and tr['completed'] and not tr['timeout'],r['name']
    assert tr['final_state']['ground_contact'] and tr['final_state']['disarm']
    assert engine['obstacle_contact_steps']==0 and r['arrival_error_m']<.2
    assert engine['sha256']['firmware/src/flight_controller.c']==core_sha
    assert engine['sha256']['build/sitl/esp32_flight_core.dll']==dll_sha
    pc=json.loads((folder/'pc-mission.json').read_text(encoding='utf8'))
    if r['kind']=='manual':
        assert pc['mission_requests']==0 and tr['companion_packets']==0
    else:
        assert pc['mission_requests']==1 and pc['accepted'] and tr['companion_packets']>0
        assert tr['yolo_frames']>0 and tr['pnp_frames']>0 and 'phase=7,done=1' in tr['mission_status']
        for line in (folder/'perception.jsonl').read_text().splitlines():
            record=json.loads(line)
            if record['kind']=='PnP':assert all(.7-1e-6<=v[2]<=1.1+1e-6 for v in record['centers'])
        for line in (folder/'wire-inputs.jsonl').read_text().splitlines():
            wire=json.loads(line)
            assert all(len(bytes.fromhex(h))==16 for h in wire['xbox'])
            for text in wire['companion']:
                frame=bytes.fromhex(text);assert len(frame)==28 and frame[:4]==b'QN\1\1'
                assert zlib.crc32(frame[:24])==int.from_bytes(frame[24:],'little')
                assert all(math.isfinite(v) for v in struct.unpack('<4f',frame[8:24]))
    with (folder/'xbox-reports.csv').open(newline='') as f:
        pilot=list(csv.DictReader(f));assert len(pilot)>100 and all(len(bytes.fromhex(row['raw_hid_hex']))==16 for row in pilot)
    video=cv2.VideoCapture(str(folder/'gazebo.mp4'));frames=int(video.get(cv2.CAP_PROP_FRAME_COUNT));fps=video.get(cv2.CAP_PROP_FPS);ok,image=video.read();video.release()
    assert ok and frames>tr['simulated_seconds']*4.8 and abs(fps-5)<.01
    checks.append(dict(name=r['name'],passed=True,video_frames=frames,video_fps=fps,original_files_sha256={f.name:sha(f) for f in sorted(folder.iterdir()) if f.is_file()}))
out=report_path.parent/'evidence-audit.json'
out.write_text(json.dumps(dict(passed=True,checked_runs=len(checks),scope='Original file/CRC/result/video/source consistency audit. No new simulation or physical test.',runs=checks),indent=2)+'\n',encoding='utf8')
print('PASS: 15 original runs; role boundaries, endpoints, no native panel contact, fixture prior, CRC frames, videos and active C/DLL fingerprints.')
