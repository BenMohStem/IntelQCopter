"""Offline regressions: run py -3 tools/test_mag_probe_capture.py"""
from pathlib import Path
import importlib.util
spec = importlib.util.spec_from_file_location('probe', Path(__file__).with_name('mag_probe_capture.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
lines=['BOOT_PROFILE requested=100 ctrl1=0xC9 ctrl2=0x08',
       'CSV,1000000,1,2,3,0x19,1',
       'CSV,1010000,1,3,4,0x19,1',
       'CSV,1020000,1,3,5,0x19,1',
       'HEALTH profile=100 measured_fresh_hz=97.5 fresh=2 regs_ok=1 age_sec=0.01',
       'VERIFY reason=IMMEDIATE id=0x80 C1=0xC9 C2=0x08 this_read=1']
s=probe.extract_summary(lines)
assert s['csv_samples']==3 and abs(s['csv_rate_hz']-100)<.01 and s['last_health_hz']==97.5
assert s['verify_lines'] and s['boot_profiles']
assert s['v13_boot_confirmed']
decimated=probe.extract_summary(['BOOT_PROFILE requested=200 ctrl1=0xCD',
    'CSV,1000000,1,2,3,0x19,1', 'CSV,1020000,1,2,3,0x19,1',
    'HEALTH profile=200 measured_fresh_hz=199.7 fresh=800 csv_every=4'])
assert decimated['csv_rate_hz']==50 and decimated['last_health_hz']==199.7
assert decimated['peak_health_hz']==199.7 and decimated['last_health']['profile']=='200'
wrong_firmware=probe.extract_summary(['MAG ACK=1 ID=1:0x80 C1=1:0x00'])
assert not wrong_firmware['v13_boot_confirmed'] and wrong_firmware['csv_samples']==0
assert probe.extract_summary(['HEALTH profile=100 measured_fresh_hz=0 fresh=0'])['v13_boot_confirmed']
assert not probe.extract_summary(['Waiting for download'])['v13_boot_confirmed']
print('mag probe parser tests passed')
