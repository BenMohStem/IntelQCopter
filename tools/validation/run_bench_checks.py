"""Run each offline bench test once; does not open a serial port or flash."""
from pathlib import Path
import argparse
import hashlib
import importlib
import io
import json
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/calibration'))
import record_poses as transport


def flatten(suite):
    for test in suite:
        if isinstance(test,unittest.TestSuite):
            yield from flatten(test)
        else:
            yield test


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native',action='store_true',help='Also execute actual firmware with fake peripherals using installed MSVC; not ESP32 emulation')
    args=parser.parse_args()
    out=ROOT/'artifacts/bench-checks'/transport.stamp()
    out.mkdir(parents=True)
    suite=unittest.TestSuite()
    identifiers=set()
    for name in ['test_accel_xyz','test_gyro_observation','test_gyro_bias','test_gyro_selection','test_tilt_observation','test_fifo1000','test_fifo_buffer']:
        module=importlib.import_module(name)
        for test in flatten(unittest.defaultTestLoader.loadTestsFromModule(module)):
            if test.id() not in identifiers:
                identifiers.add(test.id())
                suite.addTest(test)
    output=io.StringIO()
    result=unittest.TextTestRunner(stream=output,verbosity=2).run(suite)
    (out/'offline-tests.log').write_text(output.getvalue(),encoding='utf-8')
    report={'engine':'Python protocol/arithmetic tests; no MCU or sensor execution',
            'source_sha256':hashlib.sha256((ROOT/'firmware/main.cpp').read_bytes()).hexdigest(),
            'unique_tests':result.testsRun,'success':result.wasSuccessful(),'uploaded':False,'serial_opened':False}
    if args.native and result.wasSuccessful():
        import subprocess
        native=subprocess.run([sys.executable,str(ROOT/'tools/validation/run_native_fifo_tests.py')],cwd=ROOT,capture_output=True,text=True)
        (out/'native-launch.log').write_text(native.stdout+native.stderr,encoding='utf-8')
        report['native_exit_code']=native.returncode
        report['success']=report['success'] and native.returncode==0
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(output.getvalue())
    print('Original check evidence:',out)
    if not report['success']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
