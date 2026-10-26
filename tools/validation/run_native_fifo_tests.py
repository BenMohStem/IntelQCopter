"""Compile actual firmware against fake peripherals and execute fault cases. Not ESP32 emulation."""
from pathlib import Path
import hashlib
import io
import json
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/calibration'))
import record_fifo1000 as recording
import record_gyro as gyro
import record_poses as transport


def main():
    out=ROOT/'artifacts/native-fifo-tests'/transport.stamp()
    out.mkdir(parents=True)
    sha=hashlib.sha256((ROOT/'firmware/main.cpp').read_bytes()).hexdigest()
    header=out/'source-sha.hpp'
    header.write_text('#define BENCH_SOURCE_SHA "'+sha+'"\n',encoding='utf-8')
    vswhere=Path('C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe')
    paths=subprocess.check_output([str(vswhere),'-latest','-products','*','-find','VC/Auxiliary/Build/vcvars64.bat'],text=True).strip().splitlines()
    if len(paths)!=1:
        raise RuntimeError('Need one installed x64 Visual C++ build environment')
    exe=out/'native_fifo_test.exe'
    command=out/'compile.cmd'
    lines=['@echo off',f'call "{paths[0]}"','if errorlevel 1 exit /b 1',
           f'cl /nologo /std:c++17 /EHsc /O2 /W4 /D_CRT_SECURE_NO_WARNINGS /FI"{header}" /I"{ROOT / "tools/validation/arduino_stub"}" /Fo"{out / "native_fifo_test.obj"}" /Fe"{exe}" "{ROOT / "tools/validation/native_fifo_test.cpp"}"','exit /b %errorlevel%']
    command.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    build=subprocess.run(['cmd.exe','/d','/c',str(command)],cwd=ROOT,capture_output=True)
    (out/'compiler.log').write_bytes(build.stdout+build.stderr)
    if build.returncode:
        print((build.stdout+build.stderr).decode(errors='replace'))
        raise RuntimeError('Native model build failed; original compiler output preserved')
    fit=json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
    class SavedDevice:
        def __init__(self,data):
            self.data=io.BytesIO(data)
        def write(self,value):
            assert value==b'?'
        def readline(self):
            return self.data.readline()
    results=[]
    for name in ['normal','wrap','retained','invalid_boot','invalid_gyro_boot','trim_readback_failure','fifo_enable_failure','overflow','i2c_failure','fifo_packet_failure','offset_change','slow_bus','timeout','abort','queue_full','logger_drain_failure','basis_x','basis_y','basis_z']:
        case=subprocess.run([str(exe),name],capture_output=True,timeout=30)
        (out/(name+'.serial.log')).write_bytes(case.stdout)
        (out/(name+'.stderr.log')).write_bytes(case.stderr)
        if case.returncode:
            raise RuntimeError(f'Host case {name} failed, exit {case.returncode}; evidence {out}')
        result={'case':name,'exit_code':case.returncode,'result':case.stderr.decode(errors='replace').strip()}
        if name in ('normal','wrap','retained','basis_x','basis_y','basis_z'):
            device=SavedDevice(case.stdout)
            _,profile=gyro.read_header(device,io.BytesIO(),sha,fit)
            stream=recording.Samples(profile)
            started=False
            for raw in device.data:
                line=raw.decode('ascii').strip()
                if line=='BEGIN,20000':
                    started=True
                elif started and line.startswith('S,'):
                    stream.add(line)
                elif started and line.startswith('D,'):
                    stream.diagnostic(line)
                elif started and line.startswith('END,'):
                    trailer=stream.finish(line)
                    result['simulated_clock_trailer_NOT_MCU_TIMING']=trailer
                    break
            else:
                raise RuntimeError('No complete verified host stream')
        results.append(result)
        print(result['result'])
    report={'engine':'Native x64 execution of actual firmware/main.cpp compiled with MSVC, fake Arduino/I2C/serial peripherals',
            'ESP32_emulation':False,'physical_test':False,'source_sha256':sha,'cases':results,
            'limitations':'Cooperative task scheduling, synthetic clock, FIFO source, UART and I2C costs; cannot validate actual MCU timing, electrical behavior, silicon FIFO ordering, serial throughput or motor readiness.',
            'firmware_uploaded':False,'original_evidence_directory':str(out.relative_to(ROOT))}
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Original host evidence:',out)


if __name__=='__main__':
    main()
