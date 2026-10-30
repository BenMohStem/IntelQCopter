"""Compile actual estimator helper + pinned library on host; no ESP hardware model."""
from pathlib import Path
import subprocess
import hashlib
import json
root=Path(__file__).resolve().parents[3]
out=root/'artifacts/fusion-host';out.mkdir(exist_ok=True)
source=(root/'lessons/attitude-preview/main.cpp').read_text(encoding='utf-8')
helper=source[source.index('static Adafruit_Mahony attitude;'):source.index('static void process_one_sample(void)')]
library=root/'lessons/attitude-preview/.pio/libdeps/preview/Adafruit AHRS/src'
tests=r"""
int main()
{
    float level[3] = {0, 0, 9.80665f};
    float zero[3] = {0, 0, 0};
    float no_accel[3] = {0, 0, 0};
    int cases = 0;
    for (int axis = 0; axis < 3; axis++)
    {
        for (int direction = -1; direction <= 1; direction += 2)
        {
            attitude = Adafruit_Mahony();
            attitude_started = false;
            assert(update_attitude(level, zero));
            float rate[3] = {0, 0, 0};
            rate[axis] = direction * 1.57079632679f;
            for (int i = 0; i < 1000; i++)
            {
                assert(update_attitude(no_accel, rate));
            }
            assert(fabsf(attitude_q[0] - 0.70710678f) < 0.002f);
            assert(fabsf(attitude_q[axis + 1] - direction * 0.70710678f) < 0.002f);
            cases++;
        }
    }
    attitude = Adafruit_Mahony();
    attitude_started = false;
    float tilt[3] = {0, 4.903325f, 8.492808f};
    assert(update_attitude(tilt, zero));
    attitude.setQuaternion(1, 0, 0, 0);
    for (int i = 0; i < 20000; i++) assert(update_attitude(tilt, zero));
    assert(fabsf(attitude_q[1] - 0.258819f) < 0.005f);
    cases++;
    float bad[3] = {NAN, 0, 0};
    assert(!update_attitude(level, bad));
    cases++;
    attitude_started = false;
    assert(!update_attitude(no_accel, zero));
    cases++;
    printf("PASS %d actual-source host cases; no hardware acceptance\n", cases);
}
"""
test=out/'test.cpp'
test.write_text('#include <cmath>\n#include <cstdio>\n#include <cassert>\n#include <Adafruit_AHRS_Mahony.h>\nstatic const float radians_to_degrees=57.29577951308232f;\n'+helper+tests)
exe=out/'test.exe'
subprocess.run(['C:/WinLibs/zig/zig.exe','c++','-O2','-I'+str(library),str(test),str(library/'Adafruit_AHRS_Mahony.cpp'),'-o',str(exe)],check=True)
result=subprocess.run([str(exe)],check=True,capture_output=True,text=True)
print(result.stdout)
Path(__file__).with_name('host-results.json').write_text(json.dumps({'engine':'Native host compiler, actual helper and Adafruit source; not ESP timing or physical simulation','source_sha256':hashlib.sha256((root/'lessons/attitude-preview/main.cpp').read_bytes()).hexdigest(),'library_cpp_sha256':hashlib.sha256((library/'Adafruit_AHRS_Mahony.cpp').read_bytes()).hexdigest(),'result':result.stdout.strip()},indent=2)+'\n')
