from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[3]
source=(root/'lessons/attitude-preview/main.cpp').read_text(encoding='utf-8')
code=source[source.index('struct PidPreview'):source.index('// Core 1 owns sensor state.')]
tests=r"""
int main(){
 PidPreview p={};update_pid_preview(&p,0);assert(p.output==0 && p.d==0);
 PidPreview step={};update_pid_preview(&step,1);assert(step.p<0 && step.d==0 && step.output<0);
 PidPreview saturated={};
 for(int i=0;i<100000;i++){update_pid_preview(&saturated,10);assert(fabsf(saturated.output)<=0.200001f);assert(isfinite(saturated.output));}
 assert(saturated.i==0); // outward integration prevented throughout saturation
 update_pid_preview(&saturated,-10);assert(fabsf(saturated.output)<=0.200001f);
 for(int i=0;i<10000;i++) update_pid_preview(&saturated,0);
 assert(fabsf(saturated.output)<0.001f);
 puts("PASS actual PID helper: zero, restoring P, no startup derivative kick, bounded saturation, antiwindup, reversal and recovery; host only");
}
"""
out=root/'artifacts/pid-host';out.mkdir(exist_ok=True)
cpp=out/'test.cpp';cpp.write_text('#include <math.h>\n#include <cassert>\n#include <cstdio>\n'+code+tests)
exe=out/'test.exe'
subprocess.run(['C:/WinLibs/zig/zig.exe','c++','-O2',str(cpp),'-o',str(exe)],check=True)
result=subprocess.run([str(exe)],check=True,capture_output=True,text=True)
print(result.stdout)
Path(__file__).with_name('test-results.txt').write_text(result.stdout)
