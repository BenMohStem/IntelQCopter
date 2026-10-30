from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[3]
source=(root/'lessons/attitude-preview/main.cpp').read_text(encoding='utf-8')
actual=source[source.index('static float stick_command'):source.index('static void xbox_worker')]
packet=source[source.index('struct TargetPacket'):source.index('static float stick_command')]
stub=r"""
#include <cmath>
#include <cstdint>
#include <cassert>
#include <cstdio>
using QueueHandle_t=int;
uint32_t clock_ms=0;
uint32_t millis(){return clock_ms;}
struct FakeXbox {
 bool connected=false; uint32_t received=0;
 struct {uint16_t joyRHori=32768,joyRVert=32768,joyLHori=32768,joyLVert=32768;} xboxNotif;
 bool isConnected(){return connected;}
 bool isWaitingForFirstNotification(){return false;}
 uint32_t getReceiveNotificationAt(){return received;}
} xbox;
"""
tail=r"""
TargetPacket observed={};
int xQueueOverwrite(int,const TargetPacket* p){observed=*p;return 1;}
"""
tests=r"""
void tick(uint32_t now){clock_ms=now;report_targets();}
int main(){
 assert(stick_command(0)==-1);assert(stick_command(65535)==1);
 assert(stick_command(32768)==0);assert(stick_command(32767)==0);
 float last=-1;
 for(int i=0;i<65536;i++){float v=stick_command(i);assert(v>=last-1e-6f);assert(fabsf(v)<=1);last=v;}
 tick(100);assert(!observed.valid);
 xbox.connected=true;xbox.received=200;tick(200);assert(observed.valid);assert(observed.roll==0);
 xbox.xboxNotif.joyRHori=65535;xbox.received=300;tick(300);assert(observed.valid && observed.roll==20);
 xbox.connected=false;tick(400);assert(!observed.valid && observed.roll==0);
 xbox.connected=true;xbox.received=500;tick(500);assert(!observed.valid);
 xbox.xboxNotif.joyRHori=32768;xbox.received=600;tick(600);assert(observed.valid);
 tick(1000);assert(!observed.valid);
 puts("PASS actual stick mapping: all65536codes bounded/monotonic, neutral, startup, disconnect, recenter, stale guard; host only");
}
"""
out=root/'artifacts/target-host';out.mkdir(exist_ok=True)
cpp=out/'test.cpp';cpp.write_text(stub+packet+tail+actual+tests)
exe=out/'test.exe'
subprocess.run(['C:/WinLibs/zig/zig.exe','c++','-O2',str(cpp),'-o',str(exe)],check=True)
result=subprocess.run([str(exe)],check=True,capture_output=True,text=True)
print(result.stdout)
Path(__file__).with_name('target-test-results.txt').write_text(result.stdout)
