#include "qcopter_autonomy/mission.hpp"
#undef NDEBUG
#include <cassert>
#include <iostream>
int main(){
 qcopter::Mission m;qcopter::State s{0,0,.082,0,0,0,0};qcopter::Command nav;
 assert(!m.request(s,{1,0,.082},{8,0,.082},1.25));
 assert(m.request(s,{0,0,.082},{8,0,.082},1.25));
 for(int i=0;i<22;i++)m.step(s,nav,true,true,true,.05);
 s.z=1.25;for(int i=0;i<30;i++)m.step(s,nav,true,true,false,.05);
 auto stale=m.step(s,nav,false,true,false,.05);assert(!stale.command.active && stale.motor_gate);
 m.request_land(s);s.z=.082;
 auto no_contact=m.step(s,nav,true,true,false,.05);assert(!no_contact.done && no_contact.motor_gate);
 for(int i=0;i<10;i++)m.step(s,nav,true,true,true,.05);
 auto landed=m.step(s,nav,true,true,true,.05);assert(landed.done && !landed.motor_gate);
 std::cout<<"Mission C++ checks passed: start mismatch, state progression, stale pose, contact-gated landing; no hardware arming.\n";
}
