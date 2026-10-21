#include "qcopter_autonomy/mission.hpp"
#include <algorithm>
#include <cmath>
namespace qcopter {
bool Mission::request(const State&s,const std::array<double,3>&start,const std::array<double,3>&finish,double cruise){
  const double values[]={s.x,s.y,s.z,s.vx,s.vy,s.vz,s.yaw,start[0],start[1],start[2],finish[0],finish[1],finish[2],cruise};
  for(double v:values)if(!std::isfinite(v))return false;
  if(phase_!=Phase::Idle&&phase_!=Phase::Landed)return false;
  if(std::hypot(s.x-start[0],s.y-start[1])>.25||std::abs(s.z-start[2])>.15
     ||std::hypot(s.vx,s.vy)>.15||std::abs(s.vz)>.15||cruise<.6||cruise>2.5
     ||start[0]<-1||start[0]>9||std::abs(start[1])>3||std::abs(finish[2]-start[2])>.15||cruise<start[2]+.4
     ||finish[0]<-1||finish[0]>9||std::abs(finish[1])>3)return false;
  navigation_finish_enabled_=true;hold_heading_=s.yaw;finish_=finish;cruise_=cruise;transition(Phase::Spool);return true;
}
MissionOutput Mission::step(const State&s,const Command&nav,bool odometry_fresh,bool perception_fresh,bool contact,double dt){
  MissionOutput o;o.phase=phase_;
  if(!std::isfinite(dt)||dt<=0||dt>.1)return o;
  const double v[]={s.x,s.y,s.z,s.vx,s.vy,s.vz,s.yaw};for(double x:v)if(!std::isfinite(x))return o;
  if(phase_==Phase::Idle||phase_==Phase::Landed){o.done=phase_==Phase::Landed;return o;}
  elapsed_+=dt;
  o.command.active=odometry_fresh;o.motor_gate=true;o.spool=1;
  if(!odometry_fresh)return o; // invalid command asks downstream FC to retain/fallback; never airborne motor-cut
  constexpr double todeg=180/3.141592653589793;
  // Brake horizontal drift while taking off/holding/descending. Same angle/rate FC interface.
  const bool landing_hold=phase_==Phase::Approach||phase_==Phase::Land;
  const double ax=std::clamp((landing_hold?1.2*(finish_[0]-s.x):0)-1.8*s.vx,-1.,1.);
  const double ay=std::clamp((landing_hold?1.2*(finish_[1]-s.y):0)-1.8*s.vy,-1.,1.);
  const double bx=std::cos(s.yaw)*ax+std::sin(s.yaw)*ay,by=-std::sin(s.yaw)*ax+std::cos(s.yaw)*ay;
  o.command.roll_deg=-std::atan2(by,9.80665)*todeg;o.command.pitch_deg=std::atan2(bx,9.80665)*todeg;
  o.command.yaw_rate_dps=std::clamp(1.2*std::remainder(hold_heading_-s.yaw,2*3.141592653589793)*todeg,-45.,45.);
  double vz=std::clamp(1.0*(cruise_-s.z),-.45,.45);
  switch(phase_){
   case Phase::Spool:o.spool=std::clamp(elapsed_,0.,1.);vz=0;if(elapsed_>=1)transition(Phase::Takeoff);break;
   case Phase::Takeoff:
     if(std::abs(s.z-cruise_)<.10&&std::abs(s.vz)<.15)transition(Phase::Hover);break;
   case Phase::Hover:if(elapsed_>1.2)transition(Phase::Navigate);break;
   case Phase::Navigate:
     if(perception_fresh&&nav.active){
       o.command=nav;
       hold_heading_=s.yaw; // retain arrival heading for approach/landing
     }
     if(navigation_finish_enabled_&&std::hypot(s.x-finish_[0],s.y-finish_[1])<.18&&std::hypot(s.vx,s.vy)<.15)transition(Phase::Approach);
     break;
   case Phase::Approach:
     dwell_=std::hypot(s.x-finish_[0],s.y-finish_[1])<.10&&std::hypot(s.vx,s.vy)<.10 ? dwell_+dt : 0;
     if(dwell_>1.0)transition(Phase::Land);break;
   case Phase::Land:
     vz=s.z-finish_[2]>.35 ? -.25 : -.10;
     dwell_=contact&&std::abs(s.vz)<.12 ? dwell_+dt : 0;
     if(dwell_>.35){transition(Phase::Landed);o.motor_gate=false;o.done=true;o.spool=0;}
     break;
   default:break;
  }
  if(phase_!=Phase::Navigate || !perception_fresh || !nav.active)o.command.throttle=.5+vz/4;
  o.phase=phase_;return o;
}
}
#ifdef QCOPTER_NAV_DLL
#define MISSION_EXPORT __declspec(dllexport)
static qcopter::Mission mission;
extern "C" MISSION_EXPORT int qc_mission_start(const double*s,const double*a,const double*b,double h){
 mission=qcopter::Mission();return mission.request({s[0],s[1],s[2],s[3],s[4],s[5],s[6]},
  {a[0],a[1],a[2]},{b[0],b[1],b[2]},h);}
extern "C" MISSION_EXPORT void qc_mission_step(const double*s,const double*n,double dt,unsigned flags,double*out){
 qcopter::Command c;c.roll_deg=n[0];c.pitch_deg=n[1];c.yaw_rate_dps=n[2];c.throttle=n[3];c.active=n[4]>0;
 auto o=mission.step({s[0],s[1],s[2],s[3],s[4],s[5],s[6]},c,flags&1,flags&2,flags&4,dt);
 out[0]=o.command.roll_deg;out[1]=o.command.pitch_deg;out[2]=o.command.yaw_rate_dps;out[3]=o.command.throttle;
 out[4]=o.command.active;out[5]=int(o.phase);out[6]=o.spool;out[7]=o.motor_gate;out[8]=o.done;}
#endif
