#pragma once
#include "qcopter_autonomy/navigation.hpp"
namespace qcopter {
enum class Phase {Idle,Spool,Takeoff,Hover,Navigate,Approach,Land,Landed};
struct MissionOutput { Command command; Phase phase=Phase::Idle; double spool=0; bool motor_gate=false,done=false; };
class Mission {
 public:
  bool request(const State& actual,const std::array<double,3>&start,const std::array<double,3>&finish,double cruise);
  MissionOutput step(const State&,const Command&navigation,bool odometry_fresh,bool perception_fresh,bool contact,double dt);
  void request_land(const State&s){if(phase_!=Phase::Idle&&phase_!=Phase::Landed){finish_[0]=s.x;finish_[1]=s.y;transition(Phase::Land);}}
 private:
  Phase phase_=Phase::Idle;std::array<double,3> finish_{};double cruise_=1.5,elapsed_=0,dwell_=0;
  void transition(Phase phase){phase_=phase;elapsed_=dwell_=0;}
};
}
