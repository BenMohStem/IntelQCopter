#pragma once
#include <array>
#include <cstddef>
#include <vector>

namespace qcopter {
struct Obstacle { double x, y, radius; };
struct State { double x,y,z,vx,vy,vz,yaw; };
struct Command { double roll_deg=0,pitch_deg=0,yaw_rate_dps=0,throttle=0.5; bool active=false,reached=false; };
struct NavigationConfig {double position_gain=1.,velocity_gain=1.8,max_speed_mps=.8,max_acceleration_mps2=1.4,max_angle_deg=15.,heading_gain=1.2,max_yaw_rate_dps=45.;};
class Navigator {
 public:
  bool configure(const NavigationConfig&);
  const char* reason() const{return reason_;}
  bool plan(const State&, const std::array<double,3>&, const std::vector<Obstacle>&);
  Command step(const State&, bool sensing_fresh=true);
  const std::vector<std::array<double,2>>& path() const { return path_; }
 private:
  std::vector<std::array<double,2>> path_;
  std::array<double,3> goal_{};
  NavigationConfig config_{};std::vector<Obstacle> observed_;bool escaping_=false;const char*reason_="IDLE";
  std::size_t waypoint_=0;
  double heading_target_=0;bool heading_initialized_=false;
};
}
