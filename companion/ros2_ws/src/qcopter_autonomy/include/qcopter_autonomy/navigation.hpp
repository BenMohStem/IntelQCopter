#pragma once
#include <array>
#include <cstddef>
#include <vector>

namespace qcopter {
struct Obstacle { double x, y, radius; };
struct State { double x,y,z,vx,vy,vz,yaw; };
struct Command { double roll_deg=0,pitch_deg=0,yaw_rate_dps=0,throttle=0.5; bool active=false,reached=false; };
class Navigator {
 public:
  bool plan(const State&, const std::array<double,3>&, const std::vector<Obstacle>&);
  Command step(const State&, bool sensing_fresh=true);
  const std::vector<std::array<double,2>>& path() const { return path_; }
 private:
  std::vector<std::array<double,2>> path_;
  std::array<double,3> goal_{};
  std::size_t waypoint_=0;
  double heading_target_=0;bool heading_initialized_=false;
};
}
