#include "qcopter_autonomy/navigation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace qcopter {
namespace {
constexpr int nx=61,ny=41;
constexpr double resolution=0.2,xmin=-2.0,ymin=-4.0,inflation=0.65;
double clip(double v,double limit){ return std::clamp(v,-limit,limit); }
}
bool Navigator::configure(const NavigationConfig&c){
  const double v[]={c.position_gain,c.velocity_gain,c.max_speed_mps,c.max_acceleration_mps2,c.max_angle_deg,c.heading_gain,c.max_yaw_rate_dps};
  for(double x:v)if(!std::isfinite(x)||x<=0)return false;
  if(c.position_gain>20||c.velocity_gain>20||c.max_speed_mps>10||c.max_acceleration_mps2>20||c.max_angle_deg>45||c.heading_gain>20||c.max_yaw_rate_dps>180)return false;
  config_=c;return true;
}
bool Navigator::plan(const State& s,const std::array<double,3>& goal,
                      const std::vector<Obstacle>& obstacles) {
  path_.clear(); waypoint_=0; goal_=goal;reason_="INVALID_INPUT";observed_=obstacles;
  if(!std::isfinite(s.x)||!std::isfinite(s.y)||!std::isfinite(s.z)||!std::isfinite(s.yaw))return false;
  if(!heading_initialized_){heading_target_=s.yaw;heading_initialized_=true;}
  for(double v:goal)if(!std::isfinite(v))return false;
  std::vector<bool> occupied(nx*ny,false);
  for(const auto&o:obstacles)if(!std::isfinite(o.x)||!std::isfinite(o.y)||!std::isfinite(o.radius)||o.radius<0)return false;
  for(int y=0;y<ny;++y)for(int x=0;x<nx;++x)
    for(const auto&o:obstacles)if(std::hypot(xmin+x*resolution-o.x,ymin+y*resolution-o.y)<=o.radius+inflation)
      occupied[y*nx+x]=true;
  auto cell=[](double x,double y){
    if(x<xmin||x>xmin+(nx-1)*resolution||y<ymin||y>ymin+(ny-1)*resolution)return -1;
    int ix=static_cast<int>(std::lround((x-xmin)/resolution));
    int iy=static_cast<int>(std::lround((y-ymin)/resolution));
    return ix<0||ix>=nx||iy<0||iy>=ny ? -1 : iy*nx+ix;
  };
  int start=cell(s.x,s.y),finish=cell(goal[0],goal[1]);
  if(start<0||finish<0){reason_="OUTSIDE_MAP";return false;}
  auto clearance=[&](double x,double y,double margin){double d=1e9;for(const auto&o:obstacles)d=std::min(d,std::hypot(x-o.x,y-o.y)-o.radius-margin);return d;};
  if(clearance(s.x,s.y,.34)<=0){reason_="START_INSIDE_OBSTACLE";return false;}
  if(occupied[finish]){reason_="GOAL_INSIDE_CLEARANCE";return false;}
  escaping_=clearance(s.x,s.y,inflation)<0;reason_=escaping_?"ESCAPING_CLEARANCE_MARGIN":"NO_PATH";
  std::vector<double> cost(nx*ny,std::numeric_limits<double>::infinity());
  std::vector<int> parent(nx*ny,-1);
  using Item=std::pair<double,int>;
  std::priority_queue<Item,std::vector<Item>,std::greater<Item>> frontier;
  cost[start]=0;frontier.push({0,start});
  while(!frontier.empty()){
    int u=frontier.top().second;frontier.pop(); if(u==finish)break;
    int ux=u%nx,uy=u/nx;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
      if(dx==0&&dy==0)continue;
      int x=ux+dx,y=uy+dy;if(x<0||x>=nx||y<0||y>=ny)continue;
      int v=y*nx+x;
      const double fromx=u==start?s.x:xmin+ux*resolution,fromy=u==start?s.y:ymin+uy*resolution;
      const double tox=xmin+x*resolution,toy=ymin+y*resolution;
      const bool margin_exit=occupied[u]||(u==start&&escaping_);
      if(occupied[v]&&!margin_exit)continue;
      if(margin_exit){
        // The start may enter a newly enlarged uncertainty margin while still
        // outside the observed obstacle plus aircraft radius. Permit only a
        // sampled, outward, non-decreasing-clearance exit; never clear cells
        // blindly or permit crossing an observed obstacle.
        bool safe=true;double previous=clearance(fromx,fromy,inflation);
        for(int k=1;k<=12;k++){double a=k/12.;double px=fromx+a*(tox-fromx),py=fromy+a*(toy-fromy),d=clearance(px,py,inflation);
          if(clearance(px,py,.34)<=0||d+1e-8<previous){safe=false;break;}previous=d;}
        if(!safe)continue;
      }else if(dx&&dy&&(occupied[uy*nx+x]||occupied[y*nx+ux]))continue;
      double candidate=cost[u]+resolution*std::hypot(dx,dy);
      if(candidate<cost[v]){cost[v]=candidate;parent[v]=u;
        double h=resolution*std::hypot(x-finish%nx,y-finish/nx);
        frontier.push({candidate+h,v});}
    }
  }
  if(!std::isfinite(cost[finish])){reason_="NO_PATH";return false;}
  for(int v=finish;v!=-1;v=parent[v])path_.push_back({xmin+(v%nx)*resolution,ymin+(v/nx)*resolution});
  std::reverse(path_.begin(),path_.end());
  path_.back()={goal[0],goal[1]};
  reason_=escaping_?"ESCAPING_CLEARANCE_MARGIN":"OK";waypoint_=path_.size()>1?1:0;return true;
}
Command Navigator::step(const State&s,bool sensing_fresh){
  Command c;
  if(!sensing_fresh||path_.empty())return c;
  const double vals[]={s.x,s.y,s.z,s.vx,s.vy,s.vz,s.yaw};
  for(double v:vals)if(!std::isfinite(v))return c;
  if(escaping_){double d=1e9;for(const auto&o:observed_)d=std::min(d,std::hypot(s.x-o.x,s.y-o.y)-o.radius-inflation);if(d>=0){escaping_=false;reason_="OK";}}
  while(waypoint_+1<path_.size()&&std::hypot(path_[waypoint_][0]-s.x,path_[waypoint_][1]-s.y)<(escaping_?.10:.4))++waypoint_;
  double ex=path_[waypoint_][0]-s.x,ey=path_[waypoint_][1]-s.y;
  double vx=config_.position_gain*ex,vy=config_.position_gain*ey,vnorm=std::hypot(vx,vy);
  const double max_speed=escaping_?std::min(.25,config_.max_speed_mps):config_.max_speed_mps;
  if(vnorm>max_speed){vx*=max_speed/vnorm;vy*=max_speed/vnorm;}
  double ax=clip(config_.velocity_gain*(vx-s.vx),config_.max_acceleration_mps2),ay=clip(config_.velocity_gain*(vy-s.vy),config_.max_acceleration_mps2);
  double bx=std::cos(s.yaw)*ax+std::sin(s.yaw)*ay;
  double by=-std::sin(s.yaw)*ax+std::cos(s.yaw)*ay;
  c.roll_deg=clip(-std::atan2(by,9.80665)*180.0/3.141592653589793,config_.max_angle_deg);
  c.pitch_deg=clip(std::atan2(bx,9.80665)*180.0/3.141592653589793,config_.max_angle_deg);
  // Face the intended path velocity, not measured braking drift. Hold the last
  // heading near a node instead of snapping to zero as speed crosses a threshold.
  if(std::hypot(vx,vy)>.2)heading_target_=std::atan2(vy,vx);
  c.yaw_rate_dps=clip(config_.heading_gain*std::remainder(heading_target_-s.yaw,2*3.141592653589793)*180.0/3.141592653589793,config_.max_yaw_rate_dps);
  c.throttle=0.5+clip(goal_[2]-s.z,0.6)/4.0;c.active=true;
  c.reached=std::hypot(goal_[0]-s.x,goal_[1]-s.y)<0.2&&std::abs(goal_[2]-s.z)<0.2&&std::hypot(s.vx,s.vy)<0.15;
  return c;
}
}

#if defined(QCOPTER_NAV_DLL)
#if defined(_WIN32)
#define NAV_EXPORT __declspec(dllexport)
#else
#define NAV_EXPORT __attribute__((visibility("default")))
#endif
static qcopter::Navigator navigator;
extern "C" NAV_EXPORT int qc_nav_plan(const double*state,const double*goal,const double*obs,int count){
  qcopter::State s{state[0],state[1],state[2],state[3],state[4],state[5],state[6]};
  std::vector<qcopter::Obstacle> obstacles;
  if(count<0||count>10000)return 0;
  for(int i=0;i<count;++i)obstacles.push_back({obs[3*i],obs[3*i+1],obs[3*i+2]});
  return navigator.plan(s,{goal[0],goal[1],goal[2]},obstacles)?1:0;
}
extern "C" NAV_EXPORT void qc_nav_step(const double*state,int fresh,double*out){
  auto c=navigator.step({state[0],state[1],state[2],state[3],state[4],state[5],state[6]},fresh!=0);
  out[0]=c.roll_deg;out[1]=c.pitch_deg;out[2]=c.yaw_rate_dps;out[3]=c.throttle;out[4]=c.active;out[5]=c.reached;
}
extern "C" NAV_EXPORT int qc_nav_path(double*out,int capacity){
  const auto&p=navigator.path();int n=std::min(capacity,static_cast<int>(p.size()));
  for(int i=0;i<n;++i){out[2*i]=p[i][0];out[2*i+1]=p[i][1];}return n;
}
#endif
