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
bool Navigator::plan(const State& s,const std::array<double,3>& goal,
                      const std::vector<Obstacle>& obstacles) {
  path_.clear(); waypoint_=0; goal_=goal;
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
  if(start<0||finish<0||occupied[start]||occupied[finish])return false;
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
      int v=y*nx+x;if(occupied[v])continue;
      if(dx&&dy&&(occupied[uy*nx+x]||occupied[y*nx+ux]))continue;
      double candidate=cost[u]+resolution*std::hypot(dx,dy);
      if(candidate<cost[v]){cost[v]=candidate;parent[v]=u;
        double h=resolution*std::hypot(x-finish%nx,y-finish/nx);
        frontier.push({candidate+h,v});}
    }
  }
  if(!std::isfinite(cost[finish]))return false;
  for(int v=finish;v!=-1;v=parent[v])path_.push_back({xmin+(v%nx)*resolution,ymin+(v/nx)*resolution});
  std::reverse(path_.begin(),path_.end());
  path_.back()={goal[0],goal[1]};
  waypoint_=path_.size()>1?1:0;return true;
}
Command Navigator::step(const State&s,bool sensing_fresh){
  Command c;
  if(!sensing_fresh||path_.empty())return c;
  const double vals[]={s.x,s.y,s.z,s.vx,s.vy,s.vz,s.yaw};
  for(double v:vals)if(!std::isfinite(v))return c;
  while(waypoint_+1<path_.size()&&std::hypot(path_[waypoint_][0]-s.x,path_[waypoint_][1]-s.y)<0.4)++waypoint_;
  double ex=path_[waypoint_][0]-s.x,ey=path_[waypoint_][1]-s.y;
  double vx=1.0*ex,vy=1.0*ey,vnorm=std::hypot(vx,vy);
  if(vnorm>0.8){vx*=0.8/vnorm;vy*=0.8/vnorm;}
  double ax=clip(1.8*(vx-s.vx),1.4),ay=clip(1.8*(vy-s.vy),1.4);
  double bx=std::cos(s.yaw)*ax+std::sin(s.yaw)*ay;
  double by=-std::sin(s.yaw)*ax+std::cos(s.yaw)*ay;
  c.roll_deg=clip(-std::atan2(by,9.80665)*180.0/3.141592653589793,15.0);
  c.pitch_deg=clip(std::atan2(bx,9.80665)*180.0/3.141592653589793,15.0);
  // Face the intended path velocity, not measured braking drift. Hold the last
  // heading near a node instead of snapping to zero as speed crosses a threshold.
  if(std::hypot(vx,vy)>.2)heading_target_=std::atan2(vy,vx);
  c.yaw_rate_dps=clip(1.2*std::remainder(heading_target_-s.yaw,2*3.141592653589793)*180.0/3.141592653589793,45.0);
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
