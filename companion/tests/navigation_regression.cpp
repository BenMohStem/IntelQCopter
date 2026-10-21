#include "qcopter_autonomy/navigation.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
 qcopter::Navigator n;
 qcopter::State s{8.282,.264,1.25,0,0,0,0};
 std::vector<qcopter::Obstacle> obs{{6.835,-.064,.910}};
 assert(n.plan(s,{7.07,1.76,1.25},obs));
 assert(std::string(n.reason())=="ESCAPING_CLEARANCE_MARGIN");
 auto path=n.path();double previous=std::hypot(s.x-6.835,s.y+.064)-.910-.65;
 // The initial recovery segment stays outside obstacle+aircraft and moves outward.
 for(size_t i=1;i<path.size();++i){
  double x0=i==1?s.x:path[i-1][0],y0=i==1?s.y:path[i-1][1];
  for(int k=1;k<=12;k++){double a=k/12.,x=x0+a*(path[i][0]-x0),y=y0+a*(path[i][1]-y0),d=std::hypot(x-6.835,y+.064)-.910-.65;
   assert(d>-.31);if(previous<0)assert(d+1e-8>=previous);previous=d;
  }
 }
 assert(n.step(s,true).active);assert(!n.step(s,false).active);
 s.x=6.9;s.y=0;assert(!n.plan(s,{7.07,1.76,1.25},obs));assert(std::string(n.reason())=="START_INSIDE_OBSTACLE");
 s={0,0,1.25,0,0,0,0};assert(!n.plan(s,{6.835,-.064,1.25},obs));assert(std::string(n.reason())=="GOAL_INSIDE_CLEARANCE");
 assert(n.plan(s,{8,2,1.25},obs));
 qcopter::NavigationConfig c;c.max_speed_mps=0;assert(!n.configure(c));c.max_speed_mps=.4;assert(n.configure(c));
 std::cout<<"PASS: margin exit, hard obstacle block, unsafe goal block, normal path, freshness and config validation\n";
}
