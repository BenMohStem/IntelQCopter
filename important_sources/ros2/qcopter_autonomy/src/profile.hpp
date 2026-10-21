#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <sstream>
#include <cstdint>

// Wall-time callback profiling: independent of /clock and simulation speed.
// Single-threaded executor ownership; fixed counters, no per-sample log/file I/O.
class CallbackProfile {
 using Clock=std::chrono::steady_clock;
 rclcpp::Node &node_; int level_;double budget_;
 uint64_t calls_=0,rejected_=0,over_=0;double sum_=0,max_=0;
 std::array<uint64_t,10> bins_{};
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
 Clock::time_point last_=Clock::now();
 public:
 explicit CallbackProfile(rclcpp::Node &node):node_(node){
  level_=node.declare_parameter<int>("debug_level",2);
  budget_=node.declare_parameter<double>("profile_budget_us",20000.);
  if(level_<0||level_>3||!std::isfinite(budget_)||budget_<=0)throw std::invalid_argument("Invalid debug/profile settings");
  pub_=node.create_publisher<std_msgs::msg::String>("debug/profile",10);
 }
 void reject(){if(level_)rejected_++;}
 void finish(Clock::time_point start){
  if(!level_)return;
  const auto now=Clock::now();double us=std::chrono::duration<double,std::micro>(now-start).count();
  calls_++;sum_+=us;max_=std::max(max_,us);over_+=us>budget_;
  constexpr double limits[]={50,100,250,500,1000,5000,20000,50000,200000};
  unsigned i=0;while(i<9&&us>limits[i])i++;bins_[i]++;
  if(std::chrono::duration<double>(now-last_).count()<2)return;
  last_=now;std::ostringstream text;
  text<<"component="<<node_.get_name()<<",clock=steady_wall,level="<<level_<<",calls="<<calls_<<",rejected="<<rejected_
      <<",mean_us="<<sum_/calls_<<",max_us="<<max_<<",budget_us="<<budget_<<",over_budget="<<over_<<",histogram=";
  for(auto n:bins_)text<<n<<':';
  std_msgs::msg::String m;m.data=text.str();pub_->publish(m);
  if(level_>=3)RCLCPP_INFO(node_.get_logger(),"PROFILE %s",m.data.c_str());
 }
 struct Scope{CallbackProfile &p;Clock::time_point start=Clock::now();explicit Scope(CallbackProfile &owner):p(owner){}~Scope(){p.finish(start);}};
};
