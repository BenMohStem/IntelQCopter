#include "qcopter_autonomy/navigation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "qcopter_autonomy/msg/autonomy_setpoint.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <cmath>

// Same source on Pi 4/5 and Jetson. No actuator driver or arming authority.
// Map circles: [x,y,radius,...], in the same local ENU frame as odometry.
class NavigationNode : public rclcpp::Node {
 public:
  NavigationNode() : Node("qcopter_navigator") {
    auto map=declare_parameter<std::vector<double>>("obstacles",{});
    if(map.size()%3) throw std::invalid_argument("obstacles must be x,y,r triplets");
    for(size_t i=0;i<map.size();i+=3) obstacles_.push_back({map[i],map[i+1],map[i+2]});
    pub_=create_publisher<qcopter_autonomy::msg::AutonomySetpoint>("autonomy/setpoint",10);
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,[this](auto m){
      if(m->header.frame_id!="map" && m->header.frame_id!="odom") return;
      if(m->child_frame_id!=m->header.frame_id) return; // require explicit world-twist adapter
      if(have_state_ && m->header.frame_id==frame_ && rclcpp::Time(m->header.stamp)<=stamp_) return;
      if(have_state_ && m->header.frame_id!=frame_) planned_=false;
      const auto &p=m->pose.pose.position;const auto &q=m->pose.pose.orientation;
      const auto &v=m->twist.twist.linear;
      // Require velocity expressed in the world frame via the documented adapter.
      state_={p.x,p.y,p.z,v.x,v.y,v.z,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
      stamp_=m->header.stamp;frame_=m->header.frame_id;have_state_=true;
    });
    goal_=create_subscription<geometry_msgs::msg::PoseStamped>("goal",10,[this](auto m){
      if(!have_state_ || m->header.frame_id!=frame_) return;
      const auto &p=m->pose.position; planned_=navigator_.plan(state_,{p.x,p.y,p.z},obstacles_);
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      const double age=(now()-stamp_).seconds();
      auto c=navigator_.step(state_,planned_ && have_state_ && age>=0 && age<0.25);
      qcopter_autonomy::msg::AutonomySetpoint m;m.header.stamp=now();m.header.frame_id="base_link";
      constexpr double rad=3.14159265358979323846/180.;
      m.roll_angle_rad=c.roll_deg*rad;m.pitch_angle_rad=c.pitch_deg*rad;
      m.yaw_rate_rad_s=c.yaw_rate_dps*rad;m.centered_throttle=c.throttle;
      m.valid=c.active;m.goal_reached=c.reached;pub_->publish(m);
    });
  }
 private:
  qcopter::Navigator navigator_;qcopter::State state_{};
  std::vector<qcopter::Obstacle> obstacles_;bool have_state_=false,planned_=false;
  std::string frame_;
  rclcpp::Time stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
  rclcpp::Publisher<qcopter_autonomy::msg::AutonomySetpoint>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<NavigationNode>());rclcpp::shutdown();}
