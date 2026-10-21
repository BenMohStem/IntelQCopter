#include "qcopter_autonomy/mission.hpp"
#include "qcopter_autonomy/msg/autonomy_setpoint.hpp"
#include "qcopter_autonomy/msg/metric_obstacles.hpp"
#include "qcopter_autonomy/srv/request_mission.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <cmath>

class MissionNode:public rclcpp::Node {
 public:
  MissionNode():Node("qcopter_mission") {
    // Defaults inactive. This is mission setpoint authority, never physical arming.
    enabled_=declare_parameter<bool>("enable_simulation_missions",false);
    publisher_=create_publisher<qcopter_autonomy::msg::AutonomySetpoint>("autonomy/setpoint",10);
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if((m->header.frame_id!="map"&&m->header.frame_id!="odom")||m->child_frame_id!=m->header.frame_id)return;
      if(have_state_ && (m->header.frame_id!=frame_ || rclcpp::Time(m->header.stamp)<=odom_time_))return;
      const auto&q=m->pose.pose.orientation;const auto&p=m->pose.pose.position;const auto&v=m->twist.twist.linear;
      state_={p.x,p.y,p.z,v.x,v.y,v.z,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
      odom_time_=m->header.stamp;frame_=m->header.frame_id;have_state_=true;
    });
    obstacles_=create_subscription<qcopter_autonomy::msg::MetricObstacles>("perception/metric_obstacles",10,[this](qcopter_autonomy::msg::MetricObstacles::ConstSharedPtr m){
      if(!have_state_||m->header.frame_id!=frame_||m->centers.size()!=m->radii_m.size()||!m->image_processed)return;
      const double age=(now()-rclcpp::Time(m->header.stamp)).seconds();if(age<0||age>.5)return;
      for(size_t i=0;i<m->centers.size();i++) {
        const auto&p=m->centers[i];double r=m->radii_m[i];
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(r)||r<.1||r>2)continue;
        bool found=false;
        for(auto &o:map_)if(std::hypot(o.x-p.x,o.y-p.y)<.65){o.x=.8*o.x+.2*p.x;o.y=.8*o.y+.2*p.y;o.radius=std::max(o.radius,r);found=true;break;}
        if(!found&&map_.size()<200)map_.push_back({p.x,p.y,r});
      }
      perception_time_=m->header.stamp;replan_=true;
    });
    contact_=create_subscription<std_msgs::msg::Bool>("simulation/ground_contact",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){contact_state_=m->data;contact_time_=now();});
    request_=create_service<qcopter_autonomy::srv::RequestMission>("mission/request",[this](qcopter_autonomy::srv::RequestMission::Request::SharedPtr req,qcopter_autonomy::srv::RequestMission::Response::SharedPtr res){
      const auto&a=req->start;const auto&b=req->finish;
      bool fresh=have_state_&&(now()-odom_time_).seconds()>=0&&(now()-odom_time_).seconds()<.25;
      res->accepted=enabled_&&fresh&&mission_.request(state_,{a.x,a.y,a.z},{b.x,b.y,b.z},req->cruise_height_m);
      res->reason=res->accepted?"Simulation mission accepted; no physical arming":"Inactive, stale pose, busy, invalid points, or start differs from actual pose";
      if(res->accepted){finish_={b.x,b.y,req->cruise_height_m};replan_=true;}
    });
    land_=create_service<std_srvs::srv::Trigger>("mission/land",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
      response->success=enabled_&&have_state_&&(now()-odom_time_).seconds()>=0&&(now()-odom_time_).seconds()<.25;
      if(response->success)mission_.request_land(state_);
      response->message=response->success?"Land at current horizontal position; not a motor-cut command":"No fresh authorized simulation pose";
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      const bool pose_fresh=have_state_&&(now()-odom_time_).seconds()>=0&&(now()-odom_time_).seconds()<.25;
      const bool camera_fresh=(now()-perception_time_).seconds()>=0&&(now()-perception_time_).seconds()<.55;
      if(replan_&&pose_fresh){navigator_.plan(state_,finish_,map_);replan_=false;}
      auto nav=navigator_.step(state_,camera_fresh&&pose_fresh);
      const bool contact_fresh=(now()-contact_time_).seconds()>=0&&(now()-contact_time_).seconds()<.15;
      auto output=mission_.step(state_,nav,pose_fresh,camera_fresh,contact_state_&&contact_fresh,.05);
      qcopter_autonomy::msg::AutonomySetpoint msg;msg.header.stamp=now();msg.header.frame_id="base_link";
      constexpr double rad=3.141592653589793/180;
      msg.roll_angle_rad=output.command.roll_deg*rad;msg.pitch_angle_rad=output.command.pitch_deg*rad;
      msg.yaw_rate_rad_s=output.command.yaw_rate_dps*rad;msg.centered_throttle=output.command.throttle;
      msg.valid=enabled_&&output.command.active;msg.goal_reached=output.done;publisher_->publish(msg);
      // output.motor_gate is intentionally NOT published as hardware arming permission.
    });
  }
 private:
  bool enabled_=false,have_state_=false,contact_state_=false,replan_=false;std::string frame_;
  qcopter::State state_{};qcopter::Mission mission_;qcopter::Navigator navigator_;
  std::vector<qcopter::Obstacle> map_;std::array<double,3>finish_{};
  rclcpp::Time odom_time_{0,0,RCL_ROS_TIME},perception_time_{0,0,RCL_ROS_TIME},contact_time_{0,0,RCL_ROS_TIME};
  rclcpp::Publisher<qcopter_autonomy::msg::AutonomySetpoint>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<qcopter_autonomy::msg::MetricObstacles>::SharedPtr obstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_;
  rclcpp::Service<qcopter_autonomy::srv::RequestMission>::SharedPtr request_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<MissionNode>());rclcpp::shutdown();}
