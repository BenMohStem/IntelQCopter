#include "qcopter_autonomy/mission.hpp"
#include "qcopter_autonomy/msg/autonomy_setpoint.hpp"
#include "qcopter_autonomy/msg/metric_obstacles.hpp"
#include "qcopter_autonomy/srv/request_mission.hpp"
#include "qcopter_autonomy/srv/request_route.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

static uint32_t crc32(const uint8_t*p,size_t n){uint32_t c=~0u;while(n--){c^=*p++;for(int i=0;i<8;i++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
static void u32(std::vector<uint8_t>&p,unsigned offset,uint32_t v){for(unsigned i=0;i<4;i++)p[offset+i]=uint8_t(v>>(8*i));}

// The same ROS C++ mission computer runs on Windows simulation, Pi or Jetson.
// PC supplies endpoints only; obstacles arrive from the RGB perception pipeline.
class PilotMissionNode:public rclcpp::Node {
 public:
  PilotMissionNode():Node("drone_mission_computer") {
    frames_=create_publisher<std_msgs::msg::UInt8MultiArray>("fc/companion_tx",10);
    status_=create_publisher<std_msgs::msg::String>("mission/status",10);
    path_=create_publisher<std_msgs::msg::String>("mission/path",10);
    mode_=create_subscription<std_msgs::msg::UInt8MultiArray>("fc/status",10,[this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m){
      const auto&p=m->data;if(p.size()!=16||p[0]!='Q'||p[1]!='S'||p[2]!=1)return;
      uint32_t c=0;for(unsigned i=0;i<4;i++)c|=uint32_t(p[12+i])<<(8*i);
      if(c!=crc32(p.data(),12))return;
      mode_nav_=p[3]==1;disarmed_=p[4]!=0;pilot_fresh_=p[5]!=0;
      if((!mode_nav_||disarmed_)&&active_){active_=false;mission_=qcopter::Mission{};navigator_=qcopter::Navigator{};planned_=false;}
    });
    contact_=create_subscription<std_msgs::msg::Bool>("simulation/ground_contact",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){ground_contact_=m->data;});
    obstacles_=create_subscription<qcopter_autonomy::msg::MetricObstacles>("perception/metric_obstacles",10,[this](qcopter_autonomy::msg::MetricObstacles::ConstSharedPtr m){
      if(m->header.frame_id!="map"||!m->image_processed||m->centers.size()!=m->radii_m.size())return;
      double t=rclcpp::Time(m->header.stamp).seconds();if(t<camera_time_)return;camera_time_=t;images_++;
      for(size_t i=0;i<m->centers.size();i++){
        auto p=m->centers[i];double r=m->radii_m[i];
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(r)||r<.1||r>2)continue;
        bool found=false;
        for(auto&o:map_)if(std::hypot(o.x-p.x,o.y-p.y)<.75){
          if(std::hypot(o.x-p.x,o.y-p.y)>.12)dirty_=true;
          o.x=.9*o.x+.1*p.x;o.y=.9*o.y+.1*p.y;o.radius=std::max(o.radius,r);found=true;break;}
        if(!found&&map_.size()<100){map_.push_back({p.x,p.y,r});dirty_=true;
          RCLCPP_INFO(get_logger(),"RGB_OBSTACLE x=%.3f y=%.3f r=%.3f count=%zu",p.x,p.y,r,map_.size());}
      }
    });
    request_=create_service<qcopter_autonomy::srv::RequestMission>("mission/request",[this](qcopter_autonomy::srv::RequestMission::Request::SharedPtr req,qcopter_autonomy::srv::RequestMission::Response::SharedPtr res){
      res->accepted=start_route({{req->start.x,req->start.y,req->start.z},{req->finish.x,req->finish.y,req->finish.z}},req->cruise_height_m);
      res->reason=res->accepted?"Accepted by drone ROS2 computer":"Require navigation, fresh pilot/pose, current start and valid route";
    });
    route_request_=create_service<qcopter_autonomy::srv::RequestRoute>("mission/route",[this](qcopter_autonomy::srv::RequestRoute::Request::SharedPtr req,qcopter_autonomy::srv::RequestRoute::Response::SharedPtr res){
      std::vector<std::array<double,3>> points;for(const auto&p:req->points)points.push_back({p.x,p.y,p.z});
      res->accepted=start_route(points,req->cruise_height_m);
      res->reason=res->accepted?"Route accepted; intermediate waypoint flown before final landing":"Require 2 or 3 valid points, current start, fresh pilot/pose and Xbox navigation mode";
    });
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if(m->header.frame_id!="map"||m->child_frame_id!="map")return;
      double t=rclcpp::Time(m->header.stamp).seconds();if(have_pose_&&t<=pose_time_)return;
      double dt=have_pose_?t-pose_time_:.02;pose_time_=t;have_pose_=true;
      const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;const auto&v=m->twist.twist.linear;
      state_={p.x,p.y,p.z,v.x,v.y,v.z,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
      tick(std::clamp(dt,.001,.1));
    });
    RCLCPP_INFO(get_logger(),"READY: endpoints via mission/request; authority via Xbox FC mode; raw UART frames on fc/companion_tx");
  }
 private:
  bool start_route(const std::vector<std::array<double,3>>&points,double cruise){
    if(!mode_nav_||disarmed_||!pilot_fresh_||!have_pose_||active_||points.size()<2||points.size()>3)return false;
    for(size_t i=0;i<points.size();i++){
      const auto&p=points[i];for(double v:p)if(!std::isfinite(v))return false;
      if(p[0]<-1||p[0]>9||std::abs(p[1])>3||std::abs(p[2]-points.front()[2])>.15)return false;
      if(i&&std::hypot(p[0]-points[i-1][0],p[1]-points[i-1][1])<.4)return false;
    }
    if(!mission_.request(state_,points.front(),points.back(),cruise))return false;
    route_=points;route_index_=1;cruise_=cruise;active_=true;dirty_=true;requests_++;
    finish_={points[1][0],points[1][1],cruise};mission_.enable_finish(route_.size()==2);
    RCLCPP_INFO(get_logger(),"PC_ROUTE_ACCEPTED points=%zu finish=(%.2f,%.2f)",points.size(),points.back()[0],points.back()[1]);return true;
  }
  void tick(double dt){
    if(!active_||!mode_nav_||disarmed_||!pilot_fresh_)return;
    if(phase_==qcopter::Phase::Navigate&&route_index_+1<route_.size()
       &&std::hypot(state_.x-finish_[0],state_.y-finish_[1])<.2
       &&std::hypot(state_.vx,state_.vy)<.2&&std::abs(state_.z-cruise_)<.15){
      RCLCPP_INFO(get_logger(),"ROUTE_WAYPOINT_REACHED index=%zu x=%.3f y=%.3f",route_index_,state_.x,state_.y);
      ++route_index_;finish_={route_[route_index_][0],route_[route_index_][1],cruise_};dirty_=true;
      mission_.enable_finish(route_index_+1==route_.size());
    }
    bool camera_fresh=pose_time_-camera_time_>=-.1&&pose_time_-camera_time_<.65;
    if(dirty_){planned_=navigator_.plan(state_,finish_,map_);dirty_=false;plans_++;
      std::ostringstream out;for(auto p:navigator_.path())out<<p[0]<<','<<p[1]<<';';std_msgs::msg::String msg;msg.data=out.str();path_->publish(msg);}
    auto nav=navigator_.step(state_,camera_fresh&&planned_);
    auto out=mission_.step(state_,nav,true,camera_fresh,ground_contact_,dt);
    std_msgs::msg::UInt8MultiArray frame;frame.data.resize(28,0);auto&p=frame.data;
    p[0]='Q';p[1]='N';p[2]=1;p[3]=1;u32(p,4,++sequence_);
    float vals[]={float(out.command.roll_deg),float(out.command.pitch_deg),float(out.command.yaw_rate_dps),float((out.command.throttle-.5)*4)};
    for(unsigned i=0;i<4;i++){uint32_t bits;std::memcpy(&bits,vals+i,4);u32(p,8+4*i,bits);}u32(p,24,crc32(p.data(),24));
    frames_->publish(frame);
    std::ostringstream text;text<<"phase="<<int(out.phase)<<",done="<<out.done<<",mapped="<<map_.size()<<",plans="<<plans_<<",images="<<images_<<",requests="<<requests_<<",camera="<<camera_fresh<<",waypoint="<<route_index_<<",points="<<route_.size();
    std_msgs::msg::String msg;msg.data=text.str();status_->publish(msg);
    if(out.phase!=phase_){RCLCPP_INFO(get_logger(),"MISSION %s",msg.data.c_str());phase_=out.phase;}
    if(out.done)active_=false;
  }
  bool mode_nav_=false,disarmed_=false,pilot_fresh_=false,have_pose_=false,ground_contact_=false,active_=false,dirty_=false,planned_=false;
  double pose_time_=0,camera_time_=-100,cruise_=1.25;
  size_t route_index_=0;std::vector<std::array<double,3>>route_;
  unsigned sequence_=0,plans_=0,images_=0,requests_=0;
  qcopter::State state_{};qcopter::Mission mission_;qcopter::Navigator navigator_;qcopter::Phase phase_=qcopter::Phase::Idle;
  std::array<double,3>finish_{};std::vector<qcopter::Obstacle>map_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr frames_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,path_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr mode_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<qcopter_autonomy::msg::MetricObstacles>::SharedPtr obstacles_;
  rclcpp::Service<qcopter_autonomy::srv::RequestMission>::SharedPtr request_;
  rclcpp::Service<qcopter_autonomy::srv::RequestRoute>::SharedPtr route_request_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<PilotMissionNode>());rclcpp::shutdown();}
