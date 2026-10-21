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
#include "profile.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

static uint32_t crc32(const uint8_t*p,size_t n){uint32_t c=~0u;while(n--){c^=*p++;for(int i=0;i<8;i++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
static void u32(std::vector<uint8_t>&p,unsigned offset,uint32_t v){for(unsigned i=0;i<4;i++)p[offset+i]=uint8_t(v>>(8*i));}

// The same ROS C++ mission computer runs on Windows simulation, Pi or Jetson.
// PC supplies endpoints only; obstacles arrive from the RGB perception pipeline.
class PilotMissionNode:public rclcpp::Node {
 public:
  PilotMissionNode():Node("drone_mission_computer"),profile_(*this) {
    nav_config_.position_gain=declare_parameter<double>("nav_position_gain",1.0);
    nav_config_.velocity_gain=declare_parameter<double>("nav_velocity_gain",1.8);
    nav_config_.max_speed_mps=declare_parameter<double>("nav_max_speed_mps",0.8);
    nav_config_.max_acceleration_mps2=declare_parameter<double>("nav_max_acceleration_mps2",1.4);
    nav_config_.max_angle_deg=declare_parameter<double>("nav_max_angle_deg",15.0);
    nav_config_.heading_gain=declare_parameter<double>("nav_heading_gain",1.2);
    nav_config_.max_yaw_rate_dps=declare_parameter<double>("nav_max_yaw_rate_dps",45.0);
    if(!navigator_.configure(nav_config_))throw std::runtime_error("Invalid navigation settings");
    RCLCPP_INFO(get_logger(),"NAV_CONFIG position_gain=%.6g velocity_gain=%.6g max_speed_mps=%.6g max_acceleration_mps2=%.6g max_angle_deg=%.6g heading_gain=%.6g max_yaw_rate_dps=%.6g",nav_config_.position_gain,nav_config_.velocity_gain,nav_config_.max_speed_mps,nav_config_.max_acceleration_mps2,nav_config_.max_angle_deg,nav_config_.heading_gain,nav_config_.max_yaw_rate_dps);
    frames_=create_publisher<std_msgs::msg::UInt8MultiArray>("fc/companion_tx",10);
    status_=create_publisher<std_msgs::msg::String>("mission/status",10);
    path_=create_publisher<std_msgs::msg::String>("mission/path",10);
    mode_=create_subscription<std_msgs::msg::UInt8MultiArray>("fc/status",10,[this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m){
      const auto&p=m->data;if(p.size()!=16||p[0]!='Q'||p[1]!='S'||p[2]!=1)return;
      uint32_t c=0;for(unsigned i=0;i<4;i++)c|=uint32_t(p[12+i])<<(8*i);
      if(c!=crc32(p.data(),12))return;
      status_time_=now().seconds();mode_nav_=p[3]==1;disarmed_=p[4]!=0;pilot_fresh_=p[5]!=0;
      if((!mode_nav_||disarmed_)&&active_){active_=false;mission_=qcopter::Mission{};navigator_=qcopter::Navigator{};navigator_.configure(nav_config_);planned_=false;}
    });
    contact_=create_subscription<std_msgs::msg::Bool>("simulation/ground_contact",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){ground_contact_=m->data;contact_time_=now().seconds();});
    obstacles_=create_subscription<qcopter_autonomy::msg::MetricObstacles>("perception/metric_obstacles",10,[this](qcopter_autonomy::msg::MetricObstacles::ConstSharedPtr m){
      if(m->header.frame_id!="map"||!m->image_processed||m->centers.size()!=m->radii_m.size())return;
      double t=rclcpp::Time(m->header.stamp).seconds();
      const double age=now().seconds()-t;
      if(t<=camera_time_||age<-.1||age>.65||m->centers.size()>256){profile_.reject();return;}
      camera_time_=t;images_++;
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
      res->reason=res->accepted?"Accepted by drone ROS2 computer":last_route_error_;
    });
    route_request_=create_service<qcopter_autonomy::srv::RequestRoute>("mission/route",[this](qcopter_autonomy::srv::RequestRoute::Request::SharedPtr req,qcopter_autonomy::srv::RequestRoute::Response::SharedPtr res){
      std::vector<std::array<double,3>> points;for(const auto&p:req->points)points.push_back({p.x,p.y,p.z});
      res->accepted=start_route(points,req->cruise_height_m);
      res->reason=res->accepted?"Approved route accepted; visiting all intermediate nodes before final landing":last_route_error_;
    });
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if(m->header.frame_id!="map"||m->child_frame_id!="map")return;
      double t=rclcpp::Time(m->header.stamp).seconds();if(have_pose_&&t<=pose_time_)return;
      const double age=now().seconds()-t;
      if(age<-.1||age>.25){profile_.reject();return;}
      double dt=have_pose_?t-pose_time_:.02;
      const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;const auto&v=m->twist.twist.linear;
      const double values[]={p.x,p.y,p.z,v.x,v.y,v.z,q.x,q.y,q.z,q.w};
      for(double x:values)if(!std::isfinite(x)){profile_.reject();return;}
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(std::abs(norm-1)>.02||std::abs(p.x)>100000||std::abs(p.y)>100000||std::abs(p.z)>100000
         ||std::hypot(v.x,v.y)>50||std::abs(v.z)>50){profile_.reject();return;}
      pose_time_=t;have_pose_=true;
      state_={p.x,p.y,p.z,v.x,v.y,v.z,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
      tick(std::clamp(dt,.001,.1));
    });
    RCLCPP_INFO(get_logger(),"READY: endpoints via mission/request; authority via Xbox FC mode; raw UART frames on fc/companion_tx");
  }
 private:
  CallbackProfile profile_;
  bool start_route(const std::vector<std::array<double,3>>&points,double cruise){
    auto reject=[this](const std::string&why){last_route_error_=why;return false;};
    if(disarmed_)return reject("FC disarm is latched; restart the simulator");
    if(!mode_nav_)return reject("FC is in stable mode; select navigation or enable simulator navigation selection");
    if(!pilot_fresh_)return reject("No fresh Xbox pilot reports reach the FC");
    if(!have_pose_||now().seconds()-pose_time_>.25||now().seconds()-pose_time_<-.1)return reject("Drone odometry unavailable or stale");
    if(now().seconds()-status_time_>.75||now().seconds()-status_time_<0)return reject("FC status stale");
    if(active_)return reject("A mission is already running; switch to stable mode to cancel it first");
    if(points.size()<2||points.size()>64)return reject("Choose 2 to 64 nodes");
    for(size_t i=0;i<points.size();i++){
      const auto&p=points[i];for(double v:p)if(!std::isfinite(v))return reject("Node "+std::to_string(i+1)+": coordinates must be finite");
      if(p[0]<-1||p[0]>9||std::abs(p[1])>3)return reject("Node "+std::to_string(i+1)+": outside x=-1..9, y=-3..3 metres");
      if(std::abs(p[2]-points.front()[2])>.15)return reject("All nodes must use the same ground height; intermediate nodes are flown at cruise height");
      if(i&&std::hypot(p[0]-points[i-1][0],p[1]-points[i-1][1])<.4)return reject("Consecutive nodes must be at least 0.4 metres apart");
      for(const auto&o:map_)if(i&&std::hypot(p[0]-o.x,p[1]-o.y)<o.radius+.65)return reject("Node "+std::to_string(i+1)+": inside observed obstacle clearance; move it away from the orange region");
    }
    if(!mission_.request(state_,points.front(),points.back(),cruise))return reject("Start must match the stationary drone on the ground; cruise z must be 0.6..2.5m and at least 0.4m above start");
    route_=points;route_index_=1;cruise_=cruise;active_=true;dirty_=true;requests_++;
    finish_={points[1][0],points[1][1],cruise};mission_.enable_finish(route_.size()==2);
    RCLCPP_INFO(get_logger(),"PC_ROUTE_ACCEPTED points=%zu finish=(%.2f,%.2f)",points.size(),points.back()[0],points.back()[1]);return true;
  }
  void tick(double dt){
    CallbackProfile::Scope scope(profile_);
    if(!active_||!mode_nav_||disarmed_||!pilot_fresh_)return;
    if(now().seconds()-status_time_>.75||now().seconds()-status_time_<0){profile_.reject();return;}
    const bool contact_fresh=now().seconds()-contact_time_>=0&&now().seconds()-contact_time_<.2;
    if(phase_==qcopter::Phase::Navigate&&route_index_+1<route_.size()
       &&std::hypot(state_.x-finish_[0],state_.y-finish_[1])<.2
       &&std::hypot(state_.vx,state_.vy)<.2&&std::abs(state_.z-cruise_)<.15){
      RCLCPP_INFO(get_logger(),"ROUTE_WAYPOINT_REACHED index=%zu x=%.3f y=%.3f",route_index_,state_.x,state_.y);
      ++route_index_;finish_={route_[route_index_][0],route_[route_index_][1],cruise_};dirty_=true;
      mission_.enable_finish(route_index_+1==route_.size());
    }
    bool camera_fresh=pose_time_-camera_time_>=-.1&&pose_time_-camera_time_<.65;
    if(dirty_||(!planned_&&pose_time_-last_plan_time_>1.0)){last_plan_time_=pose_time_;planned_=navigator_.plan(state_,finish_,map_);dirty_=false;plans_++;RCLCPP_INFO(get_logger(),"PLANNER waypoint=%zu planned=%d reason=%s",route_index_,planned_,navigator_.reason());
      std::ostringstream out;for(auto p:navigator_.path())out<<p[0]<<','<<p[1]<<';';std_msgs::msg::String msg;msg.data=out.str();path_->publish(msg);}
    auto nav=navigator_.step(state_,camera_fresh&&planned_);
    auto out=mission_.step(state_,nav,true,camera_fresh,ground_contact_&&contact_fresh,dt);
    std_msgs::msg::UInt8MultiArray frame;frame.data.resize(28,0);auto&p=frame.data;
    p[0]='Q';p[1]='N';p[2]=1;p[3]=out.done&&ground_contact_&&contact_fresh?3:1;u32(p,4,++sequence_);
    float vals[]={float(out.command.roll_deg),float(out.command.pitch_deg),float(out.command.yaw_rate_dps),float((out.command.throttle-.5)*4)};
    for(unsigned i=0;i<4;i++){uint32_t bits;std::memcpy(&bits,vals+i,4);u32(p,8+4*i,bits);}u32(p,24,crc32(p.data(),24));
    frames_->publish(frame);
    std::ostringstream text;text<<"phase="<<int(out.phase)<<",done="<<out.done<<",mapped="<<map_.size()<<",plans="<<plans_<<",images="<<images_<<",requests="<<requests_<<",camera="<<camera_fresh<<",waypoint="<<route_index_<<",points="<<route_.size()<<",planned="<<planned_<<",planner_reason="<<navigator_.reason();
    std_msgs::msg::String msg;msg.data=text.str();status_->publish(msg);
    if(out.phase!=phase_){RCLCPP_INFO(get_logger(),"MISSION %s",msg.data.c_str());phase_=out.phase;}
    if(out.done)active_=false;
  }
  bool mode_nav_=false,disarmed_=false,pilot_fresh_=false,have_pose_=false,ground_contact_=false,active_=false,dirty_=false,planned_=false;
  std::string last_route_error_;
  qcopter::NavigationConfig nav_config_{};double last_plan_time_=-100;
  double pose_time_=0,camera_time_=-100,cruise_=1.25,status_time_=-100,contact_time_=-100;
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
