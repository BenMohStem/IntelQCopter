#include <rclcpp/rclcpp.hpp>
#include "qcopter_autonomy/srv/request_mission.hpp"
#include <iostream>
#include <chrono>
int main(int argc,char**argv){
  rclcpp::init(argc,argv);auto node=rclcpp::Node::make_shared("qcopter_mission_selector");
  // Explicit local coordinates; selects a mission, does not move or arm hardware.
  auto req=std::make_shared<qcopter_autonomy::srv::RequestMission::Request>();
  std::cout<<"Start x y z, finish x y z, cruise height (metres, local frame): "<<std::flush;
  if(!(std::cin>>req->start.x>>req->start.y>>req->start.z>>req->finish.x>>req->finish.y>>req->finish.z>>req->cruise_height_m))return 2;
  auto client=node->create_client<qcopter_autonomy::srv::RequestMission>("mission/request");
  if(!client->wait_for_service(std::chrono::seconds(5))){std::cerr<<"Mission service unavailable\n";return 3;}
  auto future=client->async_send_request(req);
  if(rclcpp::spin_until_future_complete(node,future,std::chrono::seconds(5))!=rclcpp::FutureReturnCode::SUCCESS)return 4;
  auto reply=future.get();std::cout<<reply->reason<<'\n';rclcpp::shutdown();return reply->accepted?0:1;
}
