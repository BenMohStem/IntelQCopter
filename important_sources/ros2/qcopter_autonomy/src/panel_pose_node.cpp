#include <rclcpp/rclcpp.hpp>
#include "profile.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "image_view.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "qcopter_autonomy/msg/semantic_detections.hpp"
#include "qcopter_autonomy/msg/metric_obstacles.hpp"
#include <algorithm>
#include <cmath>
#include <climits>
#include <deque>

// Controlled known-size blue panels; not general monocular obstacle ranging.
// Same camera->PnP->world geometry as the executed Python reference.
class PanelPoseNode:public rclcpp::Node {
 public:
  PanelPoseNode():Node("qcopter_panel_pose"),profile_(*this) {
    width_=declare_parameter<double>("fixture_inner_width_m",1.09);
    height_=declare_parameter<double>("fixture_inner_height_m",1.69);
    center_z_=declare_parameter<double>("fixture_center_z_m",.9);
    height_tolerance_=declare_parameter<double>("fixture_center_tolerance_m",.2);
    if(!std::isfinite(center_z_)||!std::isfinite(height_tolerance_)||height_tolerance_<=0||height_tolerance_>.5)
      throw std::invalid_argument("Invalid known-fixture height prior");
    if(!declare_parameter<bool>("known_fixture_model_confirmed",false))
      throw std::invalid_argument("Confirm known-size panel model and use its trained detector");
    pub_=create_publisher<qcopter_autonomy::msg::MetricObstacles>("perception/metric_obstacles",10);
    rgb_=create_subscription<sensor_msgs::msg::Image>("camera/image",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Image::ConstSharedPtr m){image_=m;images_.push_back(m);if(images_.size()>24)images_.pop_front();});
    intrinsics_=create_subscription<sensor_msgs::msg::CameraInfo>("camera/camera_info",10,[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){camera_=m;});
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){pose_=m;poses_.push_back(m);if(poses_.size()>150)poses_.pop_front();});
    detections_=create_subscription<qcopter_autonomy::msg::SemanticDetections>("perception/semantics",10,[this](qcopter_autonomy::msg::SemanticDetections::ConstSharedPtr m){process(m);});
    RCLCPP_INFO(get_logger(),"READY_PNP: known fixture %.2fx%.2f m; map centre z=%.2f +/-%.2f m",width_,height_,center_z_,height_tolerance_);
  }
 private:
  CallbackProfile profile_;
  void process(const qcopter_autonomy::msg::SemanticDetections::ConstSharedPtr&m){
    CallbackProfile::Scope scope(profile_);
    const double stamp=rclcpp::Time(m->header.stamp).seconds();
    double image_error=1e9,pose_error=1e9;
    for(auto image:images_){double e=std::abs(rclcpp::Time(image->header.stamp).seconds()-stamp);if(e<image_error){image_=image;image_error=e;}}
    for(auto pose:poses_){double e=std::abs(rclcpp::Time(pose->header.stamp).seconds()-stamp);if(e<pose_error){pose_=pose;pose_error=e;}}
    if(image_error>.001||pose_error>.06)return;
    if(!image_||!camera_||!pose_||camera_->k[0]<=0||camera_->k[4]<=0||width_<=0||height_<=0)return;
    if(image_->header.frame_id!=m->header.frame_id||camera_->header.frame_id!=m->header.frame_id)return;
    if(std::abs((rclcpp::Time(image_->header.stamp)-rclcpp::Time(m->header.stamp)).seconds())>.04
      ||std::abs((rclcpp::Time(pose_->header.stamp)-rclcpp::Time(m->header.stamp)).seconds())>.10)return;
    try {
      const auto bgr=qcopter::image_copy(*image_);cv::Mat hsv,mask;
      cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);cv::inRange(hsv,cv::Scalar(95,110,70),cv::Scalar(130,255,255),mask);
      cv::Mat k=(cv::Mat_<double>(3,3)<<camera_->k[0],0,camera_->k[2],0,camera_->k[4],camera_->k[5],0,0,1);
      // Images must be rectified: require explicit zero distortion, do not ignore unknown distortion.
      for(double d:camera_->d)if(!std::isfinite(d)||std::abs(d)>1e-8)return;
      qcopter_autonomy::msg::MetricObstacles out;out.header.stamp=m->header.stamp;out.header.frame_id=pose_->header.frame_id;out.image_processed=true;
      const auto &q=pose_->pose.pose.orientation;
      const double norm=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);if(!std::isfinite(norm)||std::abs(norm-1)>.01)return;
      cv::Matx33d r(1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w),
        2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w),
        2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y));
      for(const auto&det:m->detections){
        if(det.class_id!=0||det.confidence<.35)continue;
        bool finite=std::isfinite(det.confidence);for(float v:det.xyxy_pixels)finite=finite&&std::isfinite(v)&&std::abs(v)<1e7f;
        if(!finite)continue;
        cv::Mat roi=cv::Mat::zeros(mask.size(),CV_8U);
        int x1=std::clamp(int(det.xyxy_pixels[0])-5,0,bgr.cols),y1=std::clamp(int(det.xyxy_pixels[1])-5,0,bgr.rows);
        int x2=std::clamp(int(det.xyxy_pixels[2])+6,0,bgr.cols),y2=std::clamp(int(det.xyxy_pixels[3])+6,0,bgr.rows);
        if(x2<=x1||y2<=y1)continue;mask(cv::Rect(x1,y1,x2-x1,y2-y1)).copyTo(roi(cv::Rect(x1,y1,x2-x1,y2-y1)));
        std::vector<std::vector<cv::Point>> contours;cv::findContours(roi,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        std::sort(contours.begin(),contours.end(),[](auto&a,auto&b){return cv::contourArea(a)>cv::contourArea(b);});
        for(const auto&c:contours){
          if(cv::contourArea(c)<100)continue;std::vector<cv::Point> poly;cv::approxPolyDP(c,poly,.025*cv::arcLength(c,true),true);
          if(poly.size()!=4||!cv::isContourConvex(poly))continue;
          bool clipped=false;for(auto p:poly)if(p.x<3||p.y<3||p.x>bgr.cols-4||p.y>bgr.rows-4)clipped=true;if(clipped)continue;
          std::vector<cv::Point2f> corner(4);int minsum=INT_MAX,maxsum=INT_MIN,mindiff=INT_MAX,maxdiff=INT_MIN;
          for(auto p:poly){int sum=p.x+p.y,diff=p.y-p.x;if(sum<minsum){minsum=sum;corner[0]=p;}if(sum>maxsum){maxsum=sum;corner[2]=p;}
            if(diff<mindiff){mindiff=diff;corner[1]=p;}if(diff>maxdiff){maxdiff=diff;corner[3]=p;}}
          std::vector<cv::Point3f> object={{float(-width_/2),float(-height_/2),0},{float(width_/2),float(-height_/2),0},
            {float(width_/2),float(height_/2),0},{float(-width_/2),float(height_/2),0}};
          cv::Mat rv,tv;if(!cv::solvePnP(object,corner,k,cv::Mat::zeros(5,1,CV_64F),rv,tv,false,cv::SOLVEPNP_IPPE))continue;
          if(!cv::checkRange(tv)||tv.at<double>(2)<.3||tv.at<double>(2)>9)continue;
          std::vector<cv::Point2f> projected;cv::projectPoints(object,rv,tv,k,cv::Mat::zeros(5,1,CV_64F),projected);
          double error=0;for(size_t i=0;i<4;i++){auto e=projected[i]-corner[i];error+=e.dot(e);}if(std::sqrt(error/4)>2)continue;
          // Optical right/down/forward -> FLU forward/left/up, then calibrated .18m,.02m lever arm.
          cv::Vec3d body(tv.at<double>(2)+.18,-tv.at<double>(0),-tv.at<double>(1)+.02);auto world=r*body;
          geometry_msgs::msg::Point p;p.x=world[0]+pose_->pose.pose.position.x;p.y=world[1]+pose_->pose.pose.position.y;p.z=world[2]+pose_->pose.pose.position.z;
          // An occluded panel can leave a smaller four-corner blue silhouette.
          // PnP may fit that silhouette with a false distance. These controlled
          // fixtures stand on a known flat floor: reject inconsistent centres.
          // This prior is not a general monocular ranging solution.
          if(!std::isfinite(p.z)||std::abs(p.z-center_z_)>height_tolerance_)continue;
          out.centers.push_back(p);out.radii_m.push_back(.61+std::min(.3,2*(.04*tv.at<double>(2)+.03)));break;
        }
      }
      pub_->publish(out);
    }catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Panel pose rejected: %s",e.what());}
  }
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> poses_;
  double width_,height_,center_z_,height_tolerance_;sensor_msgs::msg::Image::ConstSharedPtr image_;sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_;nav_msgs::msg::Odometry::ConstSharedPtr pose_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_;rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr intrinsics_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;rclcpp::Subscription<qcopter_autonomy::msg::SemanticDetections>::SharedPtr detections_;
  rclcpp::Publisher<qcopter_autonomy::msg::MetricObstacles>::SharedPtr pub_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<PanelPoseNode>());rclcpp::shutdown();}
