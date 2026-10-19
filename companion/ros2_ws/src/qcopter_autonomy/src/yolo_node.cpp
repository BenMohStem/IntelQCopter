#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include "qcopter_autonomy/msg/semantic_detections.hpp"
#include <algorithm>
#include <cmath>

// YOLO11 raw ONNX output only: [1,4+class_count,anchors], no embedded NMS.
// This semantic node does NOT directly mark free space or command navigation.
class YoloNode : public rclcpp::Node {
 public:
  YoloNode():Node("qcopter_yolo") {
    auto model=declare_parameter<std::string>("onnx_model","");
    if(model.empty())throw std::invalid_argument("Set onnx_model to exported YOLO11 path");
    net_=cv::dnn::readNetFromONNX(model);
    if(declare_parameter<bool>("use_cuda",false)) {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    pub_=create_publisher<qcopter_autonomy::msg::SemanticDetections>("perception/semantics",10);
    depth_sub_=create_subscription<sensor_msgs::msg::Image>("camera/depth_registered",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m){depth_=m;});
    rgb_sub_=create_subscription<sensor_msgs::msg::Image>("camera/image",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m){infer(m);});
  }
 private:
  void infer(const sensor_msgs::msg::Image::ConstSharedPtr &m) {
    try {
      auto frame=cv_bridge::toCvShare(m,"bgr8")->image;
      const double scale=std::min(640./frame.cols,640./frame.rows);
      const int w=static_cast<int>(std::round(frame.cols*scale)),h=static_cast<int>(std::round(frame.rows*scale));
      const int left=(640-w)/2,top=(640-h)/2;
      cv::Mat resized,input(640,640,CV_8UC3,cv::Scalar(114,114,114));
      cv::resize(frame,resized,{w,h});resized.copyTo(input(cv::Rect(left,top,w,h)));
      net_.setInput(cv::dnn::blobFromImage(input,1./255.,{640,640},cv::Scalar(),true,false));
      auto raw=net_.forward();
      if(raw.dims!=3 || raw.size[0]!=1 || raw.size[1]<5 || raw.size[1]>1000)
        throw std::runtime_error("Expected raw YOLO11 [1,4+C,N] ONNX output");
      cv::Mat channels(raw.size[1],raw.size[2],CV_32F,raw.ptr<float>()),rows;
      cv::transpose(channels,rows);
      std::vector<cv::Rect> boxes;std::vector<float> scores;std::vector<int> ids;
      for(int i=0;i<rows.rows;i++) {
        const float *p=rows.ptr<float>(i);int cls=0;float confidence=0;
        if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||!std::isfinite(p[3])
           ||std::abs(p[0])>1e7||std::abs(p[1])>1e7||p[2]<=0||p[3]<=0||p[2]>1e7||p[3]>1e7)continue;
        for(int j=4;j<rows.cols;j++)if(p[j]>confidence){confidence=p[j];cls=j-4;}
        if(confidence<.35f || !std::isfinite(confidence))continue;
        const int x1=std::clamp(static_cast<int>((p[0]-.5f*p[2]-left)/scale),0,frame.cols);
        const int y1=std::clamp(static_cast<int>((p[1]-.5f*p[3]-top)/scale),0,frame.rows);
        const int x2=std::clamp(static_cast<int>((p[0]+.5f*p[2]-left)/scale),0,frame.cols);
        const int y2=std::clamp(static_cast<int>((p[1]+.5f*p[3]-top)/scale),0,frame.rows);
        if(x2<=x1||y2<=y1)continue;
        boxes.emplace_back(x1,y1,x2-x1,y2-y1);scores.push_back(confidence);ids.push_back(cls);
      }
      std::vector<int> kept;
      // Class-agnostic NMS is deliberately conservative for overlapping detections.
      cv::dnn::NMSBoxes(boxes,scores,.35f,.45f,kept);
      cv::Mat depth;
      if(depth_ && depth_->header.frame_id==m->header.frame_id && depth_->width==m->width && depth_->height==m->height
        && std::abs((rclcpp::Time(depth_->header.stamp)-rclcpp::Time(m->header.stamp)).seconds())<.05) {
        if(depth_->encoding=="32FC1")depth=cv_bridge::toCvShare(depth_,"32FC1")->image.clone();
        else if(depth_->encoding=="16UC1")cv_bridge::toCvShare(depth_,"16UC1")->image.convertTo(depth,CV_32F,.001);
      }
      qcopter_autonomy::msg::SemanticDetections result;result.header=m->header;
      for(int i:kept) {
        const auto b=boxes[i];qcopter_autonomy::msg::SemanticDetection d;
        d.class_id=ids[i];d.confidence=scores[i];
        d.xyxy_pixels={float(b.x),float(b.y),float(b.x+b.width),float(b.y+b.height)};
        std::vector<float> ranges;
        if(!depth.empty())for(int y=b.y+b.height/4;y<b.y+3*b.height/4;y+=2)
          for(int x=b.x+b.width/4;x<b.x+3*b.width/4;x+=2) {
            const float z=depth.at<float>(y,x);if(std::isfinite(z)&&z>.2f&&z<8)ranges.push_back(z);
          }
        d.range_valid=ranges.size()>=8;d.range_m=0;
        if(d.range_valid){auto middle=ranges.begin()+ranges.size()/2;std::nth_element(ranges.begin(),middle,ranges.end());d.range_m=*middle;}
        result.detections.push_back(d);
      }
      pub_->publish(result);
    }catch(const std::exception&e){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"YOLO frame rejected: %s",e.what());}
  }
  cv::dnn::Net net_;sensor_msgs::msg::Image::ConstSharedPtr depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_,depth_sub_;
  rclcpp::Publisher<qcopter_autonomy::msg::SemanticDetections>::SharedPtr pub_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<YoloNode>());rclcpp::shutdown();}
